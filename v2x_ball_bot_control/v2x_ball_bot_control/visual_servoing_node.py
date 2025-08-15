#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import time
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from geometry_msgs.msg import Twist
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from message_filters import Subscriber as MFSubscriber
from message_filters import ApproximateTimeSynchronizer

def saturate(x, lo, hi):
    return max(lo, min(hi, x))

class VisualServoingNode(Node):
    '''
    Astra 정합 Depth + HSV로 컬러를 받아 공 중심 추정정 → (v, ω) P제어 → /cmd_vel_servo 퍼블리시, 상태는 /servo/status로 냄
    - 선속도: 목표 거리(target_dist_m) 대비 오차
    - 각속도: 화면 중심 대비 X오프셋
    - 출력 토픽: /cmd_vel_servo (twist_mux 입력)
    - 상태 토픽: /servo/status ("ACTIVE" | "DONE" | "LOST" | "IDLE")
    '''

    def __init__(self):
        super().__init__('visual_servoing')

        # ---------------- Params ----------------
        self.declare_parameter('color_topic', '/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('v_max', 0.15)         # m/s
        self.declare_parameter('w_max', 0.6)          # rad/s
        self.declare_parameter('k_v', 0.8)            # 선속도 P게인
        self.declare_parameter('k_w', 0.004)          # 각속도 P게인 (px^-1)
        self.declare_parameter('stop_radius_px', 20.0)
        self.declare_parameter('target_dist_m', 0.45) # 프리그랩 목표 거리
        self.declare_parameter('depth_scale', 0.001)  # Orbbec U16(mm) → m: 0.001
        self.declare_parameter('depth_valid_min', 0.15)
        self.declare_parameter('depth_valid_max', 4.0)
        self.declare_parameter('lost_timeout', 1.0)   # s
        self.declare_parameter('search_yaw_rate', 0.0) # 로스트 시 탐색 회전(rad/s). 0이면 정지.

        # 공 색상(HVS) 기본 범위(형광 테니스볼 대략치) — 필요 시 튜닝
        self.declare_parameter('hsv_lower', [20, 80, 80])   # H,S,V
        self.declare_parameter('hsv_upper', [45, 255, 255])

        # 토픽 이름
        color_topic = self.get_parameter('color_topic').get_parameter_value().string_value
        depth_topic = self.get_parameter('depth_topic').get_parameter_value().string_value

        # ---------------- QoS ----------------
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # ---------------- Pub ----------------
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_servo', 10)
        self.status_pub = self.create_publisher(String, '/servo/status', 10)

        # ---------------- Sub (sync) ----------------
        self.bridge = CvBridge()
        self.color_sub = MFSubscriber(self, Image, color_topic, qos_profile=sensor_qos)
        self.depth_sub = MFSubscriber(self, Image, depth_topic, qos_profile=sensor_qos)
        self.sync = ApproximateTimeSynchronizer([self.color_sub, self.depth_sub], queue_size=10, slop=0.05)
        self.sync.registerCallback(self.cb_sync)

        # 상태 관리
        self.last_seen_ts = time.time()
        self.last_status = 'IDLE'

        self.get_logger().info(f'VisualServoingNode started. color="{color_topic}", depth="{depth_topic}"')

    # -------------- Core callback --------------
    def cb_sync(self, color_msg: Image, depth_msg: Image):
        v_max = float(self.get_parameter('v_max').value)
        w_max = float(self.get_parameter('w_max').value)
        k_v   = float(self.get_parameter('k_v').value)
        k_w   = float(self.get_parameter('k_w').value)
        stop_r= float(self.get_parameter('stop_radius_px').value)
        target= float(self.get_parameter('target_dist_m').value)
        dscale= float(self.get_parameter('depth_scale').value)
        dmin  = float(self.get_parameter('depth_valid_min').value)
        dmax  = float(self.get_parameter('depth_valid_max').value)
        lost_t= float(self.get_parameter('lost_timeout').value)
        search_rate = float(self.get_parameter('search_yaw_rate').value)

        hsv_lower = np.array(self.get_parameter('hsv_lower').value, dtype=np.uint8)
        hsv_upper = np.array(self.get_parameter('hsv_upper').value, dtype=np.uint8)

        # Convert
        try:
            color = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding='bgr8')
            depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().warn(f'CV bridge conversion failed: {e}')
            return

        H, W = color.shape[:2]

        # --- Detect tennis ball by HSV (placeholder; 실제론 YOLO 결과를 받아도 됨) ---
        hsv = cv2.cvtColor(color, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, hsv_lower, hsv_upper)
        mask = cv2.medianBlur(mask, 5)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cx = cy = None
        if contours:
            c = max(contours, key=cv2.contourArea)
            (x, y), radius = cv2.minEnclosingCircle(c)
            if radius > 3:  # 너무 작은 노이즈 제외
                cx, cy = int(x), int(y)

        now = time.time()
        twist = Twist()

        if cx is None or cy is None:
            # 로스트 처리
            if (now - self.last_seen_ts) > lost_t:
                self.last_status = 'LOST'
                self.status_pub.publish(String(data=self.last_status))
                # 탐색 회전 옵션
                twist.angular.z = saturate(search_rate, -w_max, w_max)
                self.cmd_pub.publish(twist)
            else:
                # 최근에 봤던 상태면 ACTIVE 유지(혹은 0 유지)
                self.last_status = 'ACTIVE'
                self.status_pub.publish(String(data=self.last_status))
            return

        # 공을 봤다!
        self.last_seen_ts = now

        # --- Depth 읽기 (center 픽셀 근처 median으로 노이즈 억제) ---
        win = 3
        x0 = max(0, cx - win); x1 = min(W, cx + win + 1)
        y0 = max(0, cy - win); y1 = min(H, cy + win + 1)
        patch = depth[y0:y1, x0:x1].astype(np.float32)

        if patch.ndim == 2:
            vals = patch.flatten()
        else:
            vals = patch[..., 0].flatten()

        vals = vals[np.isfinite(vals)]
        vals = vals[vals > 0]
        if vals.size == 0:
            dist_m = float('nan')
        else:
            dist_m = np.median(vals) * dscale

        # --- 픽셀 오프셋, 제어 ---
        ex_px = cx - (W / 2.0)  # +면 우측 → 좌회전(양의 ω)로 맞출지, 반대일지 선택
        # 각속도(오른손좌표 기준): 오른쪽이면 +z 회전이 카메라 기준 왼쪽보기 → 보통 ex_px에 +k_w
        w_cmd = k_w * (-ex_px)  # 화면 우측(+)이면 -회전으로 중심에 맞춤
        w_cmd = saturate(w_cmd, -w_max, w_max)

        v_cmd = 0.0
        depth_ok = (not math.isnan(dist_m)) and (dmin <= dist_m <= dmax)
        if depth_ok:
            v_cmd = k_v * (dist_m - target)  # 멀면 +, 가까우면 -
            v_cmd = saturate(v_cmd, -v_max, v_max)

        # --- 완료 조건 ---
        if abs(ex_px) <= stop_r and depth_ok and abs(dist_m - target) < 0.05:
            # 정렬/거리 OK → 정지 & DONE
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_pub.publish(twist)
            if self.last_status != 'DONE':
                self.last_status = 'DONE'
                self.status_pub.publish(String(data=self.last_status))
            return

        # --- 활성 상태 ---
        twist.linear.x = v_cmd
        twist.angular.z = w_cmd
        self.cmd_pub.publish(twist)

        if self.last_status != 'ACTIVE':
            self.last_status = 'ACTIVE'
            self.status_pub.publish(String(data=self.last_status))

    # 안전을 위해 종료 시 정지 명령 1회 전송
    def destroy_node(self):
        try:
            self.cmd_pub.publish(Twist())
        except Exception:
            pass
        super().destroy_node()

def main():
    rclpy.init()
    node = VisualServoingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
