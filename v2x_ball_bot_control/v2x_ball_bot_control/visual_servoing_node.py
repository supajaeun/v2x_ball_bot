#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import time
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import (
    QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
)

from geometry_msgs.msg import Twist, PointStamped
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import message_filters
from message_filters import Subscriber as MFSubscriber

# 퍼셉션/TF
from v2x_ball_bot_msgs.msg import BallArray
import tf2_ros
from tf2_ros import TransformException
from tf2_geometry_msgs import do_transform_point


def saturate(x, lo, hi):
    return max(lo, min(hi, x))


class VisualServoingNode(Node):
    """
    우선순위:
      1) /balls (map 좌표) 기반 → TF로 base_link에 투영해 yaw/거리 제어
      2) 백업: 컬러+Depth 동기화 HSV로 픽셀 중심 추정 → P제어
    상태 토픽: /servo/status  (TRANSIENT_LOCAL + 0.5s 하트비트)
    속도 토픽: /cmd_vel_servo (geometry_msgs/Twist)
    """

    def __init__(self):
        super().__init__('visual_servoing')

        # -------- Params --------
        # 카메라 기본값은 현재 Orbbec 토픽에 맞춤
        self.declare_parameter('color_topic', '/color/image_raw')
        self.declare_parameter('depth_topic', '/depth/image_raw')

        # 제어/임계
        self.declare_parameter('v_max', 0.15)          # m/s
        self.declare_parameter('w_max', 0.6)           # rad/s
        self.declare_parameter('k_v', 0.8)             # 선속도 P게인
        self.declare_parameter('k_w', 0.004)           # 각속도 P게인(px^-1, HSV용)
        self.declare_parameter('target_dist_m', 0.45)  # 픽업 전 정지 거리
        self.declare_parameter('stop_radius_px', 20.0) # HSV 중심 정렬 허용(px)
        self.declare_parameter('yaw_deadzone_deg', 2.0)# /balls 사용 시 각도 허용
        self.declare_parameter('k_yaw', 1.2)           # yaw 제어 게인(rad/rad)

        # 깊이/검증
        self.declare_parameter('depth_scale', 0.001)   # U16(mm)→m
        self.declare_parameter('depth_valid_min', 0.15)
        self.declare_parameter('depth_valid_max', 4.0)

        # 로스트/탐색
        self.declare_parameter('lost_timeout', 1.0)    # s
        self.declare_parameter('search_yaw_rate', 0.0) # LOST 때 탐색 회전(rad/s)

        # 퍼셉션 사용
        self.declare_parameter('use_balls', True)
        self.declare_parameter('ball_topic', '/balls')
        self.declare_parameter('ball_conf_thresh', 0.3)
        self.declare_parameter('ball_max_age', 0.5)    # s

        # HSV 백업
        self.declare_parameter('hsv_lower', [20, 80, 80])
        self.declare_parameter('hsv_upper', [45, 255, 255])
        self.declare_parameter('min_area_px', 200.0)

        color_topic = self.get_parameter('color_topic').get_parameter_value().string_value
        depth_topic = self.get_parameter('depth_topic').get_parameter_value().string_value

        # -------- QoS --------
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # -------- Publishers --------
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_servo', 10)

        # 상태 퍼블리셔: 라치 유사(새 구독자도 마지막 값 즉시 수신)
        status_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.status_pub = self.create_publisher(String, '/servo/status', status_qos)

        # -------- Subscribers --------
        self.bridge = CvBridge()
        self.color_sub = MFSubscriber(self, Image, color_topic, qos_profile=sensor_qos)
        self.depth_sub = MFSubscriber(self, Image, depth_topic, qos_profile=sensor_qos)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.color_sub, self.depth_sub], queue_size=10, slop=0.05
        )
        self.sync.registerCallback(self.cb_sync)

        # /balls (map 좌표) 구독
        self.last_ball = None  # (PointStamped(map), score, t_sec)
        self.balls_sub = self.create_subscription(
            BallArray,
            self.get_parameter('ball_topic').get_parameter_value().string_value,
            self.cb_balls,
            10
        )

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # 상태/하트비트
        self.last_seen_ts = None
        self.last_status = 'LOST'  # 시작은 당연히 못 본 상태
        self.status_pub.publish(String(data=self.last_status))
        self.create_timer(0.5, self._heartbeat)

        self.get_logger().info(f'VisualServoing started. color="{color_topic}", depth="{depth_topic}"')

    # ---- heartbeat ----
    def _heartbeat(self):
        try:
            self.status_pub.publish(String(data=self.last_status))
        except Exception:
            pass

    # ---- /balls callback ----
    def cb_balls(self, msg: BallArray):
        if not msg.balls:
            return
        best = max(msg.balls, key=lambda b: b.score)
        p = PointStamped()
        p.header.stamp = self.get_clock().now().to_msg()
        p.header.frame_id = 'map'
        p.point.x, p.point.y, p.point.z = best.x, best.y, best.z
        self.last_ball = (p, float(best.score), self.get_clock().now().nanoseconds / 1e9)

    # ---- main image callback ----
    def cb_sync(self, color_msg: Image, depth_msg: Image):
        # 파라미터 로드
        v_max = float(self.get_parameter('v_max').value)
        w_max = float(self.get_parameter('w_max').value)
        k_v   = float(self.get_parameter('k_v').value)
        k_w   = float(self.get_parameter('k_w').value)
        target= float(self.get_parameter('target_dist_m').value)
        stop_r= float(self.get_parameter('stop_radius_px').value)

        dscale= float(self.get_parameter('depth_scale').value)
        dmin  = float(self.get_parameter('depth_valid_min').value)
        dmax  = float(self.get_parameter('depth_valid_max').value)

        lost_t= float(self.get_parameter('lost_timeout').value)
        search_rate = float(self.get_parameter('search_yaw_rate').value)

        use_balls = bool(self.get_parameter('use_balls').value)
        conf_th   = float(self.get_parameter('ball_conf_thresh').value)
        max_age   = float(self.get_parameter('ball_max_age').value)
        yaw_dead  = math.radians(float(self.get_parameter('yaw_deadzone_deg').value))
        k_yaw     = float(self.get_parameter('k_yaw').value)

        hsv_lower = np.array(self.get_parameter('hsv_lower').value, dtype=np.uint8)
        hsv_upper = np.array(self.get_parameter('hsv_upper').value, dtype=np.uint8)
        min_area  = float(self.get_parameter('min_area_px').value)

        # 이미지 변환
        try:
            color = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding='bgr8')
            depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().warn(f'CV bridge conversion failed: {e}')
            return

        H, W = color.shape[:2]
        now = time.time()

        # ----------------------------
        # 1) /balls 우선 경로
        # ----------------------------
        if use_balls and (self.last_ball is not None):
            p_map, conf, t = self.last_ball
            if ((now - t) <= max_age) and (conf >= conf_th):
                try:
                    # map -> base_link 변환(최신)
                    tf = self.tf_buffer.lookup_transform('base_link', p_map.header.frame_id, Time())
                    p_base = do_transform_point(p_map, tf).point
                    dx, dy = float(p_base.x), float(p_base.y)
                    dist   = math.hypot(dx, dy)
                    yaw_err= math.atan2(dy, dx)  # +: 좌측 → +z 회전

                    # 제어량
                    w_cmd = saturate(k_yaw * yaw_err, -w_max, w_max)
                    v_cmd = 0.0
                    if dmin <= dist <= dmax:
                        v_cmd = saturate(k_v * (dist - target), -v_max, v_max)

                    # 완료 조건
                    if (abs(yaw_err) <= yaw_dead) and (abs(dist - target) < 0.05):
                        self._publish_cmd(0.0, 0.0)
                        self._set_status('DONE')
                        return

                    self._publish_cmd(v_cmd, w_cmd)
                    self._set_status('ACTIVE')
                    return

                except TransformException as e:
                    self.get_logger().warn(f'TF failed (map->base_link), fallback to HSV: {e}')
                except Exception as e:
                    self.get_logger().warn(f'Fallback to HSV due to error: {e}')

        # ----------------------------
        # 2) HSV 백업 경로 (픽셀 기반)
        # ----------------------------
        cx = cy = None
        hsv = cv2.cvtColor(color, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, hsv_lower, hsv_upper)
        mask = cv2.medianBlur(mask, 5)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            c = max(contours, key=cv2.contourArea)
            if cv2.contourArea(c) >= min_area:
                (x, y), radius = cv2.minEnclosingCircle(c)
                if radius > 3:
                    cx, cy = int(x), int(y)

        twist = Twist()

        if cx is None or cy is None:
            # 못 봄 → LOST/SEARCHING
            if (self.last_seen_ts is None) or ((now - self.last_seen_ts) > lost_t):
                self._set_status('LOST')
                if search_rate != 0.0:
                    self._publish_cmd(0.0, saturate(search_rate, -w_max, w_max))
            else:
                self._set_status('SEARCHING')
            return

        # 본 경우
        self.last_seen_ts = now

        # Depth로 거리(중앙 근방 median)
        win = 3
        x0 = max(0, cx - win); x1 = min(W, cx + win + 1)
        y0 = max(0, cy - win); y1 = min(H, cy + win + 1)
        patch = depth[y0:y1, x0:x1].astype(np.float32)
        vals = patch.flatten() if patch.ndim == 2 else patch[..., 0].flatten()
        vals = vals[np.isfinite(vals)]
        vals = vals[vals > 0]
        dist_m = (np.median(vals) * dscale) if vals.size > 0 else float('nan')

        # 픽셀 오프셋 제어
        ex_px = cx - (W / 2.0)
        w_cmd = saturate(-k_w * ex_px, -w_max, w_max)  # 우측(+)면 -회전
        v_cmd = 0.0
        depth_ok = (not math.isnan(dist_m)) and (dmin <= dist_m <= dmax)
        if depth_ok:
            v_cmd = saturate(k_v * (dist_m - target), -v_max, v_max)

        # 완료 조건
        if abs(ex_px) <= stop_r and depth_ok and abs(dist_m - target) < 0.05:
            self._publish_cmd(0.0, 0.0)
            self._set_status('DONE')
            return

        # 명령
        self._publish_cmd(v_cmd, w_cmd)
        self._set_status('ACTIVE')

    # ---- helpers ----
    def _publish_cmd(self, v_x, w_z):
        twist = Twist()
        twist.linear.x = float(v_x)
        twist.angular.z = float(w_z)
        self.cmd_pub.publish(twist)

    def _set_status(self, s: str):
        if self.last_status != s:
            self.last_status = s
            self.status_pub.publish(String(data=self.last_status))

    # 안전 종료 시 정지 1회
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
