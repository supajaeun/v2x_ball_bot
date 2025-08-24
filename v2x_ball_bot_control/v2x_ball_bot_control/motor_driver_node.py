#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, String, Bool

def sat(x, lo, hi): return max(lo, min(hi, x))

class MotorDriverNode(Node):
    """
    /cmd_vel_out (v[m/s], w[rad/s]) ->
      left/right wheel target velocity [rad/s] 퍼블리시
    안전: estop, 타임아웃 시 0 출력
    """
    def __init__(self):
        super().__init__('motor_driver')

        # 파라미터
        self.declare_parameter('cmd_topic', '/cmd_vel_out')
        self.declare_parameter('wheel_radius', 0.05)   # m
        self.declare_parameter('wheel_base',   0.30)   # m (바퀴간 거리)
        self.declare_parameter('v_max',        0.6)    # m/s
        self.declare_parameter('w_max',        1.8)    # rad/s
        self.declare_parameter('timeout',      0.5)    # s (명령 미수신 시 정지)
        self.declare_parameter('estop_topic',  '/safety/estop')

        cmd_topic  = self.get_parameter('cmd_topic').get_parameter_value().string_value
        estop_t    = self.get_parameter('estop_topic').get_parameter_value().string_value

        # QoS
        qos = QoSProfile(depth=10)
        qos.reliability = QoSReliabilityPolicy.RELIABLE
        qos.history = QoSHistoryPolicy.KEEP_LAST

        # 퍼블리셔(모터 목표 각속도 rad/s)
        self.pub_l = self.create_publisher(Float32, '/wheel_left/vel_cmd', 10)
        self.pub_r = self.create_publisher(Float32, '/wheel_right/vel_cmd', 10)
        self.status_pub = self.create_publisher(String, '/driver/status', 10)

        # 구독
        self.sub_cmd  = self.create_subscription(Twist, cmd_topic, self.cb_cmd, qos)
        self.sub_estop= self.create_subscription(Bool, estop_t, self.cb_estop, 10)

        # 상태
        self._last_msg_time = self.get_clock().now()
        self._estop = False

        # 주기 타이머: 타임아웃/하트비트
        self.create_timer(0.05, self._tick)  # 20 Hz
        self.create_timer(0.5,  self._heartbeat)

        self.get_logger().info(f'MotorDriver started. cmd="{cmd_topic}"')

    def cb_estop(self, msg: Bool):
        self._estop = bool(msg.data)
        if self._estop:
            self._publish_wheels(0.0, 0.0)
            self.status_pub.publish(String(data='ESTOP'))

    def cb_cmd(self, msg: Twist):
        self._last_msg_time = self.get_clock().now()

        # 파라미터 로드
        R = float(self.get_parameter('wheel_radius').value)
        L = float(self.get_parameter('wheel_base').value)
        v_max = float(self.get_parameter('v_max').value)
        w_max = float(self.get_parameter('w_max').value)

        # 입력 제한
        v = sat(msg.linear.x, -v_max, v_max)
        w = sat(msg.angular.z, -w_max, w_max)

        # 좌/우 바퀴 선속도(m/s)
        v_l = v - w * (L / 2.0)
        v_r = v + w * (L / 2.0)

        # 각속도(rad/s) 변환
        wl = v_l / R
        wr = v_r / R

        if self._estop:
            wl = wr = 0.0

        self._publish_wheels(wl, wr)

    def _publish_wheels(self, wl: float, wr: float):
        self.pub_l.publish(Float32(data=float(wl)))
        self.pub_r.publish(Float32(data=float(wr)))

    def _tick(self):
        # 타임아웃 시 정지
        dt = (self.get_clock().now() - self._last_msg_time).nanoseconds / 1e9
        if dt > float(self.get_parameter('timeout').value):
            self._publish_wheels(0.0, 0.0)

    def _heartbeat(self):
        s = 'ESTOP' if self._estop else 'OK'
        self.status_pub.publish(String(data=s))

def main():
    rclpy.init()
    node = MotorDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
