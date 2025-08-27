#!/usr/bin/env python3
# ROS 2 Humble / rclpy / Python 3
# Twist (/cmd_vel_out) -> differential drive wheel commands
# - backend: topic or serial
# - publishes wheel angular velocity [rad/s]
#   topics: /left_wheel/vel (Float32), /right_wheel/vel (Float32)

import math
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

try:
    import serial  # optional, only if using serial backend
except ImportError:
    serial = None


class MotorDriver(Node):
    def __init__(self):
        super().__init__('motor_driver')

        # ---------------- Params ----------------
        # inputs
        self.cmd_topic = self.declare_parameter('cmd_topic', '/cmd_vel_out').value
        # robot geom
        self.wheel_radius = float(self.declare_parameter('wheel_radius', 0.05).value)  # [m]
        self.wheel_base   = float(self.declare_parameter('wheel_base',   0.30).value)  # [m] distance between wheel centers
        # limits
        self.max_wheel_radps = float(self.declare_parameter('max_wheel_radps', 20.0).value)  # [rad/s]
        self.max_accel_radps2 = float(self.declare_parameter('max_accel_radps2', 50.0).value)  # accel limit
        # safety
        self.cmd_timeout_s = float(self.declare_parameter('cmd_timeout_s', 0.5).value)
        self.estop_topic = self.declare_parameter('estop_topic', '/safety/estop_cmd').value
        self.use_estop = bool(self.declare_parameter('use_estop', False).value)
        # wheel direction (set True if your wiring inverts a side)
        self.invert_left  = bool(self.declare_parameter('invert_left',  False).value)
        self.invert_right = bool(self.declare_parameter('invert_right', False).value)
        # backend: "topic" or "serial"
        self.backend = self.declare_parameter('backend', 'topic').value  # 'topic' | 'serial'
        # topic backend topic names
        self.left_topic  = self.declare_parameter('left_topic',  '/left_wheel/vel').value
        self.right_topic = self.declare_parameter('right_topic', '/right_wheel/vel').value
        # serial backend params
        self.serial_port = self.declare_parameter('serial_port', '/dev/ttyACM0').value
        self.serial_baud = int(self.declare_parameter('serial_baud', 115200).value)
        self.serial_timeout = float(self.declare_parameter('serial_timeout', 0.02).value)

        # ---------------- I/O ----------------
        self.sub_cmd = self.create_subscription(Twist, self.cmd_topic, self.on_cmd, 10)
        if self.use_estop:
            from std_msgs.msg import Bool
            self.sub_estop = self.create_subscription(Bool, self.estop_topic, self.on_estop, 10)
            self.estop_engaged = False
        else:
            self.estop_engaged = False

        if self.backend == 'topic':
            self.pub_left  = self.create_publisher(Float32, self.left_topic,  10)
            self.pub_right = self.create_publisher(Float32, self.right_topic, 10)
            self.get_logger().info(f"Publishing wheel speeds to topics: {self.left_topic}, {self.right_topic}")
            self.ser = None
        elif self.backend == 'serial':
            if serial is None:
                self.get_logger().fatal("pyserial not installed but backend=serial. Install with: pip install pyserial")
                raise RuntimeError("pyserial missing")
            try:
                self.ser = serial.Serial(self.serial_port, self.serial_baud, timeout=self.serial_timeout)
                self.get_logger().info(f"Serial opened: {self.serial_port} @ {self.serial_baud}")
            except Exception as e:
                self.get_logger().fatal(f"Failed to open serial: {e}")
                raise
        else:
            self.get_logger().fatal(f"Unknown backend: {self.backend}")
            raise RuntimeError("Unknown backend")

        # state
        self.last_cmd_time = 0.0
        self.target_left  = 0.0
        self.target_right = 0.0
        self.curr_left    = 0.0
        self.curr_right   = 0.0

        # control loop timer (50 Hz)
        self.dt = 0.02
        self.timer = self.create_timer(self.dt, self.update)

        self.get_logger().info(
            f"MotorDriver ready. cmd_topic={self.cmd_topic}, wheel_r={self.wheel_radius:.3f}, "
            f"wheel_base={self.wheel_base:.3f}, backend={self.backend}"
        )

    # -------- callbacks --------
    def on_estop(self, msg):
        # std_msgs/Bool: True means stop
        self.estop_engaged = bool(msg.data)
        if self.estop_engaged:
            self.get_logger().warn("E-STOP engaged! Forcing zero command.")

    def on_cmd(self, msg: Twist):
        # Twist: linear.x [m/s], angular.z [rad/s]
        v = float(msg.linear.x)
        w = float(msg.angular.z)

        # diff-drive kinematics: wheel linear speeds
        v_l = v - (w * self.wheel_base / 2.0)
        v_r = v + (w * self.wheel_base / 2.0)

        # convert to wheel angular speeds [rad/s]
        wl = v_l / max(self.wheel_radius, 1e-6)
        wr = v_r / max(self.wheel_radius, 1e-6)

        # direction invert if needed
        if self.invert_left:  wl = -wl
        if self.invert_right: wr = -wr

        # clamp by max
        wl = max(-self.max_wheel_radps, min(self.max_wheel_radps, wl))
        wr = max(-self.max_wheel_radps, min(self.max_wheel_radps, wr))

        self.target_left  = wl
        self.target_right = wr
        self.last_cmd_time = time.time()

    # -------- control loop --------
    def update(self):
        now = time.time()

        # safety: timeout or estop => zero target
        if (now - self.last_cmd_time) > self.cmd_timeout_s or self.estop_engaged:
            self.target_left  = 0.0
            self.target_right = 0.0

        # accel limit (slew rate)
        self.curr_left  = self._slew(self.curr_left,  self.target_left,  self.max_accel_radps2 * self.dt)
        self.curr_right = self._slew(self.curr_right, self.target_right, self.max_accel_radps2 * self.dt)

        # output
        self._send(self.curr_left, self.curr_right)

    def _slew(self, curr: float, target: float, step: float) -> float:
        if target > curr:
            return min(target, curr + step)
        else:
            return max(target, curr - step)

    # -------- backends --------
    def _send(self, wl: float, wr: float):
        if self.backend == 'topic':
            self.pub_left.publish(Float32(data=float(wl)))
            self.pub_right.publish(Float32(data=float(wr)))
        else:
            # serial: send a simple CSV line "V,<wl>,<wr>\n"
            try:
                line = f"V,{wl:.3f},{wr:.3f}\n"
                self.ser.write(line.encode('utf-8'))
            except Exception as e:
                self.get_logger().error(f"Serial write failed: {e}")

def main():
    rclpy.init()
    node = MotorDriver()
    try:
        rclpy.spin(node)
    finally:
        if hasattr(node, 'ser') and node.ser:
            try:
                node.ser.close()
            except Exception:
                pass
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
