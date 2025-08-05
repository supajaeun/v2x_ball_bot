#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math

class OdomTfPublisher(Node):
    def __init__(self):
        super().__init__('odom_tf_pub')
        self.broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.05, self.timer_callback)  # 20Hz

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

    def timer_callback(self):
        t = TransformStamped()
        now = self.get_clock().now().to_msg()
        t.header.stamp = now
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.z = math.sin(self.theta / 2.0)
        t.transform.rotation.w = math.cos(self.theta / 2.0)
        self.broadcaster.sendTransform(t)

def main():
    rclpy.init()
    node = OdomTfPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
