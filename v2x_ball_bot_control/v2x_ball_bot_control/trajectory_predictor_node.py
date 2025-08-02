#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from v2x_ball_bot_msgs.msg import BallPixel, BallPosition
from cv_bridge import CvBridge
import numpy as np

class TrajectoryPredictorNode(Node):
    def __init__(self):
        super().__init__('trajectory_predictor_node')
        self.bridge = CvBridge()

        # 카메라 내부 파라미터
        self.fx = self.fy = self.cx = self.cy = None
        self.depth_image = None

        # 구독자
        self.create_subscription(CameraInfo, '/depth/camera_info', self.camera_info_callback, 10)
        self.create_subscription(Image, '/depth/image_raw', self.depth_callback, 10)
        self.create_subscription(BallPixel, '/ball_uv', self.uv_callback, 10)

        # 퍼블리셔
        self.publisher = self.create_publisher(BallPosition, '/ball_xyz', 10)

    def camera_info_callback(self, msg):
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]
        self.get_logger().info(f"Camera intrinsics loaded: fx={self.fx}, fy={self.fy}, cx={self.cx}, cy={self.cy}")

    def depth_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def uv_callback(self, msg):
        if self.depth_image is None or self.fx is None:
            self.get_logger().warn("Depth image or camera intrinsics not ready.")
            return

        u = int(msg.u)
        v = int(msg.v)

        if v >= self.depth_image.shape[0] or u >= self.depth_image.shape[1]:
            self.get_logger().warn(f"Invalid pixel coordinates: ({u}, {v})")
            return

        z = float(self.depth_image[v, u]) / 1000.0  # mm → meters
        if z == 0.0:
            self.get_logger().warn("Depth value is 0. Skipping.")
            return

        x = (u - self.cx) * z / self.fx
        y = (v - self.cy) * z / self.fy

        pos_msg = BallPosition()
        pos_msg.x = x
        pos_msg.y = y
        pos_msg.z = z
        self.publisher.publish(pos_msg)

        self.get_logger().info(f"Published BallPosition: x={x:.2f}, y={y:.2f}, z={z:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPredictorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()