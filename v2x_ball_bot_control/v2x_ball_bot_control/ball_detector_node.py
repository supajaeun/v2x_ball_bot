#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from v2x_ball_bot_msgs.msg import BallPosition
from cv_bridge import CvBridge
import numpy as np
import cv2
from ultralytics import YOLO
import os

class BallDetectorNode(Node):
    def __init__(self):
        super().__init__('ball_detector_node')

        # YOLO 모델 로드
        self.model = YOLO(os.path.join(
            os.path.dirname(__file__),
            'ball_detector_v2.pt'
        ))
        self.bridge = CvBridge()

        self.create_subscription(Image, '/color/image_raw', self.rgb_callback, 10)
        self.create_subscription(Image, '/depth/image_raw', self.depth_callback, 10)
        self.create_subscription(CameraInfo, '/depth/camera_info', self.camera_info_callback, 10)

        self.publisher = self.create_publisher(BallPosition, '/ball/position', 10)

        self.latest_depth = None
        self.fx = self.fy = self.cx = self.cy = None

    def camera_info_callback(self, msg):
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]

    def depth_callback(self, msg):
        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().warn(f'Depth image 변환 실패: {e}')

    def rgb_callback(self, msg):
        if self.latest_depth is None or self.fx is None:
            return

        try:
            color_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            return

        results = self.model(color_img)
        boxes = results[0].boxes.xyxy.cpu().numpy()
        if len(boxes) == 0:
            return

        best_idx = np.argmax(results[0].boxes.conf.cpu().numpy())
        x1, y1, x2, y2 = boxes[best_idx]
        u = int((x1 + x2) / 2)
        v = int((y1 + y2) / 2)

        scale_x = self.latest_depth.shape[1] / color_img.shape[1]
        scale_y = self.latest_depth.shape[0] / color_img.shape[0]
        du = int(u * scale_x)
        dv = int(v * scale_y)

        if dv >= self.latest_depth.shape[0] or du >= self.latest_depth.shape[1]:
            return

        depth = float(self.latest_depth[dv, du])
        if depth == 0 or np.isnan(depth) or depth < 400:  # 0.4m 이하는 무시
            return

        z = depth / 1000.0 if self.latest_depth.dtype != np.float32 else depth
        x = (u - self.cx) * z / self.fx
        y = (v - self.cy) * z / self.fy

        msg = BallPosition()
        msg.x = float(x)
        msg.y = float(y)
        msg.z = float(z)

        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = BallDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
