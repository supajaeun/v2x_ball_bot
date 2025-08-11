#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from v2x_ball_bot_msgs.msg import BallPosition
from visualization_msgs.msg import Marker
from cv_bridge import CvBridge
import numpy as np
import cv2
from ultralytics import YOLO
import os

class BallDetectorNode(Node):
    def __init__(self):
        super().__init__('ball_detector_node')

        # YOLO 모델 상대 경로로 로드
        package_dir = os.path.dirname(__file__)
        model_path = os.path.join(package_dir, 'ball_detector_v3.pt')
        self.model = YOLO(model_path)
        print(self.model.names)

        self.bridge = CvBridge()

        self.create_subscription(Image, '/color/image_raw', self.rgb_callback, 10)
        self.create_subscription(Image, '/depth/image_raw', self.depth_callback, 10)
        self.create_subscription(CameraInfo, '/depth/camera_info', self.camera_info_callback, 10)

        self.publisher = self.create_publisher(BallPosition, '/ball/position', 10)
        self.marker_pub = self.create_publisher(Marker, '/ball_marker', 10)

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

        # YOLO 추론
        results = self.model(color_img)

        # YOLO가 인식한 결과 시각화
        vis_img = results[0].plot()
        cv2.imshow("YOLO Detections", vis_img)
        cv2.waitKey(1)

        # 공 탐지 및 위치 계산
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
        if depth == 0 or np.isnan(depth) or depth < 400:
            return

        z = depth / 1000.0 if self.latest_depth.dtype != np.float32 else depth
        x = (u - self.cx) * z / self.fx
        y = (v - self.cy) * z / self.fy

        msg = BallPosition()
        msg.x = float(x)
        msg.y = float(y)
        msg.z = float(z)
        self.publisher.publish(msg)

        self.publish_marker(x, y, z)

    def publish_marker(self, x, y, z):
        marker = Marker()
        marker.header.frame_id = 'orbbec_camera_color_optical_frame'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'ball'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        self.marker_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = BallDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()  

if __name__ == '__main__':
    main()
