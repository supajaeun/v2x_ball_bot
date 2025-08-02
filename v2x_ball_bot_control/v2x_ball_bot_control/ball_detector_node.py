#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class BallDetectorNode(Node):
    def __init__(self):
        super().__init__('ball_detector_node')

        # 이미지 구독자
        self.subscription = self.create_subscription(
            Image,
            '/color/image_raw',
            self.listener_callback,
            10)

        # 디버깅용 이미지 퍼블리셔
        self.debug_image_pub = self.create_publisher(Image, '/debug/ball_detection_image', 10)

        self.bridge = CvBridge()
        self.get_logger().info("🎯 Ball Detector Node Started!")

    def listener_callback(self, msg):
        # ROS 이미지 → OpenCV 이미지로 변환
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # BGR → HSV 변환
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # 노란색 범위 지정 (HSV 기준)
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([40, 255, 255])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # 마스크 → 윤곽선 추출
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # 가장 큰 윤곽선 탐지
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            if cv2.contourArea(largest_contour) > 300:
                x, y, w, h = cv2.boundingRect(largest_contour)
                cx = x + w // 2
                cy = y + h // 2

                # 디버깅 메시지
                self.get_logger().info(f"🎾 Ball detected at (u,v): ({cx}, {cy})")

                # 시각화용 그리기
                cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
                cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
            else:
                self.get_logger().info("❗공이 너무 작아서 무시됨")
        else:
            self.get_logger().info("😶 공 없음")

        # 결과 이미지를 퍼블리시
        debug_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.debug_image_pub.publish(debug_msg)

def main(args=None):
    rclpy.init(args=args)
    node = BallDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
