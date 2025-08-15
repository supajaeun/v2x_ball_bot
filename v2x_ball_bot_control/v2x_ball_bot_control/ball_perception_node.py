#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from v2x_ball_bot_msgs.msg import Ball, BallArray
from visualization_msgs.msg import MarkerArray, Marker
from cv_bridge import CvBridge
from ultralytics import YOLO
import numpy as np
import cv2
import os
import math
from collections import deque
import tf2_ros
import tf2_geometry_msgs
import message_filters
from rclpy.duration import Duration


class BallPerceptionNode(Node):
    def __init__(self):
        super().__init__('ball_perception_node')

        # === YOLO 모델 로드 ===
        package_dir = os.path.dirname(__file__)
        model_path = os.path.join(package_dir, 'ball_detector_v3.pt')
        self.model = YOLO(model_path)
        self.get_logger().info(f"YOLO model loaded: {self.model.names}")

        self.bridge = CvBridge()

        # === 파라미터 ===
        self.rgb_topic = self.declare_parameter('rgb_topic', '/color/image_raw').value
        self.depth_topic = self.declare_parameter('depth_topic', '/depth/image_raw').value
        self.camera_info_topic = self.declare_parameter('camera_info_topic', '/color/camera_info').value
        self.publish_frame = self.declare_parameter('publish_frame', 'map').value

        # === 카메라 파라미터 ===
        self.fx = self.fy = self.cx = self.cy = None
        self.create_subscription(CameraInfo, self.camera_info_topic, self.camera_info_callback, 10)

        # === TF 버퍼/리스너 ===
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # === RGB & Depth 동기화 ===
        self.rgb_sub = message_filters.Subscriber(self, Image, self.rgb_topic)
        self.depth_sub = message_filters.Subscriber(self, Image, self.depth_topic)
        self.sync = message_filters.ApproximateTimeSynchronizer([self.rgb_sub, self.depth_sub], 10, 0.05)
        self.sync.registerCallback(self.synced_callback)

        # === 퍼블리셔 ===
        self.balls_pub = self.create_publisher(BallArray, '/balls', 10)
        self.markers_pub = self.create_publisher(MarkerArray, '/ball_markers', 10)

        # === 추적 데이터 ===
        self.track_history = {}
        self.next_id = 0

    def camera_info_callback(self, msg):
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]
        self.get_logger().info(f"[DEBUG] Camera Info loaded: fx={self.fx}, fy={self.fy}, cx={self.cx}, cy={self.cy}")

    def synced_callback(self, rgb_msg, depth_msg):
        if self.fx is None:
            self.get_logger().warn("[DEBUG] Camera intrinsics not received yet")
            return

        try:
            color_img = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
            depth_img = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f"이미지 변환 실패: {e}")
            return

        # === YOLO 추론 ===
        results = self.model(color_img)
        boxes = results[0].boxes.xyxy.cpu().numpy()
        scores = results[0].boxes.conf.cpu().numpy()

        # YOLO 시각화
        vis_img = results[0].plot()
        cv2.imshow("YOLO Detections", vis_img)
        cv2.waitKey(1)

        if len(boxes) == 0:
            self.get_logger().info("[DEBUG] No detections from YOLO")
            return

        # === 메시지 준비 ===
        ball_array = BallArray()
        ball_array.stamp = rgb_msg.header.stamp
        markers = MarkerArray()

        # ✅ 기존 마커 삭제
        delete_all = Marker()
        delete_all.header.frame_id = self.publish_frame
        delete_all.header.stamp = self.get_clock().now().to_msg()
        delete_all.action = Marker.DELETEALL
        markers.markers.append(delete_all)

        # === 첫 번째 공만 처리 ===
        box, score = boxes[0], scores[0]
        if score >= 0.3:
            p_map = self.pixel_to_map(box, depth_img, rgb_msg)
            if p_map:
                ball_msg = Ball()
                ball_msg.stamp = rgb_msg.header.stamp
                ball_msg.id = "0"  # 항상 같은 ID
                ball_msg.x = p_map.point.x
                ball_msg.y = p_map.point.y
                ball_msg.z = p_map.point.z
                ball_msg.score = float(score)

                _, ball_msg.is_static = self.update_tracking(
                    0, p_map.point.x, p_map.point.y, rgb_msg.header.stamp.sec
                )

                ball_array.balls.append(ball_msg)
                markers.markers.append(self.make_marker(ball_msg))

        self.get_logger().info(f"[DEBUG] Publishing {len(markers.markers)-1} ADD markers")
        self.balls_pub.publish(ball_array)
        self.markers_pub.publish(markers)

    def pixel_to_map(self, box, depth_img, rgb_msg):
        x1, y1, x2, y2 = box
        u = int((x1 + x2) / 2)
        v = int((y1 + y2) / 2)

        # 중앙 5x5 median depth
        h, w = depth_img.shape[:2]
        if not (0 <= u < w and 0 <= v < h):
            self.get_logger().warn(f"[DEBUG] Pixel out of bounds: ({u}, {v}) in {w}x{h}")
            return None

        u0, v0 = max(2, u), max(2, v)
        u1, v1 = min(w - 3, u), min(h - 3, v)
        roi = depth_img[v0 - 2:v1 + 3, u0 - 2:u1 + 3]
        depth_vals = roi[np.isfinite(roi)]

        if depth_vals.size == 0:
            self.get_logger().warn(f"[DEBUG] No valid depth at ({u},{v})")
            return None

        depth = float(np.median(depth_vals))
        z = depth / 1000.0 if depth_img.dtype != np.float32 else depth

        self.get_logger().info(f"[DEBUG] Box: {box}, Center: ({u}, {v}), Depth median: {z:.3f} m")

        if z <= 0.0 or z < 0.4:
            self.get_logger().warn(f"[DEBUG] Invalid depth z={z:.3f} m at ({u},{v})")
            return None

        # 카메라 좌표계
        x_cam = (u - self.cx) * z / self.fx
        y_cam = (v - self.cy) * z / self.fy
        p_cam = PointStamped()
        p_cam.header = rgb_msg.header
        p_cam.header.frame_id = rgb_msg.header.frame_id
        p_cam.point.x, p_cam.point.y, p_cam.point.z = x_cam, y_cam, z

        # map 프레임으로 변환
        try:
            tf = self.tf_buffer.lookup_transform(
                self.publish_frame,
                p_cam.header.frame_id,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.05)
            )
            return tf2_geometry_msgs.do_transform_point(p_cam, tf)
        except Exception as e:
            self.get_logger().warn(f"[DEBUG] TF 변환 실패: {e}")
            return None

    def update_tracking(self, obj_id, x, y, t):
        if obj_id not in self.track_history:
            self.track_history[obj_id] = deque(maxlen=5)
        self.track_history[obj_id].append((x, y, t))

        if len(self.track_history[obj_id]) >= 2:
            (x1, y1, t1), (x2, y2, t2) = self.track_history[obj_id][0], self.track_history[obj_id][-1]
            dt = t2 - t1
            dist = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
            speed = dist / dt if dt > 0 else 0.0
            is_static = speed < 0.05
            return speed, is_static
        return 0.0, False

    def make_marker(self, ball_msg):
        marker = Marker()
        marker.header.frame_id = self.publish_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'balls'
        marker.id = int(ball_msg.id)
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = ball_msg.x
        marker.pose.position.y = ball_msg.y
        marker.pose.position.z = ball_msg.z
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        return marker


def main(args=None):
    rclpy.init(args=args)
    node = BallPerceptionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
