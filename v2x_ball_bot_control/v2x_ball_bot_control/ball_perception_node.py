#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from v2x_ball_bot_msgs.msg import Ball, BallArray
from visualization_msgs.msg import MarkerArray
from cv_bridge import CvBridge
import message_filters
import os

from v2x_ball_bot_control.ball_perception import (
    BallDetector, DepthMapper, SimpleTracker,
    make_marker, make_delete_all_marker
)

BALL_RADIUS = 0.065  # 공 반지름 (약 6.5cm → z 고정용)

class BallPerceptionNode(Node):
    def __init__(self):
        super().__init__('ball_perception_node')

        package_dir = os.path.dirname(__file__)
        self.detector = BallDetector(package_dir)
        self.bridge = CvBridge()

        self.rgb_topic = self.declare_parameter('rgb_topic', '/color/image_raw').value
        self.depth_topic = self.declare_parameter('depth_topic', '/depth/image_raw').value
        self.camera_info_topic = self.declare_parameter('camera_info_topic', '/color/camera_info').value
        self.publish_frame = self.declare_parameter('publish_frame', 'map').value

        self.fx = self.fy = self.cx = self.cy = None
        self.create_subscription(CameraInfo, self.camera_info_topic, self.camera_info_callback, 10)

        import tf2_ros
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.rgb_sub = message_filters.Subscriber(self, Image, self.rgb_topic)
        self.depth_sub = message_filters.Subscriber(self, Image, self.depth_topic)
        self.sync = message_filters.ApproximateTimeSynchronizer([self.rgb_sub, self.depth_sub], 10, 0.05)
        self.sync.registerCallback(self.synced_callback)

        self.balls_pub = self.create_publisher(BallArray, '/balls', 10)
        self.markers_pub = self.create_publisher(MarkerArray, '/ball_markers', 10)

        self.tracker = SimpleTracker()

        # 이전 프레임 좌표 저장
        self.last_x = None
        self.last_y = None

    def camera_info_callback(self, msg):
        self.fx, self.fy, self.cx, self.cy = msg.k[0], msg.k[4], msg.k[2], msg.k[5]
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

        boxes, scores, vis_img = self.detector.detect(color_img)
        if len(boxes) == 0:
            self.get_logger().info("[DEBUG] No detections from YOLO")
            return

        mapper = DepthMapper(self.fx, self.fy, self.cx, self.cy, self.tf_buffer, self.publish_frame)
        ball_array = BallArray()
        ball_array.stamp = rgb_msg.header.stamp
        markers = MarkerArray()
        markers.markers.append(make_delete_all_marker(self.publish_frame, self.get_clock().now().to_msg()))

        box, score = boxes[0], scores[0]
        if score >= 0.3:
            p_map = mapper.pixel_to_map(box, depth_img, rgb_msg, self.get_logger())
            if p_map:
                ball_msg = Ball()
                ball_msg.stamp = rgb_msg.header.stamp
                ball_msg.id = "0"

                # 새 좌표
                new_x, new_y = p_map.point.x, p_map.point.y

                # 튀는 값 필터링 (20cm 이상 튀면 무시)
                if self.last_x is not None and self.last_y is not None:
                    dx = abs(new_x - self.last_x)
                    dy = abs(new_y - self.last_y)
                    if dx < 0.2 and dy < 0.2:
                        ball_msg.x, ball_msg.y = new_x, new_y
                    else:
                        self.get_logger().warn("[DEBUG] 튀는 값 무시됨")
                        ball_msg.x, ball_msg.y = self.last_x, self.last_y
                else:
                    ball_msg.x, ball_msg.y = new_x, new_y

                # 항상 z는 공 반지름으로 고정 (바닥 위에 올려둔다)
                ball_msg.z = BALL_RADIUS

                self.last_x, self.last_y = ball_msg.x, ball_msg.y

                ball_msg.score = float(score)
                _, ball_msg.is_static = self.tracker.update(
                    0, ball_msg.x, ball_msg.y, rgb_msg.header.stamp.sec
                )
                ball_array.balls.append(ball_msg)
                markers.markers.append(
                    make_marker(ball_msg, self.publish_frame, self.get_clock().now().to_msg())
                )

        self.balls_pub.publish(ball_array)
        self.markers_pub.publish(markers)

def main(args=None):
    rclpy.init(args=args)
    node = BallPerceptionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
