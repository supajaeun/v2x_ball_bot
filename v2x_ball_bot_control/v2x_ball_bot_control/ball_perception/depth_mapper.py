# depth_mapper.py
import numpy as np
from geometry_msgs.msg import PointStamped
import tf2_geometry_msgs
import rclpy
from rclpy.duration import Duration

class DepthMapper:
    def __init__(self, fx, fy, cx, cy, tf_buffer, publish_frame):
        self.fx, self.fy, self.cx, self.cy = fx, fy, cx, cy
        self.tf_buffer = tf_buffer
        self.publish_frame = publish_frame

    def pixel_to_map(self, box, depth_img, rgb_msg, logger):
        x1, y1, x2, y2 = box
        u = int((x1 + x2) / 2)
        v = int((y1 + y2) / 2)

        h, w = depth_img.shape[:2]
        if not (0 <= u < w and 0 <= v < h):
            logger.warn(f"[DEBUG] Pixel out of bounds: ({u},{v})")
            return None

        # 중앙 5×5 영역 median depth
        roi = depth_img[max(0, v - 2):v + 3, max(0, u - 2):u + 3]
        depth_vals = roi[np.isfinite(roi)]
        if depth_vals.size == 0:
            return None

        depth = float(np.median(depth_vals))
        z = depth / 1000.0 if depth_img.dtype != np.float32 else depth
        if z <= 0.0 or z < 0.4:
            return None

        # 카메라 좌표계
        x_cam = (u - self.cx) * z / self.fx
        y_cam = (v - self.cy) * z / self.fy
        p_cam = PointStamped()
        p_cam.header = rgb_msg.header
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
            logger.warn(f"[DEBUG] TF 변환 실패: {e}")
            return None
