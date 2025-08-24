# marker_utils.py
from visualization_msgs.msg import Marker

def make_marker(ball_msg, frame_id, now):
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = now
    marker.ns = 'balls'
    marker.id = 0  # 공 하나만 표시 → 항상 같은 ID 사용
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD

    marker.pose.position.x = ball_msg.x
    marker.pose.position.y = ball_msg.y
    marker.pose.position.z = ball_msg.z

    marker.scale.x = marker.scale.y = marker.scale.z = 0.1
    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    return marker


def make_delete_all_marker(frame_id, now):
    """
    RViz 마커 초기화용.
    노드 시작 직후에만 한 번 사용하고,
    이후에는 매 프레임마다 호출하지 말 것!
    """
    delete_all = Marker()
    delete_all.header.frame_id = frame_id
    delete_all.header.stamp = now
    delete_all.action = Marker.DELETEALL
    return delete_all
