from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    rplidar = Node(
        package='rplidar_ros', executable='rplidar_composition', name='rplidar',
        parameters=[{
            'channel_type': 'serial',
            'serial_port': '/dev/ttyUSB0',      # 필요 시 /dev/ttyUSB1
            'serial_baudrate': 115200,          # Express 모델이면 256000
            'frame_id': 'laser',
            'angle_compensate': True,
            'scan_mode': 'Standard',            # Express 모델이면 'Express'
        }],
    )
    tf_laser = Node(
        package='tf2_ros', executable='static_transform_publisher',
        arguments=['0','0','0.12','0','0','0','base_link','laser'] # 라이다 실제 높이로 조정
    )
    # 임시(테스트용) odom->base_link 항등 TF — 나중에 실제 오도메로 교체
    tf_odom = Node(
        package='tf2_ros', executable='static_transform_publisher',
        arguments=['0','0','0','0','0','0','odom','base_link']
    )
    slam = Node(
        package='slam_toolbox', executable='sync_slam_toolbox_node', name='slam_toolbox',
        parameters=[{
            'odom_frame': 'odom',
            'map_frame': 'map',
            'base_frame': 'base_link',
            'scan_topic': '/scan',
            'publish_tf': True,
            'resolution': 0.05,
        }],
        output='screen'
    )
    return LaunchDescription([rplidar, tf_laser, tf_odom, slam])
