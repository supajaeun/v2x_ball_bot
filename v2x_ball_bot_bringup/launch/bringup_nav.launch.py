from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    lidar_port = LaunchConfiguration("lidar_port")
    lidar_baud = LaunchConfiguration("lidar_baud")
    lidar_mode = LaunchConfiguration("lidar_scan_mode")
    laser_z    = LaunchConfiguration("laser_z")

    rplidar = Node(
        package="rplidar_ros", executable="rplidar_composition", name="rplidar",
        parameters=[{
            "channel_type": "serial",
            "serial_port": lidar_port,
            "serial_baudrate": lidar_baud,
            "frame_id": "laser",
            "angle_compensate": True,
            "scan_mode": lidar_mode,
        }],
        output="screen", respawn=True, respawn_delay=1.0
    )

    # laser -> base_link (라이다 실제 장착 높이/자세로 조정)
    tf_laser = Node(
        package="tf2_ros", executable="static_transform_publisher",
        arguments=["0","0", laser_z, "0","0","0","base_link","laser"]
    )

    # ⚠️ 실제 오도메 노드가 준비될 때까지 임시 항등 TF 유지
    tf_odom = Node(
        package="tf2_ros", executable="static_transform_publisher",
        arguments=["0","0","0","0","0","0","odom","base_link"]
    )

    # 라이다 먼저 → SLAM 약간 지연
    slam = TimerAction(
        period=2.0,
        actions=[Node(
            package="slam_toolbox", executable="sync_slam_toolbox_node", name="slam_toolbox",
            parameters=[{
                "odom_frame": "odom",
                "map_frame": "map",
                "base_frame": "base_link",
                "scan_topic": "/scan",
                "publish_tf": True,
                "resolution": 0.05,
            }],
            output="screen"
        )]
    )

    return LaunchDescription([
        DeclareLaunchArgument("lidar_port", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("lidar_baud", default_value="115200"),
        DeclareLaunchArgument("lidar_scan_mode", default_value="Standard"),
        DeclareLaunchArgument("laser_z", default_value="0.12"),
        rplidar, tf_laser, tf_odom, slam
    ])
