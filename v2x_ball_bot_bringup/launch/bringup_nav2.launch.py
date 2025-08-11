from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    lidar_port = LaunchConfiguration("lidar_port")
    lidar_baud = LaunchConfiguration("lidar_baud")
    lidar_mode = LaunchConfiguration("lidar_scan_mode")
    laser_z    = LaunchConfiguration("laser_z")

    bringup_share = get_package_share_directory("v2x_ball_bot_bringup")
    nav2_share    = get_package_share_directory("nav2_bringup")

    base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_share, "launch", "bringup_nav.launch.py")),
        launch_arguments={
            "lidar_port": lidar_port,
            "lidar_baud": lidar_baud,
            "lidar_scan_mode": lidar_mode,
            "laser_z": laser_z
        }.items()
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_share, "launch", "bringup_launch.py")),
        launch_arguments={
            "params_file": os.path.join(bringup_share, "config", "nav2_params.yaml")
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument("lidar_port", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("lidar_baud", default_value="115200"),
        DeclareLaunchArgument("lidar_scan_mode", default_value="Standard"),
        DeclareLaunchArgument("laser_z", default_value="0.12"),
        base, nav2
    ])
