# v2x_ball_bot_control/launch/twist_mux.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('v2x_ball_bot_control')
    params_file = os.path.join(pkg_share, 'config', 'twist_mux.yaml')

    return LaunchDescription([
        Node(
            package='twist_mux',
            executable='twist_mux',
            name='twist_mux',
            output='screen',
            parameters=[params_file],   # ← dict 대신 파일 경로
        )
    ])
