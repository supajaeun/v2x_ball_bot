from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    bringup_dir = get_package_share_directory('v2x_ball_bot_bringup')
    params_file = os.path.join(bringup_dir, 'config', 'nav2_params.yaml')
    map_file = os.path.join(bringup_dir, 'maps', 'map.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        Node(
            package='nav2_bringup',
            executable='navigation_launch.py',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
            arguments=['--map', map_file, '--params-file', params_file]
        )
    ])
