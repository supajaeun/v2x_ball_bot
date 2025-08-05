from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),

        # SLAM Toolbox 실행
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                os.path.join(
                    get_package_share_directory('v2x_ball_bot_bringup'),
                    'config', 'mapper_params_online_async.yaml'
                )
            ]
        ),

        # odom → base_link TF 퍼블리셔 노드 실행
        Node(
            package='v2x_ball_bot_control',
            executable='odom_tf_pub.py',
            name='odom_tf_pub',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        )
    ])
