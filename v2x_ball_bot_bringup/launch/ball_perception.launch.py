from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    bringup_dir = os.path.join(
        get_package_share_directory('v2x_ball_bot_bringup'), 'config'
    )

    rviz_config_path = os.path.join(bringup_dir, 'ball_perception.rviz')

    return LaunchDescription([
        Node(
            package='v2x_ball_bot_control',
            executable='ball_perception_node',
            name='ball_perception',
            output='screen',
            parameters=[{
                'rgb_topic': '/color/image_raw',
                'depth_topic': '/aligned_depth_to_color/image_raw',
                'camera_info_topic': '/color/camera_info',
                'publish_frame': 'map'
            }]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path],
            output='screen'
        )
    ])
