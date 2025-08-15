from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='orbbec_camera',
            executable='orbbec_camera_node',
            name='orbbec_camera_node',
            output='screen',
            parameters=[
                {'enable_color': True},
                {'enable_depth': True},
                {'enable_align_depth_to_color': True},
                {'color_width': 640},
                {'color_height': 480},
                {'color_fps': 30},
                {'depth_width': 640},
                {'depth_height': 480},
                {'depth_fps': 30},
                {'depth_format': 'Y12'}
            ]
        )
    ])
