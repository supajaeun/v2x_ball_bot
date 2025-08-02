from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='orbbec_camera',
            executable='orbbec_camera_node',
            name='orbbec_camera_node',
            parameters=[{
                'enable_color': True,
                'color_width': 640,
                'color_height': 480,
                'color_fps': 30,
                'depth_width': 320,
                'depth_height': 240,
                'depth_fps': 30,
                'depth_format': 'Y11'
            }],
            output='screen'
        )
    ])
