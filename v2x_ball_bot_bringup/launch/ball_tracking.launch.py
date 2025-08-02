from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='v2x_ball_bot_control',
            executable='ball_detector_node',
            name='ball_detector_node',
            output='screen'
        ),
        Node(
            package='v2x_ball_bot_control',
            executable='trajectory_predictor_node',
            name='trajectory_predictor_node',
            output='screen'
        )
    ])
