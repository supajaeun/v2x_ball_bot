import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command

def generate_launch_description():
    bringup_dir = get_package_share_directory('v2x_ball_bot_bringup')
    description_dir = get_package_share_directory('v2x_ball_bot_description')

    ball_detector_params = os.path.join(bringup_dir, 'config', 'ball_detector.yaml')
    motion_controller_params = os.path.join(bringup_dir, 'config', 'motion_controller.yaml')
    xacro_file = os.path.join(description_dir, 'urdf', 'v2x_ball_bot.xacro')
    rviz_config_file = os.path.join(description_dir, 'rviz', 'v2x_ball_bot.rviz')

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['xacro ', xacro_file])
        }]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file] if os.path.exists(rviz_config_file) else []
    )

    # ✅ 제어 노드: 모두 'v2x_ball_bot_control' 패키지로 수정
    ball_detector_node = Node(
        package='v2x_ball_bot_control',
        executable='ball_detector_node',
        name='ball_detector_node',
        parameters=[ball_detector_params],
        output='screen'
    )
    trajectory_predictor_node = Node(
        package='v2x_ball_bot_control',
        executable='trajectory_predictor_node',
        name='trajectory_predictor_node',
        output='screen'
    )
    path_planner_node = Node(
        package='v2x_ball_bot_control',
        executable='path_planner_node',
        name='path_planner_node',
        output='screen'
    )
    motion_controller_node = Node(
        package='v2x_ball_bot_control',
        executable='motion_controller_node',
        name='motion_controller_node',
        parameters=[motion_controller_params],
        output='screen'
    )
    obstacle_avoidance_node = Node(
        package='v2x_ball_bot_control',
        executable='obstacle_avoidance_node',
        name='obstacle_avoidance_node',
        output='screen'
    )
    ball_pickup_node = Node(
        package='v2x_ball_bot_control',
        executable='ball_pickup_node',
        name='ball_pickup_node',
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher_node,
        rviz_node,
        ball_detector_node,
        trajectory_predictor_node,
        path_planner_node,
        motion_controller_node,
        obstacle_avoidance_node,
        ball_pickup_node
    ])
