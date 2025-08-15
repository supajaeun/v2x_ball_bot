# 정밀접근(모션) + 픽업만 구동
# 정밀 접근(모션) 노드의 /cmd_vel을 런치에서 /cmd_vel_servo로 리맵해 twist_mux 입력으로 보냅니다.

#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    motion_controller = Node(
        package='v2x_ball_bot_control',
        executable='motion_controller_node',   # console_scripts 등록명
        name='motion_controller',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[
            ('/cmd_vel', '/cmd_vel_servo'),    # twist_mux 입력으로 보냄
        ]
    )

    ball_pickup = Node(
        package='v2x_ball_bot_control',
        executable='ball_pickup_node',         # console_scripts 등록명
        name='ball_pickup',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        motion_controller,
        ball_pickup
    ])
