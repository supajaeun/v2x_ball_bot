# 먹스, 코어 런치 둘을 포함해서 한 방에 구동(편의용)

#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    twist_mux_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('v2x_ball_bot_control'), 'launch', 'twist_mux.launch.py'
        ]))
    )

    approach_core_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('v2x_ball_bot_control'), 'launch', 'approach_core.launch.py'
        ])),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        twist_mux_launch,
        approach_core_launch
    ])
