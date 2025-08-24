#!/usr/bin/env python3
# servo + mux + driver 런치 파일

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    output_cmd = LaunchConfiguration('output_cmd', default='/cmd_vel_out')
    color_topic = LaunchConfiguration('color_topic', default='/color/image_raw')
    depth_topic = LaunchConfiguration('depth_topic', default='/depth/image_raw')
    ball_topic  = LaunchConfiguration('ball_topic',  default='/balls')

    twist_mux_yaml = PathJoinSubstitution([
        FindPackageShare('v2x_ball_bot_control'), 'config', 'twist_mux.yaml'
    ])

    mux = Node(
        package='twist_mux', executable='twist_mux', name='twist_mux',
        parameters=[twist_mux_yaml],
        remappings=[('/cmd_vel_out', output_cmd)]
    )

    servo = Node(
        package='v2x_ball_bot_control', executable='visual_servoing_node', name='visual_servoing',
        parameters=[{
            'color_topic': color_topic,
            'depth_topic': depth_topic,
            'use_balls': True,
            'ball_topic': ball_topic
        }]
    )

    driver = Node(
        package='v2x_ball_bot_control', executable='motor_driver_node', name='motor_driver',
        parameters=[{
            'cmd_topic': output_cmd,
            'wheel_radius': 0.05,
            'wheel_base': 0.30
        }]
    )

    return LaunchDescription([
        DeclareLaunchArgument('output_cmd', default_value='/cmd_vel_out'),
        DeclareLaunchArgument('color_topic', default_value='/color/image_raw'),
        DeclareLaunchArgument('depth_topic', default_value='/depth/image_raw'),
        DeclareLaunchArgument('ball_topic',  default_value='/balls'),
        mux, servo, driver
    ])
