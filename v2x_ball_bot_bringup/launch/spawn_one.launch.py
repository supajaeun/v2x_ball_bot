from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('ns', default_value='car1'),
        DeclareLaunchArgument('xacro_pkg', default_value='yahboomcar_description'),
        DeclareLaunchArgument('xacro_relpath', default_value='yahboomcar_X3.urdf.xacro'),
        DeclareLaunchArgument('xacro_extra_args', default_value=''),

        ExecuteProcess(cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
                       output='screen'),

        Node(package='robot_state_publisher', executable='robot_state_publisher',
             namespace=LaunchConfiguration('ns'),
             parameters=[{
                 'use_sim_time': True,
                 'robot_description':
                    ('xacro ' +
                     os.path.join(get_package_share_directory(LaunchConfiguration('xacro_pkg').perform(None)),
                                  LaunchConfiguration('xacro_relpath').perform(None)) +
                     ' ns:=' + LaunchConfiguration('ns').perform(None) + ' ' +
                     LaunchConfiguration('xacro_extra_args').perform(None))
             }], output='screen'),

        Node(package='gazebo_ros', executable='spawn_entity.py', output='screen',
             arguments=['-entity', LaunchConfiguration('ns'),
                        '-topic', '/' + LaunchConfiguration('ns').perform(None) + '/robot_description',
                        '-x', '0', '-y', '0', '-z', '0.1']),
    ])
