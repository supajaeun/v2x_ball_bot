from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition
import os

def _setup(context, *args, **kwargs):
    ns = LaunchConfiguration('ns').perform(context)
    xacro_pkg = LaunchConfiguration('xacro_pkg').perform(context)
    # 기본: yahboomcar_X3.urdf.xacro (원하면 인자로 교체)
    xacro_rel = LaunchConfiguration('xacro_relpath').perform(context)
    share = get_package_share_directory(xacro_pkg)
    xacro_path = os.path.join(share, xacro_rel)

    # robot_state_publisher: Xacro → robot_description
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=ns,
        parameters=[{
            'use_sim_time': True,
            'robot_description': Command(['xacro ', xacro_path, ' ns:=', ns])
        }],
        output='screen'
    )

    # Gazebo에 엔티티 스폰 (topic에서 읽기)
    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', ns, '-topic', f'/{ns}/robot_description', '-z', '0.1'],
        output='screen'
    )

    # base_link → base_footprint 정적 TF (ROS1 파일에 있던 내용 이식)
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0','0','0','0','0','0', f'{ns}/base_link', f'{ns}/base_footprint'],
        output='screen'
    )

    # (옵션) /calibrated true 1회 퍼블리시 (ROS1 파일의 rostopic pub 대응)
    pub_cal = ExecuteProcess(
        cmd=['ros2', 'topic', 'pub', '--once', '/calibrated', 'std_msgs/Bool', '{data: true}'],
        condition=IfCondition(LaunchConfiguration('publish_calibrated')),
        output='screen'
    )

    return [rsp, spawn, static_tf, pub_cal]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('ns', default_value='car1'),
        DeclareLaunchArgument('xacro_pkg', default_value='v2x_ball_bot_description'),
        DeclareLaunchArgument('xacro_relpath', default_value='urdf/yahboomcar_X3.urdf.xacro'),
        DeclareLaunchArgument('publish_calibrated', default_value='false'),

        # Gazebo Classic 실행 (빈 월드)
        ExecuteProcess(cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
                       output='screen'),

        OpaqueFunction(function=_setup),
    ])
