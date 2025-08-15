from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def _spawn_robot(context, ns, xacro_pkg, xacro_relpath, extra_xacro_args):
    # xacro -> robot_description
    xacro_path = os.path.join(get_package_share_directory(xacro_pkg), xacro_relpath)
    xacro_cmd = ['xacro', xacro_path, f'ns:={ns}'] + extra_xacro_args
    robot_description = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'use_sim_time': True,
            'robot_description': (' '.join(xacro_cmd))
        }],
        namespace=ns,
        output='screen'
    )

    # Gazebo에 엔티티 스폰
    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', ns,
                   '-topic', f'/{ns}/robot_description',
                   '-x', '0', '-y', '0', '-z', '0.1'],
        output='screen'
    )
    return [robot_description, spawn]

def _multi_spawn(context):
    ns_csv      = LaunchConfiguration('ns_list').perform(context)
    xacro_pkg   = LaunchConfiguration('xacro_pkg').perform(context)
    xacro_rel   = LaunchConfiguration('xacro_relpath').perform(context)
    extra_args  = LaunchConfiguration('xacro_extra_args').perform(context)
    extra_xacro = extra_args.split() if extra_args else []

    actions = []
    for i, ns in enumerate([s.strip() for s in ns_csv.split(',') if s.strip()]):
        actions += _spawn_robot(context, ns, xacro_pkg, xacro_rel, extra_xacro)
    return actions

def generate_launch_description():
    return LaunchDescription([
        # 인자들
        DeclareLaunchArgument('ns_list', default_value='car1',
                              description='스폰할 로봇 네임스페이스 목록(쉼표 구분)'),
        DeclareLaunchArgument('xacro_pkg', default_value='yahboomcar_description',
                              description='yahboomcar X3 xacro가 들어있는 패키지명'),
        DeclareLaunchArgument('xacro_relpath', default_value='yahboomcar_X3.urdf.xacro',
                              description='패키지 내 xacro 상대 경로'),
        DeclareLaunchArgument('xacro_extra_args', default_value='',
                              description='추가 xacro 인자 (예: "left_joint:=L right_joint:=R")'),

        # Gazebo Classic 실행 (빈 월드)
        ExecuteProcess(cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
                       output='screen'),

        # 여러 대 스폰
        OpaqueFunction(function=_multi_spawn),
    ])
