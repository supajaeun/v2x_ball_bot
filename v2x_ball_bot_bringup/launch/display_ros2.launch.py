from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition, UnlessCondition
import os

def _setup(context, *args, **kwargs):
    ns = LaunchConfiguration('ns').perform(context)
    use_gui = LaunchConfiguration('use_gui').perform(context)
    xacro_pkg = LaunchConfiguration('xacro_pkg').perform(context)
    robot_type = LaunchConfiguration('robot_type').perform(context)

    share = get_package_share_directory(xacro_pkg)

    # 우선순위: yahboomcar_{robot_type}.urdf.xacro → 없으면 .urdf
    xacro_rel = f'urdf/yahboomcar_{robot_type}.urdf.xacro'
    xacro_path = os.path.join(share, xacro_rel)
    if not os.path.exists(xacro_path):
        xacro_rel = f'urdf/yahboomcar_{robot_type}.urdf'
        xacro_path = os.path.join(share, xacro_rel)

    # robot_description
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

    jsp_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        namespace=ns,
        condition=IfCondition(LaunchConfiguration('use_gui')),
        output='screen'
    )

    jsp = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        namespace=ns,
        condition=UnlessCondition(LaunchConfiguration('use_gui')),
        output='screen'
    )

    rviz_cfg = os.path.join(share, 'rviz', 'yahboomcar.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_cfg] if os.path.exists(rviz_cfg) else [],
        output='screen'
    )

    return [jsp_gui, jsp, rsp, rviz]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('ns', default_value='robot1'),
        DeclareLaunchArgument('use_gui', default_value='true'),
        DeclareLaunchArgument('xacro_pkg', default_value='v2x_ball_bot_description'),
        DeclareLaunchArgument('robot_type', default_value='X3'),
        OpaqueFunction(function=_setup),
    ])
