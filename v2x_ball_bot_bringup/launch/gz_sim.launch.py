from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os

def generate_launch_description():
    ns = LaunchConfiguration('ns')
    world = PathJoinSubstitution([get_package_share_directory('v2x_ball_bot_bringup'),
                                  'worlds', 'empty_gz.world.sdf'])
    model_sdf = PathJoinSubstitution([get_package_share_directory('v2x_ball_bot_description'),
                                      'gz', 'yahboomcar_x3_gz.sdf'])

    # Gazebo Sim 실행
    gz = ExecuteProcess(cmd=['gz', 'sim', '-r', world], output='screen')

    # 모델 SDF를 임시파일로 복사하면서 __NAME__ 치환
    tmp_model = '/tmp/yahboomcar_x3_{}.sdf'.format(LaunchConfiguration('ns').perform({}) if isinstance(ns, LaunchConfiguration) else 'car1')

    # 치환은 셸에서 처리(간단): sed 사용
    prep = ExecuteProcess(
        cmd=['bash', '-lc', f"sed 's/__NAME__/'\"$NS\"'/g' {model_sdf.perform({})} > {tmp_model}"],
        additional_env={'NS': LaunchConfiguration('ns').perform({}) if isinstance(ns, LaunchConfiguration) else 'car1'},
        output='screen'
    )

    # 스폰
    spawn = ExecuteProcess(
        cmd=['ros2','run','ros_gz_sim','create','-file', tmp_model, '-name', ns, '-z','0.1'],
        output='screen'
    )

    # 브릿지: cmd_vel, odom, scan, tf
    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', output='screen',
        arguments=[
            # ROS ↔ GZ
            f'/model/{ns.perform({}) if isinstance(ns, LaunchConfiguration) else "car1"}/cmd_vel'
            + '@geometry_msgs/msg/Twist@gz.msgs.Twist',
            f'/model/{ns.perform({}) if isinstance(ns, LaunchConfiguration) else "car1"}/odometry'
            + '@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            f'/model/{ns.perform({}) if isinstance(ns, LaunchConfiguration) else "car1"}/scan'
            + '@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V'
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument('ns', default_value='car1'),
        gz, prep, spawn, bridge
    ])
