from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os

def make_runtime_nodes(context):
    ns = LaunchConfiguration('ns').perform(context)
    bringup_share = get_package_share_directory('v2x_ball_bot_bringup')
    desc_share = get_package_share_directory(LaunchConfiguration('desc_pkg').perform(context))

    model_rel = LaunchConfiguration('model_relpath').perform(context)
    model_path = os.path.join(desc_share, model_rel)

    spawn = ExecuteProcess(
        cmd=['ros2','run','ros_gz_sim','create','-file', model_path, '-name', ns, '-z','0.1'],
        output='screen'
    )

    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', output='screen',
        arguments=[
            f'/model/{ns}/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            f'/model/{ns}/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            f'/model/{ns}/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V'
        ]
    )
    return [spawn, bridge]

def generate_launch_description():
    world = PathJoinSubstitution([
        get_package_share_directory('v2x_ball_bot_bringup'),
        'worlds', 'empty_gz.world.sdf'
    ])
    gz = ExecuteProcess(cmd=['gz', 'sim', '-r', world], output='screen')

    return LaunchDescription([
        DeclareLaunchArgument('ns', default_value='car1'),
        DeclareLaunchArgument('desc_pkg', default_value='v2x_ball_bot_description'),
        DeclareLaunchArgument('model_relpath', default_value='gz/yahboomcar_x3_gz.sdf'),
        gz,
        OpaqueFunction(function=make_runtime_nodes)
    ])
