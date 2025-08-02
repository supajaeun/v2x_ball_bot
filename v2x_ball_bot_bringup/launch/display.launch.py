from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import xacro

def generate_launch_description():
    # v2x_ball_bot_description 패키지의 경로
    description_pkg = get_package_share_directory('v2x_ball_bot_description')

    # .xacro 파일 경로
    xacro_file = os.path.join(description_pkg, 'urdf', 'v2x_ball_bot.xacro')

    # xacro → URDF 변환
    doc = xacro.process_file(xacro_file)
    robot_description_config = doc.toxml()

    # robot_state_publisher 노드
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_config}]
    )

    # RViz 노드
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
    )

    return LaunchDescription([
        robot_state_publisher_node,
        rviz_node
    ])
