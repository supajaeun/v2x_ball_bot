from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    bringup_dir = os.path.join(
        get_package_share_directory('v2x_ball_bot_bringup'), 'config'
    )

    rviz_config_path = os.path.join(bringup_dir, 'ball_perception.rviz')

    return LaunchDescription([
        # Ball perception node
        Node(
            package='v2x_ball_bot_control',
            executable='ball_perception_node',
            name='ball_perception',
            output='screen',
            parameters=[{
                'rgb_topic': '/color/image_raw',
                'depth_topic': '/depth/image_raw',
                'camera_info_topic': '/color/camera_info',
                'publish_frame': 'map'
            }]
        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path],
            output='screen'
        ),

        # Static TF: map -> base_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_map_to_base',
            arguments=['0', '0', '0', '0', '0', '0',
                       'map', 'base_link'],
            output='screen'
        ),

        # Static TF: base_link -> orbbec_camera_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_base_to_camera',
            arguments=['0', '0', '0', '0', '0', '0',
                       'base_link', 'orbbec_camera_link'],
            output='screen'
        ),

        # Static TF: orbbec_camera_link -> orbbec_camera_color_optical_frame
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_camera_to_color_optical',
            arguments=['0', '0', '0', '0', '0', '0',
                       'orbbec_camera_link', 'orbbec_camera_color_optical_frame'],
            output='screen'
        ),
    ])
