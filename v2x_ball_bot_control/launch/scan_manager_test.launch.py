#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 패키지 경로 가져오기
    pkg_share = get_package_share_directory('v2x_ball_bot_control')
    config_dir = os.path.join(pkg_share, 'config')
    
    # 파라미터 파일 경로
    scan_manager_config = os.path.join(config_dir, 'scan_manager.yaml')
    
    # 스캔매니저 노드
    scan_manager_node = Node(
        package='v2x_ball_bot_control',
        executable='scan_manager',
        name='scan_manager',
        output='screen',
        parameters=[scan_manager_config],
        arguments=['--ros-args', '--log-level', 'scan_manager:=info']
    )
    
    # TF 브로드캐스터 (테스트용)
    tf_broadcaster = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        output='screen'
    )
    
    # 테스트용 공 발행자 (시뮬레이션)
    test_ball_publisher = Node(
        package='v2x_ball_bot_control',
        executable='ball_perception_node',
        name='test_ball_publisher',
        output='screen',
        parameters=[{
            'publish_test_balls': True,
            'test_ball_count': 1,
            'test_ball_static': False
        }]
    )
    
    return LaunchDescription([
        # 스캔매니저 실행
        scan_manager_node,
        
        # TF 설정 (테스트용)
        tf_broadcaster,
        
        # 테스트용 공 발행자 (선택사항)
        # test_ball_publisher,
    ])
