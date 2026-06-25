#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
ArUco 도킹 런치파일
- aruco_detector: 카메라 영상에서 마커 인식
- aruco_moving (MarkerDockingController): 마커 기반 도킹 제어
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 런치 인자 선언
    marker_length_arg = DeclareLaunchArgument(
        'marker_length', default_value='0.05',
        description='ArUco 마커 한 변의 실제 길이 (미터)')

    target_marker_id_arg = DeclareLaunchArgument(
        'target_marker_id', default_value='-1',
        description='인식할 마커 ID (-1이면 모든 마커)')

    camera_topic_arg = DeclareLaunchArgument(
        'camera_topic', default_value='/camera/color/image_raw',
        description='카메라 이미지 토픽')

    # ArUco 감지 노드
    aruco_detector_node = Node(
        package='aruco_moving',
        executable='aruco_detector',
        name='aruco_detector',
        output='screen',
        parameters=[{
            'marker_length': LaunchConfiguration('marker_length'),
            'target_marker_id': LaunchConfiguration('target_marker_id'),
            'camera_topic': LaunchConfiguration('camera_topic'),
        }],
    )

    # 도킹 컨트롤러 노드
    docking_controller_node = Node(
        package='aruco_moving',
        executable='aruco_moving',
        name='marker_docking_controller',
        output='screen',
    )

    return LaunchDescription([
        marker_length_arg,
        target_marker_id_arg,
        camera_topic_arg,
        aruco_detector_node,
        docking_controller_node,
    ])
