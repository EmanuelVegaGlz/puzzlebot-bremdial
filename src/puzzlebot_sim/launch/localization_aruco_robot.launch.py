#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from puzzlebot_sim.dual_device_launch import dual_device_launch_arguments
from puzzlebot_sim.dual_device_launch import dual_device_nodes


def generate_launch_description():
    enable_aruco = LaunchConfiguration('enable_aruco')

    aruco_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('puzzlebot_sim'),
                'launch',
                'aruco_jetson.launch.py',
            ])
        ),
        launch_arguments={
            'camera_resource': LaunchConfiguration('camera_resource'),
            'width': LaunchConfiguration('width'),
            'height': LaunchConfiguration('height'),
            'codec': LaunchConfiguration('codec'),
            'loop': LaunchConfiguration('loop'),
            'latency': LaunchConfiguration('latency'),
            'camera_calibration_file': LaunchConfiguration('camera_calibration_file'),
            'camera_info_frame_id': LaunchConfiguration('camera_info_frame_id'),
            'marker_size': LaunchConfiguration('marker_size'),
            'reference_frame': LaunchConfiguration('reference_frame'),
            'camera_frame': LaunchConfiguration('camera_frame'),
            'image_topic': LaunchConfiguration('image_topic'),
            'image_is_rectified': LaunchConfiguration('image_is_rectified'),
        }.items(),
        condition=IfCondition(enable_aruco),
    )

    return LaunchDescription([
        *dual_device_launch_arguments(),
        DeclareLaunchArgument(
            'enable_aruco',
            default_value='false',
            description=(
                'Start the puzzlebot_sim camera stack. Keep false when the '
                'external puzzlebot_ros aruco_jetson launch is active.'
            ),
        ),
        DeclareLaunchArgument(
            'enable_rviz',
            default_value='false',
            description=(
                'Deprecated compatibility argument. RViz now runs from '
                'localization_aruco_computer.launch.py on the workstation.'
            ),
        ),
        DeclareLaunchArgument('camera_resource', default_value='csi://0'),
        DeclareLaunchArgument('width', default_value='1280'),
        DeclareLaunchArgument('height', default_value='720'),
        DeclareLaunchArgument('codec', default_value='unknown'),
        DeclareLaunchArgument('loop', default_value='0'),
        DeclareLaunchArgument('latency', default_value='2000'),
        DeclareLaunchArgument(
            'camera_calibration_file',
            default_value='file:///home/puzzlebot/.ros/jetson_cam.yaml',
        ),
        DeclareLaunchArgument(
            'camera_info_frame_id',
            default_value='camera_link_optical',
        ),
        DeclareLaunchArgument('marker_size', default_value='0.094'),
        DeclareLaunchArgument('reference_frame', default_value='base_footprint'),
        DeclareLaunchArgument('camera_frame', default_value='camera_link_optical'),
        DeclareLaunchArgument('image_topic', default_value='/video_source/raw'),
        DeclareLaunchArgument('image_is_rectified', default_value='true'),
        aruco_launch,
        *dual_device_nodes('robot'),
    ])
