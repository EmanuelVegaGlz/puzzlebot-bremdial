#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    camera_resource = LaunchConfiguration('camera_resource')
    width = LaunchConfiguration('width')
    height = LaunchConfiguration('height')
    codec = LaunchConfiguration('codec')
    loop = LaunchConfiguration('loop')
    latency = LaunchConfiguration('latency')
    camera_calibration_file = LaunchConfiguration('camera_calibration_file')
    camera_info_frame_id = LaunchConfiguration('camera_info_frame_id')
    marker_size = LaunchConfiguration('marker_size')
    reference_frame = LaunchConfiguration('reference_frame')
    camera_frame = LaunchConfiguration('camera_frame')
    image_topic = LaunchConfiguration('image_topic')
    image_is_rectified = LaunchConfiguration('image_is_rectified')

    camera = Node(
        package='ros_deep_learning',
        executable='video_source',
        name='video_source',
        parameters=[{
            'resource': camera_resource,
            'width': ParameterValue(width, value_type=int),
            'height': ParameterValue(height, value_type=int),
            'codec': codec,
            'loop': ParameterValue(loop, value_type=int),
            'latency': ParameterValue(latency, value_type=int),
        }],
        output='screen',
    )

    camera_info = Node(
        package='camera_info_publisher',
        executable='camera_info_publisher',
        name='camera_info_publisher',
        parameters=[{
            'camera_calibration_file': camera_calibration_file,
            'frame_id': camera_info_frame_id,
        }],
        output='screen',
    )

    aruco = Node(
        package='aruco_ros',
        executable='marker_publisher',
        name='marker_publisher',
        parameters=[{
            'image_is_rectified': ParameterValue(image_is_rectified, value_type=bool),
            'marker_size': ParameterValue(marker_size, value_type=float),
            'reference_frame': reference_frame,
            'camera_frame': camera_frame,
        }],
        remappings=[('/image', image_topic)],
        output='screen',
    )

    return LaunchDescription([
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
        camera,
        camera_info,
        aruco,
    ])
