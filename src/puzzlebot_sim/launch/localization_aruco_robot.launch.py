#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_aruco = LaunchConfiguration('enable_aruco')
    enable_aruco_ekf = LaunchConfiguration('enable_aruco_ekf')
    enable_bug = LaunchConfiguration('enable_bug')
    enable_path_generator = LaunchConfiguration('enable_path_generator')

    localization_params = LaunchConfiguration('localization_params')
    path_params = LaunchConfiguration('path_params')
    odom_frame = LaunchConfiguration('odom_frame')
    base_frame = LaunchConfiguration('base_frame')

    initial_x = LaunchConfiguration('initial_x')
    initial_y = LaunchConfiguration('initial_y')
    initial_theta = LaunchConfiguration('initial_theta')

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

    localization_node = Node(
        package='puzzlebot_sim',
        executable='localization',
        name='localization',
        output='screen',
        parameters=[
            localization_params,
            {
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                'odom_frame': odom_frame,
                'base_frame': base_frame,
                'initial_x': ParameterValue(initial_x, value_type=float),
                'initial_y': ParameterValue(initial_y, value_type=float),
                'initial_theta': ParameterValue(initial_theta, value_type=float),
            },
        ],
    )

    aruco_ekf_node = Node(
        package='puzzlebot_sim',
        executable='aruco_ekf_localization',
        name='aruco_ekf_localization',
        output='screen',
        parameters=[
            localization_params,
            {
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                'odom_frame': odom_frame,
                'base_frame': base_frame,
            },
        ],
        condition=IfCondition(enable_aruco_ekf),
    )

    controller_node = Node(
        package='puzzlebot_sim',
        executable='controller',
        name='controller',
        output='screen',
        parameters=[
            path_params,
            {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)},
        ],
        condition=IfCondition(enable_bug),
    )

    path_generator_node = Node(
        package='puzzlebot_sim',
        executable='path_generator',
        name='path_generator',
        output='screen',
        parameters=[
            path_params,
            {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)},
        ],
        condition=IfCondition(enable_path_generator),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('enable_aruco', default_value='true'),
        DeclareLaunchArgument('enable_aruco_ekf', default_value='true'),
        DeclareLaunchArgument(
            'enable_rviz',
            default_value='false',
            description=(
                'Deprecated compatibility argument. RViz now runs from '
                'localization_aruco_computer.launch.py on the workstation.'
            ),
        ),
        DeclareLaunchArgument('enable_bug', default_value='false'),
        DeclareLaunchArgument('enable_path_generator', default_value='false'),
        DeclareLaunchArgument('odom_frame', default_value='odom'),
        DeclareLaunchArgument('base_frame', default_value='base_footprint'),
        DeclareLaunchArgument(
            'localization_params',
            default_value=PathJoinSubstitution([
                FindPackageShare('puzzlebot_sim'),
                'config',
                'aruco_ekf_params.yaml',
            ]),
        ),
        DeclareLaunchArgument(
            'path_params',
            default_value=PathJoinSubstitution([
                FindPackageShare('puzzlebot_sim'),
                'config',
                'path_params.yaml',
            ]),
        ),
        DeclareLaunchArgument('initial_x', default_value='0.0'),
        DeclareLaunchArgument('initial_y', default_value='0.0'),
        DeclareLaunchArgument('initial_theta', default_value='0.0'),
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
        DeclareLaunchArgument('camera_info_frame_id', default_value='camera'),
        DeclareLaunchArgument('marker_size', default_value='0.094'),
        DeclareLaunchArgument('reference_frame', default_value='base_footprint'),
        DeclareLaunchArgument('camera_frame', default_value='camera'),
        DeclareLaunchArgument('image_topic', default_value='/video_source/raw'),
        DeclareLaunchArgument('image_is_rectified', default_value='true'),
        aruco_launch,
        localization_node,
        aruco_ekf_node,
        controller_node,
        path_generator_node,
    ])
