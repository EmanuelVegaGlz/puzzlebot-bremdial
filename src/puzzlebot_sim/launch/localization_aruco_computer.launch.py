#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from puzzlebot_sim.dual_device_launch import dual_device_launch_arguments
from puzzlebot_sim.dual_device_launch import dual_device_nodes


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_rviz = LaunchConfiguration('enable_rviz')
    enable_image_view = LaunchConfiguration('enable_image_view')
    rviz_config = LaunchConfiguration('rviz_config')
    image_topic = LaunchConfiguration('image_topic')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config, '--ros-args', '--log-level', 'rviz2:=warn'],
        output='screen',
        parameters=[{'use_sim_time': ParameterValue(use_sim_time, value_type=bool)}],
        condition=IfCondition(enable_rviz),
    )

    image_view_node = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='rqt_image_view',
        arguments=[image_topic],
        output='screen',
        condition=IfCondition(enable_image_view),
    )

    return LaunchDescription([
        *dual_device_launch_arguments(),
        DeclareLaunchArgument('enable_rviz', default_value='true'),
        DeclareLaunchArgument('enable_image_view', default_value='false'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=PathJoinSubstitution([
                FindPackageShare('puzzlebot_sim'),
                'rviz',
                'puzzlebot_rviz.rviz',
            ]),
        ),
        DeclareLaunchArgument('image_topic', default_value='/video_source/raw'),
        rviz_node,
        #image_view_node,
        *dual_device_nodes('computer'),
    ])
