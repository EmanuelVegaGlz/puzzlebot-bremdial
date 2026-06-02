import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_aruco_ekf = LaunchConfiguration('enable_aruco_ekf')
    odom_frame = LaunchConfiguration('odom_frame')
    base_frame = LaunchConfiguration('base_frame')
    aruco_params = LaunchConfiguration('aruco_params')

    urdf_path = os.path.join(
        get_package_share_directory('puzzlebot_sim'),
        'urdf',
        'puzzlebot.urdf',
    )
    path_params = os.path.join(
        get_package_share_directory('puzzlebot_sim'),
        'config',
        'path_params.yaml',
    )
    default_aruco_params = os.path.join(
        get_package_share_directory('puzzlebot_sim'),
        'config',
        'aruco_ekf_params.yaml',
    )
    rviz_config = os.path.join(
        get_package_share_directory('puzzlebot_sim'),
        'rviz',
        'puzzlebot_rviz.rviz',
    )

    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
        }],
    )

    localization_node = Node(
        package='puzzlebot_sim',
        executable='localization',
        name='localization',
        output='screen',
        parameters=[
            aruco_params,
            {
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                'odom_frame': odom_frame,
                'base_frame': base_frame,
            },
        ],
    )

    aruco_ekf_localization_node = Node(
        package='puzzlebot_sim',
        executable='aruco_ekf_localization',
        name='aruco_ekf_localization',
        output='screen',
        parameters=[
            aruco_params,
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
            {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)},
            path_params,
        ],
    )

    path_generator_node = Node(
        package='puzzlebot_sim',
        executable='path_generator',
        name='path_generator',
        output='screen',
        parameters=[
            {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)},
            path_params,
        ],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config, '--ros-args', '--log-level', 'rviz2:=warn'],
        output='screen',
        parameters=[{'use_sim_time': ParameterValue(use_sim_time, value_type=bool)}],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock when true; set false on the real robot.',
        ),
        DeclareLaunchArgument(
            'enable_aruco_ekf',
            default_value='false',
            description='Launch the ArUco EKF correction node.',
        ),
        DeclareLaunchArgument('odom_frame', default_value='odom'),
        DeclareLaunchArgument('base_frame', default_value='base_footprint'),
        DeclareLaunchArgument('aruco_params', default_value=default_aruco_params),
        robot_state_publisher_node,
        localization_node,
        aruco_ekf_localization_node,
        controller_node,
        path_generator_node,
        rviz_node,
    ])
