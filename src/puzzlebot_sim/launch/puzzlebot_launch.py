import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():


    urdf_file_name = 'puzzlebot.urdf'
    urdf_path = os.path.join(
        get_package_share_directory('puzzlebot_sim'),
        'urdf',
        urdf_file_name
    )

    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )

    puzzlebot_node = Node(
        package='puzzlebot_sim',
        executable='puzzlebot_sim',
        name='puzzlebot_sim',
        output='screen',
    )

    static_transform_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '2', '1', '0',   # x y z
            '0', '0', '0',   # roll pitch yaw
            'map',
            'odom'
        ]
    )

    rviz_config = os.path.join(
        get_package_share_directory('puzzlebot_sim'),
        'rviz',
        'puzzlebot_rviz.rviz'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    rqt_tf_tree_node = Node(
        package='rqt_tf_tree',
        executable='rqt_tf_tree',
        name='rqt_tf_tree'
    )

    return LaunchDescription([
        robot_state_publisher_node,
        puzzlebot_node,
        static_transform_map_odom,
        rviz_node,
        rqt_tf_tree_node
    ])