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

    config = os.path.join(
        get_package_share_directory('puzzlebot_sim'),
        'config',
        'path_params.yaml'
    )

    aruco_config = os.path.join(
        get_package_share_directory('puzzlebot_sim'),
        'config',
        'aruco_ekf_params.yaml'
    )


    rviz_config = os.path.join(
        get_package_share_directory('puzzlebot_sim'),
        'rviz',
        'puzzlebot_rviz_d.rviz'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config, '--ros-args', '--log-level', 'rviz2:=warn'],
        output='screen'
    )

    rqt_tf_tree_node = Node(
        package='rqt_tf_tree',
        executable='rqt_tf_tree',
        name='rqt_tf_tree'
    )

    localization_node = Node(
        package='puzzlebot_sim',
        executable='localization',
        name='localization',
        output='screen',
        parameters=[{'use_sim_time': True}, aruco_config],

    )

    controller_node = Node(
        package='puzzlebot_sim',
        executable='controller',
        name='controller',
        output='screen',
        parameters=[config],
    )

    path_generator_node = Node(
        package='puzzlebot_sim',
        executable='path_generator',
        name='path_generator',
        output='screen',
        parameters=[{'use_sim_time': True}, config],
    )

    rqt_graph_node = Node(
        package='rqt_graph',
        executable='rqt_graph',
        name='rqt_graph',
        output='screen',
    )

    rqt_plot_node = Node(
        package='rqt_plot',
        executable='rqt_plot',
        name='rqt_plot',
        output='screen',
        #arguments=['/sim_x', '/sim_y', '/wr', '/wl']
    )

    wall_follower_node = Node(
        package='puzzlebot_sim',
        executable='wall_follower',
        name='wall_follower',
        output='screen',
        )

    return LaunchDescription([
        robot_state_publisher_node,
        puzzlebot_node,
        rqt_tf_tree_node,
        localization_node,
        #wall_follower_node,
        controller_node,
        path_generator_node,
        joint_state_pub_node,
        rqt_plot_node,
        rqt_graph_node,
        rviz_node,
    ])
