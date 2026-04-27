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



    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc, 'frame_prefix': 'robot_1/'},],
        namespace='robot_1',
    )

    puzzlebot_node = Node(
        package='puzzlebot_sim',
        executable='puzzlebot_sim',
        name='puzzlebot_sim',
        output='screen',
        namespace='robot_1',
        parameters=[{'x0': 0.0}, {'y0': 0.0}, {'theta0': 0.0}],
    )
   
    joint_state_pub_node = Node(
        package='puzzlebot_sim',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        namespace='robot_1',
        parameters=[{'odom_frame': 'odom'}, {'base_frame': 'robot_1/base_footprint'}]
    )

    localization_node = Node(
        package='puzzlebot_sim',
        executable='localization',
        name='localization',
        output='screen',
        namespace='robot_1',
    )

    controller_node = Node(
        package='puzzlebot_sim',
        executable='controller',
        name='controller',
        output='screen',
        namespace='robot_1',
    )

    path_generator_node = Node(
        package='puzzlebot_sim',
        executable='path_generator',
        name='path_generator',
        output='screen',
        parameters=[{'use_sim_time': True}, config],
        namespace='robot_1',
    )





    robot_state_publisher_node2 = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc, 'frame_prefix': 'robot_2/'}],
        namespace='robot_2',
    )

    puzzlebot_node2 = Node(
        package='puzzlebot_sim',
        executable='puzzlebot_sim',
        name='puzzlebot_sim',
        output='screen',
        namespace='robot_2',
        parameters=[{'x0': 1.0}, {'y0': 0.0}, {'theta0': 0.0}],
    )
   
    joint_state_pub_node2 = Node(
        package='puzzlebot_sim',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        namespace='robot_2',
        parameters=[{'odom_frame': 'odom'}, {'base_frame': 'robot_2/base_footprint'}],
    )

    localization_node2 = Node(
        package='puzzlebot_sim',
        executable='localization',
        name='localization',
        output='screen',
        namespace='robot_2',
    )

    controller_node2 = Node(
        package='puzzlebot_sim',
        executable='controller',
        name='controller',
        output='screen',
        namespace='robot_2',
    )

    path_generator_node2 = Node(
        package='puzzlebot_sim',
        executable='path_generator',
        name='path_generator',
        output='screen',
        parameters=[{'use_sim_time': True}, config],
        namespace='robot_2',
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
        rqt_tf_tree_node,
        localization_node,
        controller_node,
        path_generator_node,
        joint_state_pub_node,
        rqt_plot_node,
        rqt_graph_node,
        rviz_node,
        
        robot_state_publisher_node2,
        puzzlebot_node2,
        localization_node2,
        controller_node2,
        path_generator_node2,
        joint_state_pub_node2,

    ])