import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace

def create_robot_group(namespace, robot_desc, config, ):

    return GroupAction([
        PushRosNamespace(namespace),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),

        Node(
            package='puzzlebot_sim',
            executable='puzzlebot_sim',
            name='puzzlebot_sim',
            output='screen',
        ),

        Node(
            package='puzzlebot_sim',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
        ),

        Node(
            package='puzzlebot_sim',
            executable='localization',
            name='localization',
            output='screen',
        ),

        Node(
            package='puzzlebot_sim',
            executable='controller',
            name='controller',
            output='screen',
        ),

        Node(
            package='puzzlebot_sim',
            executable='path_generator',
            name='path_generator',
            output='screen',
            parameters=[{'use_sim_time': True}, config],
        ),
    ])

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


    robot1 = create_robot_group('robot1', robot_desc, config)
    robot2 = create_robot_group('robot2', robot_desc, config)

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
        robot1,
        robot2,
        rqt_tf_tree_node,
        rqt_plot_node,
        rqt_graph_node,
        rviz_node,
    ])