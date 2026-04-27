import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    # Get package paths
    pkg_name = 'puzzlebot_sim'
    
    # URDF file path
    urdf_file_name = 'puzzlebot.urdf'
    urdf_path = os.path.join(
        get_package_share_directory(pkg_name),
        'urdf',
        urdf_file_name
    )

    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    # Robot 1 config
    robot1_config = os.path.join(
        get_package_share_directory(pkg_name),
        'config',
        'robot1_params.yaml'
    )

    # Robot 2 config
    robot2_config = os.path.join(
        get_package_share_directory(pkg_name),
        'config',
        'robot2_params.yaml'
    )

    # RViz config
    rviz_config = os.path.join(
        get_package_share_directory(pkg_name),
        'rviz',
        'puzzlebot_rviz.rviz'
    )

    # List to hold all nodes
    nodes = []

    # ============================================
    # ROBOT 1 NODES (namespace: /robot1)
    # ============================================
    
    # Robot State Publisher for Robot 1
    nodes.append(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace='robot1',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    ))

    # Puzzlebot Simulation Node for Robot 1
    nodes.append(Node(
        package=pkg_name,
        executable='puzzlebot_sim',
        name='puzzlebot_sim',
        namespace='robot1',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'initial_x': 0.0,
            'initial_y': 0.0,
            'initial_theta': 0.0
        }]
    ))

    # Joint State Publisher for Robot 1
    nodes.append(Node(
        package=pkg_name,
        executable='joint_state_publisher',
        name='joint_state_publisher',
        namespace='robot1',
        output='screen',
    ))

    # Localization Node for Robot 1
    nodes.append(Node(
        package=pkg_name,
        executable='localization',
        name='localization',
        namespace='robot1',
        output='screen',
    ))

    # Controller Node for Robot 1
    nodes.append(Node(
        package=pkg_name,
        executable='controller',
        name='controller',
        namespace='robot1',
        output='screen',
    ))

    # Path Generator Node for Robot 1
    nodes.append(Node(
        package=pkg_name,
        executable='path_generator',
        name='path_generator',
        namespace='robot1',
        output='screen',
        parameters=[{'use_sim_time': True}, robot1_config],
    ))

    # ============================================
    # ROBOT 2 NODES (namespace: /robot2)
    # ============================================

    # Robot State Publisher for Robot 2
    nodes.append(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace='robot2',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    ))

    # Puzzlebot Simulation Node for Robot 2
    nodes.append(Node(
        package=pkg_name,
        executable='puzzlebot_sim',
        name='puzzlebot_sim',
        namespace='robot2',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'initial_x': 2.0,
            'initial_y': 2.0,
            'initial_theta': 1.57  # 90 degrees
        }]
    ))

    # Joint State Publisher for Robot 2
    nodes.append(Node(
        package=pkg_name,
        executable='joint_state_publisher',
        name='joint_state_publisher',
        namespace='robot2',
        output='screen',
    ))

    # Localization Node for Robot 2
    nodes.append(Node(
        package=pkg_name,
        executable='localization',
        name='localization',
        namespace='robot2',
        output='screen',
    ))

    # Controller Node for Robot 2
    nodes.append(Node(
        package=pkg_name,
        executable='controller',
        name='controller',
        namespace='robot2',
        output='screen',
    ))

    # Path Generator Node for Robot 2
    nodes.append(Node(
        package=pkg_name,
        executable='path_generator',
        name='path_generator',
        namespace='robot2',
        output='screen',
        parameters=[{'use_sim_time': True}, robot2_config],
    ))

    # ============================================
    # VISUALIZATION NODES
    # ============================================

    # RViz Node
    nodes.append(Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    ))

    # TF Tree visualization
    nodes.append(Node(
        package='rqt_tf_tree',
        executable='rqt_tf_tree',
        name='rqt_tf_tree'
    ))

    # RQT Graph
    nodes.append(Node(
        package='rqt_graph',
        executable='rqt_graph',
        name='rqt_graph',
        output='screen',
    ))

    return LaunchDescription(nodes)