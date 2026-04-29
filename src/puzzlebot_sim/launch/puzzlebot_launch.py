import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import GroupAction

def make_prefixed_urdf(robot_desc: str, prefix: str) -> str:

    links  = ['base_footprint', 'base_link', 'wheel_left',
              'wheel_right', 'caster_wheel']
    joints = ['base_link_joint', 'wheel_left_joint',
              'wheel_right_joint', 'caster_wheel_joint']

    result = robot_desc
    for name in links + joints:
        result = result.replace(f'"{name}"', f'"{prefix}/{name}"')
    return result

def generate_launch_description():

    urdf_path = os.path.join(
        get_package_share_directory('puzzlebot_sim'), 'urdf', 'puzzlebot.urdf')
    with open(urdf_path, 'r') as f:
        robot_desc_base = f.read()

    config = os.path.join(
        get_package_share_directory('puzzlebot_sim'), 'config', 'path_params.yaml')

    rviz_config = os.path.join(
        get_package_share_directory('puzzlebot_sim'), 'rviz', 'puzzlebot_rviz.rviz')

    def robot_group(namespace, x0, y0, theta0, path_points):
        fp = namespace  # frame prefix

        robot_desc = make_prefixed_urdf(robot_desc_base, fp)

        return GroupAction([
            PushRosNamespace(namespace),

            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='screen',
                parameters=[{'robot_description': robot_desc}],
            ),

            Node(
                package='puzzlebot_sim',
                executable='puzzlebot_sim',
                name='puzzlebot_sim',
                output='screen',
                parameters=[{
                    'x0': float(x0),
                    'y0': float(y0),
                    'theta0': float(theta0),
                    'robot_frame_prefix': fp,
                }],
            ),

            Node(
                package='puzzlebot_sim',
                executable='joint_state_publisher',
                name='joint_state_publisher',
                output='screen',
                parameters=[{'robot_frame_prefix': fp}],
            ),

            Node(
                package='puzzlebot_sim',
                executable='localization',
                name='localization',
                output='screen',
                parameters=[{'robot_frame_prefix': fp}],
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
                parameters=[
                    {'use_sim_time': False},
                    config,
                    {'path_points': path_points},
                ],
            ),
        ])

    robot1 = robot_group(
        namespace='robot1',
        x0=0.0, y0=0.0, theta0=0.0,
        path_points=[0.0, 2.0, 2.0, 2.0, 2.0, 0.0, 0.0, 0.0],
    )

    robot2 = robot_group(
        namespace='robot2',
        x0=1.0, y0=0.0, theta0=1.5708,
        path_points=[1.0, 3.0, 3.0, 3.0, 3.0, 1.0, 1.0, 1.0],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
    )

    rqt_graph_node = Node(
        package='rqt_graph',
        executable='rqt_graph',
        name='rqt_graph',
        output='screen',
    )

    rqt_tf_tree_node = Node(
        package='rqt_tf_tree',
        executable='rqt_tf_tree',
        name='rqt_tf_tree',
        output='screen',
    )

    return LaunchDescription([
        robot1,
        robot2,
        rviz_node,
        rqt_graph_node,
        rqt_tf_tree_node,
    ])