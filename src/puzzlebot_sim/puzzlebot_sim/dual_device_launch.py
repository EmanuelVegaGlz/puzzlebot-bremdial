from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.logging import get_logger
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from puzzlebot_sim.path_selection import resolve_path_selection


def _enabled_on_device(enable_arg, device_arg, this_device):
    return IfCondition(PythonExpression([
        "'", LaunchConfiguration(enable_arg), "'.lower() in ['true', '1', 'yes', 'on']",
        " and ",
        "'", LaunchConfiguration(device_arg), "'.lower() == '", this_device, "'",
    ]))


def _controller_enabled_on_device(this_device):
    true_values = "['true', '1', 'yes', 'on']"
    return IfCondition(PythonExpression([
        "('", LaunchConfiguration('enable_controller'), "'.lower() in ", true_values,
        " or ",
        "'", LaunchConfiguration('enable_bug'), "'.lower() in ", true_values, ")",
        " and ",
        "'", LaunchConfiguration('controller_device'), "'.lower() == '", this_device, "'",
    ]))


def dual_device_launch_arguments():
    return [
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'enable_localization',
            default_value='true',
            description='Start the wheel odometry localization node on the selected device.',
        ),
        DeclareLaunchArgument(
            'enable_aruco_ekf',
            default_value='true',
            description='Start the ArUco EKF correction node on the selected device.',
        ),
        DeclareLaunchArgument(
            'enable_bug',
            default_value='false',
            description='Compatibility alias for enable_controller.',
        ),
        DeclareLaunchArgument(
            'enable_controller',
            default_value='false',
            description='Start the controller node on the selected device.',
        ),
        DeclareLaunchArgument(
            'enable_path_generator',
            default_value='false',
            description='Start the path generator node on the selected device.',
        ),
        DeclareLaunchArgument(
            'localization_device',
            default_value='robot',
            description='Device that runs localization: robot or computer.',
        ),
        DeclareLaunchArgument(
            'aruco_ekf_device',
            default_value='robot',
            description='Device that runs ArUco EKF correction: robot or computer.',
        ),
        DeclareLaunchArgument(
            'controller_device',
            default_value='robot',
            description='Device that runs the controller: robot or computer.',
        ),
        DeclareLaunchArgument(
            'path_generator_device',
            default_value='robot',
            description='Device that runs the path generator: robot or computer.',
        ),
        DeclareLaunchArgument('world_frame', default_value='world_origin'),
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
        DeclareLaunchArgument('initial_x', default_value='0.3'),
        DeclareLaunchArgument('initial_y', default_value='-0.3'),
        DeclareLaunchArgument('initial_theta', default_value='0.0'),
        DeclareLaunchArgument(
            'initial_point',
            default_value='-1',
            description=(
                'Index of the path point used as the initial x/y pose. '
                'Use -1 to keep initial_x and initial_y.'
            ),
        ),
        DeclareLaunchArgument(
            'goal_point',
            default_value='-1',
            description=(
                'Index of the first path point published as the goal. '
                'Use -1 to keep the legacy first-point behavior.'
            ),
        ),
    ]


def _perform(context, name):
    return context.perform_substitution(LaunchConfiguration(name))


def _dual_device_nodes(context, this_device):
    use_sim_time = LaunchConfiguration('use_sim_time')
    localization_params = LaunchConfiguration('localization_params')
    path_params = LaunchConfiguration('path_params')
    world_frame = LaunchConfiguration('world_frame')
    odom_frame = LaunchConfiguration('odom_frame')
    base_frame = LaunchConfiguration('base_frame')
    initial_theta = LaunchConfiguration('initial_theta')

    initial_point = _perform(context, 'initial_point')
    goal_point = _perform(context, 'goal_point')
    initial_x, initial_y, goal_point = resolve_path_selection(
        _perform(context, 'path_params'),
        initial_point,
        goal_point,
        _perform(context, 'initial_x'),
        _perform(context, 'initial_y'),
    )

    if initial_point != '-1' or goal_point != '-1':
        get_logger('dual_device_launch').info(
            f'Selected path segment initial_point={initial_point} at '
            f'({initial_x:.3f}, {initial_y:.3f}), '
            f'goal_point={goal_point}'
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
                'world_frame': world_frame,
                'odom_frame': odom_frame,
                'base_frame': base_frame,
                'initial_x': initial_x,
                'initial_y': initial_y,
                'initial_theta': ParameterValue(
                    initial_theta,
                    value_type=float,
                ),
            },
        ],
        condition=_enabled_on_device(
            'enable_localization',
            'localization_device',
            this_device,
        ),
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
                'world_frame': world_frame,
                'base_frame': base_frame,
            },
        ],
        condition=_enabled_on_device(
            'enable_aruco_ekf',
            'aruco_ekf_device',
            this_device,
        ),
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
        condition=_controller_enabled_on_device(this_device),
    )

    path_generator_node = Node(
        package='puzzlebot_sim',
        executable='path_generator',
        name='path_generator',
        output='screen',
        parameters=[
            path_params,
            {
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                'goal_point': goal_point,
            },
        ],
        condition=_enabled_on_device(
            'enable_path_generator',
            'path_generator_device',
            this_device,
        ),
    )

    return [
        localization_node,
        aruco_ekf_node,
        controller_node,
        path_generator_node,
    ]


def dual_device_nodes(this_device):
    return [
        OpaqueFunction(
            function=_dual_device_nodes,
            kwargs={'this_device': this_device},
        )
    ]
