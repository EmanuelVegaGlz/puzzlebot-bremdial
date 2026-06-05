from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


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
    ]


def dual_device_nodes(this_device):
    use_sim_time = LaunchConfiguration('use_sim_time')
    localization_params = LaunchConfiguration('localization_params')
    path_params = LaunchConfiguration('path_params')
    odom_frame = LaunchConfiguration('odom_frame')
    base_frame = LaunchConfiguration('base_frame')
    initial_x = LaunchConfiguration('initial_x')
    initial_y = LaunchConfiguration('initial_y')
    initial_theta = LaunchConfiguration('initial_theta')

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
                'odom_frame': odom_frame,
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
            {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)},
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
