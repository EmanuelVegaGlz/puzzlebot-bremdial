from pathlib import Path

import yaml


WORKSPACE = Path(__file__).resolve().parents[3]


def test_external_aruco_launch_uses_optical_camera_frame():
    launch_source = (
        WORKSPACE / 'src/puzzlebot_ros/launch/aruco_jetson.launch.py'
    ).read_text()

    assert "default_value='camera_link_optical'" in launch_source
    assert "default_value='base_footprint'" in launch_source
    assert "'frame_id': camera_frame" in launch_source
    assert "'camera_frame': camera_frame" in launch_source


def test_localization_and_controller_use_world_origin_interfaces():
    with (
        WORKSPACE / 'src/puzzlebot_sim/config/aruco_ekf_params.yaml'
    ).open() as stream:
        localization_params = yaml.safe_load(stream)['/**']['ros__parameters']
    with (
        WORKSPACE / 'src/puzzlebot_sim/config/path_params.yaml'
    ).open() as stream:
        path_params = yaml.safe_load(stream)

    assert localization_params['world_frame'] == 'world_origin'
    assert localization_params['raw_odom_topic'] == 'odom'
    assert localization_params['localization_topic'] == 'localization/odom'
    assert localization_params['odom_topic'] == 'localization/odom'
    assert localization_params['marker_measurement_frame'] == 'robot_xy'
    assert path_params['path_generator']['ros__parameters']['path_frame'] == (
        'world_origin'
    )
    assert path_params['controller']['ros__parameters']['pose_topic'] == (
        '/localization/odom'
    )
