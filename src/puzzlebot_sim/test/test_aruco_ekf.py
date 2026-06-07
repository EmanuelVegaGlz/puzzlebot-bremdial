import math

import numpy as np

from puzzlebot_sim.aruco_ekf_localization import (
    build_marker_map,
    ekf_range_bearing_update,
    expected_marker_measurement,
    map_xy_to_world_xy,
    marker_frame_matches,
    marker_observation_is_fresh,
    normalize_angle,
    normalize_marker_measurement_frame,
    normalized_innovation_squared,
    optical_translation_to_range_bearing,
    range_bearing_jacobian,
    robot_xy_translation_to_range_bearing,
    should_use_marker,
)


def test_normalize_angle_wraps_to_pi_interval():
    assert math.isclose(normalize_angle(3.0 * math.pi), math.pi)
    assert math.isclose(normalize_angle(-3.0 * math.pi), -math.pi)


def test_robot_xy_translation_uses_x_forward_and_negative_y_right():
    measurement = robot_xy_translation_to_range_bearing(0.3, -0.4, 99.0)

    assert math.isclose(measurement[0], 0.5)
    assert measurement[1] < 0.0
    assert math.isclose(measurement[1], math.atan2(-0.4, 0.3))


def test_optical_translation_remains_available_as_fallback():
    measurement = optical_translation_to_range_bearing(
        0.037467774,
        -0.014119647,
        0.298310250,
    )

    assert math.isclose(measurement[0], 0.300652, rel_tol=1e-5)
    assert measurement[1] < 0.0
    assert math.isclose(
        measurement[1],
        math.atan2(-0.037467774, 0.298310250),
        rel_tol=1e-9,
    )


def test_measurement_frame_aliases_are_normalized():
    assert normalize_marker_measurement_frame('robot_xy') == 'robot_xy'
    assert normalize_marker_measurement_frame('base_xy') == 'robot_xy'
    assert normalize_marker_measurement_frame('base') == 'robot_xy'
    assert normalize_marker_measurement_frame('base_footprint') == 'robot_xy'
    assert normalize_marker_measurement_frame('camera_optical') == 'optical'


def test_map_coordinates_scale_and_rotate_into_world_frame():
    assert np.allclose(map_xy_to_world_xy(184.0, -30.0, scale=0.01), [1.84, -0.30])
    assert np.allclose(
        map_xy_to_world_xy(100.0, 0.0, scale=0.01, origin_x=2.0, origin_y=3.0),
        [3.0, 3.0],
    )
    assert np.allclose(
        map_xy_to_world_xy(100.0, 0.0, scale=0.01, yaw=math.pi / 2.0),
        [0.0, 1.0],
        atol=1e-12,
    )


def test_expected_measurement_and_jacobian_match_worked_example_shape():
    state = np.array([0.1, 0.0, 0.1])
    marker_xy = np.array([3.0, 4.0])

    measurement = expected_marker_measurement(state, marker_xy)
    jacobian = range_bearing_jacobian(state, marker_xy)

    assert np.allclose(
        measurement,
        [math.sqrt(2.9 ** 2 + 4.0 ** 2), math.atan2(4.0, 2.9) - 0.1],
        atol=1e-12,
    )
    assert np.allclose(
        jacobian,
        [
            [-0.5870, -0.8096, 0.0],
            [0.1639, -0.1188, -1.0],
        ],
        atol=5e-4,
    )


def test_ekf_update_keeps_matching_state_and_reduces_covariance():
    state = np.array([0.1, 0.0, 0.1])
    covariance = np.array([
        [0.5, 0.01, 0.01],
        [0.01, 0.5, 0.01],
        [0.01, 0.01, 0.2],
    ])
    marker_xy = np.array([3.0, 4.0])
    measurement = expected_marker_measurement(state, marker_xy)
    measurement_noise = np.diag([0.1, 0.02])

    updated_state, updated_covariance = ekf_range_bearing_update(
        state,
        covariance,
        marker_xy,
        measurement,
        measurement_noise,
    )

    assert np.allclose(updated_state, state, atol=1e-12)
    assert np.trace(updated_covariance) < np.trace(covariance)
    assert np.all(np.linalg.eigvalsh(updated_covariance) >= -1e-12)


def test_ekf_offset_range_moves_robot_toward_marker():
    state = np.array([0.0, 0.0, 0.0])
    covariance = np.diag([0.5, 0.5, 0.2])
    marker_xy = np.array([3.0, 0.0])
    measurement = np.array([2.5, 0.0])
    measurement_noise = np.diag([0.05, 0.02])

    updated_state, _ = ekf_range_bearing_update(
        state,
        covariance,
        marker_xy,
        measurement,
        measurement_noise,
    )

    assert updated_state[0] > state[0]
    assert abs(updated_state[1]) < 1e-12
    assert abs(updated_state[2]) < 1e-12


def test_marker_map_filters_sentinel_and_confidence():
    marker_map = build_marker_map([-1, 703], [0.0, 1.2], [0.0, 2.3])

    assert -1 not in marker_map
    assert 703 in marker_map
    assert should_use_marker(703, 0.8, marker_map, 0.5)
    assert not should_use_marker(703, 0.2, marker_map, 0.5)
    assert not should_use_marker(42, 1.0, marker_map, 0.5)


def test_marker_map_applies_map_to_world_conversion():
    marker_map = build_marker_map([70], [184.0], [-30.0], scale=0.01)

    assert np.allclose(marker_map[70], [1.84, -0.30])


def test_marker_timestamp_rejects_zero_stale_and_future_data():
    class Stamp:
        sec = 0
        nanosec = 0

    stamp = Stamp()
    assert not marker_observation_is_fresh(stamp, 10_000_000_000, 0.5)

    stamp.sec = 9
    stamp.nanosec = 600_000_000
    assert marker_observation_is_fresh(stamp, 10_000_000_000, 0.5)

    stamp.sec = 9
    stamp.nanosec = 400_000_000
    assert not marker_observation_is_fresh(stamp, 10_000_000_000, 0.5)

    stamp.sec = 10
    stamp.nanosec = 200_000_000
    assert not marker_observation_is_fresh(stamp, 10_000_000_000, 0.5)


def test_marker_frame_requires_nonempty_base_footprint_frame():
    assert marker_frame_matches('base_footprint', 'base_footprint')
    assert marker_frame_matches('/base_footprint', 'base_footprint')
    assert not marker_frame_matches('', 'base_footprint')
    assert not marker_frame_matches('camera_link_optical', 'base_footprint')


def test_normalized_innovation_gate_rejects_large_outlier():
    state = np.array([0.0, 0.0, 0.0])
    covariance = np.diag([0.04, 0.04, 0.06854])
    marker_xy = np.array([2.0, 0.0])
    measurement_noise = np.diag([0.01, 0.02])
    measurement = np.array([0.2, math.pi / 2.0])

    _, _, _, nis = normalized_innovation_squared(
        state,
        covariance,
        marker_xy,
        measurement,
        measurement_noise,
    )

    assert nis > 9.21
    try:
        ekf_range_bearing_update(
            state,
            covariance,
            marker_xy,
            measurement,
            measurement_noise,
            innovation_gate=9.21,
        )
    except ValueError as exc:
        assert 'normalized innovation' in str(exc)
    else:
        raise AssertionError('Expected the innovation gate to reject the outlier')
