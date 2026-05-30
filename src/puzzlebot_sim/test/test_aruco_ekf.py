import math

import numpy as np

from puzzlebot_sim.localization import (
    build_marker_map,
    ekf_range_bearing_update,
    expected_marker_measurement,
    normalize_angle,
    optical_translation_to_range_bearing,
    range_bearing_jacobian,
    should_use_marker,
)


def test_normalize_angle_wraps_to_pi_interval():
    assert math.isclose(normalize_angle(3.0 * math.pi), math.pi)
    assert math.isclose(normalize_angle(-3.0 * math.pi), -math.pi)


def test_optical_translation_uses_z_forward_and_negative_x_left():
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
