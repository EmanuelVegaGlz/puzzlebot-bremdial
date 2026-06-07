import math

import numpy as np

from puzzlebot_sim.localization import integrate_differential_drive
from puzzlebot_sim.localization import parent_to_child_transform
from puzzlebot_sim.localization import propagate_differential_drive_covariance


WHEEL_SEPARATION = 0.19
VARIANCE_DENSITY = 0.0004


def test_stationary_increment_does_not_change_pose_or_covariance():
    state = np.array([1.0, -2.0, 0.4])
    covariance = np.diag([0.04, 0.04, 0.06854])

    updated_state = integrate_differential_drive(
        state,
        0.0,
        0.0,
        WHEEL_SEPARATION,
    )
    updated_covariance = propagate_differential_drive_covariance(
        state,
        covariance,
        0.0,
        0.0,
        WHEEL_SEPARATION,
        VARIANCE_DENSITY,
    )

    assert np.array_equal(updated_state, state)
    assert np.array_equal(updated_covariance, covariance)


def test_forward_motion_increases_covariance_and_moves_along_heading():
    state = np.array([0.0, 0.0, 0.0])
    covariance = np.zeros((3, 3))

    updated_state = integrate_differential_drive(
        state,
        0.2,
        0.2,
        WHEEL_SEPARATION,
    )
    updated_covariance = propagate_differential_drive_covariance(
        state,
        covariance,
        0.2,
        0.2,
        WHEEL_SEPARATION,
        VARIANCE_DENSITY,
    )

    assert np.allclose(updated_state, [0.2, 0.0, 0.0])
    assert np.trace(updated_covariance) > 0.0
    assert np.allclose(updated_covariance, updated_covariance.T)
    assert np.all(np.linalg.eigvalsh(updated_covariance) >= -1e-12)


def test_turning_motion_changes_heading_and_keeps_covariance_valid():
    state = np.array([0.0, 0.0, 0.0])
    covariance = np.zeros((3, 3))

    updated_state = integrate_differential_drive(
        state,
        0.05,
        0.15,
        WHEEL_SEPARATION,
    )
    updated_covariance = propagate_differential_drive_covariance(
        state,
        covariance,
        0.05,
        0.15,
        WHEEL_SEPARATION,
        VARIANCE_DENSITY,
    )

    assert updated_state[0] > 0.0
    assert updated_state[1] > 0.0
    assert updated_state[2] > 0.0
    assert np.all(np.linalg.eigvalsh(updated_covariance) >= -1e-12)


def test_world_to_odom_transform_preserves_continuous_odom_base_pose():
    odom_base = np.array([1.2, -0.4, 0.3])
    world_base = np.array([2.0, 1.0, 0.8])

    world_odom = parent_to_child_transform(world_base, odom_base)
    cos_yaw = math.cos(world_odom[2])
    sin_yaw = math.sin(world_odom[2])
    reconstructed_world_base = np.array([
        world_odom[0] + cos_yaw * odom_base[0] - sin_yaw * odom_base[1],
        world_odom[1] + sin_yaw * odom_base[0] + cos_yaw * odom_base[1],
        world_odom[2] + odom_base[2],
    ])

    assert np.allclose(reconstructed_world_base, world_base)


def test_global_correction_changes_world_odom_not_odom_base():
    odom_base = np.array([0.7, 0.1, -0.2])
    original_world_base = np.array([1.0, 1.5, 0.1])
    corrected_world_base = np.array([1.2, 1.4, 0.15])

    original_world_odom = parent_to_child_transform(
        original_world_base,
        odom_base,
    )
    corrected_world_odom = parent_to_child_transform(
        corrected_world_base,
        odom_base,
    )

    assert np.array_equal(odom_base, np.array([0.7, 0.1, -0.2]))
    assert not np.allclose(original_world_odom, corrected_world_odom)
