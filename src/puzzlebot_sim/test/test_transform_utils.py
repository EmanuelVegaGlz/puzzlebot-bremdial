import math

from puzzlebot_sim.transform_utils import quaternion_from_yaw
from puzzlebot_sim.transform_utils import yaw_from_quaternion_wxyz
from puzzlebot_sim.transform_utils import yaw_from_quaternion_xyzw


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def test_quaternion_from_yaw_uses_wxyz_order():
    q = quaternion_from_yaw(math.pi / 2.0)

    assert math.isclose(q[0], math.sqrt(0.5))
    assert q[1] == 0.0
    assert q[2] == 0.0
    assert math.isclose(q[3], math.sqrt(0.5))


def test_yaw_round_trips_from_wxyz_and_xyzw_order():
    for yaw in [-math.pi, -1.3, 0.0, 2.4, math.pi]:
        w, x, y, z = quaternion_from_yaw(yaw)

        assert math.isclose(
            normalize_angle(yaw_from_quaternion_wxyz(w, x, y, z)),
            normalize_angle(yaw),
            abs_tol=1e-12,
        )
        assert math.isclose(
            normalize_angle(yaw_from_quaternion_xyzw(x, y, z, w)),
            normalize_angle(yaw),
            abs_tol=1e-12,
        )
