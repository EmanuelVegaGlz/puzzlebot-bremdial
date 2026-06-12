import math


def quaternion_from_yaw(yaw):
    half_yaw = float(yaw) * 0.5
    return (
        math.cos(half_yaw),
        0.0,
        0.0,
        math.sin(half_yaw),
    )


def yaw_from_quaternion_xyzw(x, y, z, w):
    siny_cosp = 2.0 * (float(w) * float(z) + float(x) * float(y))
    cosy_cosp = 1.0 - 2.0 * (float(y) * float(y) + float(z) * float(z))
    return math.atan2(siny_cosp, cosy_cosp)


def yaw_from_quaternion_wxyz(w, x, y, z):
    return yaw_from_quaternion_xyzw(x, y, z, w)
