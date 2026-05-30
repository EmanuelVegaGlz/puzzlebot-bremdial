import math

import numpy as np
import rclpy
from aruco_msgs.msg import MarkerArray
from nav_msgs.msg import Odometry
from rclpy import qos
from rclpy.node import Node
from std_msgs.msg import Float32
import transforms3d


DEFAULT_PROCESS_NOISE = [
    0.000273, 0.00026, 0.00026,
    0.00026, 0.000273, 0.00026,
    0.00026, 0.00026, 0.001406,
]

MIN_LANDMARK_DISTANCE_SQ = 1e-9


def normalize_angle(angle):
    return float(math.atan2(math.sin(angle), math.cos(angle)))


def optical_translation_to_range_bearing(x, y, z):
    del y
    forward = float(z)
    left = -float(x)
    marker_range = math.hypot(forward, left)
    bearing = normalize_angle(math.atan2(left, forward))
    return np.array([marker_range, bearing], dtype=float)


def marker_pose_to_range_bearing(marker_pose):
    position = marker_pose.pose.position
    return optical_translation_to_range_bearing(
        position.x,
        position.y,
        position.z,
    )


def expected_marker_measurement(state, marker_xy):
    x, y, theta = state
    mx, my = marker_xy
    dx = float(mx) - float(x)
    dy = float(my) - float(y)
    distance_sq = dx * dx + dy * dy

    if distance_sq <= MIN_LANDMARK_DISTANCE_SQ:
        raise ValueError('Robot is too close to marker for a stable EKF update')

    marker_range = math.sqrt(distance_sq)
    bearing = normalize_angle(math.atan2(dy, dx) - theta)
    return np.array([marker_range, bearing], dtype=float)


def range_bearing_jacobian(state, marker_xy):
    x, y, _ = state
    mx, my = marker_xy
    dx = float(mx) - float(x)
    dy = float(my) - float(y)
    distance_sq = dx * dx + dy * dy

    if distance_sq <= MIN_LANDMARK_DISTANCE_SQ:
        raise ValueError('Robot is too close to marker for a stable EKF update')

    marker_range = math.sqrt(distance_sq)
    return np.array([
        [-dx / marker_range, -dy / marker_range, 0.0],
        [dy / distance_sq, -dx / distance_sq, -1.0],
    ], dtype=float)


def ekf_range_bearing_update(state, covariance, marker_xy, measurement, measurement_noise):
    state = np.asarray(state, dtype=float).reshape(3)
    covariance = np.asarray(covariance, dtype=float).reshape(3, 3)
    measurement = np.asarray(measurement, dtype=float).reshape(2)
    measurement_noise = np.asarray(measurement_noise, dtype=float).reshape(2, 2)

    z_hat = expected_marker_measurement(state, marker_xy)
    G = range_bearing_jacobian(state, marker_xy)
    residual = measurement - z_hat
    residual[1] = normalize_angle(residual[1])

    innovation_covariance = G @ covariance @ G.T + measurement_noise
    try:
        kalman_gain = np.linalg.solve(
            innovation_covariance.T,
            (covariance @ G.T).T,
        ).T
    except np.linalg.LinAlgError:
        kalman_gain = covariance @ G.T @ np.linalg.pinv(innovation_covariance)

    updated_state = state + kalman_gain @ residual
    updated_state[2] = normalize_angle(updated_state[2])

    identity = np.eye(3)
    correction = identity - kalman_gain @ G
    updated_covariance = (
        correction @ covariance @ correction.T
        + kalman_gain @ measurement_noise @ kalman_gain.T
    )
    updated_covariance = 0.5 * (updated_covariance + updated_covariance.T)
    return updated_state, updated_covariance


def build_marker_map(marker_ids, marker_x, marker_y):
    ids = [int(marker_id) for marker_id in marker_ids]
    xs = [float(x) for x in marker_x]
    ys = [float(y) for y in marker_y]

    if not (len(ids) == len(xs) == len(ys)):
        raise ValueError('marker_ids, marker_x, and marker_y must have equal length')

    return {
        marker_id: np.array([x, y], dtype=float)
        for marker_id, x, y in zip(ids, xs, ys)
        if marker_id >= 0
    }


def should_use_marker(marker_id, confidence, marker_map, min_confidence):
    return int(marker_id) in marker_map and float(confidence) >= float(min_confidence)


class localization(Node):

    def __init__(self):
        super().__init__('localization')

        self.declare_parameter('robot_frame_prefix', '')
        self.declare_parameter('marker_topic', '/marker_publisher/markers')
        self.declare_parameter('marker_ids', [-1])
        self.declare_parameter('marker_x', [0.0])
        self.declare_parameter('marker_y', [0.0])
        self.declare_parameter('min_confidence', 0.5)
        self.declare_parameter('measurement_range_variance', 0.01)
        self.declare_parameter('measurement_bearing_variance', 0.02)
        self.declare_parameter('process_noise', DEFAULT_PROCESS_NOISE)

        prefix = self.get_parameter('robot_frame_prefix').value
        ns = self.get_namespace().strip('/')
        fp = prefix if prefix else ns

        def frame(name):
            return f'{fp}/{name}' if fp else name

        marker_topic = self.get_parameter('marker_topic').value
        self.min_confidence = self.get_parameter('min_confidence').value
        self.marker_map = self._load_marker_map()
        self.process_noise = self._load_process_noise()
        self.measurement_noise = np.diag([
            float(self.get_parameter('measurement_range_variance').value),
            float(self.get_parameter('measurement_bearing_variance').value),
        ])

        self.wr_sub = self.create_subscription(
            Float32,
            'VelocityEncR',
            self.wr_callback,
            qos.qos_profile_sensor_data,
        )
        self.wl_sub = self.create_subscription(
            Float32,
            'VelocityEncL',
            self.wl_callback,
            qos.qos_profile_sensor_data,
        )
        self.marker_sub = self.create_subscription(
            MarkerArray,
            marker_topic,
            self.marker_callback,
            qos.qos_profile_sensor_data,
        )

        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)

        self.odom_frame = frame('world')
        self.base_link_frame = frame('base_link')

        self.r = 0.05
        self.L = 0.19

        self.wr = 0.0
        self.wl = 0.0
        self.x = 0.0
        self.y = 2.0
        self.theta = 0.0
        self.prev_time_ns = self.get_clock().now().nanoseconds

        self.P = np.zeros((3, 3), dtype=float)

        self.timer = self.create_timer(0.02, self.timer_callback)

        if self.marker_map:
            marker_ids = ', '.join(str(marker_id) for marker_id in self.marker_map)
            self.get_logger().info(f'ArUco EKF enabled for marker IDs: {marker_ids}')
        else:
            self.get_logger().warn(
                'No valid ArUco markers configured; localization will run as dead reckoning.'
            )
        self.get_logger().info('Localization node initialized')

    def _load_marker_map(self):
        try:
            return build_marker_map(
                self.get_parameter('marker_ids').value,
                self.get_parameter('marker_x').value,
                self.get_parameter('marker_y').value,
            )
        except ValueError as exc:
            self.get_logger().error(f'Invalid marker map parameters: {exc}')
            return {}

    def _load_process_noise(self):
        raw = list(self.get_parameter('process_noise').value)
        if len(raw) != 9:
            self.get_logger().error(
                'process_noise must contain 9 values; using default process noise.'
            )
            raw = DEFAULT_PROCESS_NOISE
        return np.array(raw, dtype=float).reshape(3, 3)

    @property
    def state(self):
        return np.array([self.x, self.y, self.theta], dtype=float)

    def set_state(self, state):
        self.x = float(state[0])
        self.y = float(state[1])
        self.theta = normalize_angle(float(state[2]))

    def timer_callback(self):
        now_ns = self.get_clock().now().nanoseconds
        dt = (now_ns - self.prev_time_ns) / 1e9
        self.prev_time_ns = now_ns

        if dt > 0.0:
            v, w = self.get_robot_vel(self.wr, self.wl)
            self.predict(v, w, dt)

        self.odom_pub.publish(self.fill_odom_message(self.x, self.y, self.theta))

    def wr_callback(self, msg):
        self.wr = msg.data

    def wl_callback(self, msg):
        self.wl = msg.data

    def marker_callback(self, msg):
        updates = 0
        for marker in msg.markers:
            if not should_use_marker(
                marker.id,
                marker.confidence,
                self.marker_map,
                self.min_confidence,
            ):
                continue

            marker_xy = self.marker_map[int(marker.id)]
            measurement = marker_pose_to_range_bearing(marker.pose)

            try:
                updated_state, updated_covariance = ekf_range_bearing_update(
                    self.state,
                    self.P,
                    marker_xy,
                    measurement,
                    self.measurement_noise,
                )
            except ValueError as exc:
                self.get_logger().debug(f'Skipping marker {marker.id}: {exc}')
                continue

            self.set_state(updated_state)
            self.P = updated_covariance
            updates += 1

        if updates:
            self.get_logger().debug(f'Applied {updates} ArUco EKF update(s)')

    def predict(self, v, w, dt):
        theta = self.theta
        self.x += v * math.cos(theta) * dt
        self.y += v * math.sin(theta) * dt
        self.theta = normalize_angle(theta + w * dt)

        motion_jacobian = np.array([
            [1.0, 0.0, -v * dt * math.sin(theta)],
            [0.0, 1.0, v * dt * math.cos(theta)],
            [0.0, 0.0, 1.0],
        ], dtype=float)

        self.P = motion_jacobian @ self.P @ motion_jacobian.T + self.process_noise
        self.P = 0.5 * (self.P + self.P.T)

    def get_robot_vel(self, wr, wl):
        v = self.r * (wr + wl) / 2.0
        w = self.r * (wr - wl) / self.L
        return v, w

    def fill_odom_message(self, x, y, yaw):
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_link_frame
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        quat = transforms3d.euler.euler2quat(0, 0, yaw)
        odom.pose.pose.orientation.w = quat[0]
        odom.pose.pose.orientation.x = quat[1]
        odom.pose.pose.orientation.y = quat[2]
        odom.pose.pose.orientation.z = quat[3]
        odom.pose.covariance = self._odom_pose_covariance()
        return odom

    def _odom_pose_covariance(self):
        covariance = [0.0] * 36
        covariance[0] = float(self.P[0, 0])
        covariance[1] = float(self.P[0, 1])
        covariance[5] = float(self.P[0, 2])
        covariance[6] = float(self.P[1, 0])
        covariance[7] = float(self.P[1, 1])
        covariance[11] = float(self.P[1, 2])
        covariance[30] = float(self.P[2, 0])
        covariance[31] = float(self.P[2, 1])
        covariance[35] = float(self.P[2, 2])
        return covariance


def main(args=None):
    rclpy.init(args=args)
    node = localization()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
