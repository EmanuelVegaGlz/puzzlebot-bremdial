import math
import sys
from collections import Counter, deque

import numpy as np
import rclpy
from aruco_msgs.msg import MarkerArray as ArucoMarkerArray
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from rclpy import qos
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray as VisualizationMarkerArray

from puzzlebot_sim.localization import (
    covariance_3x3_to_odom,
    odom_covariance_to_3x3,
    yaw_from_odom,
)
from puzzlebot_sim.transform_utils import quaternion_from_yaw


MIN_LANDMARK_DISTANCE_SQ = 1e-9
DEFAULT_MARKER_MEASUREMENT_FRAME = 'robot_xy'
DEFAULT_MARKER_TIMESTAMP_POLICY = 'soft'
MARKER_TIMESTAMP_POLICIES = {'soft', 'strict'}
MARKER_MEASUREMENT_FRAMES = {
    'base': 'robot_xy',
    'base_footprint': 'robot_xy',
    'base_link': 'robot_xy',
    'base_xy': 'robot_xy',
    'robot': 'robot_xy',
    'robot_xy': 'robot_xy',
    'optical': 'optical',
    'camera_optical': 'optical',
}


def normalize_angle(angle):
    return float(math.atan2(math.sin(angle), math.cos(angle)))


def optical_translation_to_range_bearing(x, y, z):
    del y
    forward = float(z)
    left = -float(x)
    marker_range = math.hypot(forward, left)
    bearing = normalize_angle(math.atan2(left, forward))
    return np.array([marker_range, bearing], dtype=float)


def robot_xy_translation_to_range_bearing(x, y, z):
    del z
    forward = float(x)
    left = float(y)
    marker_range = math.hypot(forward, left)
    bearing = normalize_angle(math.atan2(left, forward))
    return np.array([marker_range, bearing], dtype=float)


def normalize_marker_measurement_frame(measurement_frame):
    frame = str(measurement_frame).strip().lower()
    if frame not in MARKER_MEASUREMENT_FRAMES:
        valid = ', '.join(sorted(MARKER_MEASUREMENT_FRAMES))
        raise ValueError(
            f'Unsupported marker_measurement_frame {measurement_frame}; '
            f'use one of: {valid}'
        )
    return MARKER_MEASUREMENT_FRAMES[frame]


def normalize_frame_id(frame_id):
    return str(frame_id).strip().lstrip('/')


def marker_frame_matches(frame_id, expected_frame):
    normalized_frame = normalize_frame_id(frame_id)
    return bool(normalized_frame) and (
        normalized_frame == normalize_frame_id(expected_frame)
    )


def stamp_to_nanoseconds(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def normalize_marker_timestamp_policy(policy):
    normalized = str(policy).strip().lower()
    if normalized not in MARKER_TIMESTAMP_POLICIES:
        valid = ', '.join(sorted(MARKER_TIMESTAMP_POLICIES))
        raise ValueError(
            f'Unsupported marker_timestamp_policy {policy}; '
            f'use one of: {valid}'
        )
    return normalized


def resolve_marker_timestamp_ns(marker_stamp, array_stamp):
    marker_stamp_ns = stamp_to_nanoseconds(marker_stamp)
    if marker_stamp_ns > 0:
        return marker_stamp_ns, 'marker'

    array_stamp_ns = stamp_to_nanoseconds(array_stamp)
    if array_stamp_ns > 0:
        return array_stamp_ns, 'array'
    return None, 'missing'


def classify_marker_timestamp(
    stamp_ns,
    now_ns,
    max_age,
    future_tolerance=0.1,
):
    if stamp_ns is None or int(stamp_ns) <= 0:
        return 'missing', None

    age = (int(now_ns) - int(stamp_ns)) / 1e9
    if age < -max(0.0, float(future_tolerance)):
        return 'future', age
    if float(max_age) > 0.0 and age > float(max_age):
        return 'stale', age
    return 'valid', age


def marker_timestamp_action(policy, timestamp_status, can_deskew):
    normalized_policy = normalize_marker_timestamp_policy(policy)
    if timestamp_status != 'valid':
        return 'reject' if normalized_policy == 'strict' else 'arrival'
    return 'deskew' if can_deskew else 'arrival'


def marker_observation_is_fresh(stamp, now_ns, max_age, future_tolerance=0.1):
    status, _ = classify_marker_timestamp(
        stamp_to_nanoseconds(stamp),
        now_ns,
        max_age,
        future_tolerance,
    )
    return status == 'valid'


def interpolate_odom_state(history, target_ns):
    if not history:
        return None

    target_ns = int(target_ns)
    first_stamp, first_state = history[0]
    last_stamp, last_state = history[-1]
    if target_ns < int(first_stamp) or target_ns > int(last_stamp):
        return None
    if target_ns == int(first_stamp):
        return np.array(first_state, dtype=float)
    if target_ns == int(last_stamp):
        return np.array(last_state, dtype=float)

    for index in range(1, len(history)):
        upper_stamp, upper_state = history[index]
        if target_ns > int(upper_stamp):
            continue

        lower_stamp, lower_state = history[index - 1]
        span_ns = int(upper_stamp) - int(lower_stamp)
        if span_ns <= 0:
            return np.array(upper_state, dtype=float)

        fraction = (target_ns - int(lower_stamp)) / span_ns
        lower_state = np.asarray(lower_state, dtype=float).reshape(3)
        upper_state = np.asarray(upper_state, dtype=float).reshape(3)
        interpolated = lower_state + fraction * (upper_state - lower_state)
        yaw_delta = normalize_angle(upper_state[2] - lower_state[2])
        interpolated[2] = normalize_angle(
            lower_state[2] + fraction * yaw_delta
        )
        return interpolated
    return None


def deskew_range_bearing(
    measurement,
    observation_odom_state,
    target_odom_state,
):
    marker_range, bearing = np.asarray(measurement, dtype=float).reshape(2)
    observation_odom_state = np.asarray(
        observation_odom_state,
        dtype=float,
    ).reshape(3)
    target_odom_state = np.asarray(target_odom_state, dtype=float).reshape(3)

    marker_in_observation = np.array([
        marker_range * math.cos(bearing),
        marker_range * math.sin(bearing),
    ])
    observation_yaw = observation_odom_state[2]
    marker_in_odom = observation_odom_state[:2] + np.array([
        math.cos(observation_yaw) * marker_in_observation[0]
        - math.sin(observation_yaw) * marker_in_observation[1],
        math.sin(observation_yaw) * marker_in_observation[0]
        + math.cos(observation_yaw) * marker_in_observation[1],
    ])

    target_delta = marker_in_odom - target_odom_state[:2]
    target_yaw = target_odom_state[2]
    marker_in_target = np.array([
        math.cos(target_yaw) * target_delta[0]
        + math.sin(target_yaw) * target_delta[1],
        -math.sin(target_yaw) * target_delta[0]
        + math.cos(target_yaw) * target_delta[1],
    ])
    return np.array([
        math.hypot(marker_in_target[0], marker_in_target[1]),
        normalize_angle(math.atan2(marker_in_target[1], marker_in_target[0])),
    ])


def map_xy_to_world_xy(
    map_x,
    map_y,
    scale=1.0,
    origin_x=0.0,
    origin_y=0.0,
    yaw=0.0,
):
    scaled_x = float(map_x) * float(scale)
    scaled_y = float(map_y) * float(scale)
    cos_yaw = math.cos(float(yaw))
    sin_yaw = math.sin(float(yaw))

    world_x = float(origin_x) + cos_yaw * scaled_x - sin_yaw * scaled_y
    world_y = float(origin_y) + sin_yaw * scaled_x + cos_yaw * scaled_y
    return np.array([world_x, world_y], dtype=float)


def marker_pose_to_range_bearing(
    marker_pose,
    measurement_frame=DEFAULT_MARKER_MEASUREMENT_FRAME,
):
    position = marker_pose.pose.position
    frame = normalize_marker_measurement_frame(measurement_frame)
    if frame == 'optical':
        return optical_translation_to_range_bearing(
            position.x,
            position.y,
            position.z,
        )
    return robot_xy_translation_to_range_bearing(
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


def normalized_innovation_squared(
    state,
    covariance,
    marker_xy,
    measurement,
    measurement_noise,
):
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
        solved_residual = np.linalg.solve(innovation_covariance, residual)
    except np.linalg.LinAlgError:
        solved_residual = np.linalg.pinv(innovation_covariance) @ residual
    nis = float(residual.T @ solved_residual)
    return residual, G, innovation_covariance, nis


def ekf_range_bearing_update(
    state,
    covariance,
    marker_xy,
    measurement,
    measurement_noise,
    innovation_gate=None,
):
    state = np.asarray(state, dtype=float).reshape(3)
    covariance = np.asarray(covariance, dtype=float).reshape(3, 3)
    measurement_noise = np.asarray(measurement_noise, dtype=float).reshape(2, 2)

    residual, G, innovation_covariance, nis = normalized_innovation_squared(
        state,
        covariance,
        marker_xy,
        measurement,
        measurement_noise,
    )
    if innovation_gate is not None and nis > float(innovation_gate):
        raise ValueError(
            f'normalized innovation {nis:.3f} exceeds gate {innovation_gate}'
        )

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


def build_marker_map(
    marker_ids,
    marker_x,
    marker_y,
    scale=1.0,
    origin_x=0.0,
    origin_y=0.0,
    yaw=0.0,
):
    ids = [int(marker_id) for marker_id in marker_ids]
    xs = [float(x) for x in marker_x]
    ys = [float(y) for y in marker_y]

    if not (len(ids) == len(xs) == len(ys)):
        raise ValueError('marker_ids, marker_x, and marker_y must have equal length')

    return {
        marker_id: map_xy_to_world_xy(x, y, scale, origin_x, origin_y, yaw)
        for marker_id, x, y in zip(ids, xs, ys)
        if marker_id >= 0
    }


def should_use_marker(marker_id, confidence, marker_map, min_confidence):
    return int(marker_id) in marker_map and float(confidence) >= float(min_confidence)


class ArucoEkfLocalization(Node):

    def __init__(self):
        super().__init__('aruco_ekf_localization')

        self.declare_parameter('odom_topic', 'localization/odom')
        self.declare_parameter('raw_odom_topic', 'odom')
        self.declare_parameter('correction_topic', 'aruco_ekf/odom_correction')
        self.declare_parameter('marker_topic', '/marker_publisher/markers')
        self.declare_parameter('world_frame', 'world_origin')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('map_marker_visualization_topic', 'aruco_ekf/map_markers')
        self.declare_parameter(
            'detected_marker_visualization_topic',
            'aruco_ekf/detected_markers',
        )
        self.declare_parameter('marker_measurement_frame', DEFAULT_MARKER_MEASUREMENT_FRAME)
        self.declare_parameter('marker_ids', [-1])
        self.declare_parameter('marker_x', [0.0])
        self.declare_parameter('marker_y', [0.0])
        self.declare_parameter('marker_map_scale', 1.0)
        self.declare_parameter('marker_map_origin_x', 0.0)
        self.declare_parameter('marker_map_origin_y', 0.0)
        self.declare_parameter('marker_map_yaw', 0.0)
        self.declare_parameter('min_confidence', 0.5)
        self.declare_parameter('measurement_range_variance', 0.01)
        self.declare_parameter('measurement_bearing_variance', 0.02)
        self.declare_parameter(
            'marker_timestamp_policy',
            DEFAULT_MARKER_TIMESTAMP_POLICY,
        )
        self.declare_parameter('marker_max_age', 0.5)
        self.declare_parameter('marker_future_tolerance', 0.1)
        self.declare_parameter('marker_odom_history_duration', 2.0)
        self.declare_parameter('innovation_gate', 9.21)

        self.marker_measurement_frame = self._load_marker_measurement_frame()
        self.marker_timestamp_policy = self._load_marker_timestamp_policy()
        self.min_confidence = self.get_parameter('min_confidence').value
        self.marker_max_age = float(self.get_parameter('marker_max_age').value)
        self.marker_future_tolerance = max(
            0.0,
            float(self.get_parameter('marker_future_tolerance').value),
        )
        self.marker_odom_history_duration = max(
            0.0,
            float(self.get_parameter('marker_odom_history_duration').value),
        )
        self.innovation_gate = float(self.get_parameter('innovation_gate').value)
        self.marker_map = self._load_marker_map()
        self.measurement_noise = np.diag([
            float(self.get_parameter('measurement_range_variance').value),
            float(self.get_parameter('measurement_bearing_variance').value),
        ])

        self.state = np.zeros(3, dtype=float)
        self.P = np.zeros((3, 3), dtype=float)
        self.have_odom = False
        self.state_stamp_ns = None
        self.world_frame = self.get_parameter('world_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.child_frame = self.get_parameter('base_frame').value
        self.raw_odom_history = deque()
        self.last_warning_ns = {}

        self.odom_sub = self.create_subscription(
            Odometry,
            self.get_parameter('odom_topic').value,
            self.odom_callback,
            10,
        )
        self.raw_odom_sub = self.create_subscription(
            Odometry,
            self.get_parameter('raw_odom_topic').value,
            self.raw_odom_callback,
            100,
        )
        self.marker_sub = self.create_subscription(
            ArucoMarkerArray,
            self.get_parameter('marker_topic').value,
            self.marker_callback,
            qos.qos_profile_sensor_data,
        )
        self.correction_pub = self.create_publisher(
            Odometry,
            self.get_parameter('correction_topic').value,
            10,
        )

        map_marker_qos = QoSProfile(depth=1)
        map_marker_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.map_marker_pub = self.create_publisher(
            VisualizationMarkerArray,
            self.get_parameter('map_marker_visualization_topic').value,
            map_marker_qos,
        )
        self.detected_marker_pub = self.create_publisher(
            VisualizationMarkerArray,
            self.get_parameter('detected_marker_visualization_topic').value,
            10,
        )

        self.publish_map_markers()
        self.get_logger().info(
            f'ArUco EKF correction node initialized in frame '
            f'{self.world_frame} '
            f'with {self.marker_timestamp_policy} timestamp handling'
        )

    def _load_marker_measurement_frame(self):
        value = self.get_parameter('marker_measurement_frame').value
        try:
            return normalize_marker_measurement_frame(value)
        except ValueError as exc:
            self.get_logger().error(f'{exc}; using {DEFAULT_MARKER_MEASUREMENT_FRAME}')
            return DEFAULT_MARKER_MEASUREMENT_FRAME

    def _load_marker_timestamp_policy(self):
        value = self.get_parameter('marker_timestamp_policy').value
        try:
            return normalize_marker_timestamp_policy(value)
        except ValueError as exc:
            self.get_logger().error(
                f'{exc}; using {DEFAULT_MARKER_TIMESTAMP_POLICY}'
            )
            return DEFAULT_MARKER_TIMESTAMP_POLICY

    def _load_marker_map(self):
        try:
            return build_marker_map(
                self.get_parameter('marker_ids').value,
                self.get_parameter('marker_x').value,
                self.get_parameter('marker_y').value,
                self.get_parameter('marker_map_scale').value,
                self.get_parameter('marker_map_origin_x').value,
                self.get_parameter('marker_map_origin_y').value,
                self.get_parameter('marker_map_yaw').value,
            )
        except ValueError as exc:
            self.get_logger().error(f'Invalid marker map parameters: {exc}')
            return {}

    def odom_callback(self, msg):
        if (
            normalize_frame_id(msg.header.frame_id)
            != normalize_frame_id(self.world_frame)
        ):
            self._warn_throttled(
                'odom_frame',
                f'Ignoring localization odometry in {msg.header.frame_id!r}; '
                f'expected {self.world_frame!r}.',
            )
            return
        if (
            normalize_frame_id(msg.child_frame_id)
            != normalize_frame_id(self.child_frame)
        ):
            self._warn_throttled(
                'odom_child',
                f'Ignoring localization odometry for {msg.child_frame_id!r}; '
                f'expected {self.child_frame!r}.',
            )
            return

        self.state = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            yaw_from_odom(msg),
        ], dtype=float)
        self.P = odom_covariance_to_3x3(msg.pose.covariance)
        self.P = 0.5 * (self.P + self.P.T)
        stamp_ns = stamp_to_nanoseconds(msg.header.stamp)
        self.state_stamp_ns = (
            stamp_ns if stamp_ns > 0 else self.get_clock().now().nanoseconds
        )
        self.have_odom = True

    def raw_odom_callback(self, msg):
        if (
            normalize_frame_id(msg.header.frame_id)
            != normalize_frame_id(self.odom_frame)
        ):
            self._warn_throttled(
                'raw_odom_frame',
                f'Ignoring raw odometry in {msg.header.frame_id!r}; '
                f'expected {self.odom_frame!r}.',
            )
            return
        if (
            normalize_frame_id(msg.child_frame_id)
            != normalize_frame_id(self.child_frame)
        ):
            self._warn_throttled(
                'raw_odom_child',
                f'Ignoring raw odometry for {msg.child_frame_id!r}; '
                f'expected {self.child_frame!r}.',
            )
            return

        stamp_ns = stamp_to_nanoseconds(msg.header.stamp)
        if stamp_ns <= 0:
            self._warn_throttled(
                'raw_odom_stamp',
                'Ignoring raw odometry with a missing timestamp.',
            )
            return

        state = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            yaw_from_odom(msg),
        ], dtype=float)
        self._append_raw_odom_state(stamp_ns, state)

    def _append_raw_odom_state(self, stamp_ns, state):
        stamp_ns = int(stamp_ns)
        if self.raw_odom_history and stamp_ns <= self.raw_odom_history[-1][0]:
            if stamp_ns == self.raw_odom_history[-1][0]:
                self.raw_odom_history[-1] = (stamp_ns, state)
            return

        self.raw_odom_history.append((stamp_ns, np.array(state, dtype=float)))
        history_ns = int(self.marker_odom_history_duration * 1e9)
        cutoff_ns = stamp_ns - history_ns
        while (
            len(self.raw_odom_history) > 1
            and self.raw_odom_history[1][0] < cutoff_ns
        ):
            self.raw_odom_history.popleft()

    def marker_callback(self, msg):
        detected_markers = self._delete_all_visualization_markers()
        if not self.have_odom:
            self.detected_marker_pub.publish(detected_markers)
            return

        corrected_state = np.array(self.state, dtype=float)
        corrected_covariance = np.array(self.P, dtype=float)
        updates = 0
        now_ns = self.get_clock().now().nanoseconds
        rejection_counts = Counter()

        for marker in msg.markers:
            if not self._marker_has_expected_frame(marker):
                rejection_counts['frame'] += 1
                continue
            marker_id = int(marker.id)
            if marker_id not in self.marker_map:
                rejection_counts['map_id'] += 1
                continue
            if float(marker.confidence) < float(self.min_confidence):
                rejection_counts['confidence'] += 1
                continue

            marker_xy = self.marker_map[marker_id]
            measurement = marker_pose_to_range_bearing(
                marker.pose,
                self.marker_measurement_frame,
            )
            measurement, timing_action = self._prepare_marker_measurement(
                marker,
                msg.header.stamp,
                measurement,
                now_ns,
            )
            if measurement is None:
                rejection_counts['timestamp'] += 1
                continue

            try:
                corrected_state, corrected_covariance = ekf_range_bearing_update(
                    corrected_state,
                    corrected_covariance,
                    marker_xy,
                    measurement,
                    self.measurement_noise,
                    innovation_gate=self.innovation_gate,
                )
            except ValueError as exc:
                self.get_logger().debug(f'Skipping marker {marker.id}: {exc}')
                reason = (
                    'innovation'
                    if 'normalized innovation' in str(exc)
                    else 'measurement'
                )
                rejection_counts[reason] += 1
                continue

            self._append_detected_marker_visualization(
                detected_markers,
                marker_id,
                updates,
                corrected_state,
                measurement,
            )
            updates += 1
            self.get_logger().debug(
                f'Accepted marker {marker_id} using {timing_action} timing'
            )

        self.detected_marker_pub.publish(detected_markers)
        if updates:
            self.correction_pub.publish(
                self.fill_correction_message(corrected_state, corrected_covariance)
            )
            self.get_logger().debug(
                f'Published ArUco EKF correction from {updates} marker(s)'
            )
        elif msg.markers:
            summary = ', '.join(
                f'{reason}={count}'
                for reason, count in sorted(rejection_counts.items())
            )
            self._warn_throttled(
                'marker_no_correction',
                'Received marker observations but published no correction'
                + (f': {summary}.' if summary else '.'),
            )

    def _prepare_marker_measurement(
        self,
        marker,
        array_stamp,
        measurement,
        now_ns,
    ):
        stamp_ns, stamp_source = resolve_marker_timestamp_ns(
            marker.header.stamp,
            array_stamp,
        )
        timestamp_status, age = classify_marker_timestamp(
            stamp_ns,
            now_ns,
            self.marker_max_age,
            self.marker_future_tolerance,
        )

        observation_odom = None
        target_odom = None
        if timestamp_status == 'valid' and self.state_stamp_ns is not None:
            observation_odom = interpolate_odom_state(
                self.raw_odom_history,
                stamp_ns,
            )
            target_odom = interpolate_odom_state(
                self.raw_odom_history,
                self.state_stamp_ns,
            )
        can_deskew = observation_odom is not None and target_odom is not None
        action = marker_timestamp_action(
            self.marker_timestamp_policy,
            timestamp_status,
            can_deskew,
        )

        if action == 'reject':
            self._warn_marker_timestamp(
                marker.id,
                timestamp_status,
                age,
                stamp_source,
                'rejected',
            )
            return None, action

        if action == 'deskew':
            age_text = f'{age:+.3f} s' if age is not None else 'unknown'
            self.get_logger().debug(
                f'Marker {int(marker.id)} timestamp is valid '
                f'(age {age_text}, source {stamp_source}); deskewed.'
            )
            return (
                deskew_range_bearing(
                    measurement,
                    observation_odom,
                    target_odom,
                ),
                action,
            )

        fallback_reason = (
            timestamp_status
            if timestamp_status != 'valid'
            else 'out_of_history'
        )
        self._warn_marker_timestamp(
            marker.id,
            fallback_reason,
            age,
            stamp_source,
            'used at arrival time',
        )
        return measurement, action

    def _warn_marker_timestamp(
        self,
        marker_id,
        reason,
        age,
        stamp_source,
        action,
    ):
        age_text = 'unknown'
        if age is not None:
            age_text = f'{age:+.3f} s'
        self._warn_throttled(
            f'marker_timestamp_{reason}_{action}',
            f'Marker {int(marker_id)} timestamp is {reason} '
            f'(age {age_text}, source {stamp_source}); {action}.',
        )

    def fill_correction_message(self, state, covariance):
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = self.world_frame
        odom.child_frame_id = self.child_frame
        odom.pose.pose.position.x = float(state[0])
        odom.pose.pose.position.y = float(state[1])
        quat = quaternion_from_yaw(float(state[2]))
        odom.pose.pose.orientation.w = quat[0]
        odom.pose.pose.orientation.x = quat[1]
        odom.pose.pose.orientation.y = quat[2]
        odom.pose.pose.orientation.z = quat[3]
        odom.pose.covariance = covariance_3x3_to_odom(covariance)
        return odom

    def _marker_has_expected_frame(self, marker):
        if not marker_frame_matches(marker.header.frame_id, self.child_frame):
            self._warn_throttled(
                'marker_frame',
                f'Ignoring marker observations in {marker.header.frame_id!r}; '
                f'expected {self.child_frame!r}.',
            )
            return False
        return True

    def _warn_throttled(self, key, message):
        now_ns = self.get_clock().now().nanoseconds
        last_ns = self.last_warning_ns.get(key)
        if last_ns is None or now_ns - last_ns >= int(5e9):
            self.get_logger().warn(message)
            self.last_warning_ns[key] = now_ns

    def publish_map_markers(self):
        marker_array = self._delete_all_visualization_markers()
        for index, (marker_id, marker_xy) in enumerate(self.marker_map.items()):
            marker = self._visualization_marker('aruco_map', index, Marker.CYLINDER)
            marker.pose.position.x = float(marker_xy[0])
            marker.pose.position.y = float(marker_xy[1])
            marker.pose.position.z = 0.035
            marker.scale.x = 0.14
            marker.scale.y = 0.14
            marker.scale.z = 0.07
            marker.color.r = 0.0
            marker.color.g = 0.8
            marker.color.b = 1.0
            marker.color.a = 0.85
            marker_array.markers.append(marker)

            label = self._visualization_marker(
                'aruco_map_labels',
                index,
                Marker.TEXT_VIEW_FACING,
            )
            label.pose.position.x = float(marker_xy[0])
            label.pose.position.y = float(marker_xy[1])
            label.pose.position.z = 0.24
            label.scale.z = 0.16
            label.color.r = 0.7
            label.color.g = 1.0
            label.color.b = 1.0
            label.color.a = 1.0
            label.text = f'id {marker_id}'
            marker_array.markers.append(label)

        self.map_marker_pub.publish(marker_array)

    def _delete_all_visualization_markers(self):
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = self.world_frame
        marker.action = Marker.DELETEALL
        return VisualizationMarkerArray(markers=[marker])

    def _visualization_marker(self, namespace, marker_id, marker_type):
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = self.world_frame
        marker.ns = namespace
        marker.id = int(marker_id)
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        return marker

    def _append_detected_marker_visualization(
        self,
        marker_array,
        marker_id,
        index,
        state,
        measurement,
    ):
        marker_range, bearing = measurement
        marker_angle = state[2] + bearing
        marker_x = state[0] + marker_range * math.cos(marker_angle)
        marker_y = state[1] + marker_range * math.sin(marker_angle)

        detected = self._visualization_marker('aruco_detected', index, Marker.SPHERE)
        detected.pose.position.x = float(marker_x)
        detected.pose.position.y = float(marker_y)
        detected.pose.position.z = 0.13
        detected.scale.x = 0.18
        detected.scale.y = 0.18
        detected.scale.z = 0.18
        detected.color.r = 1.0
        detected.color.g = 0.45
        detected.color.b = 0.0
        detected.color.a = 0.95
        detected.lifetime.sec = 1
        marker_array.markers.append(detected)

        ray = self._visualization_marker('aruco_detection_rays', index, Marker.LINE_LIST)
        ray.scale.x = 0.025
        ray.color.r = 1.0
        ray.color.g = 0.85
        ray.color.b = 0.0
        ray.color.a = 0.85
        ray.points = [
            Point(x=float(state[0]), y=float(state[1]), z=0.08),
            Point(x=float(marker_x), y=float(marker_y), z=0.08),
        ]
        ray.lifetime.sec = 1
        marker_array.markers.append(ray)

        label = self._visualization_marker(
            'aruco_detected_labels',
            index,
            Marker.TEXT_VIEW_FACING,
        )
        label.pose.position.x = float(marker_x)
        label.pose.position.y = float(marker_y)
        label.pose.position.z = 0.36
        label.scale.z = 0.16
        label.color.r = 1.0
        label.color.g = 0.8
        label.color.b = 0.35
        label.color.a = 1.0
        label.text = f'seen {marker_id}'
        label.lifetime.sec = 1
        marker_array.markers.append(label)


def main(args=None):
    cli_args = sys.argv[1:] if args is None else list(args)
    if '--help' in cli_args:
        print(
            'usage: ros2 run puzzlebot_sim aruco_ekf_localization '
            '[--ros-args --params-file PATH]'
        )
        print('Publishes ArUco EKF corrections on aruco_ekf/odom_correction.')
        return

    rclpy.init(args=args)
    node = ArucoEkfLocalization()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
