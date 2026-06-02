import math

import numpy as np
import rclpy
from aruco_msgs.msg import MarkerArray as ArucoMarkerArray
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from rclpy import qos
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
import transforms3d
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray as VisualizationMarkerArray

from puzzlebot_sim.localization import (
    covariance_3x3_to_odom,
    odom_covariance_to_3x3,
    yaw_from_odom,
)


MIN_LANDMARK_DISTANCE_SQ = 1e-9
DEFAULT_MARKER_MEASUREMENT_FRAME = 'base_xy'
MARKER_MEASUREMENT_FRAMES = {
    'base': 'base_xy',
    'base_xy': 'base_xy',
    'robot': 'base_xy',
    'robot_xy': 'base_xy',
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


def map_xy_to_world_xy(map_x, map_y, scale=1.0, origin_x=0.0, origin_y=0.0, yaw=0.0):
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

        self.declare_parameter('odom_topic', 'odom')
        self.declare_parameter('correction_topic', 'aruco_ekf/odom_correction')
        self.declare_parameter('marker_topic', '/marker_publisher/markers')
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

        self.marker_measurement_frame = self._load_marker_measurement_frame()
        self.min_confidence = self.get_parameter('min_confidence').value
        self.marker_map = self._load_marker_map()
        self.measurement_noise = np.diag([
            float(self.get_parameter('measurement_range_variance').value),
            float(self.get_parameter('measurement_bearing_variance').value),
        ])

        self.state = np.zeros(3, dtype=float)
        self.P = np.zeros((3, 3), dtype=float)
        self.have_odom = False
        self.odom_frame = 'world'
        self.child_frame = 'base_link'

        self.odom_sub = self.create_subscription(
            Odometry,
            self.get_parameter('odom_topic').value,
            self.odom_callback,
            10,
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
        self.get_logger().info('ArUco EKF localization correction node initialized')

    def _load_marker_measurement_frame(self):
        value = self.get_parameter('marker_measurement_frame').value
        try:
            return normalize_marker_measurement_frame(value)
        except ValueError as exc:
            self.get_logger().error(f'{exc}; using {DEFAULT_MARKER_MEASUREMENT_FRAME}')
            return DEFAULT_MARKER_MEASUREMENT_FRAME

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
        self.state = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            yaw_from_odom(msg),
        ], dtype=float)
        self.P = odom_covariance_to_3x3(msg.pose.covariance)
        self.P = 0.5 * (self.P + self.P.T)
        self.odom_frame = msg.header.frame_id or self.odom_frame
        self.child_frame = msg.child_frame_id or self.child_frame
        self.have_odom = True

    def marker_callback(self, msg):
        detected_markers = self._delete_all_visualization_markers()
        if not self.have_odom:
            self.detected_marker_pub.publish(detected_markers)
            return

        corrected_state = np.array(self.state, dtype=float)
        corrected_covariance = np.array(self.P, dtype=float)
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
            measurement = marker_pose_to_range_bearing(
                marker.pose,
                self.marker_measurement_frame,
            )

            try:
                corrected_state, corrected_covariance = ekf_range_bearing_update(
                    corrected_state,
                    corrected_covariance,
                    marker_xy,
                    measurement,
                    self.measurement_noise,
                )
            except ValueError as exc:
                self.get_logger().debug(f'Skipping marker {marker.id}: {exc}')
                continue

            self._append_detected_marker_visualization(
                detected_markers,
                int(marker.id),
                updates,
                corrected_state,
                measurement,
            )
            updates += 1

        self.detected_marker_pub.publish(detected_markers)
        if updates:
            self.correction_pub.publish(
                self.fill_correction_message(corrected_state, corrected_covariance)
            )
            self.get_logger().debug(f'Published ArUco EKF correction from {updates} marker(s)')

    def fill_correction_message(self, state, covariance):
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.child_frame
        odom.pose.pose.position.x = float(state[0])
        odom.pose.pose.position.y = float(state[1])
        quat = transforms3d.euler.euler2quat(0, 0, float(state[2]))
        odom.pose.pose.orientation.w = quat[0]
        odom.pose.pose.orientation.x = quat[1]
        odom.pose.pose.orientation.y = quat[2]
        odom.pose.pose.orientation.z = quat[3]
        odom.pose.covariance = covariance_3x3_to_odom(covariance)
        return odom

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
        marker.header.frame_id = self.odom_frame
        marker.action = Marker.DELETEALL
        return VisualizationMarkerArray(markers=[marker])

    def _visualization_marker(self, namespace, marker_id, marker_type):
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = self.odom_frame
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
    rclpy.init(args=args)
    node = ArucoEkfLocalization()
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
