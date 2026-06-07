import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy import qos
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float32
from tf2_ros import TransformBroadcaster

from puzzlebot_sim.transform_utils import quaternion_from_yaw
from puzzlebot_sim.transform_utils import yaw_from_quaternion_wxyz


DEFAULT_INITIAL_COVARIANCE = [
    0.04, 0.0, 0.0,
    0.0, 0.04, 0.0,
    0.0, 0.0, 0.06854,
]


def normalize_angle(angle):
    return float(np.arctan2(np.sin(angle), np.cos(angle)))


def covariance_3x3_to_odom(covariance):
    covariance = np.asarray(covariance, dtype=float).reshape(3, 3)
    odom_covariance = [0.0] * 36
    odom_covariance[0] = float(covariance[0, 0])
    odom_covariance[1] = float(covariance[0, 1])
    odom_covariance[5] = float(covariance[0, 2])
    odom_covariance[6] = float(covariance[1, 0])
    odom_covariance[7] = float(covariance[1, 1])
    odom_covariance[11] = float(covariance[1, 2])
    odom_covariance[30] = float(covariance[2, 0])
    odom_covariance[31] = float(covariance[2, 1])
    odom_covariance[35] = float(covariance[2, 2])
    return odom_covariance


def odom_covariance_to_3x3(odom_covariance):
    return np.array([
        [odom_covariance[0], odom_covariance[1], odom_covariance[5]],
        [odom_covariance[6], odom_covariance[7], odom_covariance[11]],
        [odom_covariance[30], odom_covariance[31], odom_covariance[35]],
    ], dtype=float)


def yaw_from_odom(odom):
    orientation = odom.pose.pose.orientation
    yaw = yaw_from_quaternion_wxyz(
        orientation.w,
        orientation.x,
        orientation.y,
        orientation.z,
    )
    return normalize_angle(yaw)


def integrate_differential_drive(state, left_distance, right_distance, wheel_separation):
    state = np.asarray(state, dtype=float).reshape(3)
    center_distance = 0.5 * (float(right_distance) + float(left_distance))
    heading_delta = (
        float(right_distance) - float(left_distance)
    ) / float(wheel_separation)
    midpoint_heading = state[2] + 0.5 * heading_delta

    updated = np.array(state, dtype=float)
    updated[0] += center_distance * np.cos(midpoint_heading)
    updated[1] += center_distance * np.sin(midpoint_heading)
    updated[2] = normalize_angle(state[2] + heading_delta)
    return updated


def propagate_differential_drive_covariance(
    state,
    covariance,
    left_distance,
    right_distance,
    wheel_separation,
    wheel_distance_variance_density,
):
    state = np.asarray(state, dtype=float).reshape(3)
    covariance = np.asarray(covariance, dtype=float).reshape(3, 3)
    left_distance = float(left_distance)
    right_distance = float(right_distance)
    wheel_separation = float(wheel_separation)
    density = max(0.0, float(wheel_distance_variance_density))

    center_distance = 0.5 * (right_distance + left_distance)
    heading_delta = (right_distance - left_distance) / wheel_separation
    midpoint_heading = state[2] + 0.5 * heading_delta
    cos_heading = np.cos(midpoint_heading)
    sin_heading = np.sin(midpoint_heading)

    state_jacobian = np.array([
        [1.0, 0.0, -center_distance * sin_heading],
        [0.0, 1.0, center_distance * cos_heading],
        [0.0, 0.0, 1.0],
    ], dtype=float)

    wheel_jacobian = np.array([
        [
            0.5 * cos_heading + center_distance * sin_heading / (2.0 * wheel_separation),
            0.5 * cos_heading - center_distance * sin_heading / (2.0 * wheel_separation),
        ],
        [
            0.5 * sin_heading - center_distance * cos_heading / (2.0 * wheel_separation),
            0.5 * sin_heading + center_distance * cos_heading / (2.0 * wheel_separation),
        ],
        [-1.0 / wheel_separation, 1.0 / wheel_separation],
    ], dtype=float)
    wheel_covariance = np.diag([
        density * abs(left_distance),
        density * abs(right_distance),
    ])

    propagated = (
        state_jacobian @ covariance @ state_jacobian.T
        + wheel_jacobian @ wheel_covariance @ wheel_jacobian.T
    )
    return 0.5 * (propagated + propagated.T)


def parent_to_child_transform(parent_base_state, child_base_state):
    parent_base_state = np.asarray(parent_base_state, dtype=float).reshape(3)
    child_base_state = np.asarray(child_base_state, dtype=float).reshape(3)
    yaw = normalize_angle(parent_base_state[2] - child_base_state[2])
    cos_yaw = np.cos(yaw)
    sin_yaw = np.sin(yaw)
    rotated_child_translation = np.array([
        cos_yaw * child_base_state[0] - sin_yaw * child_base_state[1],
        sin_yaw * child_base_state[0] + cos_yaw * child_base_state[1],
    ])
    translation = parent_base_state[:2] - rotated_child_translation
    return np.array([translation[0], translation[1], yaw], dtype=float)


class localization(Node):

    def __init__(self):
        super().__init__('localization')

        self.declare_parameter('robot_frame_prefix', '')
        self.declare_parameter('world_frame', 'world_origin')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('raw_odom_topic', 'odom')
        self.declare_parameter('localization_topic', 'localization/odom')
        self.declare_parameter('correction_topic', 'aruco_ekf/odom_correction')
        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('initial_theta', 0.0)
        self.declare_parameter('initial_covariance', DEFAULT_INITIAL_COVARIANCE)
        self.declare_parameter('wheel_radius', 0.05)
        self.declare_parameter('wheel_separation', 0.19)
        self.declare_parameter('wheel_distance_variance_density', 0.0004)
        self.declare_parameter('encoder_timeout', 0.5)

        prefix = self.get_parameter('robot_frame_prefix').value
        ns = self.get_namespace().strip('/')
        frame_prefix = prefix if prefix else ns

        def frame(name):
            return f'{frame_prefix}/{name}' if frame_prefix else name

        self.world_frame = frame(self.get_parameter('world_frame').value)
        self.odom_frame = frame(self.get_parameter('odom_frame').value)
        self.base_frame = frame(self.get_parameter('base_frame').value)
        self.wheel_radius = float(self.get_parameter('wheel_radius').value)
        self.wheel_separation = float(self.get_parameter('wheel_separation').value)
        self.wheel_variance_density = float(
            self.get_parameter('wheel_distance_variance_density').value
        )
        self.encoder_timeout = float(self.get_parameter('encoder_timeout').value)

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
        self.correction_sub = self.create_subscription(
            Odometry,
            self.get_parameter('correction_topic').value,
            self.correction_callback,
            10,
        )

        self.odom_pub = self.create_publisher(
            Odometry,
            self.get_parameter('raw_odom_topic').value,
            10,
        )
        self.localization_pub = self.create_publisher(
            Odometry,
            self.get_parameter('localization_topic').value,
            10,
        )
        self.tf_br = TransformBroadcaster(self)

        self.wr = 0.0
        self.wl = 0.0
        self.wr_stamp_ns = None
        self.wl_stamp_ns = None
        self.local_state = np.zeros(3, dtype=float)
        self.global_state = np.array([
            float(self.get_parameter('initial_x').value),
            float(self.get_parameter('initial_y').value),
            normalize_angle(float(self.get_parameter('initial_theta').value)),
        ])
        self.local_covariance = np.zeros((3, 3), dtype=float)
        self.global_covariance = self._load_initial_covariance()
        self.prev_time_ns = self.get_clock().now().nanoseconds
        self.last_encoder_warning_ns = None

        self.timer = self.create_timer(0.02, self.timer_callback)
        self.get_logger().info(
            'Localization initialized with TF tree '
            f'{self.world_frame} -> {self.odom_frame} -> {self.base_frame}'
        )

    def _load_initial_covariance(self):
        raw = list(self.get_parameter('initial_covariance').value)
        if len(raw) != 9:
            self.get_logger().error(
                'initial_covariance must contain 9 values; using default covariance.'
            )
            raw = DEFAULT_INITIAL_COVARIANCE
        covariance = np.array(raw, dtype=float).reshape(3, 3)
        return 0.5 * (covariance + covariance.T)

    def timer_callback(self):
        now = self.get_clock().now()
        now_ns = now.nanoseconds
        dt = (now_ns - self.prev_time_ns) / 1e9
        self.prev_time_ns = now_ns

        right_velocity, left_velocity = self._fresh_wheel_velocities(now_ns)
        linear_velocity = 0.0
        angular_velocity = 0.0
        if dt > 0.0:
            left_distance = self.wheel_radius * left_velocity * dt
            right_distance = self.wheel_radius * right_velocity * dt
            linear_velocity = 0.5 * (right_distance + left_distance) / dt
            angular_velocity = (
                right_distance - left_distance
            ) / (self.wheel_separation * dt)
            self._integrate_increment(left_distance, right_distance)

        stamp = now.to_msg()
        raw_odom = self.fill_odom_message(
            self.local_state,
            self.local_covariance,
            self.odom_frame,
            linear_velocity,
            angular_velocity,
            stamp,
        )
        global_odom = self.fill_odom_message(
            self.global_state,
            self.global_covariance,
            self.world_frame,
            linear_velocity,
            angular_velocity,
            stamp,
        )
        self.odom_pub.publish(raw_odom)
        self.localization_pub.publish(global_odom)
        self.publish_transforms(stamp)

    def wr_callback(self, msg):
        self.wr = float(msg.data)
        self.wr_stamp_ns = self.get_clock().now().nanoseconds

    def wl_callback(self, msg):
        self.wl = float(msg.data)
        self.wl_stamp_ns = self.get_clock().now().nanoseconds

    def correction_callback(self, msg):
        if msg.header.frame_id and msg.header.frame_id != self.world_frame:
            self.get_logger().warn(
                f'Ignoring correction in {msg.header.frame_id}; '
                f'expected {self.world_frame}.',
                throttle_duration_sec=5.0,
            )
            return
        if msg.child_frame_id and msg.child_frame_id != self.base_frame:
            self.get_logger().warn(
                f'Ignoring correction for {msg.child_frame_id}; '
                f'expected {self.base_frame}.',
                throttle_duration_sec=5.0,
            )
            return

        self.global_state = np.array([
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
            yaw_from_odom(msg),
        ])
        self.global_covariance = odom_covariance_to_3x3(msg.pose.covariance)
        self.global_covariance = 0.5 * (
            self.global_covariance + self.global_covariance.T
        )
        self.get_logger().debug('Applied ArUco correction to global localization')

    def _fresh_wheel_velocities(self, now_ns):
        if self.wr_stamp_ns is None or self.wl_stamp_ns is None:
            self._warn_stale_encoders(now_ns, 'Waiting for both encoder topics.')
            return 0.0, 0.0
        if self.encoder_timeout <= 0.0:
            return self.wr, self.wl

        timeout_ns = int(self.encoder_timeout * 1e9)
        if (
            now_ns - self.wr_stamp_ns > timeout_ns
            or now_ns - self.wl_stamp_ns > timeout_ns
        ):
            self._warn_stale_encoders(
                now_ns,
                'Encoder data is stale; holding odometry stationary.',
            )
            return 0.0, 0.0
        return self.wr, self.wl

    def _warn_stale_encoders(self, now_ns, message):
        if (
            self.last_encoder_warning_ns is None
            or now_ns - self.last_encoder_warning_ns >= int(5e9)
        ):
            self.get_logger().warn(message)
            self.last_encoder_warning_ns = now_ns

    def _integrate_increment(self, left_distance, right_distance):
        self.local_covariance = propagate_differential_drive_covariance(
            self.local_state,
            self.local_covariance,
            left_distance,
            right_distance,
            self.wheel_separation,
            self.wheel_variance_density,
        )
        self.global_covariance = propagate_differential_drive_covariance(
            self.global_state,
            self.global_covariance,
            left_distance,
            right_distance,
            self.wheel_separation,
            self.wheel_variance_density,
        )
        self.local_state = integrate_differential_drive(
            self.local_state,
            left_distance,
            right_distance,
            self.wheel_separation,
        )
        self.global_state = integrate_differential_drive(
            self.global_state,
            left_distance,
            right_distance,
            self.wheel_separation,
        )

    def fill_odom_message(
        self,
        state,
        covariance,
        parent_frame,
        linear_velocity,
        angular_velocity,
        stamp,
    ):
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = parent_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = float(state[0])
        odom.pose.pose.position.y = float(state[1])
        quat = quaternion_from_yaw(state[2])
        odom.pose.pose.orientation.w = quat[0]
        odom.pose.pose.orientation.x = quat[1]
        odom.pose.pose.orientation.y = quat[2]
        odom.pose.pose.orientation.z = quat[3]
        odom.pose.covariance = covariance_3x3_to_odom(covariance)
        odom.twist.twist.linear.x = float(linear_velocity)
        odom.twist.twist.angular.z = float(angular_velocity)
        return odom

    def publish_transforms(self, stamp):
        odom_base = self._state_transform(
            self.odom_frame,
            self.base_frame,
            self.local_state,
            stamp,
        )
        world_odom_state = parent_to_child_transform(
            self.global_state,
            self.local_state,
        )
        world_odom = self._state_transform(
            self.world_frame,
            self.odom_frame,
            world_odom_state,
            stamp,
        )
        self.tf_br.sendTransform([world_odom, odom_base])

    def _state_transform(self, parent_frame, child_frame, state, stamp):
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = parent_frame
        transform.child_frame_id = child_frame
        transform.transform.translation.x = float(state[0])
        transform.transform.translation.y = float(state[1])
        transform.transform.translation.z = 0.0
        quat = quaternion_from_yaw(state[2])
        transform.transform.rotation.w = quat[0]
        transform.transform.rotation.x = quat[1]
        transform.transform.rotation.y = quat[2]
        transform.transform.rotation.z = quat[3]
        return transform


def main(args=None):
    rclpy.init(args=args)
    node = localization()
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
