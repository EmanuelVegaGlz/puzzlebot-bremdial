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


DEFAULT_PROCESS_NOISE = [
    0.000273, 0.00026, 0.00026,
    0.00026, 0.000273, 0.00026,
    0.00026, 0.00026, 0.001406,
]

DEFAULT_INITIAL_COVARIANCE = [
    0.0, 0.0, 0.0,
    0.0, 0.0, 0.0,
    0.0, 0.0, 0.0,
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


class localization(Node):

    def __init__(self):
        super().__init__('localization')

        self.declare_parameter('robot_frame_prefix', '')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('correction_topic', 'aruco_ekf/odom_correction')
        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('initial_theta', 0.0)
        self.declare_parameter('initial_covariance', DEFAULT_INITIAL_COVARIANCE)
        self.declare_parameter('process_noise', DEFAULT_PROCESS_NOISE)

        prefix = self.get_parameter('robot_frame_prefix').value
        odom_frame = self.get_parameter('odom_frame').value
        base_frame = self.get_parameter('base_frame').value
        ns = self.get_namespace().strip('/')
        fp = prefix if prefix else ns

        def frame(name):
            return f'{fp}/{name}' if fp else name

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

        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_br = TransformBroadcaster(self)

        self.odom_frame = frame(odom_frame)
        self.base_link_frame = frame(base_frame)

        self.r = 0.05
        self.L = 0.19

        self.wr = 0.0
        self.wl = 0.0
        self.x = float(self.get_parameter('initial_x').value)
        self.y = float(self.get_parameter('initial_y').value)
        self.theta = normalize_angle(float(self.get_parameter('initial_theta').value))
        self.prev_time_ns = self.get_clock().now().nanoseconds

        self.P = self._load_initial_covariance()
        self.process_noise = self._load_process_noise()

        self.timer = self.create_timer(0.02, self.timer_callback)
        self.get_logger().info(
            f'Localization node initialized: {self.odom_frame} -> {self.base_link_frame}'
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

    def _load_process_noise(self):
        raw = list(self.get_parameter('process_noise').value)
        if len(raw) != 9:
            self.get_logger().error(
                'process_noise must contain 9 values; using default process noise.'
            )
            raw = DEFAULT_PROCESS_NOISE
        covariance = np.array(raw, dtype=float).reshape(3, 3)
        return 0.5 * (covariance + covariance.T)

    def timer_callback(self):
        now_ns = self.get_clock().now().nanoseconds
        dt = (now_ns - self.prev_time_ns) / 1e9
        self.prev_time_ns = now_ns

        if dt > 0.0:
            v, w = self.get_robot_vel(self.wr, self.wl)
            self.update_pose(v, w, dt)
            self.update_covariance(v, dt)

        odom_msg = self.fill_odom_message(self.x, self.y, self.theta)
        self.odom_pub.publish(odom_msg)
        self.publish_odom_tf(self.x, self.y, self.theta, odom_msg.header.stamp)

    def wr_callback(self, msg):
        self.wr = msg.data

    def wl_callback(self, msg):
        self.wl = msg.data

    def correction_callback(self, msg):
        self.x = float(msg.pose.pose.position.x)
        self.y = float(msg.pose.pose.position.y)
        self.theta = yaw_from_odom(msg)
        self.P = odom_covariance_to_3x3(msg.pose.covariance)
        self.P = 0.5 * (self.P + self.P.T)
        self.get_logger().debug('Applied ArUco EKF odometry correction')

    def update_covariance(self, v, dt):
        motion_jacobian = np.array([
            [1.0, 0.0, -v * dt * np.sin(self.theta)],
            [0.0, 1.0, v * dt * np.cos(self.theta)],
            [0.0, 0.0, 1.0],
        ], dtype=float)

        self.P = motion_jacobian @ self.P @ motion_jacobian.T + self.process_noise
        self.P = 0.5 * (self.P + self.P.T)

    def get_robot_vel(self, wr, wl):
        v = self.r * (wr + wl) / 2.0
        w = self.r * (wr - wl) / self.L
        return v, w

    def update_pose(self, v, w, dt):
        self.x += v * np.cos(self.theta) * dt
        self.y += v * np.sin(self.theta) * dt
        self.theta = normalize_angle(self.theta + w * dt)

    def fill_odom_message(self, x, y, yaw):
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_link_frame
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        quat = quaternion_from_yaw(yaw)
        odom.pose.pose.orientation.w = quat[0]
        odom.pose.pose.orientation.x = quat[1]
        odom.pose.pose.orientation.y = quat[2]
        odom.pose.pose.orientation.z = quat[3]
        odom.pose.covariance = covariance_3x3_to_odom(self.P)
        return odom

    def publish_odom_tf(self, x, y, yaw, stamp):
        tf_msg = TransformStamped()
        tf_msg.header.stamp = stamp
        tf_msg.header.frame_id = self.odom_frame
        tf_msg.child_frame_id = self.base_link_frame
        tf_msg.transform.translation.x = x
        tf_msg.transform.translation.y = y
        tf_msg.transform.translation.z = 0.0
        quat = quaternion_from_yaw(yaw)
        tf_msg.transform.rotation.w = quat[0]
        tf_msg.transform.rotation.x = quat[1]
        tf_msg.transform.rotation.y = quat[2]
        tf_msg.transform.rotation.z = quat[3]
        self.tf_br.sendTransform(tf_msg)


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
