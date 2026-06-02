import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy import qos
from rclpy.node import Node
from std_msgs.msg import Float32
import transforms3d


def normalize_angle(angle):
    return float(np.arctan2(np.sin(angle), np.cos(angle)))


def covariance_3x3_to_odom(covariance):
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
    _, _, yaw = transforms3d.euler.quat2euler([
        orientation.w,
        orientation.x,
        orientation.y,
        orientation.z,
    ])
    return normalize_angle(yaw)


class localization(Node):

    def __init__(self):
        super().__init__('localization')

        self.declare_parameter('robot_frame_prefix', '')
        self.declare_parameter('correction_topic', 'aruco_ekf/odom_correction')

        prefix = self.get_parameter('robot_frame_prefix').value
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

        self.odom_frame = frame('world')
        self.base_link_frame = frame('base_link')

        self.r = 0.05
        self.L = 0.19

        self.wr = 0.0
        self.wl = 0.0
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.prev_time_ns = self.get_clock().now().nanoseconds

        self.P = np.zeros((3, 3), dtype=float)
        self.process_noise = np.array([
            [0.000273, 0.00026, 0.00026],
            [0.00026, 0.000273, 0.00026],
            [0.00026, 0.00026, 0.001406],
        ], dtype=float)

        self.timer = self.create_timer(0.02, self.timer_callback)
        self.get_logger().info('Regular localization node initialized')

    def timer_callback(self):
        now_ns = self.get_clock().now().nanoseconds
        dt = (now_ns - self.prev_time_ns) / 1e9
        self.prev_time_ns = now_ns

        if dt > 0.0:
            v, w = self.get_robot_vel(self.wr, self.wl)
            self.update_pose(v, w, dt)
            self.update_covariance(v, dt)

        self.odom_pub.publish(self.fill_odom_message(self.x, self.y, self.theta))

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
        quat = transforms3d.euler.euler2quat(0, 0, yaw)
        odom.pose.pose.orientation.w = quat[0]
        odom.pose.pose.orientation.x = quat[1]
        odom.pose.pose.orientation.y = quat[2]
        odom.pose.pose.orientation.z = quat[3]
        odom.pose.covariance = covariance_3x3_to_odom(self.P)
        return odom


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
