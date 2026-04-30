import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from rclpy import qos
import transforms3d
import numpy as np

class localization(Node):

    def __init__(self):
        super().__init__('localization')

        self.declare_parameter('robot_frame_prefix', '')
        prefix = self.get_parameter('robot_frame_prefix').value
        ns = self.get_namespace().strip('/')
        fp = prefix if prefix else ns

        def frame(name):
            return f'{fp}/{name}' if fp else name

        # Subscribers 
        self.wr_sub = self.create_subscription(Float32, 'wr', self.wr_callback, qos.qos_profile_sensor_data)
        self.wl_sub = self.create_subscription(Float32, 'wl', self.wl_callback, qos.qos_profile_sensor_data)

        # Publisher  
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)

        self.odom_frame = frame('odom')
        self.base_link_frame = frame('base_link')

        # Constants
        self.r = 0.05
        self.L = 0.19

        # Initial state
        self.wr    = 0.0
        self.wl    = 0.0
        self.x     = 0.0
        self.y     = 0.0
        self.theta = 0.0
        self.prev_time_ns = self.get_clock().now().nanoseconds

        self.P = np.zeros((3, 3))  # Initial covariance
        self.A = 0.01  # Variance for wheel speed noise
        self.B = 0.005 # Covariance between wheel speeds
        self.C = 0.02  # Variance for heading noise

        self.timer = self.create_timer(0.02, self.timer_callback)

    def timer_callback(self):
        v, w = self.get_robot_vel(self.wr, self.wl)
        self.update_pose(v, w)
        dt = 0.01
        self.update_covariance(v, w, dt)
        odom_msg = self.fill_odom_message(self.x, self.y, self.theta)
        odom_msg.pose.covariance = [0.0]*36
        odom_msg.pose.covariance[0] = self.P[0, 0]  # x covariance
        odom_msg.pose.covariance[7] = self.P[1, 1]  # y covariance
        odom_msg.pose.covariance[35] = self.P[2, 2] # theta covariance 
        odom_msg.pose.covariance[1] = self.P[0, 1] #cov xy
        odom_msg.pose.covariance[6] = self.P[1, 0] #cov yx
        odom_msg.pose.covariance[5] = self.P[0, 1] #cov xtheta
        odom_msg.pose.covariance[30] = self.P[2, 0] #cov theta x
        odom_msg.pose.covariance[11] = self.P[1, 2] #cov ytheta
        odom_msg.pose.covariance[31] = self.P[2, 1] #cov theta y

        self.odom_pub.publish(odom_msg)

    def wr_callback(self, msg):
        self.wr = msg.data

    def wl_callback(self, msg):
        self.wl = msg.data

    def update_covariance(self, v, w, dt):
        #Jacobian matrices
        J_h = np.array([
            [1, 0, -v * dt * np.sin(self.theta)],
            [0, 1,  v * dt * np.cos(self.theta)],
            [0, 0, 1]
        ])

        Q = np.array([
            [self.A, self.B, self.B],
            [self.B, self.A, self.B],
            [self.B, self.B, self.C]
        ])

        #Covariance propagation
        self.P = J_h @ self.P @ J_h.T + Q

    def get_robot_vel(self, wr, wl):
        v = self.r * (wr + wl) / 2.0
        w = self.r * (wr - wl) / self.L
        return v, w

    def update_pose(self, v, w):
        dt = (self.get_clock().now().nanoseconds - self.prev_time_ns) / 1e9
        self.x += v * np.cos(self.theta) * dt
        self.y += v * np.sin(self.theta) * dt
        self.theta += w * dt
        self.theta = np.arctan2(np.sin(self.theta), np.cos(self.theta))
        self.prev_time_ns = self.get_clock().now().nanoseconds

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
        return odom


def main(args=None):
    rclpy.init(args=args)
    node = localization()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        node.destroy_node()


if __name__ == '__main__':
    main()