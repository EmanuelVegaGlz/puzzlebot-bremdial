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

        self.timer = self.create_timer(0.02, self.timer_callback)

    def timer_callback(self):
        v, w = self.get_robot_vel(self.wr, self.wl)
        self.update_pose(v, w)
        self.odom_pub.publish(self.fill_odom_message(self.x, self.y, self.theta))

    def wr_callback(self, msg):
        self.wr = msg.data

    def wl_callback(self, msg):
        self.wl = msg.data

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