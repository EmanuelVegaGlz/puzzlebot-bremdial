import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from rclpy import qos
import transforms3d
import numpy as np

class localization(Node):

    def __init__(self, namespace=''):
        # Apply namespace if provided
        node_name = f'localization{namespace}' if namespace else 'localization'
        super().__init__(node_name)
        
        # Declare parameters
        self.declare_parameter('wheel_radius', 0.05)
        self.declare_parameter('wheel_separation', 0.19)
        self.declare_parameter('namespace', '')
        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('initial_theta', 0.0)
        
        # Get parameter values
        self.r = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.L = self.get_parameter('wheel_separation').get_parameter_value().double_value
        self.namespace = self.get_parameter('namespace').get_parameter_value().string_value
        
        # Build topic names with namespace
        wr_topic = f'{self.namespace}/wr' if self.namespace else 'wr'
        wl_topic = f'{self.namespace}/wl' if self.namespace else 'wl'
        odom_topic = f'{self.namespace}/odom' if self.namespace else 'odom'
        
        # Create subscribers to the wheel velocity topics
        self.wr_sub = self.create_subscription(Float32, wr_topic, self.wr_callback, qos.qos_profile_sensor_data)
        self.wl_sub = self.create_subscription(Float32, wl_topic, self.wl_callback, qos.qos_profile_sensor_data)
        
        # Create publisher for the robot pose
        self.odom_pub = self.create_publisher(Odometry, odom_topic, 10)

        # Get initial pose from parameters
        self.x = self.get_parameter('initial_x').get_parameter_value().double_value
        self.y = self.get_parameter('initial_y').get_parameter_value().double_value
        self.theta = self.get_parameter('initial_theta').get_parameter_value().double_value
        
        # Variables
        self.w = 0.0  # angular velocity
        self.v = 0.0  # linear velocity
        self.wr = 0.0  # right wheel velocity
        self.wl = 0.0  # left wheel velocity

        self.odom = Odometry()
        self.prev_time_ns = self.get_clock().now().nanoseconds

        # Create timer to update the robot pose
        timer_period = 0.02
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info(f'Localization node initialized with namespace: "{self.namespace}"')
        self.get_logger().info(f'Initial pose: x={self.x}, y={self.y}, theta={self.theta}')
        self.get_logger().info(f'Wheel radius: {self.r}, Wheel separation: {self.L}')

    def timer_callback(self):
        # Update the robot pose based on the wheel velocities
        v, w = self.get_robot_vel(self.wr, self.wl)
        self.update_pose(v, w)

        self.odom = self.fill_odom_message(self.x, self.y,self.theta)
        self.odom_pub.publish(self.odom)
        # add logger info every 1 second

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
        self.x = self.x + v * np.cos(self.theta) * dt
        self.y = self.y + v * np.sin(self.theta) * dt
        self.theta = self.theta + w * dt
        self.theta = np.arctan2(np.sin(self.theta), np.cos(self.theta))  # Normalize angle to [-pi, pi]
        self.prev_time_ns = self.get_clock().now().nanoseconds
        self.prev_time_ns = self.get_clock().now().nanoseconds

    def fill_odom_message(self, x,y,yaw):
        # Create a new Odometry message 
        odom_msg = Odometry() 
        # Fill the message with the robot's pose 
        odom_msg.header.stamp = self.get_clock().now().to_msg() # Get the current time 
        odom_msg.header.frame_id = 'odom' # Set the frame id 
        odom_msg.child_frame_id = 'base_link' # Set the child frame id 
        odom_msg.pose.pose.position.x = x # x position [m] 
        odom_msg.pose.pose.position.y = y # y position [m] 
        odom_msg.pose.pose.position.z = 0.0 # z position [m] 
        # Set the orientation using quaternion
        # Convert the yaw angle to a quaternion 
        quat = transforms3d.euler.euler2quat(0, 0, yaw)  
        odom_msg.pose.pose.orientation.w = quat[0] 
        odom_msg.pose.pose.orientation.x = quat[1] 
        odom_msg.pose.pose.orientation.y = quat[2] 
        odom_msg.pose.pose.orientation.z = quat[3]
        return odom_msg
    
def main(args=None):
    rclpy.init(args=args)
    
    # Parse arguments for namespace support
    namespace = ''
    if args:
        for arg in args:
            if arg.startswith('__ns:='):
                namespace = arg.replace('__ns:=', '')
                break
    
    node = localization(namespace=namespace)

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
    