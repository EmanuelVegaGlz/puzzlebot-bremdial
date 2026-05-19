'''
User Defined Path Controller Node with PID Control
The robot moves in the set path, calculating pose and velocity.
The robot stops after completing the path.
Goal points are set via params.
'''

import rclpy
import rclpy.logging
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from geometry_msgs.msg import Pose2D, Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry
import numpy as np
import signal
import sys
import tf_transformations

class controller(Node):
    def __init__(self):
        super().__init__('controller')
        self.wait_for_ros_time()

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist,'cmd_vel', 10)
        self.next_goal_pub = self.create_publisher(Empty,'next_goal', 10)
        self.pose_sub = self.create_subscription(Odometry, 'odom',self.pose_cb,  10)
        self.goal_sub = self.create_subscription(Pose2D,'goal',self.goal_cb,  10)
        # Lidar subscription for wall-following
        self.lidar_sub = self.create_subscription(LaserScan, 'scan', self.lidar_cb, 10)

        signal.signal(signal.SIGINT, self.shutdown_function)

        self.get_logger().set_level(rclpy.logging.LoggingSeverity.INFO)

        # Parameters
        self.robust_margin = self.declare_parameter('robust_margin',  0.9).get_parameter_value().double_value
        self.goal_threshold = self.declare_parameter('goal_threshold', 0.05).get_parameter_value().double_value
        self.kp_v = self.declare_parameter('kp_v', 0.2).get_parameter_value().double_value
        self.kp_w = self.declare_parameter('kp_w', 1.2).get_parameter_value().double_value
        self.add_on_set_parameters_callback(self.parameter_callback)

        self.goal_received = False
        self.xg = 0.0
        self.yg = 0.0
        self.xr = 0.0
        self.yr = 0.0
        self.theta_r = 0.0

        self.cmd_vel = Twist()
        # Wall-following baseline state and parameters
        self.lidar = LaserScan()
        self.d_safety = 0.2   # Stop distance [m]
        self.v_wall = 0.4     # Linear velocity [m/s]
        self.kw = 1.9         # Angular proportional gain
        self.d_wall = 0.3
        self.k_wall = 1.0
        self.last_log_time = self.get_clock().now()
        self.create_timer(0.05, self.main_timer_cb)

        self.next_goal_pub.publish(Empty())
        self.get_logger().info("Controller node initialized. Requested first goal.")

    def main_timer_cb(self):
        now = self.get_clock().now()
        log_interval = 1.0

        # Check lidar and compute wall-following override when appropriate
        use_wall_follow = False
        closest_range = float('inf')
        theta_closest = 0.0

        if getattr(self.lidar, 'ranges', None):
            # filter out NaNs
            try:
                ranges = [r for r in self.lidar.ranges if not np.isnan(r)]
            except Exception:
                ranges = list(self.lidar.ranges)

            if ranges:
                # find minimum but skip non-finite values
                finite_ranges = [r for r in ranges if np.isfinite(r)]
                if finite_ranges:
                    closest_range = min(finite_ranges)
                    # find index matching closest_range in original list
                    closest_index = list(self.lidar.ranges).index(closest_range)
                    theta_closest = (
                        self.lidar.angle_min
                        + closest_index * self.lidar.angle_increment
                    )
                    use_wall_follow = True

        if self.goal_received:
            if (now - self.last_log_time).nanoseconds * 1e-9 > log_interval:
                self.get_logger().info(f"Moving to goal: x={self.xg:.2f}, y={self.yg:.2f}")
                self.last_log_time = now
            ed, etheta = self.get_errors(self.xr, self.yr, self.xg, self.yg, self.theta_r)

            if ed < self.goal_threshold:
                self.get_logger().info(f"Goal reached: x={self.xg:.2f}, y={self.yg:.2f}")
                self.goal_received = False
                self.next_goal_pub.publish(Empty())
                self.cmd_vel.linear.x  = 0.0
                self.cmd_vel.angular.z = 0.0
            else:
                # Nominal control towards goal
                self.cmd_vel.linear.x  = min(self.kp_v * ed, 0.5)
                self.cmd_vel.angular.z = self.kp_w * etheta

                # If lidar indicates nearby obstacles, override with wall-following logic
                if use_wall_follow:
                    # Case 1: No nearby obstacles
                    if np.isinf(closest_range) or closest_range > 1.0:
                        # keep nominal cmd_vel
                        pass

                    # Case 2: Obstacle too close
                    elif closest_range < self.d_safety:
                        self.get_logger().info("Object too close, stopping")
                        self.cmd_vel.linear.x = 0.0
                        self.cmd_vel.angular.z = 0.0

                    # Case 3: Wall following
                    else:
                        theta_ao = self.get_theta_ao(theta_closest)
                        theta_fw = self.get_theta_fw(theta_ao, direction='fwccw')

                        angle_error = np.arctan2(np.sin(theta_fw), np.cos(theta_fw))
                        d_wall_error = closest_range - self.d_wall

                        w = self.kw * angle_error + self.k_wall * d_wall_error
                        w = np.clip(w, -1.0, 1.0)
                        v = self.v_wall * 0.4

                        self.cmd_vel.linear.x = v
                        self.cmd_vel.angular.z = w
        else:
            if (now - self.last_log_time).nanoseconds * 1e-9 > log_interval:
                self.get_logger().info("Waiting for goal")
                self.last_log_time = now
            self.cmd_vel.linear.x  = 0.0
            self.cmd_vel.angular.z = 0.0

        self.cmd_vel_pub.publish(self.cmd_vel)

    def get_errors(self, xr, yr, xg, yg, theta_r):
        ed     = np.sqrt((xg - xr)**2 + (yg - yr)**2)
        thetag = np.arctan2(yg - yr, xg - xr)
        etheta = np.arctan2(np.sin(thetag - theta_r), np.cos(thetag - theta_r))
        return ed, etheta

    def get_theta_ao(self, theta_closest):
        """
        Computes obstacle avoidance angle.
        """

        theta_ao = theta_closest + np.pi

        # Normalize angle
        theta_ao = np.arctan2(np.sin(theta_ao), np.cos(theta_ao))

        return theta_ao

    def get_theta_fw(self, theta_ao, direction="fwccw"):
        """
        Computes wall-following angle.

        Args:
            theta_ao : float
                Obstacle avoidance angle.

            direction : str
                "fwcw"  -> follow wall clockwise (right)
                "fwccw" -> follow wall counterclockwise (left)
        """

        if direction == "fwcw":
            # Follow wall on the right
            theta_fw = theta_ao - np.pi / 2

        else:
            # Follow wall on the left
            theta_fw = theta_ao + np.pi / 2

        # Normalize angle
        theta_fw = np.arctan2(np.sin(theta_fw), np.cos(theta_fw))

        return theta_fw

    def lidar_cb(self, lidar_msg):
        """
        Receives LaserScan messages and stores them for use in the controller loop.
        """

        self.lidar = lidar_msg

    def pose_cb(self, msg):
        self.xr = msg.pose.pose.position.x
        self.yr = msg.pose.pose.position.y
        ori = msg.pose.pose.orientation
        _, _, self.theta_r = tf_transformations.euler_from_quaternion(
            [ori.x, ori.y, ori.z, ori.w])

    def goal_cb(self, goal):
        self.xg = goal.x
        self.yg = goal.y
        self.goal_received = True
        self.get_logger().info(f"New goal: x={self.xg:.2f}, y={self.yg:.2f}")

    def wait_for_ros_time(self):
        self.get_logger().info('Waiting for ROS time...')
        while rclpy.ok():
            if self.get_clock().now().nanoseconds > 0:
                break
            rclpy.spin_once(self, timeout_sec=0.1)

    def parameter_callback(self, params):
        for p in params:
            if p.name == 'robust_margin': self.robust_margin = p.value
            elif p.name == 'goal_threshold': self.goal_threshold = p.value
            elif p.name == 'kp_v':self.kp_v = p.value
            elif p.name == 'kp_w':self.kp_w = p.value
        return SetParametersResult(successful=True)

    def shutdown_function(self, signum, frame):
        self.cmd_vel_pub.publish(Twist())
        rclpy.shutdown()
        sys.exit(0)


def main(args=None):
    rclpy.init(args=args)
    node = controller()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()