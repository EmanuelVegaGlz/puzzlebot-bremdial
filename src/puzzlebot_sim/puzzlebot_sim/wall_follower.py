import signal
import sys

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class LaserScanSub(Node):

    def __init__(self):
        super().__init__('wall_follower')

        # Handle shutdown gracefully
        signal.signal(signal.SIGINT, self.shutdown_function)

        # Subscribers and publishers
        self.sub = self.create_subscription(
            LaserScan,
            "scan",
            self.lidar_cb,
            10
        )

        self.cmd_vel_pub = self.create_publisher(
            Twist,
            "cmd_vel",
            10
        )

        # Variables
        self.lidar = LaserScan()
        self.robot_vel = Twist()

        # Parameters
        self.d_safety = 0.2   # Stop distance [m]
        self.v = 0.4          # Linear velocity [m/s]
        self.kw = 1.8         # Angular proportional gain
        self.d_wall = 0.4
        self.k_wall = 1.1
        self.front_d_safety = 0.3


        # Timer (10 Hz)
        timer_period = 0.1
        self.timer = self.create_timer(
            timer_period,
            self.timer_callback
        )

        self.get_logger().info("Node initialized!")

    def timer_callback(self):

        # Check if lidar data has been received
        if not self.lidar.ranges:
            print("No lidar data received")
            return

        # Get closest object
        closest_range, theta_closest = self.get_closest_object()

        #print(f"closest_range: {closest_range}")
        #print(f"theta_closest: {theta_closest}")

        # Case 1: No nearby obstacles
        if np.isinf(closest_range) or closest_range > 1.0:

            print("There are no objects around")

            v = self.v
            w = 0.0

        # Case 2: Obstacle too close
        elif closest_range < self.d_safety:

            print("Object too close, stopping")

            v = 0.0
            w = 0.0

        # Case 3: Wall following
        else:

            # Obstacle avoidance angle
            theta_ao = self.get_theta_ao(theta_closest)

            # Wall-following angle
            theta_fw = self.get_theta_fw(
                theta_ao,
                direction="fwccw"
            )

            # Angular control
            angle_error = np.arctan2(
                np.sin(theta_fw),
                np.cos(theta_fw)
            )



            d_wall_error = closest_range - self.d_wall

            w = self.kw * angle_error + self.k_wall * d_wall_error
            # Reduce speed while turning
            v = self.v * 0.4

            if self.get_closest_front_obstacle_distance() < self.front_d_safety:
                print("Obstacle in front, corner case")
                #turn depending cw or counter clockwise to follow next wall
                w += self.kw * np.sign(theta_fw) * np.pi / 4
                v = 0.0

            # Limit angular velocity
            w = np.clip(w, -1.2, 1.2)

            
        # Publish velocity command
        self.robot_vel.linear.x = v
        self.robot_vel.angular.z = w

        self.cmd_vel_pub.publish(self.robot_vel)

    def get_front_obstacle_distance(self):
        """
        Returns distance to the closest obstacle in front of the robot.
        """

        # Front is at index corresponding to angle 0
        front_index = int(
            (0.0 - self.lidar.angle_min) / self.lidar.angle_increment
        )

        front_distance = self.lidar.ranges[front_index]

        return front_distance
    
    def get_closest_front_obstacle_distance(self, angle_threshold=np.pi/6):
        """
        Returns distance to the closest obstacle in front of the robot within a specified angle threshold.

        Args:
            angle_threshold : float
                Maximum angle from the front direction to consider (in radians).
        """

        closest_distance = float('inf')

        for i, range in enumerate(self.lidar.ranges):
            angle = self.lidar.angle_min + i * self.lidar.angle_increment

            # Check if the angle is within the threshold
            if abs(angle) <= angle_threshold:
                if range < closest_distance:
                    closest_distance = range

        return closest_distance
    

    def get_closest_object(self):
        """
        Returns:
            closest_range : float
                Distance to closest obstacle.

            theta_closest : float
                Angle to closest obstacle [rad].
        """

        closest_range = min(self.lidar.ranges)

        closest_index = self.lidar.ranges.index(
            closest_range
        )

        theta_closest = (
            self.lidar.angle_min
            + closest_index * self.lidar.angle_increment
        )

        # Normalize angle to (-pi, pi]
        theta_closest = np.arctan2(
            np.sin(theta_closest),
            np.cos(theta_closest)
        )

        return closest_range, theta_closest

    def get_theta_ao(self, theta_closest):
        """
        Computes obstacle avoidance angle.
        """

        theta_ao = theta_closest + np.pi

        # Normalize angle
        theta_ao = np.arctan2(
            np.sin(theta_ao),
            np.cos(theta_ao)
        )

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
        theta_fw = np.arctan2(
            np.sin(theta_fw),
            np.cos(theta_fw)
        )

        return theta_fw

    def lidar_cb(self, lidar_msg):
        """
        Receives LaserScan messages.
        """

        self.lidar = lidar_msg

    def shutdown_function(self, signum, frame):
        """
        Stops the robot before shutting down.
        """

        self.get_logger().info(
            "Shutting down. Stopping robot..."
        )

        stop_twist = Twist()

        # Publish zero velocities
        self.cmd_vel_pub.publish(stop_twist)

        rclpy.shutdown()
        sys.exit(0)


def main(args=None):

    rclpy.init(args=args)
    wall_follower = LaserScanSub()
    rclpy.spin(wall_follower)
    wall_follower.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()