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


def normalize_angle(angle):
    """Normalize an angle to (-pi, pi]."""

    return np.arctan2(np.sin(angle), np.cos(angle))


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
        self.bug_mode = self.declare_parameter('bug_mode', 2).get_parameter_value().integer_value
        self.robust_margin = self.declare_parameter('robust_margin',  0.9).get_parameter_value().double_value
        self.goal_threshold = self.declare_parameter('goal_threshold', 0.05).get_parameter_value().double_value
        self.kp_v = self.declare_parameter('kp_v', 0.2).get_parameter_value().double_value
        self.kp_w = self.declare_parameter('kp_w', 1.2).get_parameter_value().double_value
        # Bug2 parameters
        self.bug_enabled = self.declare_parameter('bug_enabled', True).get_parameter_value().bool_value
        self.bug_hit_dist = self.declare_parameter('bug_hit_dist', 0.5).get_parameter_value().double_value
        self.bug_leave_tol = self.declare_parameter('bug_leave_tol', 0.15).get_parameter_value().double_value
        self.bug_leave_margin = self.declare_parameter('bug_leave_margin', 0.05).get_parameter_value().double_value
        self.bug_max_follow_time = self.declare_parameter('bug_max_follow_time', 30.0).get_parameter_value().double_value

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
        self.kw = 1.8         # Angular proportional gain
        self.d_wall = 0.4
        self.k_wall = 1.1
        self.front_d_safety = 0.3
        self.last_log_time = self.get_clock().now()
        # Bug2 state
        self.bug_state = 'nav'  # 'nav' or 'wall_follow' (Bug2) or 'avoid' (Bug0)
        self.mline_start = (0.0, 0.0)
        self.hit_distance = float('inf')
        self.wall_follow_start_time = None
        self._mline_origin = (0.0, 0.0)
        self.hit_point = (0.0, 0.0)
        # Bug0 state (simple avoid-on-contact behavior)
        self.bug0_start_time = None
        self.create_timer(0.05, self.main_timer_cb)

        self.next_goal_pub.publish(Empty())
        self.get_logger().info("Controller node initialized. Requested first goal.")

    def main_timer_cb(self):
        now = self.get_clock().now()
        log_interval = 1.0

        lidar_data = self._get_lidar_obstacle_data()
        closest_range = lidar_data['closest_range']
        theta_closest = lidar_data['theta_closest']
        has_obstacle = lidar_data['has_obstacle']

        if self.goal_received:
            if (now - self.last_log_time).nanoseconds * 1e-9 > log_interval:
                self.get_logger().info(f"Moving to goal: x={self.xg:.2f}, y={self.yg:.2f}")
                self.last_log_time = now

            ed, etheta = self.get_errors(self.xr, self.yr, self.xg, self.yg, self.theta_r)

            if ed < self.goal_threshold:
                self.get_logger().info(f"Goal reached: x={self.xg:.2f}, y={self.yg:.2f}")
                self.goal_received = False
                self.bug_state = 'nav'
                self.wall_follow_start_time = None
                self.bug0_start_time = None
                self.next_goal_pub.publish(Empty())
                self.cmd_vel.linear.x = 0.0
                self.cmd_vel.angular.z = 0.0
            else:
                # Nominal goal tracking
                self.cmd_vel.linear.x = min(self.kp_v * ed, 0.5)
                self.cmd_vel.angular.z = self.kp_w * etheta

                if self.bug_enabled and has_obstacle and closest_range < self.bug_hit_dist and self.bug_state == 'nav':
                    if int(self.bug_mode) == 2:
                        self.bug_state = 'wall_follow'
                        self.hit_distance = ed
                        # Record hit point on the m-line at the moment of contact
                        self.mline_start = (self.xr, self.yr)
                        self.hit_point = (self.xr, self.yr)
                        self.wall_follow_start_time = now
                        self.get_logger().info(
                            f"Bug2: hit obstacle at ({self.xr:.2f},{self.yr:.2f}), "
                            f"switching to wall_follow (hit_distance={self.hit_distance:.2f})"
                        )
                    else:
                        self.bug_state = 'avoid'
                        self.hit_distance = ed
                        self.bug0_start_time = now
                        self.get_logger().info(
                            f"Bug0: hit obstacle, switching to avoid (hit_distance={self.hit_distance:.2f})"
                        )

                if self.bug_state == 'wall_follow':
                    # Leave condition: on m-line AND closer to goal than hit point
                    if self._should_leave_bug2(ed):
                        self.get_logger().info("Bug2: leave condition met, switching to nav")
                        self.bug_state = 'nav'
                        self.wall_follow_start_time = None
                    else:
                        # Always follow the wall contour while in wall_follow state
                        self._apply_wall_follow_controller()

                    if self.wall_follow_start_time is not None and (
                        (now - self.wall_follow_start_time).nanoseconds * 1e-9
                    ) > self.bug_max_follow_time:
                        self.get_logger().info("Bug2: wall_follow timeout, returning to nav")
                        self.bug_state = 'nav'
                        self.wall_follow_start_time = None

                elif self.bug_state == 'avoid' and has_obstacle:
                    self._apply_wall_follow_controller()

                    if self._should_leave_bug0(closest_range):
                        self.get_logger().info("Bug0: obstacle cleared, switching to nav")
                        self.bug_state = 'nav'
                        self.bug0_start_time = None

                    if self.bug0_start_time is not None and (
                        (now - self.bug0_start_time).nanoseconds * 1e-9
                    ) > self.bug_max_follow_time:
                        self.get_logger().info("Bug0: avoid timeout, returning to nav")
                        self.bug_state = 'nav'
                        self.bug0_start_time = None
        else:
            if (now - self.last_log_time).nanoseconds * 1e-9 > log_interval:
                self.get_logger().info("Waiting for goal")
                self.last_log_time = now
            self.cmd_vel.linear.x = 0.0
            self.cmd_vel.angular.z = 0.0

        self.cmd_vel_pub.publish(self.cmd_vel)

    def _get_lidar_obstacle_data(self):
        if not getattr(self.lidar, 'ranges', None):
            return {
                'has_obstacle': False,
                'closest_range': float('inf'),
                'theta_closest': 0.0,
            }

        try:
            ranges = [r for r in self.lidar.ranges if np.isfinite(r)]
        except Exception:
            ranges = list(self.lidar.ranges)

        if not ranges:
            return {
                'has_obstacle': False,
                'closest_range': float('inf'),
                'theta_closest': 0.0,
            }

        closest_range = min(ranges)
        closest_index = list(self.lidar.ranges).index(closest_range)
        theta_closest = normalize_angle(
            self.lidar.angle_min + closest_index * self.lidar.angle_increment
        )

        return {
            'has_obstacle': True,
            'closest_range': float(closest_range),
            'theta_closest': float(theta_closest),
        }

    def _apply_wall_follow_controller(self):
        closest_range, theta_closest = self.get_closest_object()

        if np.isinf(closest_range) or closest_range > 1.0:
            self.cmd_vel.linear.x = self.v_wall
            self.cmd_vel.angular.z = 0.0
            return

        if closest_range < self.d_safety:
            self.get_logger().info("Object too close during wall_follow, stopping")
            self.cmd_vel.linear.x = 0.0
            self.cmd_vel.angular.z = 0.0
            return

        theta_ao = self.get_theta_ao(theta_closest)
        theta_fw = self.get_theta_fw(theta_ao, direction='fwccw')
        angle_error = normalize_angle(theta_fw)
        d_wall_error = closest_range - self.d_wall

        w = self.kw * angle_error + self.k_wall * d_wall_error
        v = self.v_wall * 0.4

        if self.get_closest_front_obstacle_distance() < self.front_d_safety:
            self.get_logger().info("Obstacle in front, corner case")
            w += self.kw * np.sign(theta_fw) * np.pi / 4
            v = 0.0

        self.cmd_vel.linear.x = v
        self.cmd_vel.angular.z = np.clip(w, -1.2, 1.2)

    def _apply_bug0_controller(self, closest_range, theta_closest):
        if closest_range < self.d_safety:
            self.get_logger().info("Object too close during avoid, stopping")
            self.cmd_vel.linear.x = 0.0
            self.cmd_vel.angular.z = 0.0
            return

        theta_ao = self.get_theta_ao(theta_closest)
        angle_error = normalize_angle(theta_ao - self.theta_r)
        w = np.clip(self.kp_w * angle_error, -1.0, 1.0)
        v = self.v_wall * 0.2 if abs(angle_error) < 0.5 else 0.0

        self.cmd_vel.linear.x = v
        self.cmd_vel.angular.z = w

    def _apply_bug2_line_controller(self):
        line_heading = self._get_bug2_line_heading()
        angle_error = normalize_angle(line_heading - self.theta_r)

        self.cmd_vel.linear.x = self.v_wall * 0.4
        self.cmd_vel.angular.z = np.clip(self.kp_w * angle_error, -1.2, 1.2)

    def _get_bug2_line_heading(self):
        start = self.mline_start
        end = (self.xg, self.yg)

        if start == end:
            return normalize_angle(np.arctan2(self.yg - self.yr, self.xg - self.xr))

        dx = end[0] - start[0]
        dy = end[1] - start[1]
        denom = dx * dx + dy * dy

        if denom <= 1e-9:
            return normalize_angle(np.arctan2(self.yg - self.yr, self.xg - self.xr))

        t = ((self.xr - start[0]) * dx + (self.yr - start[1]) * dy) / denom
        t = float(np.clip(t, 0.0, 1.0))

        px = start[0] + t * dx
        py = start[1] + t * dy

        return normalize_angle(np.arctan2(py - self.yr, px - self.xr))

    def _dist_to_mline(self):
        """
        Compute the perpendicular distance from the robot to the starting line (m-line).
        Starting line connects _mline_origin (x0,y0) to goal (xG,yG).
        Line form: Ax + By + C = 0
          m  = (yG - y0) / (xG - x0)
          A  = m,  B = -1,  C = y0 - m*x0
        Distance = |A*xR + B*yR + C| / sqrt(A^2 + B^2)
        For vertical line (xG == x0) use |xR - x0|.
        """
        x0, y0 = self._mline_origin
        xg, yg = self.xg, self.yg
        dx = xg - x0
        dy = yg - y0
        if abs(dx) < 1e-6:
            # Vertical line
            return abs(self.xr - x0)
        m = dy / dx
        # A*xR + B*yR + C,  A=m, B=-1, C=y0-m*x0
        A = m
        B = -1.0
        C = y0 - m * x0
        num = abs(A * self.xr + B * self.yr + C)
        den = np.hypot(A, B)
        return num / den if den > 1e-9 else float('inf')

    def _should_leave_bug2(self, ed):
        """
        Leave condition (from Bug2 spec):
          1. Progress:   d_gtg(t) < |d_gtg(t_H1) - minProgress|
          2. On m-line:  d_line < bug_leave_tol
          3. Must have moved away from hit point to avoid immediate re-trigger.
        """
        progress = ed < (self.hit_distance - self.bug_leave_margin)
        d_line = self._dist_to_mline()
        on_line = d_line < self.bug_leave_tol
        dx = self.xr - self.hit_point[0]
        dy = self.yr - self.hit_point[1]
        away_from_hit = np.hypot(dx, dy) > 0.20
        return progress and on_line and away_from_hit

    def _should_leave_bug0(self, closest_range):
        return closest_range > (self.bug_hit_dist + 0.2)

    def get_errors(self, xr, yr, xg, yg, theta_r):
        ed     = np.sqrt((xg - xr)**2 + (yg - yr)**2)
        thetag = np.arctan2(yg - yr, xg - xr)
        etheta = np.arctan2(np.sin(thetag - theta_r), np.cos(thetag - theta_r))
        return ed, etheta

    def get_front_obstacle_distance(self):
        front_index = int(
            (0.0 - self.lidar.angle_min) / self.lidar.angle_increment
        )
        return self.lidar.ranges[front_index]

    def get_closest_front_obstacle_distance(self, angle_threshold=np.pi / 6):
        closest_distance = float('inf')

        for i, distance in enumerate(self.lidar.ranges):
            angle = self.lidar.angle_min + i * self.lidar.angle_increment

            if abs(angle) <= angle_threshold and distance < closest_distance:
                closest_distance = distance

        return closest_distance

    def get_closest_object(self):
        closest_range = min(self.lidar.ranges)
        closest_index = self.lidar.ranges.index(closest_range)
        theta_closest = (
            self.lidar.angle_min
            + closest_index * self.lidar.angle_increment
        )
        theta_closest = normalize_angle(theta_closest)
        return closest_range, theta_closest

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

    def _on_mline(self, mline_start, mline_end, point, tol):
        """Return True if `point` is within `tol` distance of the m-line from mline_start to mline_end."""
        (x1, y1) = mline_start
        (x2, y2) = mline_end
        (x0, y0) = point
        # Degenerate line
        if x1 == x2 and y1 == y2:
            return False
        # distance from point to line segment
        num = abs((y2 - y1) * x0 - (x2 - x1) * y0 + x2 * y1 - y2 * x1)
        den = np.hypot(y2 - y1, x2 - x1)
        dist = num / den
        return dist <= tol

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
        # Record the true m-line origin (robot position when goal is received).
        # This defines the straight line from start to goal used by Bug2.
        self._mline_origin = (self.xr, self.yr)
        self.mline_start = (self.xr, self.yr)  # kept for compatibility
        self.bug_state = 'nav'
        self.wall_follow_start_time = None
        self.hit_point = (self.xr, self.yr)
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
            elif p.name == 'bug_enabled': self.bug_enabled = p.value
            elif p.name == 'bug_hit_dist': self.bug_hit_dist = p.value
            elif p.name == 'bug_leave_tol': self.bug_leave_tol = p.value
            elif p.name == 'bug_leave_margin': self.bug_leave_margin = p.value
            elif p.name == 'bug_max_follow_time': self.bug_max_follow_time = p.value
            elif p.name == 'bug_mode': self.bug_mode = int(p.value)
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