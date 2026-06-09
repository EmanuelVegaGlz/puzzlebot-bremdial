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
import tf_transformations
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException

class controller(Node):
    def __init__(self):
        super().__init__('controller')
        self.wait_for_ros_time()
        self.pose_topic = self.declare_parameter(
            'pose_topic',
            '/localization/odom'
        ).get_parameter_value().string_value

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist,'cmd_vel', 10)
        self.next_goal_pub = self.create_publisher(Empty,'next_goal', 10)
        self.pose_sub = self.create_subscription(
            Odometry,
            self.pose_topic,
            self.pose_cb,
            10
        )
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
        # Bug parameters
        self.bug_mode = self.declare_parameter('bug_mode', 0).get_parameter_value().integer_value
        self.bug_enabled = self.declare_parameter('bug_enabled', True).get_parameter_value().bool_value
        self.bug_hit_dist = self.declare_parameter('bug_hit_dist', 0.5).get_parameter_value().double_value
        self.bug_leave_tol = self.declare_parameter('bug_leave_tol', 0.05).get_parameter_value().double_value
        self.bug_leave_margin = self.declare_parameter('bug_leave_margin', 0.05).get_parameter_value().double_value
        self.bug_max_follow_time = self.declare_parameter('bug_max_follow_time', 30.0).get_parameter_value().double_value
        self.bug0_clear_shot_dist = self.declare_parameter('bug0_clear_shot_dist', 0.80).get_parameter_value().double_value
        self.bug0_clear_shot_sector = self.declare_parameter('bug0_clear_shot_sector', 0.25).get_parameter_value().double_value
        self.front_d_safety = self.declare_parameter('front_d_safety', 0.45).get_parameter_value().double_value
        self.front_sector = self.declare_parameter('front_sector', 0.45).get_parameter_value().double_value
        self.max_v = self.declare_parameter('max_v', 0.35).get_parameter_value().double_value
        self.max_w = self.declare_parameter('max_w', 1.0).get_parameter_value().double_value
        self.corner_enter_dist = self.declare_parameter('corner_enter_dist', 0.50).get_parameter_value().double_value
        self.corner_exit_dist = self.declare_parameter('corner_exit_dist', 0.75).get_parameter_value().double_value
        self.corner_turn_w = self.declare_parameter('corner_turn_w', 0.65).get_parameter_value().double_value
        self.corner_linear_v = self.declare_parameter('corner_linear_v', 0.02).get_parameter_value().double_value
        self.wall_follow_direction = self.declare_parameter('wall_follow_direction', 'fwccw').get_parameter_value().string_value
        self.d_wall = self.declare_parameter('d_wall', 0.28).get_parameter_value().double_value
        self.v_wall = self.declare_parameter('v_wall', 0.45).get_parameter_value().double_value
        self.wall_speed_scale = self.declare_parameter('wall_speed_scale', 0.50).get_parameter_value().double_value
        self.kw = self.declare_parameter('kw', 1.25).get_parameter_value().double_value
        self.k_wall = self.declare_parameter('k_wall', 0.55).get_parameter_value().double_value
        self.wall_sector_inner_angle = self.declare_parameter('wall_sector_inner_angle', 0.45).get_parameter_value().double_value
        self.wall_sector_outer_angle = self.declare_parameter('wall_sector_outer_angle', 1.75).get_parameter_value().double_value
        self.wall_end_enter_dist = self.declare_parameter('wall_end_enter_dist', 0.52).get_parameter_value().double_value
        self.wall_end_exit_dist = self.declare_parameter('wall_end_exit_dist', 0.36).get_parameter_value().double_value
        self.wall_end_turn_w = self.declare_parameter('wall_end_turn_w', 0.90).get_parameter_value().double_value
        self.wall_end_linear_v = self.declare_parameter('wall_end_linear_v', 0.02).get_parameter_value().double_value
        self.wall_end_sector_inner_angle = self.declare_parameter('wall_end_sector_inner_angle', 0.35).get_parameter_value().double_value
        self.wall_end_sector_outer_angle = self.declare_parameter('wall_end_sector_outer_angle', 1.35).get_parameter_value().double_value
        self.wall_end_reacquire_dist = self.declare_parameter('wall_end_reacquire_dist', 0.46).get_parameter_value().double_value
        self.wall_end_min_turn_time = self.declare_parameter('wall_end_min_turn_time', 0.8).get_parameter_value().double_value
        self.wall_end_max_turn_time = self.declare_parameter('wall_end_max_turn_time', 3.5).get_parameter_value().double_value
        self.system_paused = self.declare_parameter('system_paused', False).get_parameter_value().bool_value
        self.aruco_stop_enabled = self.declare_parameter('aruco_stop_enabled', True).get_parameter_value().bool_value
        self.aruco_stop_duration = self.declare_parameter('aruco_stop_duration', 10.0).get_parameter_value().double_value
        self.aruco_resume_ignore_duration = self.declare_parameter(
            'aruco_resume_ignore_duration',
            2.0
        ).get_parameter_value().double_value
        self.aruco_correction_topic = self.declare_parameter(
            'aruco_correction_topic',
            'aruco_ekf/odom_correction'
        ).get_parameter_value().string_value

        self.aruco_correction_sub = self.create_subscription(
            Odometry,
            self.aruco_correction_topic,
            self.aruco_correction_cb,
            10
        )

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
        self.last_log_time = self.get_clock().now()
        self.last_state_log_time = self.get_clock().now()
        # Bug2 state
        self.bug_state = 'nav'  # 'nav' or 'wall_follow'
        self.mline_start = (0.0, 0.0)
        self.hit_distance = float('inf')
        self.wall_follow_start_time = None
        self.corner_active = False
        self.wall_end_active = False
        self.wall_end_start_time = None
        self.aruco_stop_until = None
        self.aruco_stop_active = False
        self.aruco_ignore_until = None

        # Outer corner escape routine state
        self.outer_escape_active = False
        self.outer_escape_attempts = 0
        self.outer_escape_start_pose = None  # (x, y, theta) at start of each attempt
        self.outer_escape_phase = 'idle'     # 'moving' or 'idle'
        self.OUTER_ESCAPE_DIST = 0.30        # 30 cm diagonal
        self.OUTER_ESCAPE_ANGLE = np.pi / 2  # 90 degrees
        self.OUTER_ESCAPE_V = 0.15           # linear speed during escape (tune as needed)
        self.OUTER_ESCAPE_W = 0.45           # angular speed during escape (tune as needed)

        self.create_timer(0.05, self.main_timer_cb)

        self.next_goal_pub.publish(Empty())
        self.get_logger().info(
            f"Controller initialized with pose topic {self.pose_topic}. "
            "Requested first goal."
        )

    def main_timer_cb(self):
        now = self.get_clock().now()
        log_interval = 1.0

        if self.system_paused:
            self.cmd_vel.linear.x = 0.0
            self.cmd_vel.angular.z = 0.0
            self.cmd_vel_pub.publish(self.cmd_vel)
            return

        if self.aruco_stop_active:
            if self.aruco_stop_until is not None and now < self.aruco_stop_until:
                self.cmd_vel.linear.x = 0.0
                self.cmd_vel.angular.z = 0.0
                self.cmd_vel_pub.publish(self.cmd_vel)
                return

            self.aruco_stop_active = False
            self.aruco_stop_until = None
            self.aruco_ignore_until = now + Duration(
                seconds=float(self.aruco_resume_ignore_duration)
            )
            self.get_logger().info(
                f"ArUco odometry pause finished. Ignoring new ArUco corrections for "
                f"{self.aruco_resume_ignore_duration:.1f} seconds while moving away."
            )

        # Check lidar and compute wall-following override when appropriate
        use_wall_follow = False
        closest_range = float('inf')
        theta_closest = 0.0
        front_range = float('inf')
        front_theta = 0.0
        goal_path_range = float('inf')
        side_wall_range = float('inf')

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
                    front_range, front_index = self._sector_min(-self.front_sector, self.front_sector)
                    if front_index is not None:
                        front_theta = self.lidar.angle_min + front_index * self.lidar.angle_increment

        if self.goal_received:
            if (
                self.bug_state == 'nav'
                and (now - self.last_log_time).nanoseconds * 1e-9 > log_interval
            ):
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
                self.cmd_vel.linear.x  = min(self.kp_v * ed, self.max_v)
                self.cmd_vel.angular.z = float(np.clip(self.kp_w * etheta, -self.max_w, self.max_w))

                goal_angle = self._goal_angle_in_robot_frame()
                goal_path_range, goal_path_index = self._sector_min(
                    goal_angle - self.bug0_clear_shot_sector,
                    goal_angle + self.bug0_clear_shot_sector
                )
                goal_path_theta = goal_angle
                if goal_path_index is not None:
                    goal_path_theta = (
                        self.lidar.angle_min
                        + goal_path_index * self.lidar.angle_increment
                    )

                # BUG: transition to wall-follow only when the path is blocked.
                obstacle_ahead = front_range < self.bug_hit_dist
                obstacle_on_goal_path = goal_path_range < min(self.bug_hit_dist, ed)
                if (
                    self.bug_enabled
                    and use_wall_follow
                    and self.bug_state == 'nav'
                    and (obstacle_ahead or obstacle_on_goal_path)
                ):
                    self.bug_state = 'wall_follow'
                    self.hit_distance = ed
                    self.wall_follow_start_time = now
                    self.corner_active = False
                    self.wall_end_active = False
                    if obstacle_ahead:
                        closest_range = front_range
                        theta_closest = front_theta
                        entry_reason = 'front'
                    else:
                        closest_range = goal_path_range
                        theta_closest = goal_path_theta
                        entry_reason = 'goal_path'
                    self.get_logger().info(
                        f"Bug{self.bug_mode}: hit obstacle, switching to wall_follow "
                        f"(reason={entry_reason}, hit_distance={self.hit_distance:.2f}, "
                        f"front={front_range:.2f}, goal_path={goal_path_range:.2f}, "
                        f"closest={closest_range:.2f})"
                    )

                # If wall-following, execute wall-follow and check leave conditions
                if self.bug_state == 'wall_follow' and use_wall_follow:
                    # emergency stop if too close
                    front_distance = self.get_closest_front_obstacle_distance()
                    if self.corner_active:
                        self.corner_active = front_distance < self.corner_exit_dist
                    else:
                        self.corner_active = front_distance < self.corner_enter_dist

                    if self.corner_active:
                        self.wall_end_active = False
                        self.wall_end_start_time = None
                        turn_sign = -1.0 if self.wall_follow_direction == 'fwccw' else 1.0
                        if front_distance < self.front_d_safety:
                            self.cmd_vel.linear.x = 0.0
                        else:
                            self.cmd_vel.linear.x = self.corner_linear_v
                        self.cmd_vel.angular.z = turn_sign * min(self.corner_turn_w, self.max_w)
                    else:
                        side_wall_range, side_wall_theta, side_wall_found = self._wall_follow_reference(
                            closest_range,
                            theta_closest
                        )


                    # ── Outer corner escape routine (replaces wall_end logic) ──────────────
                    if self.outer_escape_active:
                        # Compute progress since start of this attempt
                        dx = self.xr - self.outer_escape_start_pose[0]
                        dy = self.yr - self.outer_escape_start_pose[1]
                        dist_traveled = np.hypot(dx, dy)
                        angle_turned = abs(
                            np.arctan2(
                                np.sin(self.theta_r - self.outer_escape_start_pose[2]),
                                np.cos(self.theta_r - self.outer_escape_start_pose[2])
                            )
                        )

                        attempt_done = (
                            dist_traveled >= self.OUTER_ESCAPE_DIST
                            and angle_turned >= self.OUTER_ESCAPE_ANGLE
                        )

                        if not attempt_done:
                            # Pure odometry: move diagonally forward-left at 45 deg arc
                            self.cmd_vel.linear.x = self.OUTER_ESCAPE_V
                            self.cmd_vel.angular.z = self.OUTER_ESCAPE_W
                            self.cmd_vel_pub.publish(self.cmd_vel)
                            return  # skip all other logic this cycle
                        else:
                            # Attempt finished — check for left wall
                            left_wall_range, _, left_wall_found = self._wall_follow_reference(
                                float('inf'), 0.0
                            )
                            has_left_wall = left_wall_found and left_wall_range < self.d_wall * 2.5

                            self.get_logger().info(
                                f"Outer escape attempt {self.outer_escape_attempts} done. "
                                f"dist={dist_traveled:.2f} angle={np.degrees(angle_turned):.1f}° "
                                f"left_wall={'yes' if has_left_wall else 'no'} ({left_wall_range:.2f}m)"
                            )

                            if has_left_wall or self.outer_escape_attempts >= 2:
                                # Wall found or max attempts reached — resume normal wall follow
                                self.outer_escape_active = False
                                self.outer_escape_attempts = 0
                                self.outer_escape_phase = 'idle'
                                self.outer_escape_start_pose = None
                                self.get_logger().info("Outer escape done. Resuming wall follow.")
                                # Fall through to normal wall follow below
                            else:
                                # Try again
                                self.outer_escape_attempts += 1
                                self.outer_escape_start_pose = (self.xr, self.yr, self.theta_r)
                                self.get_logger().info(
                                    f"No left wall found. Starting outer escape attempt {self.outer_escape_attempts}."
                                )
                                self.cmd_vel.linear.x = self.OUTER_ESCAPE_V
                                self.cmd_vel.angular.z = self.OUTER_ESCAPE_W
                                self.cmd_vel_pub.publish(self.cmd_vel)
                                return

                    else:
                        # Check if we should trigger the outer escape routine
                        end_wall_range, _, end_wall_found = self._wall_follow_reference(
                            closest_range,
                            theta_closest,
                            inner_angle=self.wall_end_sector_inner_angle,
                            outer_angle=self.wall_end_sector_outer_angle
                        )
                        wall_end_condition = (
                            (not end_wall_found)
                            or end_wall_range > self.wall_end_enter_dist
                        )

                        if wall_end_condition:
                            self.outer_escape_active = True
                            self.outer_escape_attempts = 1
                            self.outer_escape_start_pose = (self.xr, self.yr, self.theta_r)
                            self.get_logger().info(
                                "Outer corner/wall end detected. Starting escape attempt 1."
                            )
                            self.cmd_vel.linear.x = self.OUTER_ESCAPE_V
                            self.cmd_vel.angular.z = self.OUTER_ESCAPE_W
                            self.cmd_vel_pub.publish(self.cmd_vel)
                            return

                    # Normal wall follow (reached when outer_escape just finished, or no wall_end condition)
                    closest_range = side_wall_range
                    theta_closest = side_wall_theta

                            # Obstacle avoidance angle
                    theta_ao = self.get_theta_ao(theta_closest)

                    # Wall-following angle
                    theta_fw = self.get_theta_fw(
                        theta_ao,
                        direction=self.wall_follow_direction
                    )

                    # Angular control
                    angle_error = np.arctan2(
                        np.sin(theta_fw),
                        np.cos(theta_fw)
                    )


                    d_wall_error = closest_range - self.d_wall

                    w = self.kw * angle_error + self.k_wall * d_wall_error
                    # Reduce speed while turning
                    v = self.v_wall * self.wall_speed_scale

                    # Limit angular velocity
                    w = np.clip(w, -self.max_w, self.max_w)

                    self.cmd_vel.linear.x = v
                    self.cmd_vel.angular.z = w

                    leave_bug, leave_details = self._should_leave_bug(ed)
                    if self.corner_active or self.outer_escape_active:
                        leave_bug = False

                    if self.wall_follow_start_time is not None:
                        elapsed_wall_follow = self._elapsed_seconds(
                            self.wall_follow_start_time,
                            now
                        )
                        if elapsed_wall_follow > self.bug_max_follow_time:
                            leave_bug = True
                            leave_details['forced_leave'] = True
                            leave_details['wall_follow_time'] = elapsed_wall_follow
                            self.get_logger().info(
                                f"Bug leave forced by timeout: elapsed={elapsed_wall_follow:.2f}s, "
                                f"max={self.bug_max_follow_time:.2f}s, ed={ed:.2f}"
                            )

                    if leave_bug:
                        self.bug_state = 'nav'
                        self.wall_follow_start_time = None
                        self.corner_active = False
                        self.outer_escape_active = False
                        self.outer_escape_attempts = 0
                        self.outer_escape_start_pose = None
                        self.cmd_vel.linear.x = min(self.kp_v * ed, self.max_v)
                        self.cmd_vel.angular.z = float(np.clip(self.kp_w * etheta, -self.max_w, self.max_w))

                    self._log_state(
                        now, ed, etheta, closest_range, front_range,
                        leave_details=leave_details,
                        goal_path_range=goal_path_range,
                        side_wall_range=side_wall_range
                    )

                else:
                    self._log_state(
                        now, ed, etheta, closest_range, front_range,
                        goal_path_range=goal_path_range
                    )
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

    def _sector_min(self, min_angle, max_angle):
        if not getattr(self.lidar, 'ranges', None):
            return float('inf'), None

        closest_distance = float('inf')
        closest_index = None
        range_min = getattr(self.lidar, 'range_min', 0.0)
        range_max = getattr(self.lidar, 'range_max', float('inf'))

        for i, distance in enumerate(self.lidar.ranges):
            if not np.isfinite(distance):
                continue
            if distance < range_min or distance > range_max:
                continue
            angle = np.arctan2(
                np.sin(self.lidar.angle_min + i * self.lidar.angle_increment),
                np.cos(self.lidar.angle_min + i * self.lidar.angle_increment)
            )
            if self._angle_in_sector(angle, min_angle, max_angle) and distance < closest_distance:
                closest_distance = distance
                closest_index = i

        return closest_distance, closest_index

    def _angle_in_sector(self, angle, min_angle, max_angle):
        angle = np.arctan2(np.sin(angle), np.cos(angle))
        min_angle = np.arctan2(np.sin(min_angle), np.cos(min_angle))
        max_angle = np.arctan2(np.sin(max_angle), np.cos(max_angle))

        if min_angle <= max_angle:
            return min_angle <= angle <= max_angle
        return angle >= min_angle or angle <= max_angle

    def get_closest_front_obstacle_distance(self):
        distance, _ = self._sector_min(-self.front_sector, self.front_sector)
        return distance

    def _wall_follow_reference(self, fallback_range, fallback_theta, inner_angle=None, outer_angle=None):
        if inner_angle is None:
            inner_angle = self.wall_sector_inner_angle
        if outer_angle is None:
            outer_angle = self.wall_sector_outer_angle

        inner_angle = abs(inner_angle)
        outer_angle = abs(outer_angle)
        if inner_angle > outer_angle:
            inner_angle, outer_angle = outer_angle, inner_angle

        if self.wall_follow_direction == 'fwcw':
            min_angle = -outer_angle
            max_angle = -inner_angle
        else:
            min_angle = inner_angle
            max_angle = outer_angle

        wall_range, wall_index = self._sector_min(min_angle, max_angle)
        if wall_index is None:
            return float('inf'), fallback_theta, False

        wall_theta = self.lidar.angle_min + wall_index * self.lidar.angle_increment
        wall_theta = np.arctan2(np.sin(wall_theta), np.cos(wall_theta))
        return wall_range, wall_theta, True

    def _wall_side_distances(self):
        inner_angle = abs(self.wall_sector_inner_angle)
        outer_angle = abs(self.wall_sector_outer_angle)
        if inner_angle > outer_angle:
            inner_angle, outer_angle = outer_angle, inner_angle

        left_range, _ = self._sector_min(inner_angle, outer_angle)
        right_range, _ = self._sector_min(-outer_angle, -inner_angle)

        if self.wall_follow_direction == 'fwcw':
            return 'right', right_range, left_range
        return 'left', left_range, right_range

    @staticmethod
    def _format_distance(distance):
        if not np.isfinite(distance):
            return 'no_detection'
        return f'{distance:.2f}m'

    def _elapsed_seconds(self, start_time, now):
        if start_time is None:
            return 0.0
        return (now - start_time).nanoseconds * 1e-9

    def _should_leave_bug(self, ed):
        progress = ed < (self.hit_distance - self.bug_leave_margin)

        if int(self.bug_mode) == 0:
            clear_shot, clear_details = self._has_clear_shot_to_goal(ed)
            leave = progress and clear_shot
            details = {
                'progress': progress,
                'clear_shot': clear_shot,
                'goal_obs': clear_details['obstacle_distance'],
                'clear_threshold': clear_details['clear_distance'],
                'goal_angle': clear_details['goal_angle'],
                'leave': leave,
            }
            if leave:
                self.get_logger().info(
                    f"Bug0 leave: progress={progress}, clear_shot={clear_shot}, "
                    f"ed={ed:.2f}, hit={self.hit_distance:.2f}"
                )
            return leave, details

        on_mline = self._on_mline(
            self.mline_start,
            (self.xg, self.yg),
            (self.xr, self.yr),
            self.bug_leave_tol
        )
        clear_shot, clear_details = self._has_clear_shot_to_goal(ed)
        leave = progress and (on_mline or clear_shot)
        details = {
            'progress': progress,
            'on_mline': on_mline,
            'clear_shot': clear_shot,
            'goal_obs': clear_details['obstacle_distance'],
            'clear_threshold': clear_details['clear_distance'],
            'goal_angle': clear_details['goal_angle'],
            'leave': leave,
        }
        if leave:
            self.get_logger().info(
                f"Bug2 leave: progress={progress}, on_mline={on_mline}, "
                f"clear_shot={clear_shot}, ed={ed:.2f}, hit={self.hit_distance:.2f}"
            )
        return leave, details

    def _has_clear_shot_to_goal(self, goal_distance):
        goal_angle = self._goal_angle_in_robot_frame()

        obstacle_distance, _ = self._sector_min(
            goal_angle - self.bug0_clear_shot_sector,
            goal_angle + self.bug0_clear_shot_sector
        )

        goal_clear_distance = max(0.0, goal_distance - self.bug_leave_margin)
        clear_distance = min(self.bug0_clear_shot_dist, goal_clear_distance)
        clear_distance = max(self.d_wall, clear_distance)
        clear = obstacle_distance > clear_distance
        return clear, {
            'obstacle_distance': obstacle_distance,
            'clear_distance': clear_distance,
            'goal_angle': goal_angle,
        }

    def _goal_angle_in_robot_frame(self):
        goal_angle = np.arctan2(self.yg - self.yr, self.xg - self.xr) - self.theta_r
        return np.arctan2(np.sin(goal_angle), np.cos(goal_angle))

    def _log_state(
        self,
        now,
        ed,
        etheta,
        closest_range,
        front_range,
        leave_details=None,
        goal_path_range=None,
        side_wall_range=None
    ):
        if (now - self.last_state_log_time).nanoseconds * 1e-9 < 1.0:
            return
        self.last_state_log_time = now
        if goal_path_range is None:
            goal_path_range = float('inf')
        if side_wall_range is None:
            side_wall_range = float('inf')

        wall_side, followed_wall_range, opposite_wall_range = (
            self._wall_side_distances()
        )
        msg = (
            f"wall_side={wall_side} "
            f"followed_wall={self._format_distance(followed_wall_range)} "
            f"opposite_wall={self._format_distance(opposite_wall_range)} "
            f"closest={closest_range:.2f} front={front_range:.2f} "
            f"goal_path={goal_path_range:.2f} side_wall={side_wall_range:.2f} "
            f"corner={self.corner_active} wall_end={self.wall_end_active}"
        )

        if leave_details is not None:
            detail_text = ' '.join(
                f"{key}={value:.2f}" if isinstance(value, float) else f"{key}={value}"
                for key, value in leave_details.items()
            )
            msg = f"{msg} hit={self.hit_distance:.2f} {detail_text}"

        self.get_logger().info(msg)

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

    def aruco_correction_cb(self, msg):
        if not self.aruco_stop_enabled or self.system_paused:
            return

        now = self.get_clock().now()
        if self.aruco_stop_active and self.aruco_stop_until is not None and now < self.aruco_stop_until:
            return
        if self.aruco_ignore_until is not None and now < self.aruco_ignore_until:
            return

        self.cmd_vel.linear.x = 0.0
        self.cmd_vel.angular.z = 0.0
        self.aruco_stop_until = now + Duration(seconds=float(self.aruco_stop_duration))
        self.aruco_stop_active = True
        self.cmd_vel_pub.publish(self.cmd_vel)
        self.get_logger().info(
            f"ArUco odometry correction received. Stopping controller for "
            f"{self.aruco_stop_duration:.1f} seconds."
        )

    def goal_cb(self, goal):
        self.xg = goal.x
        self.yg = goal.y
        self.goal_received = True
        # Record m-line start as current robot position when goal is received
        self.mline_start = (self.xr, self.yr)
        self.bug_state = 'nav'
        self.corner_active = False
        self.wall_end_active = False
        self.wall_end_start_time = None
        self.outer_escape_active = False
        self.outer_escape_attempts = 0
        self.outer_escape_start_pose = None
        self.outer_escape_phase = 'idle'
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
            elif p.name == 'bug_mode': self.bug_mode = int(p.value)
            elif p.name == 'bug_enabled': self.bug_enabled = p.value
            elif p.name == 'bug_hit_dist': self.bug_hit_dist = p.value
            elif p.name == 'bug_leave_tol': self.bug_leave_tol = p.value
            elif p.name == 'bug_leave_margin': self.bug_leave_margin = p.value
            elif p.name == 'bug_max_follow_time': self.bug_max_follow_time = p.value
            elif p.name == 'bug0_clear_shot_dist': self.bug0_clear_shot_dist = p.value
            elif p.name == 'bug0_clear_shot_sector': self.bug0_clear_shot_sector = p.value
            elif p.name == 'front_d_safety': self.front_d_safety = p.value
            elif p.name == 'front_sector': self.front_sector = p.value
            elif p.name == 'max_v': self.max_v = p.value
            elif p.name == 'max_w': self.max_w = p.value
            elif p.name == 'corner_enter_dist': self.corner_enter_dist = p.value
            elif p.name == 'corner_exit_dist': self.corner_exit_dist = p.value
            elif p.name == 'corner_turn_w': self.corner_turn_w = p.value
            elif p.name == 'corner_linear_v': self.corner_linear_v = p.value
            elif p.name == 'wall_follow_direction': self.wall_follow_direction = p.value
            elif p.name == 'd_wall': self.d_wall = p.value
            elif p.name == 'v_wall': self.v_wall = p.value
            elif p.name == 'wall_speed_scale': self.wall_speed_scale = p.value
            elif p.name == 'kw': self.kw = p.value
            elif p.name == 'k_wall': self.k_wall = p.value
            elif p.name == 'wall_sector_inner_angle': self.wall_sector_inner_angle = p.value
            elif p.name == 'wall_sector_outer_angle': self.wall_sector_outer_angle = p.value
            elif p.name == 'wall_end_enter_dist': self.wall_end_enter_dist = p.value
            elif p.name == 'wall_end_exit_dist': self.wall_end_exit_dist = p.value
            elif p.name == 'wall_end_turn_w': self.wall_end_turn_w = p.value
            elif p.name == 'wall_end_linear_v': self.wall_end_linear_v = p.value
            elif p.name == 'wall_end_sector_inner_angle': self.wall_end_sector_inner_angle = p.value
            elif p.name == 'wall_end_sector_outer_angle': self.wall_end_sector_outer_angle = p.value
            elif p.name == 'wall_end_reacquire_dist': self.wall_end_reacquire_dist = p.value
            elif p.name == 'wall_end_min_turn_time': self.wall_end_min_turn_time = p.value
            elif p.name == 'wall_end_max_turn_time': self.wall_end_max_turn_time = p.value
            elif p.name == 'system_paused':
                self.system_paused = p.value
                if self.system_paused:
                    self.cmd_vel_pub.publish(Twist())
                    self.get_logger().info("System paused by parameter. Controller output stopped.")
                else:
                    self.get_logger().info("System pause released. Controller output active.")
            elif p.name == 'aruco_stop_enabled':
                self.aruco_stop_enabled = p.value
                if not self.aruco_stop_enabled:
                    self.aruco_stop_active = False
                    self.aruco_stop_until = None
                    self.aruco_ignore_until = None
            elif p.name == 'aruco_stop_duration': self.aruco_stop_duration = p.value
            elif p.name == 'aruco_resume_ignore_duration': self.aruco_resume_ignore_duration = p.value
        return SetParametersResult(successful=True)

    def shutdown_function(self, signum, frame):
        self.cmd_vel_pub.publish(Twist())
        if rclpy.ok():
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = controller()
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
