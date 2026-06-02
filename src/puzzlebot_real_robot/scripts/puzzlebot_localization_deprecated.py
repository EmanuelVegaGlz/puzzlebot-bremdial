#!/usr/bin/env python3

import math
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float32, Float64, Int32
from tf2_ros import TransformBroadcaster


def quaternion_from_yaw(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class PuzzlebotLocalization(Node):
    def __init__(self) -> None:
        super().__init__("puzzlebot_localization")

        self.declare_parameter("left_wheel_topic", "/VelocityEncL")
        self.declare_parameter("right_wheel_topic", "/VelocityEncR")
        self.declare_parameter("encoder_message_type", "float32")
        self.declare_parameter("encoder_units", "rad_per_sec")
        self.declare_parameter("ticks_per_revolution", 20.0)
        self.declare_parameter("wheel_radius", 0.05)
        self.declare_parameter("wheel_separation", 0.19)
        self.declare_parameter("publish_rate", 50.0)
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("odom_frame_id", "odom")
        self.declare_parameter("base_frame_id", "base_footprint")
        self.declare_parameter("publish_tf", False)

        self.left_topic = self.get_parameter("left_wheel_topic").value
        self.right_topic = self.get_parameter("right_wheel_topic").value
        self.encoder_units = str(self.get_parameter("encoder_units").value).lower()
        self.ticks_per_revolution = float(self.get_parameter("ticks_per_revolution").value)
        self.wheel_radius = float(self.get_parameter("wheel_radius").value)
        self.wheel_separation = float(self.get_parameter("wheel_separation").value)
        self.odom_frame_id = self.get_parameter("odom_frame_id").value
        self.base_frame_id = self.get_parameter("base_frame_id").value
        self.publish_tf = bool(self.get_parameter("publish_tf").value)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.left_value: Optional[float] = None
        self.right_value: Optional[float] = None
        self.prev_left_ticks: Optional[float] = None
        self.prev_right_ticks: Optional[float] = None
        self.last_time = self.get_clock().now()

        msg_type = self._message_type()
        self.create_subscription(msg_type, self.left_topic, self._left_callback, 10)
        self.create_subscription(msg_type, self.right_topic, self._right_callback, 10)
        self.odom_pub = self.create_publisher(
            Odometry, self.get_parameter("odom_topic").value, 10
        )
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

        publish_rate = float(self.get_parameter("publish_rate").value)
        self.create_timer(1.0 / publish_rate, self._update)

    def _message_type(self):
        msg_type = str(self.get_parameter("encoder_message_type").value).lower()
        if msg_type == "float64":
            return Float64
        if msg_type == "int32":
            return Int32
        return Float32

    def _left_callback(self, msg) -> None:
        self.left_value = float(msg.data)

    def _right_callback(self, msg) -> None:
        self.right_value = float(msg.data)

    def _wheel_delta(self, left: float, right: float, dt: float) -> Tuple[float, float]:
        if self.encoder_units in ("ticks", "tick", "counts", "count"):
            if self.prev_left_ticks is None or self.prev_right_ticks is None:
                self.prev_left_ticks = left
                self.prev_right_ticks = right
                return 0.0, 0.0
            left_ticks = left - self.prev_left_ticks
            right_ticks = right - self.prev_right_ticks
            self.prev_left_ticks = left
            self.prev_right_ticks = right
            scale = 2.0 * math.pi / self.ticks_per_revolution
            return left_ticks * scale, right_ticks * scale

        if self.encoder_units in ("ticks_per_sec", "ticks_per_second"):
            scale = 2.0 * math.pi / self.ticks_per_revolution
            return left * scale * dt, right * scale * dt

        if self.encoder_units == "rpm":
            scale = 2.0 * math.pi / 60.0
            return left * scale * dt, right * scale * dt

        return left * dt, right * dt

    def _update(self) -> None:
        if self.left_value is None or self.right_value is None:
            return

        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now
        if dt <= 0.0:
            return

        d_left_angle, d_right_angle = self._wheel_delta(
            self.left_value, self.right_value, dt
        )
        d_left = d_left_angle * self.wheel_radius
        d_right = d_right_angle * self.wheel_radius
        d_center = 0.5 * (d_right + d_left)
        d_theta = (d_right - d_left) / self.wheel_separation

        mid_theta = self.theta + 0.5 * d_theta
        self.x += d_center * math.cos(mid_theta)
        self.y += d_center * math.sin(mid_theta)
        self.theta = math.atan2(
            math.sin(self.theta + d_theta), math.cos(self.theta + d_theta)
        )

        linear_velocity = d_center / dt
        angular_velocity = d_theta / dt
        self._publish_odometry(now, linear_velocity, angular_velocity)

    def _publish_odometry(self, stamp, linear_velocity: float, angular_velocity: float) -> None:
        odom = Odometry()
        odom.header.stamp = stamp.to_msg()
        odom.header.frame_id = self.odom_frame_id
        odom.child_frame_id = self.base_frame_id
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = quaternion_from_yaw(self.theta)
        odom.twist.twist.linear.x = linear_velocity
        odom.twist.twist.angular.z = angular_velocity

        odom.pose.covariance[0] = 0.02
        odom.pose.covariance[7] = 0.02
        odom.pose.covariance[35] = 0.04
        odom.twist.covariance[0] = 0.02
        odom.twist.covariance[35] = 0.04
        self.odom_pub.publish(odom)

        if self.tf_broadcaster is None:
            return

        transform = TransformStamped()
        transform.header.stamp = odom.header.stamp
        transform.header.frame_id = self.odom_frame_id
        transform.child_frame_id = self.base_frame_id
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.rotation = odom.pose.pose.orientation
        self.tf_broadcaster.sendTransform(transform)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PuzzlebotLocalization()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
