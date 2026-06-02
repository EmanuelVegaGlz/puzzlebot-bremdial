#!/usr/bin/env python3

import math
from typing import Optional, Tuple

import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from std_msgs.msg import Float32, Float64, Int16, Int32, Int64, UInt16, UInt32, UInt64
from tf2_ros import TransformBroadcaster


ENCODER_TYPE_ALIASES = {
    "float32": Float32,
    "float64": Float64,
    "int16": Int16,
    "int32": Int32,
    "int64": Int64,
    "uint16": UInt16,
    "uint32": UInt32,
    "uint64": UInt64,
}

ENCODER_ROS_TYPES = {
    "std_msgs/msg/Float32": Float32,
    "std_msgs/msg/Float64": Float64,
    "std_msgs/msg/Int16": Int16,
    "std_msgs/msg/Int32": Int32,
    "std_msgs/msg/Int64": Int64,
    "std_msgs/msg/UInt16": UInt16,
    "std_msgs/msg/UInt32": UInt32,
    "std_msgs/msg/UInt64": UInt64,
}


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
        self.declare_parameter("encoder_message_type", "auto")
        self.declare_parameter("encoder_units", "rad_per_sec")
        self.declare_parameter("ticks_per_revolution", 20.0)
        self.declare_parameter("wheel_radius", 0.05)
        self.declare_parameter("wheel_separation", 0.19)
        self.declare_parameter("publish_rate", 50.0)
        self.declare_parameter("encoder_timeout", 0.5)
        self.declare_parameter("max_integration_dt", 0.25)
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("odom_frame_id", "odom")
        self.declare_parameter("base_frame_id", "base_footprint")
        self.declare_parameter("publish_tf", True)

        self.left_topic = str(self.get_parameter("left_wheel_topic").value)
        self.right_topic = str(self.get_parameter("right_wheel_topic").value)
        self.encoder_units = str(self.get_parameter("encoder_units").value).lower()
        self.ticks_per_revolution = float(self.get_parameter("ticks_per_revolution").value)
        self.wheel_radius = float(self.get_parameter("wheel_radius").value)
        self.wheel_separation = float(self.get_parameter("wheel_separation").value)
        self.encoder_timeout = float(self.get_parameter("encoder_timeout").value)
        self.max_integration_dt = float(self.get_parameter("max_integration_dt").value)
        self.odom_frame_id = str(self.get_parameter("odom_frame_id").value)
        self.base_frame_id = str(self.get_parameter("base_frame_id").value)
        self.publish_tf = bool(self.get_parameter("publish_tf").value)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.left_value: Optional[float] = None
        self.right_value: Optional[float] = None
        self.left_stamp: Optional[Time] = None
        self.right_stamp: Optional[Time] = None
        self.prev_left_ticks: Optional[float] = None
        self.prev_right_ticks: Optional[float] = None
        self.last_time = self.get_clock().now()
        self.last_encoder_warning_time = -1.0
        self.last_discovery_warning_time = -1.0

        self.encoder_message_type = str(
            self.get_parameter("encoder_message_type").value
        ).lower()
        self.left_subscription = None
        self.right_subscription = None
        self.left_subscription_type: Optional[str] = None
        self.right_subscription_type: Optional[str] = None
        self._ensure_encoder_subscriptions()

        self.odom_pub = self.create_publisher(
            Odometry, str(self.get_parameter("odom_topic").value), 10
        )
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

        publish_rate = float(self.get_parameter("publish_rate").value)
        self.create_timer(1.0 / publish_rate, self._update)

        self.get_logger().info(
            "Publishing odom from encoders: "
            f"left={self.left_topic}, right={self.right_topic}, "
            f"message_type={self.encoder_message_type}, units={self.encoder_units}, "
            f"frame={self.odom_frame_id}, "
            f"child={self.base_frame_id}, publish_tf={self.publish_tf}"
        )

    def _ensure_encoder_subscriptions(self) -> None:
        if self.left_subscription is None:
            self.left_subscription = self._create_encoder_subscription(
                self.left_topic, self._left_callback
            )
        if self.right_subscription is None:
            self.right_subscription = self._create_encoder_subscription(
                self.right_topic, self._right_callback
            )

    def _create_encoder_subscription(self, topic: str, callback):
        msg_type, type_name = self._resolve_encoder_type(topic)
        if msg_type is None:
            return None

        try:
            subscription = self.create_subscription(
                msg_type, topic, callback, qos_profile_sensor_data
            )
        except Exception as exc:
            self._warn_discovery(
                f"Could not subscribe to {topic} as {type_name}: {exc}"
            )
            return None

        if topic == self.left_topic:
            self.left_subscription_type = type_name
        if topic == self.right_topic:
            self.right_subscription_type = type_name

        self.get_logger().info(f"Subscribed to {topic} as {type_name}")
        return subscription

    def _resolve_encoder_type(self, topic: str):
        if self.encoder_message_type in ENCODER_TYPE_ALIASES:
            msg_type = ENCODER_TYPE_ALIASES[self.encoder_message_type]
            return msg_type, self.encoder_message_type

        if self.encoder_message_type != "auto":
            self.get_logger().warn(
                f"Unsupported encoder_message_type '{self.encoder_message_type}'. "
                "Falling back to auto."
            )
            self.encoder_message_type = "auto"

        topic_types = self._discover_topic_types(topic)
        for topic_type in topic_types:
            if topic_type in ENCODER_ROS_TYPES:
                return ENCODER_ROS_TYPES[topic_type], topic_type

        if topic_types:
            self._warn_discovery(
                f"Topic {topic} is publishing unsupported type(s): "
                + ", ".join(sorted(topic_types))
            )
        return None, None

    def _discover_topic_types(self, topic: str) -> set:
        normalized = self._normalize_topic(topic)
        topic_types = set()

        for topic_name, names_and_types in self.get_topic_names_and_types():
            if self._normalize_topic(topic_name) == normalized:
                topic_types.update(names_and_types)

        try:
            for info in self.get_publishers_info_by_topic(topic):
                topic_type = getattr(info, "topic_type", None)
                if topic_type:
                    topic_types.add(topic_type)
        except Exception:
            pass

        return topic_types

    def _normalize_topic(self, topic: str) -> str:
        return "/" + topic.lstrip("/")

    def _warn_discovery(self, message: str) -> None:
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        if now_sec - self.last_discovery_warning_time < 5.0:
            return
        self.last_discovery_warning_time = now_sec
        self.get_logger().warn(message)

    def _left_callback(self, msg) -> None:
        self.left_value = float(msg.data)
        self.left_stamp = self.get_clock().now()

    def _right_callback(self, msg) -> None:
        self.right_value = float(msg.data)
        self.right_stamp = self.get_clock().now()

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
        self._ensure_encoder_subscriptions()

        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now
        if dt <= 0.0:
            return
        dt = min(dt, self.max_integration_dt)

        if not self._encoders_ready(now):
            self._publish_odometry(now, 0.0, 0.0)
            self._warn_missing_encoders(now)
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

    def _encoders_ready(self, now: Time) -> bool:
        if self.left_value is None or self.right_value is None:
            return False

        if self.encoder_timeout <= 0.0:
            return True

        left_age = (now - self.left_stamp).nanoseconds * 1e-9 if self.left_stamp else math.inf
        right_age = (now - self.right_stamp).nanoseconds * 1e-9 if self.right_stamp else math.inf
        return left_age <= self.encoder_timeout and right_age <= self.encoder_timeout

    def _warn_missing_encoders(self, now: Time) -> None:
        now_sec = now.nanoseconds * 1e-9
        if now_sec - self.last_encoder_warning_time < 5.0:
            return
        self.last_encoder_warning_time = now_sec

        missing = []
        if self.left_subscription is None:
            missing.append(f"{self.left_topic} (no compatible publisher discovered)")
        elif self.left_value is None:
            missing.append(self.left_topic)
        if self.right_subscription is None:
            missing.append(f"{self.right_topic} (no compatible publisher discovered)")
        elif self.right_value is None:
            missing.append(self.right_topic)

        if missing:
            self.get_logger().warn(
                "Waiting for encoder topic(s): "
                + ", ".join(missing)
                + ". Publishing stationary /odom and TF meanwhile."
            )
            return

        self.get_logger().warn(
            "Encoder data is stale. Publishing stationary /odom and TF until "
            "fresh wheel data arrives."
        )

    def _publish_odometry(
        self, stamp, linear_velocity: float, angular_velocity: float
    ) -> None:
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
        transform.transform.translation.z = 0.0
        transform.transform.rotation = odom.pose.pose.orientation
        self.tf_broadcaster.sendTransform(transform)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PuzzlebotLocalization()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
