#!/usr/bin/env python3

from typing import Optional, Tuple

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster


class PuzzlebotJointStatePublisher(Node):
    def __init__(self) -> None:
        super().__init__("puzzlebot_joint_state_publisher")

        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("wheel_radius", 0.05)
        self.declare_parameter("wheel_separation", 0.19)
        self.declare_parameter("left_joint_name", "wheel_left_joint")
        self.declare_parameter("right_joint_name", "wheel_right_joint")
        self.declare_parameter("odom_frame_id", "odom")
        self.declare_parameter("base_frame_id", "base_footprint")
        self.declare_parameter("publish_tf", True)

        self.wheel_radius = float(self.get_parameter("wheel_radius").value)
        self.wheel_separation = float(self.get_parameter("wheel_separation").value)
        self.left_joint_name = self.get_parameter("left_joint_name").value
        self.right_joint_name = self.get_parameter("right_joint_name").value
        self.odom_frame_id = self.get_parameter("odom_frame_id").value
        self.base_frame_id = self.get_parameter("base_frame_id").value
        self.publish_tf = bool(self.get_parameter("publish_tf").value)

        self.last_odom: Optional[Odometry] = None
        self.left_position = 0.0
        self.right_position = 0.0
        self.left_velocity = 0.0
        self.right_velocity = 0.0
        self.last_stamp = None

        self.create_subscription(
            Odometry,
            self.get_parameter("odom_topic").value,
            self._odom_callback,
            10,
        )
        self.publisher = self.create_publisher(JointState, "/joint_states", 10)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

    def _odom_callback(self, msg: Odometry) -> None:
        stamp = Time.from_msg(msg.header.stamp)
        if self.last_stamp is None:
            self.last_stamp = stamp
            self.last_odom = msg
            self._publish_joint_state(msg.header.stamp)
            self._publish_tf(msg)
            return

        dt = (stamp - self.last_stamp).nanoseconds * 1e-9
        self.last_stamp = stamp
        if dt <= 0.0:
            return

        self.left_velocity, self.right_velocity = self._inverse_kinematics(msg)
        self.left_position += self.left_velocity * dt
        self.right_position += self.right_velocity * dt
        self.last_odom = msg

        self._publish_joint_state(msg.header.stamp)
        self._publish_tf(msg)

    def _inverse_kinematics(self, odom: Odometry) -> Tuple[float, float]:
        linear = odom.twist.twist.linear.x
        angular = odom.twist.twist.angular.z
        half_track = 0.5 * self.wheel_separation
        left_linear = linear - angular * half_track
        right_linear = linear + angular * half_track
        return left_linear / self.wheel_radius, right_linear / self.wheel_radius

    def _publish_joint_state(self, stamp) -> None:
        joint_state = JointState()
        joint_state.header.stamp = stamp
        joint_state.name = [self.left_joint_name, self.right_joint_name]
        joint_state.position = [self.left_position, self.right_position]
        joint_state.velocity = [self.left_velocity, self.right_velocity]
        self.publisher.publish(joint_state)

    def _publish_tf(self, odom: Odometry) -> None:
        if self.tf_broadcaster is None:
            return

        transform = TransformStamped()
        transform.header.stamp = odom.header.stamp
        transform.header.frame_id = odom.header.frame_id or self.odom_frame_id
        transform.child_frame_id = odom.child_frame_id or self.base_frame_id
        transform.transform.translation.x = odom.pose.pose.position.x
        transform.transform.translation.y = odom.pose.pose.position.y
        transform.transform.translation.z = odom.pose.pose.position.z
        transform.transform.rotation = odom.pose.pose.orientation
        self.tf_broadcaster.sendTransform(transform)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PuzzlebotJointStatePublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
