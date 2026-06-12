#!/usr/bin/env python3

import math
from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, Float64, Int32


class ServoLaserScan(Node):
    def __init__(self) -> None:
        super().__init__("servo_laser_scan")

        self.declare_parameter("distance_topic", "/LaserDistance")
        self.declare_parameter("angle_topic", "/ServoAngle")
        self.declare_parameter("message_type", "float32")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("frame_id", "laser_frame")
        self.declare_parameter("angle_units", "degrees")
        self.declare_parameter("servo_center_angle", 90.0)
        self.declare_parameter("angle_sign", 1.0)
        self.declare_parameter("distance_scale", 1.0)
        self.declare_parameter("angle_min", -1.57079632679)
        self.declare_parameter("angle_max", 1.57079632679)
        self.declare_parameter("angle_increment", 0.01745329252)
        self.declare_parameter("range_min", 0.03)
        self.declare_parameter("range_max", 4.0)
        self.declare_parameter("publish_rate", 10.0)
        self.declare_parameter("scan_time", 0.1)

        self.frame_id = self.get_parameter("frame_id").value
        self.angle_units = str(self.get_parameter("angle_units").value).lower()
        self.center_angle = float(self.get_parameter("servo_center_angle").value)
        self.angle_sign = float(self.get_parameter("angle_sign").value)
        self.distance_scale = float(self.get_parameter("distance_scale").value)
        self.angle_min = float(self.get_parameter("angle_min").value)
        self.angle_max = float(self.get_parameter("angle_max").value)
        self.angle_increment = float(self.get_parameter("angle_increment").value)
        self.range_min = float(self.get_parameter("range_min").value)
        self.range_max = float(self.get_parameter("range_max").value)
        self.scan_time = float(self.get_parameter("scan_time").value)

        self.bin_count = int(round((self.angle_max - self.angle_min) / self.angle_increment)) + 1
        self.ranges = [float("inf")] * self.bin_count
        self.last_angle: Optional[float] = None
        self.last_distance: Optional[float] = None

        msg_type = self._message_type()
        self.create_subscription(
            msg_type,
            self.get_parameter("distance_topic").value,
            self._distance_callback,
            10,
        )
        self.create_subscription(
            msg_type,
            self.get_parameter("angle_topic").value,
            self._angle_callback,
            10,
        )
        self.publisher = self.create_publisher(
            LaserScan, self.get_parameter("scan_topic").value, 10
        )
        self.create_timer(1.0 / float(self.get_parameter("publish_rate").value), self._publish)

    def _message_type(self):
        msg_type = str(self.get_parameter("message_type").value).lower()
        if msg_type == "float64":
            return Float64
        if msg_type == "int32":
            return Int32
        return Float32

    def _distance_callback(self, msg) -> None:
        self.last_distance = float(msg.data) * self.distance_scale
        self._store_sample()

    def _angle_callback(self, msg) -> None:
        centered = (float(msg.data) - self.center_angle) * self.angle_sign
        if self.angle_units in ("deg", "degree", "degrees"):
            self.last_angle = math.radians(centered)
        else:
            self.last_angle = centered
        self._store_sample()

    def _store_sample(self) -> None:
        if self.last_angle is None or self.last_distance is None:
            return
        if self.last_angle < self.angle_min or self.last_angle > self.angle_max:
            return

        index = int(round((self.last_angle - self.angle_min) / self.angle_increment))
        if index < 0 or index >= self.bin_count:
            return

        if self.range_min <= self.last_distance <= self.range_max:
            self.ranges[index] = self.last_distance
        else:
            self.ranges[index] = float("inf")

    def _publish(self) -> None:
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = self.frame_id
        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = self.angle_increment
        scan.time_increment = 0.0
        scan.scan_time = self.scan_time
        scan.range_min = self.range_min
        scan.range_max = self.range_max
        scan.ranges = list(self.ranges)
        self.publisher.publish(scan)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ServoLaserScan()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
