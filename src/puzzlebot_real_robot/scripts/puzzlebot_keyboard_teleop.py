#!/usr/bin/env python3

import select
import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


HELP = """
Puzzlebot keyboard teleop
-------------------------
w/x : increase/decrease linear speed
a/d : increase/decrease angular speed
s   : stop
space: stop
q   : quit
"""


class PuzzlebotKeyboardTeleop(Node):
    def __init__(self) -> None:
        super().__init__("puzzlebot_keyboard_teleop")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("linear_step", 0.03)
        self.declare_parameter("angular_step", 0.15)
        self.declare_parameter("max_linear_speed", 0.25)
        self.declare_parameter("max_angular_speed", 1.2)
        self.declare_parameter("publish_rate", 10.0)

        self.linear_step = float(self.get_parameter("linear_step").value)
        self.angular_step = float(self.get_parameter("angular_step").value)
        self.max_linear = float(self.get_parameter("max_linear_speed").value)
        self.max_angular = float(self.get_parameter("max_angular_speed").value)
        self.linear = 0.0
        self.angular = 0.0

        self.publisher = self.create_publisher(
            Twist, self.get_parameter("cmd_vel_topic").value, 10
        )
        self.create_timer(1.0 / float(self.get_parameter("publish_rate").value), self.publish)

    def apply_key(self, key: str) -> bool:
        if key == "w":
            self.linear = min(self.max_linear, self.linear + self.linear_step)
        elif key == "x":
            self.linear = max(-self.max_linear, self.linear - self.linear_step)
        elif key == "a":
            self.angular = min(self.max_angular, self.angular + self.angular_step)
        elif key == "d":
            self.angular = max(-self.max_angular, self.angular - self.angular_step)
        elif key in ("s", " "):
            self.linear = 0.0
            self.angular = 0.0
        elif key == "q":
            self.linear = 0.0
            self.angular = 0.0
            self.publish()
            return False
        return True

    def publish(self) -> None:
        msg = Twist()
        msg.linear.x = self.linear
        msg.angular.z = self.angular
        self.publisher.publish(msg)


def read_key(timeout: float = 0.1) -> str:
    ready, _, _ = select.select([sys.stdin], [], [], timeout)
    if ready:
        return sys.stdin.read(1)
    return ""


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PuzzlebotKeyboardTeleop()
    if not sys.stdin.isatty():
        node.get_logger().error(
            "Keyboard teleop needs an interactive terminal. Run it in a separate "
            "terminal or launch with teleop_use_xterm:=true."
        )
        node.destroy_node()
        rclpy.shutdown()
        return

    old_settings = termios.tcgetattr(sys.stdin)

    try:
        tty.setcbreak(sys.stdin.fileno())
        print(HELP)
        running = True
        while rclpy.ok() and running:
            rclpy.spin_once(node, timeout_sec=0.0)
            key = read_key()
            if key:
                running = node.apply_key(key)
    finally:
        node.linear = 0.0
        node.angular = 0.0
        node.publish()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
