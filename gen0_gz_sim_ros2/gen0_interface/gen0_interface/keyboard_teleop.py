#!/usr/bin/env python3

import select
import sys
import termios
import time
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


HELP_TEXT = """
Keyboard mapping control

Hold:
  w       forward
  a / d   forward left / right
  s       reverse
  space   stop
  h       show this help
  q       quit

This is a hold-to-run controller: if no key is received within the timeout,
it publishes zero velocity.
"""


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__("gen0_keyboard_teleop")

        self.declare_parameter("cmd_vel_topic", "/control/cmd_vel")
        self.declare_parameter("linear_speed", 0.20)
        self.declare_parameter("turn_linear_speed", 0.16)
        self.declare_parameter("reverse_speed", 0.12)
        self.declare_parameter("angular_speed", 0.25)
        self.declare_parameter("key_timeout", 0.35)
        self.declare_parameter("publish_rate", 10.0)

        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self.linear_speed = float(self.get_parameter("linear_speed").value)
        self.turn_linear_speed = float(self.get_parameter("turn_linear_speed").value)
        self.reverse_speed = float(self.get_parameter("reverse_speed").value)
        self.angular_speed = float(self.get_parameter("angular_speed").value)
        self.key_timeout = float(self.get_parameter("key_timeout").value)
        self.publish_rate = float(self.get_parameter("publish_rate").value)

        self.publisher = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.last_twist = Twist()
        self.last_key_time = 0.0
        self.running = True

        self.get_logger().info(
            f"Keyboard teleop publishing {self.cmd_vel_topic}; "
            f"linear={self.linear_speed:.2f} m/s, turn={self.angular_speed:.2f} rad/s"
        )

    def run(self):
        if not sys.stdin.isatty():
            self.get_logger().error("keyboard_teleop must be run in an interactive terminal")
            return

        print(HELP_TEXT)
        settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setcbreak(sys.stdin.fileno())
            period = 1.0 / max(self.publish_rate, 1.0)
            while rclpy.ok() and self.running:
                key = self.read_key(period)
                if key:
                    self.handle_key(key)

                if time.monotonic() - self.last_key_time > self.key_timeout:
                    self.last_twist = Twist()

                self.publisher.publish(self.last_twist)
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
            self.publish_stop()

    def read_key(self, timeout):
        ready, _, _ = select.select([sys.stdin], [], [], timeout)
        if not ready:
            return ""
        return sys.stdin.read(1)

    def handle_key(self, key):
        if key == "q":
            self.running = False
            self.last_twist = Twist()
            return

        if key == "h":
            print(HELP_TEXT)
            return

        twist = Twist()
        if key == "w":
            twist.linear.x = self.linear_speed
        elif key == "a":
            twist.linear.x = self.turn_linear_speed
            twist.angular.z = self.angular_speed
        elif key == "d":
            twist.linear.x = self.turn_linear_speed
            twist.angular.z = -self.angular_speed
        elif key == "s":
            twist.linear.x = -self.reverse_speed
        elif key == " ":
            twist = Twist()
        else:
            return

        self.last_twist = twist
        self.last_key_time = time.monotonic()

    def publish_stop(self, count=5):
        stop = Twist()
        for _ in range(count):
            self.publisher.publish(stop)
            time.sleep(0.02)


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except (Exception, KeyboardInterrupt):
            pass


if __name__ == "__main__":
    main()
