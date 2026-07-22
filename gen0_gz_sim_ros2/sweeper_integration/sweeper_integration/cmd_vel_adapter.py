#!/usr/bin/env python3

import copy

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node


def clamp(value, lower, upper):
    return max(lower, min(value, upper))


class CmdVelAdapter(Node):
    def __init__(self):
        super().__init__('cmd_vel_adapter')

        self.declare_parameter('input_topic', '/cmd_vel')
        self.declare_parameter('output_topic', '/control/cmd_vel')
        self.declare_parameter('max_linear_speed', 2.0)
        self.declare_parameter('max_angular_speed', 0.4)
        self.declare_parameter('min_linear_speed_for_rotation', 0.0)
        self.declare_parameter('linear_deadband', 0.02)
        self.declare_parameter('angular_deadband', 0.02)
        self.declare_parameter('command_timeout', 0.5)
        self.declare_parameter('publish_rate', 20.0)

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        publish_rate = float(self.get_parameter('publish_rate').value)

        self.max_linear_speed = float(self.get_parameter('max_linear_speed').value)
        self.max_angular_speed = float(self.get_parameter('max_angular_speed').value)
        self.min_linear_speed_for_rotation = max(
            0.0,
            float(self.get_parameter('min_linear_speed_for_rotation').value))
        self.linear_deadband = max(
            0.0, float(self.get_parameter('linear_deadband').value))
        self.angular_deadband = max(
            0.0, float(self.get_parameter('angular_deadband').value))
        self.command_timeout = float(self.get_parameter('command_timeout').value)
        self.last_command_time = None
        self.latest_command = Twist()

        self.publisher = self.create_publisher(Twist, output_topic, 10)
        self.subscription = self.create_subscription(
            Twist, input_topic, self.command_callback, 10)
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_command)

        self.get_logger().info(
            f'Adapting {input_topic} -> {output_topic}; '
            f'limits={self.max_linear_speed:.2f} m/s, '
            f'{self.max_angular_speed:.2f} rad/s; '
            f'rotation crawl={self.min_linear_speed_for_rotation:.2f} m/s')

    def command_callback(self, msg):
        command = copy.deepcopy(msg)
        command.linear.x = clamp(
            command.linear.x, -self.max_linear_speed, self.max_linear_speed)
        command.angular.z = clamp(
            command.angular.z, -self.max_angular_speed, self.max_angular_speed)
        if (
            self.min_linear_speed_for_rotation > 0.0 and
            abs(command.linear.x) < self.linear_deadband and
            abs(command.angular.z) > self.angular_deadband
        ):
            # The Gen0 four-wheel steering interface cannot execute a pure
            # in-place rotation because wheel speeds stay at zero. Convert
            # Nav2 spin/recovery commands into a slow forward arc.
            command.linear.x = min(
                self.min_linear_speed_for_rotation,
                self.max_linear_speed)
        self.latest_command = command
        self.last_command_time = self.get_clock().now()

    def publish_command(self):
        if self.last_command_time is None:
            self.publisher.publish(Twist())
            return

        age = (self.get_clock().now() - self.last_command_time).nanoseconds * 1e-9
        if age > self.command_timeout:
            self.publisher.publish(Twist())
        else:
            self.publisher.publish(self.latest_command)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.publisher.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
