#!/usr/bin/env python3
from copy import deepcopy

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


class Nav2ProjectedMapRelay(Node):
    def __init__(self):
        super().__init__('nav2_projected_map_relay')

        self.declare_parameter('input_topic', '/projected_map')
        self.declare_parameter('output_topic', '/map')
        self.declare_parameter('output_frame', 'map')
        self.declare_parameter('unknown_as_free', False)
        self.declare_parameter('publish_period', 0.5)

        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.output_frame = self.get_parameter('output_frame').value
        self.unknown_as_free = bool(self.get_parameter('unknown_as_free').value)
        self.publish_period = float(self.get_parameter('publish_period').value)

        if self.publish_period <= 0.0:
            self.publish_period = 0.5

        output_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self.latest_map = None
        self.have_logged_first_map = False

        self.map_pub = self.create_publisher(OccupancyGrid, self.output_topic, output_qos)
        self.create_subscription(OccupancyGrid, self.input_topic, self.map_callback, 10)
        self.create_timer(self.publish_period, self.timer_callback)

        self.get_logger().info(
            f'Relaying {self.input_topic} to {self.output_topic} as frame '
            f'{self.output_frame}, unknown_as_free={self.unknown_as_free}.'
        )

    def map_callback(self, msg):
        out = deepcopy(msg)
        out.header.frame_id = self.output_frame
        out.info.map_load_time = self.get_clock().now().to_msg()

        if self.unknown_as_free:
            out.data = [0 if value < 0 else value for value in out.data]

        self.latest_map = out
        self.map_pub.publish(out)

        if not self.have_logged_first_map:
            self.have_logged_first_map = True
            self.get_logger().info(
                f'Published projected map {out.info.width}x{out.info.height} '
                f'at {out.info.resolution:.3f} m/cell on {self.output_topic}.'
            )

    def timer_callback(self):
        if self.latest_map is None:
            return
        self.latest_map.header.stamp = self.get_clock().now().to_msg()
        self.map_pub.publish(self.latest_map)


def main(args=None):
    rclpy.init(args=args)
    node = Nav2ProjectedMapRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except KeyboardInterrupt:
                pass


if __name__ == '__main__':
    main()
