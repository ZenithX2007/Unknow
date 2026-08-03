#!/usr/bin/env python3
from array import array
from copy import deepcopy

import numpy as np
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
        self.declare_parameter('fixed_geometry', False)
        self.declare_parameter('fixed_origin_x', -25.0)
        self.declare_parameter('fixed_origin_y', -25.0)
        self.declare_parameter('fixed_width', 500)
        self.declare_parameter('fixed_height', 500)
        self.declare_parameter('fixed_resolution', 0.10)

        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.output_frame = self.get_parameter('output_frame').value
        self.unknown_as_free = bool(self.get_parameter('unknown_as_free').value)
        self.publish_period = float(self.get_parameter('publish_period').value)
        self.fixed_geometry = bool(self.get_parameter('fixed_geometry').value)
        self.fixed_origin_x = float(self.get_parameter('fixed_origin_x').value)
        self.fixed_origin_y = float(self.get_parameter('fixed_origin_y').value)
        self.fixed_width = int(self.get_parameter('fixed_width').value)
        self.fixed_height = int(self.get_parameter('fixed_height').value)
        self.fixed_resolution = float(self.get_parameter('fixed_resolution').value)

        if self.publish_period <= 0.0:
            self.publish_period = 0.5
        if self.fixed_width <= 0 or self.fixed_height <= 0:
            self.fixed_geometry = False
        if self.fixed_resolution <= 0.0:
            self.fixed_resolution = 0.0

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
            f'{self.output_frame}, unknown_as_free={self.unknown_as_free}, '
            f'fixed_geometry={self.fixed_geometry}.'
        )

    def map_callback(self, msg):
        out = self.to_fixed_geometry(msg) if self.fixed_geometry else deepcopy(msg)
        out.header.frame_id = self.output_frame
        out.info.map_load_time = self.get_clock().now().to_msg()

        if self.unknown_as_free and not self.fixed_geometry:
            out.data = [0 if value < 0 else value for value in out.data]

        self.latest_map = out
        self.map_pub.publish(out)

        if not self.have_logged_first_map:
            self.have_logged_first_map = True
            self.get_logger().info(
                f'Published projected map {out.info.width}x{out.info.height} '
                f'at {out.info.resolution:.3f} m/cell on {self.output_topic}; '
                f'origin=({out.info.origin.position.x:.2f}, '
                f'{out.info.origin.position.y:.2f}).'
            )

    def to_fixed_geometry(self, msg):
        resolution = self.fixed_resolution if self.fixed_resolution > 0.0 else float(msg.info.resolution)
        out = OccupancyGrid()
        out.header = deepcopy(msg.header)
        out.info = deepcopy(msg.info)
        out.info.resolution = resolution
        out.info.width = self.fixed_width
        out.info.height = self.fixed_height
        out.info.origin.position.x = self.fixed_origin_x
        out.info.origin.position.y = self.fixed_origin_y
        out.info.origin.position.z = 0.0
        out.info.origin.orientation.w = 1.0

        fill_value = 0 if self.unknown_as_free else -1
        canvas = np.full((self.fixed_height, self.fixed_width), fill_value, dtype=np.int8)

        input_resolution = float(msg.info.resolution)
        input_width = int(msg.info.width)
        input_height = int(msg.info.height)
        if input_resolution <= 0.0 or input_width <= 0 or input_height <= 0:
            out.data = array('b', canvas.ravel(order='C'))
            return out

        if abs(input_resolution - resolution) > 1e-6:
            self.get_logger().warn(
                'Skipping projected map update with resolution '
                f'{input_resolution:.4f}; fixed output resolution is {resolution:.4f}.'
            )
            out.data = array('b', canvas.ravel(order='C'))
            return out

        src = np.asarray(msg.data, dtype=np.int8).reshape((input_height, input_width))
        if self.unknown_as_free:
            src = np.where(src < 0, 0, src).astype(np.int8, copy=False)

        dst_x0 = int(round((float(msg.info.origin.position.x) - self.fixed_origin_x) / resolution))
        dst_y0 = int(round((float(msg.info.origin.position.y) - self.fixed_origin_y) / resolution))
        dst_x1 = dst_x0 + input_width
        dst_y1 = dst_y0 + input_height

        copy_dst_x0 = max(0, dst_x0)
        copy_dst_y0 = max(0, dst_y0)
        copy_dst_x1 = min(self.fixed_width, dst_x1)
        copy_dst_y1 = min(self.fixed_height, dst_y1)

        if copy_dst_x0 >= copy_dst_x1 or copy_dst_y0 >= copy_dst_y1:
            self.get_logger().warn(
                'Projected map is outside the fixed Nav2 map canvas; '
                'increase fixed_origin/fixed_width/fixed_height.'
            )
            out.data = array('b', canvas.ravel(order='C'))
            return out

        src_x0 = copy_dst_x0 - dst_x0
        src_y0 = copy_dst_y0 - dst_y0
        src_x1 = src_x0 + (copy_dst_x1 - copy_dst_x0)
        src_y1 = src_y0 + (copy_dst_y1 - copy_dst_y0)
        canvas[copy_dst_y0:copy_dst_y1, copy_dst_x0:copy_dst_x1] = src[src_y0:src_y1, src_x0:src_x1]

        out.data = array('b', canvas.ravel(order='C'))
        return out

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
