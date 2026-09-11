#!/usr/bin/env python3

"""Publish the TF missing from the Gazebo odometry bridge.

Gazebo provides the authoritative /odom message in the lightweight simulator
mode, but its odom -> base_link TF is not converted by this bridge version.
This node consumes that message without republishing it and supplies the
single odom -> base_footprint transform required by Nav2.
"""

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class OdomTfBroadcaster(Node):
    def __init__(self):
        super().__init__('odom_tf_broadcaster')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')

        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        odom_topic = self.get_parameter('odom_topic').value
        self.broadcaster = TransformBroadcaster(self)
        self.create_subscription(Odometry, odom_topic, self.odom_callback, 20)
        self.get_logger().info(
            f'Broadcasting {self.odom_frame} -> {self.base_frame} from {odom_topic}')

    def odom_callback(self, message):
        transform = TransformStamped()
        transform.header = message.header
        transform.header.frame_id = self.odom_frame
        transform.child_frame_id = self.base_frame
        transform.transform.translation.x = message.pose.pose.position.x
        transform.transform.translation.y = message.pose.pose.position.y
        transform.transform.translation.z = 0.0
        transform.transform.rotation = message.pose.pose.orientation
        self.broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    node = OdomTfBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
