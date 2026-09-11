#!/usr/bin/env python3

import math

from geometry_msgs.msg import PoseArray, TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


def yaw_from_quaternion(orientation):
    siny_cosp = 2.0 * (
        orientation.w * orientation.z + orientation.x * orientation.y)
    cosy_cosp = 1.0 - 2.0 * (
        orientation.y * orientation.y + orientation.z * orientation.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class GroundTruthOdometry(Node):
    def __init__(self):
        super().__init__('ground_truth_odometry')

        self.declare_parameter('pose_topic', '/gen0_model/links/poses')
        self.declare_parameter('odom_topic', '/odom')
        # Gazebo Pose_V becomes a ROS PoseArray, which discards link names and
        # does not guarantee its ordering. A negative index enables tracking.
        self.declare_parameter('pose_index', -1)
        self.declare_parameter('initial_x', -20.6991)
        self.declare_parameter('initial_y', -22.4324)
        self.declare_parameter('initial_z', 2.85)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('publish_tf', True)

        pose_topic = self.get_parameter('pose_topic').value
        odom_topic = self.get_parameter('odom_topic').value
        self.pose_index = int(self.get_parameter('pose_index').value)
        self.initial_position = (
            float(self.get_parameter('initial_x').value),
            float(self.get_parameter('initial_y').value),
            float(self.get_parameter('initial_z').value),
        )
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_tf = bool(self.get_parameter('publish_tf').value)

        self.publisher = self.create_publisher(Odometry, odom_topic, 10)
        self.subscription = self.create_subscription(
            PoseArray, pose_topic, self.pose_callback, 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.last_position = None
        self.last_yaw = None
        self.last_stamp = None
        self.reported_layout = False
        self.reported_bad_index = False
        self.reported_initial_pose = False

        self.get_logger().info(
            f'Publishing {odom_topic} and {self.odom_frame} -> {self.base_frame} '
            f'from {pose_topic} using '
            f"{'continuous pose tracking' if self.pose_index < 0 else f'pose_index={self.pose_index}'}")

    def pose_callback(self, msg):
        if not self.reported_layout:
            self.get_logger().info(
                f'Received PoseArray with {len(msg.poses)} poses; '
                f'configured pose_index={self.pose_index}')
            self.reported_layout = True

        pose = self.select_vehicle_pose(msg.poses)
        if pose is None:
            if not self.reported_bad_index:
                self.get_logger().error(
                    f'pose_index {self.pose_index} is outside PoseArray size '
                    f'{len(msg.poses)}')
                self.reported_bad_index = True
            return

        if not self.reported_initial_pose:
            self.get_logger().info(
                f'Initial vehicle pose: ({pose.position.x:.2f}, '
                f'{pose.position.y:.2f}, {pose.position.z:.2f})')
            self.reported_initial_pose = True
        stamp = rclpy.time.Time.from_msg(msg.header.stamp)
        if stamp.nanoseconds == 0:
            stamp = self.get_clock().now()
        yaw = yaw_from_quaternion(pose.orientation)
        position = (pose.position.x, pose.position.y, pose.position.z)

        odom = Odometry()
        odom.header.stamp = stamp.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = position[0]
        odom.pose.pose.position.y = position[1]
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.z = math.sin(yaw / 2.0)
        odom.pose.pose.orientation.w = math.cos(yaw / 2.0)

        if self.last_position is not None and self.last_stamp is not None:
            dt = (stamp - self.last_stamp).nanoseconds * 1e-9
            if dt > 1e-6:
                dx = position[0] - self.last_position[0]
                dy = position[1] - self.last_position[1]
                odom.twist.twist.linear.x = (
                    dx * math.cos(yaw) + dy * math.sin(yaw)) / dt
                odom.twist.twist.linear.y = (
                    -dx * math.sin(yaw) + dy * math.cos(yaw)) / dt
                odom.twist.twist.angular.z = normalize_angle(
                    yaw - self.last_yaw) / dt

        self.publisher.publish(odom)
        if self.publish_tf:
            self.publish_transform(odom)

        self.last_position = position
        self.last_yaw = yaw
        self.last_stamp = stamp

    def select_vehicle_pose(self, poses):
        if self.pose_index >= 0:
            if self.pose_index >= len(poses):
                return None
            return poses[self.pose_index]

        reference = self.initial_position
        if self.last_position is not None:
            reference = self.last_position
        return min(
            poses,
            key=lambda pose: (
                (pose.position.x - reference[0]) ** 2
                + (pose.position.y - reference[1]) ** 2
                + (pose.position.z - reference[2]) ** 2),
            default=None)

    def publish_transform(self, odom):
        transform = TransformStamped()
        transform.header = odom.header
        transform.child_frame_id = self.base_frame
        transform.transform.translation.x = odom.pose.pose.position.x
        transform.transform.translation.y = odom.pose.pose.position.y
        transform.transform.translation.z = odom.pose.pose.position.z
        transform.transform.rotation = odom.pose.pose.orientation
        self.tf_broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    node = GroundTruthOdometry()
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
