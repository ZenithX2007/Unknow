#!/usr/bin/env python3

import math

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


def normalize_quaternion(q):
    norm = math.sqrt(float(np.dot(q, q)))
    if norm < 1e-12:
        return np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
    return q / norm


def quaternion_conjugate(q):
    return np.array([-q[0], -q[1], -q[2], q[3]], dtype=np.float64)


def quaternion_multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return np.array(
        [
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz,
        ],
        dtype=np.float64,
    )


def rotate_vector(q, vector):
    rotated = quaternion_multiply(
        quaternion_multiply(q, np.array([vector[0], vector[1], vector[2], 0.0])),
        quaternion_conjugate(q),
    )
    return rotated[:3]


def yaw_from_quaternion(q):
    x, y, z, w = q
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def shortest_angle_delta(angle, previous):
    return math.atan2(math.sin(angle - previous), math.cos(angle - previous))


def stamp_to_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class StableOdom(Node):
    def __init__(self):
        super().__init__("stable_odom")

        self.declare_parameter("input_topic", "/odom")
        self.declare_parameter("output_topic", "/gen0_mapping/stable_odom")
        self.declare_parameter("odom_frame_id", "odom")
        self.declare_parameter("base_frame_id", "base_link")
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("compute_twist_from_pose", True)

        self.input_topic = self.get_parameter("input_topic").value
        self.output_topic = self.get_parameter("output_topic").value
        self.odom_frame_id = self.get_parameter("odom_frame_id").value
        self.base_frame_id = self.get_parameter("base_frame_id").value
        self.publish_tf = bool(self.get_parameter("publish_tf").value)
        self.compute_twist_from_pose = bool(
            self.get_parameter("compute_twist_from_pose").value
        )

        self.origin_position = None
        self.origin_orientation_inverse = None
        self.last_position = None
        self.last_yaw = None
        self.last_stamp_sec = None

        self.publisher = self.create_publisher(Odometry, self.output_topic, 20)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(Odometry, self.input_topic, self.odom_callback, 50)

        self.get_logger().info(
            f"Publishing normalized odom {self.input_topic} -> {self.output_topic}; "
            f"frame={self.odom_frame_id}, child={self.base_frame_id}, "
            f"publish_tf={self.publish_tf}"
        )

    def odom_callback(self, msg):
        position = np.array(
            [
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z,
            ],
            dtype=np.float64,
        )
        orientation = normalize_quaternion(
            np.array(
                [
                    msg.pose.pose.orientation.x,
                    msg.pose.pose.orientation.y,
                    msg.pose.pose.orientation.z,
                    msg.pose.pose.orientation.w,
                ],
                dtype=np.float64,
            )
        )

        if self.origin_position is None:
            self.origin_position = position
            self.origin_orientation_inverse = quaternion_conjugate(orientation)
            self.get_logger().info(
                "Stable odom origin set from first input pose: "
                f"x={position[0]:.2f}, y={position[1]:.2f}, "
                f"yaw={yaw_from_quaternion(orientation):.3f}"
            )

        relative_position = rotate_vector(
            self.origin_orientation_inverse, position - self.origin_position
        )
        relative_orientation = normalize_quaternion(
            quaternion_multiply(self.origin_orientation_inverse, orientation)
        )

        out = Odometry()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.odom_frame_id
        out.child_frame_id = self.base_frame_id
        out.pose = msg.pose
        out.pose.pose.position.x = float(relative_position[0])
        out.pose.pose.position.y = float(relative_position[1])
        out.pose.pose.position.z = float(relative_position[2])
        out.pose.pose.orientation.x = float(relative_orientation[0])
        out.pose.pose.orientation.y = float(relative_orientation[1])
        out.pose.pose.orientation.z = float(relative_orientation[2])
        out.pose.pose.orientation.w = float(relative_orientation[3])
        out.twist = msg.twist

        if self.compute_twist_from_pose:
            self.fill_twist_from_pose(out, relative_position, relative_orientation)

        self.publisher.publish(out)
        if self.publish_tf:
            self.tf_broadcaster.sendTransform(self.to_transform(out))

    def fill_twist_from_pose(self, out, position, orientation):
        stamp_sec = stamp_to_seconds(out.header.stamp)
        yaw = yaw_from_quaternion(orientation)
        if self.last_position is not None and self.last_stamp_sec is not None:
            dt = stamp_sec - self.last_stamp_sec
            if dt > 1e-4:
                delta = position - self.last_position
                cos_yaw = math.cos(yaw)
                sin_yaw = math.sin(yaw)
                out.twist.twist.linear.x = float(
                    (cos_yaw * delta[0] + sin_yaw * delta[1]) / dt
                )
                out.twist.twist.linear.y = float(
                    (-sin_yaw * delta[0] + cos_yaw * delta[1]) / dt
                )
                out.twist.twist.linear.z = float(delta[2] / dt)
                out.twist.twist.angular.z = float(
                    shortest_angle_delta(yaw, self.last_yaw) / dt
                )

        self.last_position = position.copy()
        self.last_yaw = yaw
        self.last_stamp_sec = stamp_sec

    def to_transform(self, odom):
        transform = TransformStamped()
        transform.header.stamp = odom.header.stamp
        transform.header.frame_id = self.odom_frame_id
        transform.child_frame_id = self.base_frame_id
        transform.transform.translation.x = odom.pose.pose.position.x
        transform.transform.translation.y = odom.pose.pose.position.y
        transform.transform.translation.z = odom.pose.pose.position.z
        transform.transform.rotation = odom.pose.pose.orientation
        return transform


def main(args=None):
    rclpy.init(args=args)
    node = StableOdom()
    try:
        rclpy.spin(node)
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
