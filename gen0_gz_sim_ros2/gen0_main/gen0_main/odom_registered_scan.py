#!/usr/bin/env python3

import math

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


def quaternion_to_matrix(q):
    x, y, z, w = q
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        return np.eye(3, dtype=np.float32)

    s = 2.0 / n
    xx = x * x * s
    yy = y * y * s
    zz = z * z * s
    xy = x * y * s
    xz = x * z * s
    yz = y * z * s
    wx = w * x * s
    wy = w * y * s
    wz = w * z * s

    return np.array(
        [
            [1.0 - (yy + zz), xy - wz, xz + wy],
            [xy + wz, 1.0 - (xx + zz), yz - wx],
            [xz - wy, yz + wx, 1.0 - (xx + yy)],
        ],
        dtype=np.float32,
    )


def stamp_to_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class OdomRegisteredScan(Node):
    def __init__(self):
        super().__init__("odom_registered_scan")

        self.declare_parameter(
            "input_topic", "/gen0_mapping/simulated_front3d/lidar/points"
        )
        self.declare_parameter("odom_topic", "/gen0_mapping/stable_odom")
        self.declare_parameter(
            "output_topic", "/gen0_mapping/stable_registered_scan"
        )
        self.declare_parameter("output_frame", "odom")
        self.declare_parameter("lidar_xyz_in_base", [1.9, 0.0, 1.9])
        self.declare_parameter("max_odom_age", 1.0)
        self.declare_parameter("max_points", 25000)

        self.input_topic = self.get_parameter("input_topic").value
        self.odom_topic = self.get_parameter("odom_topic").value
        self.output_topic = self.get_parameter("output_topic").value
        self.output_frame = self.get_parameter("output_frame").value
        self.lidar_xyz_in_base = self.vector_parameter(
            "lidar_xyz_in_base", [1.9, 0.0, 1.9]
        )
        self.max_odom_age = float(self.get_parameter("max_odom_age").value)
        self.max_points = int(self.get_parameter("max_points").value)

        self.odom_position = None
        self.odom_rotation = None
        self.odom_stamp_sec = None

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 50)
        self.create_subscription(PointCloud2, self.input_topic, self.cloud_callback, qos)
        self.publisher = self.create_publisher(PointCloud2, self.output_topic, 10)

        self.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(
                name="intensity", offset=12, datatype=PointField.FLOAT32, count=1
            ),
        ]

        self.get_logger().info(
            f"Publishing odom-registered scan {self.input_topic} + "
            f"{self.odom_topic} -> {self.output_topic}; output_frame={self.output_frame}"
        )

    def vector_parameter(self, name, fallback):
        value = self.get_parameter(name).value
        if len(value) != 3:
            self.get_logger().warn(
                f"Parameter {name} must contain exactly 3 values; using {fallback}"
            )
            value = fallback
        return np.array(value, dtype=np.float32)

    def odom_callback(self, msg):
        self.odom_position = np.array(
            [
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z,
            ],
            dtype=np.float32,
        )
        self.odom_rotation = quaternion_to_matrix(
            [
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w,
            ]
        )
        self.odom_stamp_sec = stamp_to_seconds(msg.header.stamp)

    def cloud_callback(self, msg):
        if self.odom_position is None or self.odom_rotation is None:
            self.get_logger().warn(
                f"Waiting for odom on {self.odom_topic} before registering scans",
                throttle_duration_sec=2.0,
            )
            return

        cloud_stamp_sec = stamp_to_seconds(msg.header.stamp)
        if (
            self.max_odom_age > 0.0
            and self.odom_stamp_sec is not None
            and abs(cloud_stamp_sec - self.odom_stamp_sec) > self.max_odom_age
        ):
            self.get_logger().warn(
                "Skipping scan because odom is stale: "
                f"age={abs(cloud_stamp_sec - self.odom_stamp_sec):.2f}s",
                throttle_duration_sec=2.0,
            )
            return

        points = self.read_points(msg)
        if points.size == 0:
            return

        if self.max_points > 0 and len(points) > self.max_points:
            indices = np.linspace(0, len(points) - 1, self.max_points, dtype=np.int64)
            points = points[indices]

        xyz_base = points[:, :3] + self.lidar_xyz_in_base
        xyz_odom = xyz_base.dot(self.odom_rotation.T) + self.odom_position
        output = np.column_stack((xyz_odom, points[:, 3])).astype(np.float32)

        header = Header()
        header.stamp = msg.header.stamp
        header.frame_id = self.output_frame
        self.publisher.publish(
            point_cloud2.create_cloud(header, self.fields, output.tolist())
        )

    def read_points(self, msg):
        available_fields = {field.name for field in msg.fields}
        has_intensity = "intensity" in available_fields
        field_names = (
            ["x", "y", "z", "intensity"] if has_intensity else ["x", "y", "z"]
        )

        try:
            structured = point_cloud2.read_points(
                msg,
                field_names=field_names,
                skip_nans=False,
            )
            xyz = np.column_stack(
                (structured["x"], structured["y"], structured["z"])
            ).astype(np.float32, copy=False)
            if has_intensity:
                intensity = structured["intensity"].astype(np.float32, copy=False)
            else:
                intensity = np.zeros(xyz.shape[0], dtype=np.float32)
        except (AssertionError, ValueError, KeyError):
            return np.empty((0, 4), dtype=np.float32)

        finite = np.isfinite(xyz).all(axis=1) & np.isfinite(intensity)
        xyz = xyz[finite]
        intensity = intensity[finite]
        if xyz.size == 0:
            return np.empty((0, 4), dtype=np.float32)

        return np.column_stack((xyz, intensity)).astype(np.float32, copy=False)


def main(args=None):
    rclpy.init(args=args)
    node = OdomRegisteredScan()
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
