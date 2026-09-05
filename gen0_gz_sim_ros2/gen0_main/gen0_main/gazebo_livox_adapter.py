#!/usr/bin/env python3

import math

import numpy as np
import rclpy
from livox_ros_driver2.msg import CustomMsg, CustomPoint
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


class GazeboLivoxAdapter(Node):
    def __init__(self):
        super().__init__("gazebo_livox_adapter")

        self.declare_parameter("input_topic", "/gen0_model/front3d/lidar/points")
        self.declare_parameter("output_topic", "/livox/lidar")
        self.declare_parameter("scan_rate", 10.0)
        self.declare_parameter("line_count", 64)
        self.declare_parameter("vertical_min_angle", -1.2)
        self.declare_parameter("vertical_max_angle", 0.8)
        self.declare_parameter("max_points", 8000)
        self.declare_parameter("min_range", 0.5)
        self.declare_parameter("max_range", 80.0)
        self.declare_parameter("self_filter_enabled", True)
        self.declare_parameter("lidar_xyz_in_base", [0.85, 0.0, 0.72])
        self.declare_parameter("self_filter_min_xyz", [-2.8, -1.4, -0.4])
        self.declare_parameter("self_filter_max_xyz", [2.8, 1.4, 2.9])

        self.input_topic = self.get_parameter("input_topic").value
        self.output_topic = self.get_parameter("output_topic").value
        self.scan_rate = max(0.1, float(self.get_parameter("scan_rate").value))
        self.line_count = max(1, min(255, int(self.get_parameter("line_count").value)))
        self.vertical_min_angle = float(
            self.get_parameter("vertical_min_angle").value
        )
        self.vertical_max_angle = float(
            self.get_parameter("vertical_max_angle").value
        )
        self.max_points = int(self.get_parameter("max_points").value)
        self.min_range = float(self.get_parameter("min_range").value)
        self.max_range = float(self.get_parameter("max_range").value)
        self.self_filter_enabled = bool(
            self.get_parameter("self_filter_enabled").value
        )
        self.lidar_xyz_in_base = self.vector_parameter(
            "lidar_xyz_in_base", [0.85, 0.0, 0.72]
        )
        self.self_filter_min_xyz = self.vector_parameter(
            "self_filter_min_xyz", [-2.8, -1.4, -0.4]
        )
        self.self_filter_max_xyz = self.vector_parameter(
            "self_filter_max_xyz", [2.8, 1.4, 2.9]
        )

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(PointCloud2, self.input_topic, self.cloud_callback, qos)
        self.publisher = self.create_publisher(CustomMsg, self.output_topic, 10)
        self.get_logger().info(
            f"Converting {self.input_topic} -> {self.output_topic} as Livox CustomMsg, "
            f"line_count={self.line_count}, max_points={self.max_points}, "
            f"range=[{self.min_range:.2f}, {self.max_range:.2f}], "
            f"self_filter_enabled={self.self_filter_enabled}"
        )

    def vector_parameter(self, name, fallback):
        value = self.get_parameter(name).value
        if len(value) != 3:
            self.get_logger().warn(
                f"Parameter {name} must contain exactly 3 values; using {fallback}"
            )
            value = fallback
        return np.array(value, dtype=np.float32)

    def cloud_callback(self, msg):
        points = self.read_points(msg)
        if points.size == 0:
            return

        if self.max_points > 0 and points.shape[0] > self.max_points:
            keep_indices = np.linspace(
                0, points.shape[0] - 1, self.max_points, dtype=np.int64
            )
            points = points[keep_indices]

        livox_msg = CustomMsg()
        livox_msg.header = msg.header
        livox_msg.header.frame_id = msg.header.frame_id or "livox_frame"
        livox_msg.timebase = (
            int(msg.header.stamp.sec) * 1000000000 + int(msg.header.stamp.nanosec)
        )
        livox_msg.lidar_id = 0
        livox_msg.rsvd = [0, 0, 0]
        livox_msg.points = self.to_livox_points(points, msg.width, msg.height)
        livox_msg.point_num = len(livox_msg.points)
        self.publisher.publish(livox_msg)

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
                intensity = np.full(xyz.shape[0], 100.0, dtype=np.float32)
        except (AssertionError, ValueError, KeyError):
            return np.empty((0, 5), dtype=np.float32)

        if xyz.size == 0:
            return np.empty((0, 5), dtype=np.float32)

        source_indices = np.arange(xyz.shape[0], dtype=np.float32)
        finite = np.isfinite(xyz).all(axis=1) & np.isfinite(intensity)
        xyz = xyz[finite]
        intensity = intensity[finite]
        source_indices = source_indices[finite]
        if xyz.size == 0:
            return np.empty((0, 5), dtype=np.float32)

        distance_sq = np.einsum("ij,ij->i", xyz, xyz)
        keep = distance_sq > self.min_range * self.min_range
        if self.max_range > self.min_range:
            keep &= distance_sq < self.max_range * self.max_range
        xyz = xyz[keep]
        intensity = intensity[keep]
        source_indices = source_indices[keep]
        if xyz.size == 0:
            return np.empty((0, 5), dtype=np.float32)

        if self.self_filter_enabled:
            xyz, intensity, source_indices = self.remove_self_points(
                xyz, intensity, source_indices
            )
            if xyz.size == 0:
                return np.empty((0, 5), dtype=np.float32)

        return np.column_stack((xyz, intensity, source_indices)).astype(
            np.float32, copy=False
        )

    def remove_self_points(self, xyz, intensity, source_indices):
        points_base = xyz + self.lidar_xyz_in_base
        outside_vehicle = np.any(
            (points_base < self.self_filter_min_xyz)
            | (points_base > self.self_filter_max_xyz),
            axis=1,
        )
        return (
            xyz[outside_vehicle],
            intensity[outside_vehicle],
            source_indices[outside_vehicle],
        )

    def to_livox_points(self, points, width, height):
        width = max(1, int(width))
        height = max(1, int(height))
        scan_period_ns = int(1000000000.0 / self.scan_rate)
        livox_points = []

        reflectivity_values = np.clip(points[:, 3], 0.0, 255.0).astype(np.uint8)
        for index, point in enumerate(points):
            original_index = int(point[4])

            row = self.point_line(point, original_index, width, height)
            column = original_index % width

            custom_point = CustomPoint()
            custom_point.x = float(point[0])
            custom_point.y = float(point[1])
            custom_point.z = float(point[2])
            custom_point.reflectivity = int(reflectivity_values[index])
            custom_point.tag = 0
            custom_point.line = int(row % self.line_count)
            custom_point.offset_time = int(
                scan_period_ns * column / max(width - 1, 1)
            )
            livox_points.append(custom_point)

        return livox_points

    def point_line(self, point, original_index, width, height):
        if height > 1:
            return int((original_index // width) % self.line_count)

        vertical_range = self.vertical_max_angle - self.vertical_min_angle
        if vertical_range <= 1e-6:
            return 0

        horizontal_distance = math.hypot(float(point[0]), float(point[1]))
        vertical_angle = math.atan2(float(point[2]), max(horizontal_distance, 1e-6))
        normalized = (vertical_angle - self.vertical_min_angle) / vertical_range
        line = int(round(normalized * (self.line_count - 1)))
        return max(0, min(self.line_count - 1, line))


def main(args=None):
    rclpy.init(args=args)
    node = GazeboLivoxAdapter()
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
