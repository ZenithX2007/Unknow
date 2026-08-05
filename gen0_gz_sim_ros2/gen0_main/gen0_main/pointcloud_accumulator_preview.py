#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2


class PointCloudAccumulatorPreview(Node):
    def __init__(self):
        super().__init__("pointcloud_accumulator_preview")

        self.declare_parameter("input_topic", "/gen0_mapping/cloud_registered")
        self.declare_parameter("output_topic", "/gen0_mapping/rviz/fast_lio_map")
        self.declare_parameter("max_points", 220000)
        self.declare_parameter("voxel_size", 0.12)
        self.declare_parameter("adaptive_voxel_size", True)
        self.declare_parameter("voxel_growth_factor", 1.5)
        self.declare_parameter("max_voxel_size", 1.5)
        self.declare_parameter("pre_sample_factor", 3)
        self.declare_parameter("color_mode", "z")
        self.declare_parameter("color_min", 0.0)
        self.declare_parameter("color_max", 0.0)
        self.declare_parameter("publish_period", 1.0)

        self.input_topic = self.get_parameter("input_topic").value
        self.output_topic = self.get_parameter("output_topic").value
        self.max_points = int(self.get_parameter("max_points").value)
        self.voxel_size = max(1e-3, float(self.get_parameter("voxel_size").value))
        self.current_voxel_size = self.voxel_size
        self.adaptive_voxel_size = bool(
            self.get_parameter("adaptive_voxel_size").value
        )
        self.voxel_growth_factor = max(
            1.05, float(self.get_parameter("voxel_growth_factor").value)
        )
        self.max_voxel_size = max(
            self.voxel_size, float(self.get_parameter("max_voxel_size").value)
        )
        self.pre_sample_factor = max(1, int(self.get_parameter("pre_sample_factor").value))
        self.color_mode = str(self.get_parameter("color_mode").value).lower()
        self.color_min = float(self.get_parameter("color_min").value)
        self.color_max = float(self.get_parameter("color_max").value)
        publish_period = float(self.get_parameter("publish_period").value)

        self.voxels = {}
        self.latest_header = None
        self.scan_count = 0
        self.hard_cap_warning_emitted = False

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(PointCloud2, self.input_topic, self.cloud_callback, qos)
        self.publisher = self.create_publisher(PointCloud2, self.output_topic, 1)
        self.create_timer(max(publish_period, 0.1), self.publish_map)

        self.get_logger().info(
            f"Accumulating {self.input_topic} -> {self.output_topic}, "
            f"max_points={self.max_points}, voxel_size={self.voxel_size}, "
            f"adaptive_voxels={self.adaptive_voxel_size}, "
            f"color_mode={self.color_mode}"
        )

    def cloud_callback(self, msg):
        points = self.read_preview_points(msg)
        if points.size == 0:
            return

        voxel_keys = np.floor(
            points[:, 0:3] / self.current_voxel_size
        ).astype(np.int32)
        _, unique_indices = np.unique(voxel_keys, axis=0, return_index=True)
        voxel_keys = voxel_keys[unique_indices]
        points = points[unique_indices]

        for key, point in zip(map(tuple, voxel_keys), points):
            self.voxels[key] = (
                float(point[0]),
                float(point[1]),
                float(point[2]),
                float(point[3]),
            )

        self.latest_header = msg.header
        self.scan_count += 1
        self.compact_if_needed()

    def read_preview_points(self, msg):
        total_points = int(msg.width * msg.height)
        if total_points <= 0:
            return np.empty((0, 4), dtype=np.float32)

        max_input_points = (
            self.max_points * self.pre_sample_factor if self.max_points > 0 else total_points
        )
        if total_points > max_input_points:
            uvs = np.linspace(0, total_points - 1, max_input_points, dtype=np.int64)
        else:
            uvs = None

        available_fields = {field.name for field in msg.fields}
        has_intensity = "intensity" in available_fields
        field_names = ["x", "y", "z", "intensity"] if has_intensity else ["x", "y", "z"]

        try:
            structured = point_cloud2.read_points(
                msg,
                field_names=field_names,
                skip_nans=False,
                uvs=uvs,
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

        if xyz.size == 0:
            return np.empty((0, 4), dtype=np.float32)

        finite = np.isfinite(xyz).all(axis=1) & np.isfinite(intensity)
        xyz = xyz[finite]
        intensity = intensity[finite]
        if xyz.size == 0:
            return np.empty((0, 4), dtype=np.float32)

        return np.column_stack((xyz, intensity)).astype(np.float32, copy=False)

    def compact_if_needed(self):
        if self.max_points <= 0 or len(self.voxels) <= self.max_points:
            return

        previous_count = len(self.voxels)
        previous_voxel_size = self.current_voxel_size

        # Preserve global coverage when the preview reaches its memory budget.
        # Removing arbitrary old keys made previously mapped areas disappear.
        while (
            len(self.voxels) > self.max_points
            and self.adaptive_voxel_size
            and self.current_voxel_size < self.max_voxel_size
        ):
            next_voxel_size = min(
                self.max_voxel_size,
                self.current_voxel_size * self.voxel_growth_factor,
            )
            if next_voxel_size <= self.current_voxel_size:
                break
            self.revoxelize(next_voxel_size)

        if len(self.voxels) > self.max_points:
            self.limit_by_spatial_hash()

        if self.current_voxel_size > previous_voxel_size:
            self.get_logger().info(
                "Compacted global point-cloud preview without dropping mapped "
                f"regions: {previous_count} -> {len(self.voxels)} voxels, "
                f"voxel_size={previous_voxel_size:.3f} -> "
                f"{self.current_voxel_size:.3f} m."
            )

    def revoxelize(self, voxel_size):
        points = np.asarray(list(self.voxels.values()), dtype=np.float32)
        if points.size == 0:
            self.current_voxel_size = voxel_size
            return

        voxel_keys = np.floor(points[:, 0:3] / voxel_size).astype(np.int32)
        _, unique_indices = np.unique(voxel_keys, axis=0, return_index=True)
        self.voxels = {
            tuple(voxel_keys[index]): tuple(float(value) for value in points[index])
            for index in unique_indices
        }
        self.current_voxel_size = voxel_size

    def limit_by_spatial_hash(self):
        keys = sorted(self.voxels, key=self.spatial_hash)
        retained_keys = keys[: self.max_points]
        self.voxels = {key: self.voxels[key] for key in retained_keys}
        if not self.hard_cap_warning_emitted:
            self.hard_cap_warning_emitted = True
            self.get_logger().warn(
                "Global point-cloud preview reached the maximum voxel size; "
                f"retaining {len(self.voxels)} spatially distributed voxels."
            )

    @staticmethod
    def spatial_hash(key):
        x, y, z = key
        return (
            (x * 73856093) ^ (y * 19349663) ^ (z * 83492791)
        ) & 0xFFFFFFFF

    def publish_map(self):
        if self.latest_header is None or not self.voxels:
            return

        preview_points = np.array(list(self.voxels.values()), dtype=np.float32)
        rgb = self.colorize(preview_points)
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name="rgb", offset=16, datatype=PointField.FLOAT32, count=1),
        ]
        out = self.create_cloud_fast(self.latest_header, fields, preview_points, rgb)
        self.publisher.publish(out)

    @staticmethod
    def create_cloud_fast(header, fields, points, rgb):
        dtype = np.dtype(
            [
                ("x", "<f4"),
                ("y", "<f4"),
                ("z", "<f4"),
                ("intensity", "<f4"),
                ("rgb", "<f4"),
            ]
        )
        cloud = np.empty(points.shape[0], dtype=dtype)
        cloud["x"] = points[:, 0]
        cloud["y"] = points[:, 1]
        cloud["z"] = points[:, 2]
        cloud["intensity"] = points[:, 3]
        cloud["rgb"] = rgb

        msg = PointCloud2()
        msg.header = header
        msg.height = 1
        msg.width = int(points.shape[0])
        msg.fields = fields
        msg.is_bigendian = False
        msg.point_step = dtype.itemsize
        msg.row_step = msg.point_step * msg.width
        msg.data = cloud.tobytes()
        msg.is_dense = False
        return msg

    def colorize(self, preview_points):
        if self.color_mode == "intensity":
            values = preview_points[:, 3]
        elif self.color_mode == "range":
            values = np.linalg.norm(preview_points[:, 0:3], axis=1)
        elif self.color_mode == "x":
            values = preview_points[:, 0]
        elif self.color_mode == "y":
            values = preview_points[:, 1]
        else:
            values = preview_points[:, 2]

        if self.color_max > self.color_min:
            lower = self.color_min
            upper = self.color_max
        else:
            lower, upper = np.percentile(values, [2.0, 98.0])
            if upper <= lower:
                upper = lower + 1.0

        normalized = np.clip((values - lower) / (upper - lower), 0.0, 1.0)
        red = (255.0 * normalized).astype(np.uint32)
        green = (255.0 * (1.0 - normalized)).astype(np.uint32)
        blue = np.zeros_like(red, dtype=np.uint32)
        packed_rgb = (red << 16) | (green << 8) | blue
        return packed_rgb.astype(np.uint32).view(np.float32)


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudAccumulatorPreview()
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
