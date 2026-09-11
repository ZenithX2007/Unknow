#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2


class PointCloudPreview(Node):
    def __init__(self):
        super().__init__("pointcloud_preview")

        self.declare_parameter("input_topic", "/gen0_mapping/fast_lio_map")
        self.declare_parameter("output_topic", "/gen0_mapping/rviz/fast_lio_map")
        self.declare_parameter("max_points", 80000)
        self.declare_parameter("voxel_size", 0.25)
        self.declare_parameter("pre_sample_factor", 8)
        self.declare_parameter("min_range", 0.0)
        self.declare_parameter("max_range", 0.0)
        self.declare_parameter("color_mode", "z")
        self.declare_parameter("color_min", 0.0)
        self.declare_parameter("color_max", 0.0)
        self.declare_parameter("terrain_ground_height", 0.03)
        self.declare_parameter("terrain_obstacle_height", 0.22)
        self.declare_parameter("publish_period", 0.5)

        self.input_topic = self.get_parameter("input_topic").value
        self.output_topic = self.get_parameter("output_topic").value
        self.max_points = int(self.get_parameter("max_points").value)
        self.voxel_size = float(self.get_parameter("voxel_size").value)
        self.pre_sample_factor = max(1, int(self.get_parameter("pre_sample_factor").value))
        self.min_range = float(self.get_parameter("min_range").value)
        self.max_range = float(self.get_parameter("max_range").value)
        self.color_mode = str(self.get_parameter("color_mode").value).lower()
        self.color_min = float(self.get_parameter("color_min").value)
        self.color_max = float(self.get_parameter("color_max").value)
        self.terrain_ground_height = float(
            self.get_parameter("terrain_ground_height").value
        )
        self.terrain_obstacle_height = float(
            self.get_parameter("terrain_obstacle_height").value
        )
        publish_period = float(self.get_parameter("publish_period").value)
        self.latest_msg = None
        self.latest_stamp = None
        self.last_published_stamp = None

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(PointCloud2, self.input_topic, self.cloud_callback, qos)
        self.publisher = self.create_publisher(PointCloud2, self.output_topic, 1)
        self.create_timer(max(publish_period, 0.05), self.publish_preview)

        self.get_logger().info(
            f"Previewing {self.input_topic} -> {self.output_topic}, "
            f"max_points={self.max_points}, voxel_size={self.voxel_size}, "
            f"range=[{self.min_range:.2f}, {self.max_range:.2f}], "
            f"color_mode={self.color_mode}"
        )

    def cloud_callback(self, msg):
        self.latest_msg = msg
        self.latest_stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)

    def publish_preview(self):
        if self.latest_msg is None or self.latest_stamp == self.last_published_stamp:
            return

        preview_points = self.sample_points(self.latest_msg)
        if preview_points.size == 0:
            return

        rgb = self.colorize(preview_points)
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name="rgb", offset=16, datatype=PointField.FLOAT32, count=1),
        ]
        out = self.create_cloud_fast(self.latest_msg.header, fields, preview_points, rgb)
        self.publisher.publish(out)
        self.last_published_stamp = self.latest_stamp

    def create_cloud_fast(self, header, fields, points, rgb):
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
        cloud["x"] = points[:, 0].astype(np.float32, copy=False)
        cloud["y"] = points[:, 1].astype(np.float32, copy=False)
        cloud["z"] = points[:, 2].astype(np.float32, copy=False)
        cloud["intensity"] = points[:, 3].astype(np.float32, copy=False)
        cloud["rgb"] = rgb.astype(np.float32, copy=False)

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
        if self.color_mode == "terrain":
            return self.colorize_terrain(preview_points)

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

    def colorize_terrain(self, preview_points):
        height_cost = np.maximum(preview_points[:, 3], 0.0)
        upper = max(self.terrain_obstacle_height, self.terrain_ground_height + 1e-3)
        normalized = np.clip(
            (height_cost - self.terrain_ground_height)
            / (upper - self.terrain_ground_height),
            0.0,
            1.0,
        )

        # SCURM's terrain map is an elevation-cost map. Keep traversable ground
        # nearly white and fade raised cells into a soft green obstacle color.
        ground = np.array([238.0, 238.0, 232.0], dtype=np.float32)
        raised = np.array([154.0, 218.0, 154.0], dtype=np.float32)
        colors = ground + (raised - ground) * normalized[:, None]

        red = colors[:, 0].astype(np.uint32)
        green = colors[:, 1].astype(np.uint32)
        blue = colors[:, 2].astype(np.uint32)
        packed_rgb = (red << 16) | (green << 8) | blue
        return packed_rgb.astype(np.uint32).view(np.float32)

    def sample_points(self, msg):
        total_points = int(msg.width * msg.height)
        if total_points <= 0:
            return np.empty((0, 4), dtype=np.float32)

        max_input_points = (
            self.max_points * self.pre_sample_factor if self.max_points > 0 else total_points
        )
        if total_points > max_input_points:
            indices = np.linspace(
                0, total_points - 1, max_input_points, dtype=np.int64
            )
        else:
            indices = None

        sampled = self.read_xyz_intensity(msg, indices)
        if sampled.size == 0:
            return np.empty((0, 4), dtype=np.float32)

        points = sampled[:, 0:3]
        intensity = sampled[:, 3]
        if points.size == 0:
            return np.empty((0, 4), dtype=np.float32)

        finite = np.isfinite(points).all(axis=1) & np.isfinite(intensity)
        points = points[finite]
        intensity = intensity[finite]

        if points.size == 0:
            return np.empty((0, 4), dtype=np.float32)

        if self.max_range > self.min_range >= 0.0:
            range_sq = np.einsum("ij,ij->i", points, points)
            range_mask = (range_sq >= self.min_range * self.min_range) & (
                range_sq <= self.max_range * self.max_range
            )
            points = points[range_mask]
            intensity = intensity[range_mask]
            if points.size == 0:
                return np.empty((0, 4), dtype=np.float32)

        if self.voxel_size > 0.0:
            voxel_keys = np.floor(points / self.voxel_size).astype(np.int32)
            _, unique_indices = np.unique(voxel_keys, axis=0, return_index=True)
            points = points[unique_indices]
            intensity = intensity[unique_indices]

        if self.max_points > 0 and points.shape[0] > self.max_points:
            keep_indices = np.linspace(
                0, points.shape[0] - 1, self.max_points, dtype=np.int64
            )
            points = points[keep_indices]
            intensity = intensity[keep_indices]

        return np.column_stack((points, intensity)).astype(np.float32, copy=False)

    def read_xyz_intensity(self, msg, indices):
        fields = {field.name: field for field in msg.fields}
        if not all(name in fields for name in ("x", "y", "z")):
            return np.empty((0, 4), dtype=np.float32)

        if (
            fields["x"].datatype != PointField.FLOAT32
            or fields["y"].datatype != PointField.FLOAT32
            or fields["z"].datatype != PointField.FLOAT32
        ):
            return self.read_xyz_intensity_slow(msg, indices)

        total_points = int(msg.width * msg.height)
        if msg.row_step != msg.point_step * msg.width:
            return self.read_xyz_intensity_slow(msg, indices)

        dtype = ">f4" if msg.is_bigendian else "<f4"
        try:
            x_view = np.ndarray(
                (total_points,),
                dtype=dtype,
                buffer=msg.data,
                offset=fields["x"].offset,
                strides=(msg.point_step,),
            )
            y_view = np.ndarray(
                (total_points,),
                dtype=dtype,
                buffer=msg.data,
                offset=fields["y"].offset,
                strides=(msg.point_step,),
            )
            z_view = np.ndarray(
                (total_points,),
                dtype=dtype,
                buffer=msg.data,
                offset=fields["z"].offset,
                strides=(msg.point_step,),
            )
        except (TypeError, ValueError):
            return np.empty((0, 4), dtype=np.float32)

        if indices is None:
            x = x_view.astype(np.float32, copy=False)
            y = y_view.astype(np.float32, copy=False)
            z = z_view.astype(np.float32, copy=False)
        else:
            x = x_view[indices].astype(np.float32, copy=False)
            y = y_view[indices].astype(np.float32, copy=False)
            z = z_view[indices].astype(np.float32, copy=False)

        intensity = np.zeros(x.shape[0], dtype=np.float32)
        intensity_field = fields.get("intensity")
        if intensity_field is not None and intensity_field.datatype == PointField.FLOAT32:
            try:
                intensity_view = np.ndarray(
                    (total_points,),
                    dtype=dtype,
                    buffer=msg.data,
                    offset=intensity_field.offset,
                    strides=(msg.point_step,),
                )
                if indices is None:
                    intensity = intensity_view.astype(np.float32, copy=False)
                else:
                    intensity = intensity_view[indices].astype(np.float32, copy=False)
            except (TypeError, ValueError):
                intensity = np.zeros(x.shape[0], dtype=np.float32)

        return np.column_stack((x, y, z, intensity)).astype(np.float32, copy=False)

    def read_xyz_intensity_slow(self, msg, indices):
        available_fields = {field.name for field in msg.fields}
        has_intensity = "intensity" in available_fields
        field_names = ["x", "y", "z", "intensity"] if has_intensity else ["x", "y", "z"]

        try:
            structured = point_cloud2.read_points(
                msg,
                field_names=field_names,
                skip_nans=False,
            )
            if indices is not None:
                structured = structured[indices]
            points = np.column_stack(
                (structured["x"], structured["y"], structured["z"])
            ).astype(np.float32, copy=False)
            if has_intensity:
                intensity = structured["intensity"].astype(np.float32, copy=False)
            else:
                intensity = np.zeros(points.shape[0], dtype=np.float32)
        except (AssertionError, ValueError, KeyError, IndexError):
            return np.empty((0, 4), dtype=np.float32)

        return np.column_stack((points, intensity)).astype(np.float32, copy=False)


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudPreview()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
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
