#!/usr/bin/env python3

import math
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import PoseArray
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


def quaternion_to_matrix(q):
    x = q.x
    y = q.y
    z = q.z
    w = q.w

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


def rotation_matrix_from_rpy(roll, pitch, yaw):
    cr = math.cos(roll)
    sr = math.sin(roll)
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    cy = math.cos(yaw)
    sy = math.sin(yaw)

    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=np.float32,
    )


class SimulatedWorldLidar(Node):
    def __init__(self):
        super().__init__("simulated_world_lidar")

        self.declare_parameter("world_obj_path", "")
        self.declare_parameter(
            "output_topic", "/gen0_mapping/simulated_front3d/lidar/points"
        )
        self.declare_parameter("pose_topic", "/gen0_model/links/poses")
        self.declare_parameter("pose_index", 15)
        self.declare_parameter("frame_id", "front_3d_lidar_link")
        self.declare_parameter("scan_rate", 10.0)
        self.declare_parameter("max_points", 24000)
        self.declare_parameter("world_voxel_size", 0.25)
        self.declare_parameter("surface_sampling_enabled", False)
        self.declare_parameter("surface_sample_count", 0)
        self.declare_parameter("add_obstacle_columns", False)
        self.declare_parameter("column_voxel_size", 0.35)
        self.declare_parameter("column_min_height", 0.5)
        self.declare_parameter("column_sample_step", 0.4)
        self.declare_parameter("column_max_rel_height", 2.0)
        self.declare_parameter("random_seed", 7)
        self.declare_parameter("world_pose_xyz", [0.0, 0.0, 0.0])
        self.declare_parameter("world_pose_rpy", [1.57079632679, 0.0, 0.0])
        self.declare_parameter("lidar_xyz_in_base", [1.9, 0.0, 1.9])
        self.declare_parameter("min_range", 0.6)
        self.declare_parameter("max_range", 80.0)
        self.declare_parameter("horizontal_min_angle", -math.pi)
        self.declare_parameter("horizontal_max_angle", math.pi)
        self.declare_parameter("vertical_min_angle", -1.2)
        self.declare_parameter("vertical_max_angle", 0.8)
        self.declare_parameter("min_base_z", -2.5)
        self.declare_parameter("max_base_z", 25.0)
        self.declare_parameter("priority_sampling_enabled", True)
        self.declare_parameter("priority_range", 12.0)
        self.declare_parameter("priority_min_base_z", -2.5)
        self.declare_parameter("priority_max_base_z", 2.8)
        self.declare_parameter("self_filter_enabled", True)
        self.declare_parameter("self_filter_min_xyz", [-2.8, -1.4, -0.4])
        self.declare_parameter("self_filter_max_xyz", [2.8, 1.4, 2.9])

        self.world_obj_path = self.get_parameter("world_obj_path").value
        self.output_topic = self.get_parameter("output_topic").value
        self.pose_topic = self.get_parameter("pose_topic").value
        self.pose_index = int(self.get_parameter("pose_index").value)
        self.frame_id = self.get_parameter("frame_id").value
        self.scan_rate = max(0.1, float(self.get_parameter("scan_rate").value))
        self.max_points = int(self.get_parameter("max_points").value)
        self.world_voxel_size = float(
            self.get_parameter("world_voxel_size").value
        )
        self.surface_sampling_enabled = bool(
            self.get_parameter("surface_sampling_enabled").value
        )
        self.surface_sample_count = max(
            0, int(self.get_parameter("surface_sample_count").value)
        )
        self.add_obstacle_columns = bool(
            self.get_parameter("add_obstacle_columns").value
        )
        self.column_voxel_size = float(
            self.get_parameter("column_voxel_size").value
        )
        self.column_min_height = float(
            self.get_parameter("column_min_height").value
        )
        self.column_sample_step = float(
            self.get_parameter("column_sample_step").value
        )
        self.column_max_rel_height = float(
            self.get_parameter("column_max_rel_height").value
        )
        self.random_seed = int(self.get_parameter("random_seed").value)
        self.world_pose_xyz = self.vector_parameter(
            "world_pose_xyz", [0.0, 0.0, 0.0]
        )
        self.world_pose_rpy = self.vector_parameter(
            "world_pose_rpy", [1.57079632679, 0.0, 0.0]
        )
        self.lidar_xyz_in_base = self.vector_parameter(
            "lidar_xyz_in_base", [1.9, 0.0, 1.9]
        )
        self.min_range = float(self.get_parameter("min_range").value)
        self.max_range = float(self.get_parameter("max_range").value)
        self.horizontal_min_angle = float(
            self.get_parameter("horizontal_min_angle").value
        )
        self.horizontal_max_angle = float(
            self.get_parameter("horizontal_max_angle").value
        )
        self.vertical_min_angle = float(
            self.get_parameter("vertical_min_angle").value
        )
        self.vertical_max_angle = float(
            self.get_parameter("vertical_max_angle").value
        )
        self.min_base_z = float(self.get_parameter("min_base_z").value)
        self.max_base_z = float(self.get_parameter("max_base_z").value)
        self.priority_sampling_enabled = bool(
            self.get_parameter("priority_sampling_enabled").value
        )
        self.priority_range = float(self.get_parameter("priority_range").value)
        self.priority_min_base_z = float(
            self.get_parameter("priority_min_base_z").value
        )
        self.priority_max_base_z = float(
            self.get_parameter("priority_max_base_z").value
        )
        self.self_filter_enabled = bool(
            self.get_parameter("self_filter_enabled").value
        )
        self.self_filter_min_xyz = self.vector_parameter(
            "self_filter_min_xyz", [-2.8, -1.4, -0.4]
        )
        self.self_filter_max_xyz = self.vector_parameter(
            "self_filter_max_xyz", [2.8, 1.4, 2.9]
        )

        self.vehicle_position = None
        self.vehicle_rotation = None
        self.world_points = self.load_world_points()

        self.create_subscription(PoseArray, self.pose_topic, self.pose_callback, 10)
        self.publisher = self.create_publisher(PointCloud2, self.output_topic, 10)
        self.create_timer(1.0 / self.scan_rate, self.publish_scan)

        self.get_logger().info(
            f"Publishing simulated world lidar {self.output_topic} from "
            f"{len(self.world_points)} environment points, pose_topic={self.pose_topic}, "
            f"max_points={self.max_points}, max_range={self.max_range:.1f}, "
            f"h=[{self.horizontal_min_angle:.2f}, {self.horizontal_max_angle:.2f}], "
            f"v=[{self.vertical_min_angle:.2f}, {self.vertical_max_angle:.2f}], "
            f"add_obstacle_columns={self.add_obstacle_columns}"
        )

    def vector_parameter(self, name, fallback):
        value = self.get_parameter(name).value
        if len(value) != 3:
            self.get_logger().warn(
                f"Parameter {name} must contain exactly 3 values; using {fallback}"
            )
            value = fallback
        return np.array(value, dtype=np.float32)

    def load_world_points(self):
        path = Path(self.world_obj_path)
        if not path.exists():
            self.get_logger().error(f"World OBJ path does not exist: {path}")
            return np.empty((0, 3), dtype=np.float32)

        vertices = []
        faces = []
        with path.open("r", errors="ignore") as stream:
            for line in stream:
                if line.startswith("v "):
                    parts = line.split()
                    if len(parts) < 4:
                        continue
                    try:
                        vertices.append(
                            (float(parts[1]), float(parts[2]), float(parts[3]))
                        )
                    except ValueError:
                        continue
                elif line.startswith("f ") and self.surface_sampling_enabled:
                    face = self.parse_face_indices(line, len(vertices))
                    if len(face) >= 3:
                        faces.append(face)

        if not vertices:
            self.get_logger().error(f"No vertices found in {path}")
            return np.empty((0, 3), dtype=np.float32)

        points = np.asarray(vertices, dtype=np.float32)
        world_rotation = rotation_matrix_from_rpy(*self.world_pose_rpy.tolist())
        points = points.dot(world_rotation.T) + self.world_pose_xyz
        if self.surface_sampling_enabled and faces and self.surface_sample_count > 0:
            surface_points = self.sample_surface_points(points, faces)
            if surface_points.size:
                points = np.vstack((points, surface_points))
                self.get_logger().info(
                    f"Added {len(surface_points)} deterministic surface samples "
                    f"from {len(faces)} OBJ faces"
                )

        if self.add_obstacle_columns:
            column_points = self.build_obstacle_columns(points)
            if column_points.size:
                points = np.vstack((points, column_points))

        if self.world_voxel_size > 0.0:
            voxel_keys = np.floor(points / self.world_voxel_size).astype(np.int32)
            _, unique_indices = np.unique(voxel_keys, axis=0, return_index=True)
            points = points[unique_indices]

        rng = np.random.default_rng(self.random_seed)
        rng.shuffle(points)
        return points.astype(np.float32, copy=False)

    def parse_face_indices(self, line, vertex_count):
        indices = []
        for token in line.split()[1:]:
            vertex_token = token.split("/", 1)[0]
            if not vertex_token:
                continue
            try:
                raw_index = int(vertex_token)
            except ValueError:
                continue

            if raw_index > 0:
                index = raw_index - 1
            else:
                index = vertex_count + raw_index

            if 0 <= index < vertex_count:
                indices.append(index)
        return indices

    def sample_surface_points(self, points, faces):
        triangles = []
        for face in faces:
            anchor = face[0]
            for offset in range(1, len(face) - 1):
                triangles.append((anchor, face[offset], face[offset + 1]))

        if not triangles:
            return np.empty((0, 3), dtype=np.float32)

        triangle_indices = np.asarray(triangles, dtype=np.int32)
        triangle_points = points[triangle_indices]
        edge_ab = triangle_points[:, 1] - triangle_points[:, 0]
        edge_ac = triangle_points[:, 2] - triangle_points[:, 0]
        areas = 0.5 * np.linalg.norm(np.cross(edge_ab, edge_ac), axis=1)
        valid = np.isfinite(areas) & (areas > 1e-6)
        if not np.any(valid):
            return np.empty((0, 3), dtype=np.float32)

        triangle_points = triangle_points[valid]
        areas = areas[valid]
        probabilities = areas / areas.sum()
        rng = np.random.default_rng(self.random_seed + 101)
        chosen = rng.choice(
            len(triangle_points),
            size=self.surface_sample_count,
            replace=True,
            p=probabilities,
        )
        chosen_triangles = triangle_points[chosen]

        r1 = rng.random(self.surface_sample_count, dtype=np.float32)
        r2 = rng.random(self.surface_sample_count, dtype=np.float32)
        sqrt_r1 = np.sqrt(r1)[:, None]

        samples = (
            (1.0 - sqrt_r1) * chosen_triangles[:, 0]
            + sqrt_r1 * (1.0 - r2[:, None]) * chosen_triangles[:, 1]
            + sqrt_r1 * r2[:, None] * chosen_triangles[:, 2]
        )
        return samples.astype(np.float32, copy=False)

    def build_obstacle_columns(self, points):
        if (
            self.column_voxel_size <= 0.0
            or self.column_min_height <= 0.0
            or self.column_sample_step <= 0.0
            or self.column_max_rel_height <= 0.0
        ):
            return np.empty((0, 3), dtype=np.float32)

        xy_keys = np.floor(points[:, :2] / self.column_voxel_size).astype(np.int32)
        unique_keys, inverse = np.unique(xy_keys, axis=0, return_inverse=True)
        min_z = np.full(len(unique_keys), np.inf, dtype=np.float32)
        max_z = np.full(len(unique_keys), -np.inf, dtype=np.float32)
        np.minimum.at(min_z, inverse, points[:, 2])
        np.maximum.at(max_z, inverse, points[:, 2])

        column_mask = (max_z - min_z) >= self.column_min_height
        if not np.any(column_mask):
            return np.empty((0, 3), dtype=np.float32)

        centers_xy = (
            unique_keys[column_mask].astype(np.float32) + 0.5
        ) * self.column_voxel_size
        column_min_z = min_z[column_mask]
        column_max_z = max_z[column_mask]

        samples = []
        levels = np.arange(
            self.column_sample_step,
            self.column_max_rel_height + 1e-6,
            self.column_sample_step,
            dtype=np.float32,
        )
        for level in levels:
            sample_z = column_min_z + level
            valid = sample_z <= column_max_z
            if not np.any(valid):
                continue
            samples.append(
                np.column_stack((centers_xy[valid], sample_z[valid])).astype(
                    np.float32,
                    copy=False,
                )
            )

        if not samples:
            return np.empty((0, 3), dtype=np.float32)
        return np.vstack(samples)

    def pose_callback(self, msg):
        if len(msg.poses) <= self.pose_index:
            self.get_logger().warn(
                f"PoseArray has {len(msg.poses)} poses, cannot read index "
                f"{self.pose_index}",
                throttle_duration_sec=2.0,
            )
            return

        pose = msg.poses[self.pose_index]
        self.vehicle_position = np.array(
            [pose.position.x, pose.position.y, pose.position.z],
            dtype=np.float32,
        )
        self.vehicle_rotation = quaternion_to_matrix(pose.orientation)

    def publish_scan(self):
        if self.vehicle_position is None or self.world_points.size == 0:
            return

        sensor_position = (
            self.vehicle_position + self.vehicle_rotation.dot(self.lidar_xyz_in_base)
        )
        points_lidar = (self.world_points - sensor_position).dot(
            self.vehicle_rotation
        )

        range_sq = np.einsum("ij,ij->i", points_lidar, points_lidar)
        keep = range_sq >= self.min_range * self.min_range
        if self.max_range > self.min_range:
            keep &= range_sq <= self.max_range * self.max_range

        if self.horizontal_max_angle > self.horizontal_min_angle:
            horizontal_angle = np.arctan2(points_lidar[:, 1], points_lidar[:, 0])
            keep &= (horizontal_angle >= self.horizontal_min_angle) & (
                horizontal_angle <= self.horizontal_max_angle
            )

        if self.vertical_max_angle > self.vertical_min_angle:
            horizontal = np.linalg.norm(points_lidar[:, :2], axis=1)
            vertical_angle = np.arctan2(
                points_lidar[:, 2], np.maximum(horizontal, 1e-6)
            )
            keep &= (vertical_angle >= self.vertical_min_angle) & (
                vertical_angle <= self.vertical_max_angle
            )

        points_base = points_lidar + self.lidar_xyz_in_base
        keep &= (points_base[:, 2] >= self.min_base_z) & (
            points_base[:, 2] <= self.max_base_z
        )

        if self.self_filter_enabled:
            outside_vehicle = np.any(
                (points_base < self.self_filter_min_xyz)
                | (points_base > self.self_filter_max_xyz),
                axis=1,
            )
            keep &= outside_vehicle

        points = points_lidar[keep]
        points_base = points_base[keep]
        range_sq = range_sq[keep]
        if points.size == 0:
            self.get_logger().warn(
                "Simulated lidar produced zero points after filtering",
                throttle_duration_sec=2.0,
            )
            return

        if self.max_points > 0 and len(points) > self.max_points:
            points = self.sample_scan_points(points, points_base, range_sq)

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.frame_id
        self.publisher.publish(point_cloud2.create_cloud_xyz32(header, points))

    def sample_scan_points(self, points, points_base, range_sq):
        if not self.priority_sampling_enabled or self.priority_range <= 0.0:
            indices = np.linspace(0, len(points) - 1, self.max_points, dtype=np.int64)
            return points[indices]

        priority_range_sq = self.priority_range * self.priority_range
        priority = (
            (range_sq <= priority_range_sq)
            & (points_base[:, 2] >= self.priority_min_base_z)
            & (points_base[:, 2] <= self.priority_max_base_z)
        )

        priority_points = points[priority]
        if len(priority_points) >= self.max_points:
            return self.even_sample(priority_points, self.max_points)

        remaining = self.max_points - len(priority_points)
        other_points = points[~priority]
        if len(other_points) <= remaining:
            return np.vstack((priority_points, other_points))

        other_range_sq = range_sq[~priority]
        near_order = np.argsort(other_range_sq)
        other_sample = self.even_sample(other_points[near_order], remaining)
        return np.vstack((priority_points, other_sample))

    @staticmethod
    def even_sample(points, count):
        if count <= 0 or len(points) == 0:
            return np.empty((0, 3), dtype=np.float32)
        if len(points) <= count:
            return points
        indices = np.linspace(0, len(points) - 1, count, dtype=np.int64)
        return points[indices]


def main(args=None):
    rclpy.init(args=args)
    node = SimulatedWorldLidar()
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
