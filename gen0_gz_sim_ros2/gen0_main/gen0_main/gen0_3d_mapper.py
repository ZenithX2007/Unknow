#!/usr/bin/env python3

import math
from pathlib import Path

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_srvs.srv import Trigger


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


class Gen03DMapper(Node):
    def __init__(self):
        super().__init__("gen0_3d_mapper")

        self.declare_parameter("cloud_topic", "/gen0_model/front3d/lidar/points")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("map_topic", "/gen0_mapping/map_cloud")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("voxel_size", 0.25)
        self.declare_parameter("publish_period", 1.0)
        self.declare_parameter("max_points", 500000)
        self.declare_parameter("max_points_per_scan", 25000)
        self.declare_parameter("min_range", 0.7)
        self.declare_parameter("max_range", 80.0)
        self.declare_parameter("min_z", -5.0)
        self.declare_parameter("max_z", 30.0)
        self.declare_parameter("min_translation_delta", 0.10)
        self.declare_parameter("min_rotation_delta", 0.02)
        self.declare_parameter("lidar_xyz_in_base", [1.9, 0.0, 1.9])
        self.declare_parameter("self_filter_enabled", True)
        self.declare_parameter("self_filter_min_xyz", [-2.8, -1.4, -0.4])
        self.declare_parameter("self_filter_max_xyz", [2.8, 1.4, 2.9])
        self.declare_parameter("save_pcd_path", "/tmp/gen0_3d_map.pcd")

        self.cloud_topic = self.get_parameter("cloud_topic").value
        self.odom_topic = self.get_parameter("odom_topic").value
        self.map_topic = self.get_parameter("map_topic").value
        self.map_frame = self.get_parameter("map_frame").value
        self.voxel_size = float(self.get_parameter("voxel_size").value)
        self.publish_period = float(self.get_parameter("publish_period").value)
        self.max_points = int(self.get_parameter("max_points").value)
        self.max_points_per_scan = int(self.get_parameter("max_points_per_scan").value)
        self.min_range = float(self.get_parameter("min_range").value)
        self.max_range = float(self.get_parameter("max_range").value)
        self.min_z = float(self.get_parameter("min_z").value)
        self.max_z = float(self.get_parameter("max_z").value)
        self.min_translation_delta = float(self.get_parameter("min_translation_delta").value)
        self.min_rotation_delta = float(self.get_parameter("min_rotation_delta").value)
        self.lidar_xyz_in_base = np.array(
            self.get_parameter("lidar_xyz_in_base").value, dtype=np.float32
        )
        self.self_filter_enabled = bool(
            self.get_parameter("self_filter_enabled").value
        )
        self.self_filter_min_xyz = np.array(
            self.get_parameter("self_filter_min_xyz").value, dtype=np.float32
        )
        self.self_filter_max_xyz = np.array(
            self.get_parameter("self_filter_max_xyz").value, dtype=np.float32
        )
        self.save_pcd_path = self.get_parameter("save_pcd_path").value

        if self.voxel_size <= 0.0:
            raise ValueError("voxel_size must be greater than zero")

        self.latest_pose = None
        self.latest_rotation = None
        self.latest_position = None
        self.last_integrated_position = None
        self.last_integrated_yaw = None
        self.voxels = {}
        self.scans_integrated = 0

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 20)
        self.create_subscription(PointCloud2, self.cloud_topic, self.cloud_callback, sensor_qos)
        self.map_pub = self.create_publisher(PointCloud2, self.map_topic, 1)
        self.create_timer(self.publish_period, self.publish_map)
        self.create_service(Trigger, "~/save_map", self.save_map_callback)
        self.create_service(Trigger, "~/reset_map", self.reset_map_callback)

        self.get_logger().info(
            f"3D mapper listening cloud={self.cloud_topic}, odom={self.odom_topic}, "
            f"publishing map={self.map_topic}"
        )

    def odom_callback(self, msg):
        self.latest_pose = msg.pose.pose
        self.latest_position = np.array(
            [
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z,
            ],
            dtype=np.float32,
        )
        self.latest_rotation = quaternion_to_matrix(msg.pose.pose.orientation)

    def cloud_callback(self, msg):
        if self.latest_pose is None:
            return

        position = self.latest_position
        rotation = self.latest_rotation
        yaw = math.atan2(rotation[1, 0], rotation[0, 0])
        if not self.should_integrate(position, yaw):
            return

        field_names = [field.name for field in msg.fields]
        if not {"x", "y", "z"}.issubset(field_names):
            self.get_logger().warn("Skipping point cloud without x/y/z fields", throttle_duration_sec=2.0)
            return

        try:
            points = point_cloud2.read_points_numpy(msg, field_names=["x", "y", "z"], skip_nans=True)
        except (AssertionError, ValueError):
            structured = point_cloud2.read_points(msg, field_names=["x", "y", "z"], skip_nans=True)
            points = np.column_stack((structured["x"], structured["y"], structured["z"]))

        if points.size == 0:
            return

        points = points.astype(np.float32, copy=False)
        if self.max_points_per_scan > 0 and len(points) > self.max_points_per_scan:
            step = max(1, int(math.ceil(len(points) / self.max_points_per_scan)))
            points = points[::step]

        range_sq = np.sum(points * points, axis=1)
        mask = (range_sq >= self.min_range * self.min_range) & (
            range_sq <= self.max_range * self.max_range
        )
        if not np.any(mask):
            return

        points_base = points[mask] + self.lidar_xyz_in_base
        if self.self_filter_enabled:
            outside_vehicle = np.any(
                (points_base < self.self_filter_min_xyz)
                | (points_base > self.self_filter_max_xyz),
                axis=1,
            )
            points_base = points_base[outside_vehicle]
            if len(points_base) == 0:
                return

        points_map = points_base.dot(rotation.T) + position
        z_mask = (points_map[:, 2] >= self.min_z) & (points_map[:, 2] <= self.max_z)
        points_map = points_map[z_mask]
        if len(points_map) == 0:
            return

        voxel_keys = np.floor(points_map / self.voxel_size).astype(np.int32)
        _, unique_indices = np.unique(voxel_keys, axis=0, return_index=True)
        unique_keys = voxel_keys[unique_indices]
        unique_points = points_map[unique_indices]

        added = 0
        for key, point in zip(unique_keys, unique_points):
            key_tuple = (int(key[0]), int(key[1]), int(key[2]))
            if key_tuple not in self.voxels:
                self.voxels[key_tuple] = (
                    float(point[0]),
                    float(point[1]),
                    float(point[2]),
                )
                added += 1

        self.trim_map()
        self.scans_integrated += 1
        self.last_integrated_position = position.copy()
        self.last_integrated_yaw = yaw

        if self.scans_integrated == 1 or self.scans_integrated % 20 == 0:
            self.get_logger().info(
                f"Integrated scans={self.scans_integrated}, added_voxels={added}, "
                f"map_points={len(self.voxels)}"
            )

    def should_integrate(self, position, yaw):
        if self.last_integrated_position is None:
            return True

        translation_delta = float(np.linalg.norm(position - self.last_integrated_position))
        yaw_delta = abs(self.normalize_angle(yaw - self.last_integrated_yaw))
        return (
            translation_delta >= self.min_translation_delta
            or yaw_delta >= self.min_rotation_delta
        )

    @staticmethod
    def normalize_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def trim_map(self):
        overflow = len(self.voxels) - self.max_points
        if overflow <= 0:
            return
        for _ in range(overflow):
            self.voxels.pop(next(iter(self.voxels)))

    def publish_map(self):
        if not self.voxels:
            return

        header = self.create_header()
        cloud = point_cloud2.create_cloud_xyz32(header, list(self.voxels.values()))
        self.map_pub.publish(cloud)

    def create_header(self):
        header = PointCloud2().header
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.map_frame
        return header

    def save_map_callback(self, request, response):
        del request
        if not self.voxels:
            response.success = False
            response.message = "map is empty"
            return response

        path = Path(self.save_pcd_path).expanduser()
        path.parent.mkdir(parents=True, exist_ok=True)
        points = list(self.voxels.values())
        with path.open("w", encoding="ascii") as pcd:
            pcd.write("# .PCD v0.7 - Point Cloud Data file format\n")
            pcd.write("VERSION 0.7\n")
            pcd.write("FIELDS x y z\n")
            pcd.write("SIZE 4 4 4\n")
            pcd.write("TYPE F F F\n")
            pcd.write("COUNT 1 1 1\n")
            pcd.write(f"WIDTH {len(points)}\n")
            pcd.write("HEIGHT 1\n")
            pcd.write("VIEWPOINT 0 0 0 1 0 0 0\n")
            pcd.write(f"POINTS {len(points)}\n")
            pcd.write("DATA ascii\n")
            for x, y, z in points:
                pcd.write(f"{x:.6f} {y:.6f} {z:.6f}\n")

        response.success = True
        response.message = f"saved {len(points)} points to {path}"
        return response

    def reset_map_callback(self, request, response):
        del request
        self.voxels.clear()
        self.scans_integrated = 0
        self.last_integrated_position = None
        self.last_integrated_yaw = None
        response.success = True
        response.message = "map reset"
        return response


def main(args=None):
    rclpy.init(args=args)
    node = Gen03DMapper()
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
