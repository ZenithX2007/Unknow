#!/usr/bin/env python3

import json
import math
import os
import time
import warnings

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Pose, PoseArray
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


def stamp_to_sec(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def stamp_key(stamp):
    return (int(stamp.sec), int(stamp.nanosec))


def quaternion_to_matrix(q):
    x = float(q.x)
    y = float(q.y)
    z = float(q.z)
    w = float(q.w)

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


def transform_to_matrix(transform):
    matrix = np.eye(4, dtype=np.float32)
    matrix[:3, :3] = quaternion_to_matrix(transform.transform.rotation)
    matrix[0, 3] = float(transform.transform.translation.x)
    matrix[1, 3] = float(transform.transform.translation.y)
    matrix[2, 3] = float(transform.transform.translation.z)
    return matrix


def transform_points(points, matrix):
    if points.size == 0:
        return points.reshape((-1, 3))
    return points.dot(matrix[:3, :3].T) + matrix[:3, 3]


def bbox_corners(min_point, max_point):
    return np.array(
        [
            [min_point[0], min_point[1], min_point[2]],
            [max_point[0], min_point[1], min_point[2]],
            [max_point[0], max_point[1], min_point[2]],
            [min_point[0], max_point[1], min_point[2]],
            [min_point[0], min_point[1], max_point[2]],
            [max_point[0], min_point[1], max_point[2]],
            [max_point[0], max_point[1], max_point[2]],
            [min_point[0], max_point[1], max_point[2]],
        ],
        dtype=np.float32,
    )


def bbox_iou(box_a, box_b):
    ax1, ay1, ax2, ay2 = box_a
    bx1, by1, bx2, by2 = box_b
    ix1 = max(ax1, bx1)
    iy1 = max(ay1, by1)
    ix2 = min(ax2, bx2)
    iy2 = min(ay2, by2)
    iw = max(0.0, ix2 - ix1)
    ih = max(0.0, iy2 - iy1)
    intersection = iw * ih
    area_a = max(0.0, ax2 - ax1) * max(0.0, ay2 - ay1)
    area_b = max(0.0, bx2 - bx1) * max(0.0, by2 - by1)
    union = area_a + area_b - intersection
    if union <= 1e-6:
        return 0.0
    return float(intersection / union)


def point_inside_bbox(point, bbox):
    x, y = point
    x1, y1, x2, y2 = bbox
    return x1 <= x <= x2 and y1 <= y <= y2


class TrashFusionDetector(Node):
    def __init__(self):
        super().__init__("trash_fusion_detector")

        self.declare_parameter("image_topic", "/gen0_model/front_camera")
        self.declare_parameter("camera_info_topic", "/gen0_model/camera_info")
        self.declare_parameter(
            "pointcloud_topic", "/gen0_mapping/simulated_front3d/lidar/points"
        )
        self.declare_parameter("model_path", "/home/zjxue2007/Unknow/best_road.pt")
        self.declare_parameter("output_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("camera_frame", "front_camera_link")
        self.declare_parameter("pointcloud_frame", "front_3d_lidar_link")
        self.declare_parameter("camera_frame_uses_optical_axes", False)
        self.declare_parameter("process_period", 0.5)
        self.declare_parameter("max_sync_slop", 0.75)
        self.declare_parameter("tf_timeout", 0.08)
        self.declare_parameter("yolo_confidence", 0.25)
        self.declare_parameter("yolo_imgsz", 640)
        self.declare_parameter("yolo_max_det", 20)
        self.declare_parameter("yolo_device", "")
        self.declare_parameter("class_filter", "")
        self.declare_parameter("max_input_points", 50000)
        self.declare_parameter("min_range", 0.4)
        self.declare_parameter("max_range", 22.0)
        self.declare_parameter("object_min_base_z", 0.03)
        self.declare_parameter("object_max_base_z", 0.80)
        self.declare_parameter("cluster_voxel_size", 0.04)
        self.declare_parameter("cluster_eps", 0.18)
        self.declare_parameter("cluster_min_points", 4)
        self.declare_parameter("cluster_max_points", 2500)
        self.declare_parameter("cluster_max_extent_xy", 1.30)
        self.declare_parameter("cluster_max_height", 1.00)
        self.declare_parameter("cluster_min_height", 0.02)
        self.declare_parameter("iou_threshold", 0.08)
        self.declare_parameter("allow_center_match", True)
        self.declare_parameter("use_roi_fallback", True)
        self.declare_parameter("roi_min_points", 4)
        self.declare_parameter("marker_lifetime", 1.0)
        self.declare_parameter("fallback_image_width", 1600)
        self.declare_parameter("fallback_image_height", 1200)
        self.declare_parameter("fallback_horizontal_fov", 1.047)
        self.declare_parameter("fallback_lidar_to_base_xyz", [0.85, 0.0, 0.72])
        self.declare_parameter("fallback_lidar_to_camera_xyz", [0.0, 0.0, 0.3])

        self.image_topic = str(self.get_parameter("image_topic").value)
        self.camera_info_topic = str(self.get_parameter("camera_info_topic").value)
        self.pointcloud_topic = str(self.get_parameter("pointcloud_topic").value)
        self.model_path = os.path.expanduser(str(self.get_parameter("model_path").value))
        self.output_frame = str(self.get_parameter("output_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.camera_frame = str(self.get_parameter("camera_frame").value)
        self.pointcloud_frame = str(self.get_parameter("pointcloud_frame").value)
        self.camera_frame_uses_optical_axes = bool(
            self.get_parameter("camera_frame_uses_optical_axes").value
        )
        self.process_period = max(0.05, float(self.get_parameter("process_period").value))
        self.max_sync_slop = max(0.0, float(self.get_parameter("max_sync_slop").value))
        self.tf_timeout = max(0.0, float(self.get_parameter("tf_timeout").value))
        self.yolo_confidence = float(self.get_parameter("yolo_confidence").value)
        self.yolo_imgsz = int(self.get_parameter("yolo_imgsz").value)
        self.yolo_max_det = int(self.get_parameter("yolo_max_det").value)
        self.yolo_device = str(self.get_parameter("yolo_device").value).strip()
        self.class_filter = self.parse_string_set(
            self.get_parameter("class_filter").value
        )
        self.max_input_points = int(self.get_parameter("max_input_points").value)
        self.min_range = float(self.get_parameter("min_range").value)
        self.max_range = float(self.get_parameter("max_range").value)
        self.object_min_base_z = float(
            self.get_parameter("object_min_base_z").value
        )
        self.object_max_base_z = float(
            self.get_parameter("object_max_base_z").value
        )
        self.cluster_voxel_size = float(
            self.get_parameter("cluster_voxel_size").value
        )
        self.cluster_eps = float(self.get_parameter("cluster_eps").value)
        self.cluster_min_points = int(self.get_parameter("cluster_min_points").value)
        self.cluster_max_points = int(self.get_parameter("cluster_max_points").value)
        self.cluster_max_extent_xy = float(
            self.get_parameter("cluster_max_extent_xy").value
        )
        self.cluster_max_height = float(
            self.get_parameter("cluster_max_height").value
        )
        self.cluster_min_height = float(
            self.get_parameter("cluster_min_height").value
        )
        self.iou_threshold = float(self.get_parameter("iou_threshold").value)
        self.allow_center_match = bool(self.get_parameter("allow_center_match").value)
        self.use_roi_fallback = bool(self.get_parameter("use_roi_fallback").value)
        self.roi_min_points = int(self.get_parameter("roi_min_points").value)
        self.marker_lifetime = max(
            0.05, float(self.get_parameter("marker_lifetime").value)
        )
        self.fallback_image_width = int(
            self.get_parameter("fallback_image_width").value
        )
        self.fallback_image_height = int(
            self.get_parameter("fallback_image_height").value
        )
        self.fallback_horizontal_fov = float(
            self.get_parameter("fallback_horizontal_fov").value
        )
        self.fallback_lidar_to_base_xyz = self.vector_parameter(
            "fallback_lidar_to_base_xyz", [0.85, 0.0, 0.72]
        )
        self.fallback_lidar_to_camera_xyz = self.vector_parameter(
            "fallback_lidar_to_camera_xyz", [0.0, 0.0, 0.3]
        )

        self.o3d = self.load_open3d()
        self.model = self.load_yolo()
        self.bridge = CvBridge()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.latest_image = None
        self.latest_cloud = None
        self.latest_camera_info = None
        self.last_processed_image_stamp = None
        self.warned_fallback_camera_info = False
        self.warned_fallbacks = set()
        self.frames = 0
        self.start_wall_time = time.monotonic()

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(Image, self.image_topic, self.image_callback, sensor_qos)
        self.create_subscription(
            PointCloud2, self.pointcloud_topic, self.cloud_callback, sensor_qos
        )
        self.create_subscription(
            CameraInfo, self.camera_info_topic, self.camera_info_callback, sensor_qos
        )

        self.marker_pub = self.create_publisher(
            MarkerArray, "/gen0_perception/trash_markers", 10
        )
        self.pose_pub = self.create_publisher(
            PoseArray, "/gen0_perception/trash_poses", 10
        )
        self.detection_pub = self.create_publisher(
            String, "/gen0_perception/trash_detections", 10
        )
        self.debug_image_pub = self.create_publisher(
            Image, "/gen0_perception/trash_debug_image", 10
        )
        self.create_timer(self.process_period, self.process_latest)

        self.get_logger().info(
            "Trash fusion detector started: "
            f"image={self.image_topic}, cloud={self.pointcloud_topic}, "
            f"camera_info={self.camera_info_topic}, output_frame={self.output_frame}, "
            f"model={self.model_path}, max_sync_slop={self.max_sync_slop:.2f}s, "
            f"yolo_imgsz={self.yolo_imgsz}, yolo_conf={self.yolo_confidence:.2f}"
        )

    def parse_string_set(self, raw_value):
        if raw_value is None:
            return set()
        if isinstance(raw_value, str):
            values = raw_value.split(",")
        else:
            values = list(raw_value)
        return {str(value).strip().lower() for value in values if str(value).strip()}

    def vector_parameter(self, name, default):
        value = self.get_parameter(name).value
        if value is None:
            value = default
        vector = np.array(list(value), dtype=np.float32)
        if vector.size != 3:
            self.get_logger().warn(
                f"Parameter {name} must contain 3 values; using {default}"
            )
            return np.array(default, dtype=np.float32)
        return vector

    def load_yolo(self):
        if not os.path.exists(self.model_path):
            raise FileNotFoundError(f"YOLO model not found: {self.model_path}")
        try:
            from ultralytics import YOLO
        except Exception as exc:
            raise RuntimeError(
                "Missing Python package ultralytics. Install it with "
                "`python3 -m pip install --user ultralytics`."
            ) from exc
        return YOLO(self.model_path)

    def load_open3d(self):
        try:
            with warnings.catch_warnings():
                warnings.filterwarnings(
                    "ignore",
                    message="A NumPy version >=.* is required for this version of SciPy.*",
                    category=UserWarning,
                )
                import open3d as o3d
        except Exception as exc:
            raise RuntimeError(
                "Missing Python package open3d. Install it with "
                "`python3 -m pip install --user open3d`."
            ) from exc
        return o3d

    def image_callback(self, msg):
        self.latest_image = msg

    def cloud_callback(self, msg):
        self.latest_cloud = msg

    def camera_info_callback(self, msg):
        if msg.k[0] > 0.0 and msg.k[4] > 0.0:
            self.latest_camera_info = msg

    def process_latest(self):
        image_msg = self.latest_image
        cloud_msg = self.latest_cloud
        if image_msg is None or cloud_msg is None:
            return

        image_stamp = stamp_key(image_msg.header.stamp)
        if image_stamp == self.last_processed_image_stamp:
            return
        self.last_processed_image_stamp = image_stamp

        time_diff = abs(
            stamp_to_sec(image_msg.header.stamp)
            - stamp_to_sec(cloud_msg.header.stamp)
        )
        if self.max_sync_slop > 0.0 and time_diff > self.max_sync_slop:
            self.get_logger().warn(
                f"Skipping unsynchronized image/cloud pair, dt={time_diff:.3f}s",
                throttle_duration_sec=2.0,
            )
            self.publish_empty(
                cloud_msg.header.stamp,
                "sync_dt_exceeded",
                {"sync_dt": float(time_diff), "max_sync_slop": self.max_sync_slop},
            )
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
        except CvBridgeError as exc:
            self.get_logger().warn(f"Failed to convert image: {exc}")
            return

        cycle_start = time.monotonic()
        detect_start = time.monotonic()
        detections = self.detect_image(cv_image)
        inference_sec = time.monotonic() - detect_start
        debug_image = cv_image.copy()
        self.draw_yolo_detections(debug_image, detections)
        if not detections:
            self.draw_status_text(
                debug_image,
                f"YOLO:0 sync:{time_diff:.2f}s infer:{inference_sec:.2f}s",
            )
            self.publish_debug_image(debug_image, image_msg.header)
            self.publish_empty(
                cloud_msg.header.stamp,
                "no_yolo_detection",
                {
                    "sync_dt": float(time_diff),
                    "yolo_count": 0,
                    "inference_sec": float(inference_sec),
                    "cycle_sec": float(time.monotonic() - cycle_start),
                },
            )
            self.log_diagnostic(
                "no_yolo_detection",
                time_diff,
                0,
                0,
                0,
                inference_sec,
                time.monotonic() - cycle_start,
            )
            return

        points = self.read_xyz_points(cloud_msg)
        if points.size == 0:
            self.draw_status_text(
                debug_image,
                f"YOLO:{len(detections)} no cloud points",
            )
            self.publish_debug_image(debug_image, image_msg.header)
            self.publish_empty(
                cloud_msg.header.stamp,
                "empty_pointcloud",
                {
                    "sync_dt": float(time_diff),
                    "yolo_count": len(detections),
                    "inference_sec": float(inference_sec),
                },
            )
            return

        cloud_frame = self.message_frame(cloud_msg.header.frame_id, self.pointcloud_frame)
        camera_frame = self.resolve_camera_frame(image_msg)
        points = self.filter_points(points, cloud_msg.header, cloud_frame)
        if points.size == 0:
            self.draw_status_text(
                debug_image,
                f"YOLO:{len(detections)} no filtered points",
            )
            self.publish_debug_image(debug_image, image_msg.header)
            self.publish_empty(
                cloud_msg.header.stamp,
                "no_filtered_points",
                {
                    "sync_dt": float(time_diff),
                    "yolo_count": len(detections),
                    "inference_sec": float(inference_sec),
                },
            )
            return

        camera_matrix = self.camera_matrix(cv_image)
        cloud_to_camera = self.lookup_matrix(
            camera_frame,
            cloud_frame,
            cloud_msg.header.stamp,
            "camera projection",
            allow_fallback=True,
            fallback_xyz=self.fallback_lidar_to_camera_xyz,
        )
        if cloud_to_camera is None:
            self.draw_status_text(
                debug_image,
                f"YOLO:{len(detections)} missing cloud->camera TF",
            )
            self.publish_debug_image(debug_image, image_msg.header)
            self.publish_empty(
                cloud_msg.header.stamp,
                "missing_cloud_to_camera_tf",
                {
                    "sync_dt": float(time_diff),
                    "yolo_count": len(detections),
                    "inference_sec": float(inference_sec),
                },
            )
            return

        cloud_to_output = self.lookup_matrix(
            self.output_frame,
            cloud_frame,
            cloud_msg.header.stamp,
            "trash map output",
            allow_fallback=False,
        )
        if cloud_to_output is None:
            self.draw_status_text(
                debug_image,
                f"YOLO:{len(detections)} missing output TF",
            )
            self.publish_debug_image(debug_image, image_msg.header)
            self.publish_empty(
                cloud_msg.header.stamp,
                "missing_output_tf",
                {
                    "sync_dt": float(time_diff),
                    "yolo_count": len(detections),
                    "inference_sec": float(inference_sec),
                },
            )
            return

        cluster_start = time.monotonic()
        clusters = self.cluster_points(points)
        cluster_sec = time.monotonic() - cluster_start
        matches = self.match_detections(
            detections,
            clusters,
            points,
            cloud_to_camera,
            cloud_to_output,
            camera_matrix,
            debug_image,
        )

        output_header = self.make_output_header(cloud_msg.header.stamp)
        cycle_sec = time.monotonic() - cycle_start
        self.draw_status_text(
            debug_image,
            (
                f"YOLO:{len(detections)} clusters:{len(clusters)} "
                f"matches:{len(matches)} sync:{time_diff:.2f}s "
                f"infer:{inference_sec:.2f}s"
            ),
        )
        self.publish_results(
            output_header,
            matches,
            cloud_msg.header,
            time_diff,
            {
                "yolo_count": len(detections),
                "cluster_count": len(clusters),
                "filtered_point_count": int(points.shape[0]),
                "inference_sec": float(inference_sec),
                "cluster_sec": float(cluster_sec),
                "cycle_sec": float(cycle_sec),
            },
        )
        self.publish_debug_image(debug_image, image_msg.header)
        self.log_diagnostic(
            "processed",
            time_diff,
            len(detections),
            len(clusters),
            len(matches),
            inference_sec,
            cycle_sec,
        )
        self.log_fps()

    def detect_image(self, cv_image):
        kwargs = {
            "conf": self.yolo_confidence,
            "verbose": False,
        }
        if self.yolo_imgsz > 0:
            kwargs["imgsz"] = self.yolo_imgsz
        if self.yolo_max_det > 0:
            kwargs["max_det"] = self.yolo_max_det
        if self.yolo_device:
            kwargs["device"] = self.yolo_device

        results = self.model.predict(cv_image, **kwargs)
        detections = []
        for result in results:
            boxes = getattr(result, "boxes", None)
            if boxes is None:
                continue
            for box in boxes:
                xyxy = box.xyxy[0].detach().cpu().numpy().astype(float)
                confidence = float(box.conf[0].detach().cpu().item())
                class_id = int(box.cls[0].detach().cpu().item())
                class_name = self.class_name(class_id)
                if self.class_filter and class_name.lower() not in self.class_filter:
                    if str(class_id) not in self.class_filter:
                        continue
                detections.append(
                    {
                        "bbox": (
                            float(xyxy[0]),
                            float(xyxy[1]),
                            float(xyxy[2]),
                            float(xyxy[3]),
                        ),
                        "confidence": confidence,
                        "class_id": class_id,
                        "class_name": class_name,
                    }
                )
        return detections

    def class_name(self, class_id):
        names = getattr(self.model, "names", {})
        if isinstance(names, dict):
            return str(names.get(class_id, class_id))
        if 0 <= class_id < len(names):
            return str(names[class_id])
        return str(class_id)

    def read_xyz_points(self, msg):
        field_names = [field.name for field in msg.fields]
        if not {"x", "y", "z"}.issubset(field_names):
            self.get_logger().warn(
                "Skipping point cloud without x/y/z fields",
                throttle_duration_sec=2.0,
            )
            return np.empty((0, 3), dtype=np.float32)

        try:
            points = point_cloud2.read_points_numpy(
                msg, field_names=["x", "y", "z"], skip_nans=True
            )
        except (AssertionError, ValueError):
            structured = point_cloud2.read_points(
                msg, field_names=["x", "y", "z"], skip_nans=True
            )
            points = np.column_stack(
                (structured["x"], structured["y"], structured["z"])
            )

        if points.size == 0:
            return np.empty((0, 3), dtype=np.float32)
        if getattr(points.dtype, "names", None):
            points = np.column_stack((points["x"], points["y"], points["z"]))
        points = np.asarray(points, dtype=np.float32).reshape((-1, 3))
        finite = np.isfinite(points).all(axis=1)
        return points[finite]

    def filter_points(self, points, header, cloud_frame):
        if points.shape[0] == 0:
            return points

        if self.max_input_points > 0 and points.shape[0] > self.max_input_points:
            step = max(1, int(math.ceil(points.shape[0] / self.max_input_points)))
            points = points[::step]

        range_sq = np.einsum("ij,ij->i", points, points)
        mask = np.ones(points.shape[0], dtype=bool)
        if self.min_range > 0.0:
            mask &= range_sq >= self.min_range * self.min_range
        if self.max_range > self.min_range:
            mask &= range_sq <= self.max_range * self.max_range
        points = points[mask]
        if points.shape[0] == 0:
            return points

        base_matrix = self.lookup_matrix(
            self.base_frame,
            cloud_frame,
            header.stamp,
            "base-height filtering",
            allow_fallback=True,
            fallback_xyz=self.fallback_lidar_to_base_xyz,
        )
        if base_matrix is None:
            return points

        base_points = transform_points(points, base_matrix)
        z_mask = (base_points[:, 2] >= self.object_min_base_z) & (
            base_points[:, 2] <= self.object_max_base_z
        )
        return points[z_mask]

    def cluster_points(self, points):
        if points.shape[0] < self.cluster_min_points:
            return []

        pcd = self.o3d.geometry.PointCloud()
        pcd.points = self.o3d.utility.Vector3dVector(points.astype(np.float64))
        if self.cluster_voxel_size > 0.0:
            pcd = pcd.voxel_down_sample(self.cluster_voxel_size)

        cluster_points = np.asarray(pcd.points, dtype=np.float32)
        if cluster_points.shape[0] < self.cluster_min_points:
            return []

        with self.o3d.utility.VerbosityContextManager(
            self.o3d.utility.VerbosityLevel.Error
        ):
            labels = np.asarray(
                pcd.cluster_dbscan(
                    eps=self.cluster_eps,
                    min_points=self.cluster_min_points,
                    print_progress=False,
                )
            )

        max_label = int(labels.max()) if labels.size > 0 else -1
        if max_label < 0:
            return []

        clusters = []
        for label in range(max_label + 1):
            label_indices = np.where(labels == label)[0]
            count = int(label_indices.size)
            if count < self.cluster_min_points or count > self.cluster_max_points:
                continue
            cluster = cluster_points[label_indices]
            min_point = np.min(cluster, axis=0)
            max_point = np.max(cluster, axis=0)
            dims = max_point - min_point
            if dims[2] < self.cluster_min_height:
                continue
            if dims[2] > self.cluster_max_height:
                continue
            if max(dims[0], dims[1]) > self.cluster_max_extent_xy:
                continue
            clusters.append(
                {
                    "points": cluster,
                    "min": min_point,
                    "max": max_point,
                    "center": (min_point + max_point) * 0.5,
                    "dimensions": dims,
                }
            )
        return clusters

    def match_detections(
        self,
        detections,
        clusters,
        points,
        cloud_to_camera,
        cloud_to_output,
        camera_matrix,
        debug_image,
    ):
        cluster_projections = []
        for cluster_index, cluster in enumerate(clusters):
            projection = self.project_box(
                cluster["min"],
                cluster["max"],
                cloud_to_camera,
                camera_matrix,
                debug_image.shape[1],
                debug_image.shape[0],
            )
            if projection is None:
                continue
            projection["cluster_index"] = cluster_index
            cluster_projections.append(projection)
            self.draw_projected_box(debug_image, projection, (0, 0, 255), 1)

        candidates = []
        for projection in cluster_projections:
            for detection_index, detection in enumerate(detections):
                iou = bbox_iou(projection["bbox"], detection["bbox"])
                center_inside = point_inside_bbox(
                    projection["center_uv"], detection["bbox"]
                )
                if iou >= self.iou_threshold or (
                    self.allow_center_match and center_inside
                ):
                    score = iou + (0.2 if center_inside else 0.0)
                    score += detection["confidence"] * 0.01
                    candidates.append(
                        (
                            score,
                            iou,
                            center_inside,
                            projection["cluster_index"],
                            detection_index,
                            projection,
                        )
                    )

        candidates.sort(key=lambda item: item[0], reverse=True)
        used_clusters = set()
        used_detections = set()
        matches = []
        for score, iou, center_inside, cluster_index, detection_index, projection in candidates:
            if cluster_index in used_clusters or detection_index in used_detections:
                continue
            used_clusters.add(cluster_index)
            used_detections.add(detection_index)
            cluster = clusters[cluster_index]
            detection = detections[detection_index]
            match = self.make_match(
                detection,
                cluster["min"],
                cluster["max"],
                cloud_to_output,
                float(score),
                float(iou),
                "iou" if iou >= self.iou_threshold else "center",
            )
            matches.append(match)
            self.draw_match(debug_image, detection, projection, match)

        if self.use_roi_fallback and len(used_detections) < len(detections):
            roi_matches = self.roi_fallback_matches(
                detections,
                used_detections,
                points,
                cloud_to_camera,
                cloud_to_output,
                camera_matrix,
                debug_image,
            )
            matches.extend(roi_matches)

        return matches

    def roi_fallback_matches(
        self,
        detections,
        used_detections,
        points,
        cloud_to_camera,
        cloud_to_output,
        camera_matrix,
        debug_image,
    ):
        camera_points = transform_points(points, cloud_to_camera)
        optical_points = self.camera_points_to_optical(camera_points)
        uv, valid = self.project_points(optical_points, camera_matrix)
        if uv.size == 0:
            return []

        matches = []
        for detection_index, detection in enumerate(detections):
            if detection_index in used_detections:
                continue
            x1, y1, x2, y2 = detection["bbox"]
            mask = (
                valid
                & (uv[:, 0] >= x1)
                & (uv[:, 0] <= x2)
                & (uv[:, 1] >= y1)
                & (uv[:, 1] <= y2)
            )
            roi_points = points[mask]
            if roi_points.shape[0] < self.roi_min_points:
                continue

            min_point = np.min(roi_points, axis=0)
            max_point = np.max(roi_points, axis=0)
            dims = max_point - min_point
            if dims[2] > self.cluster_max_height:
                continue
            if max(dims[0], dims[1]) > self.cluster_max_extent_xy:
                continue

            match = self.make_match(
                detection,
                min_point,
                max_point,
                cloud_to_output,
                float(detection["confidence"]),
                0.0,
                "roi",
            )
            matches.append(match)
            cv2.putText(
                debug_image,
                f"roi {match['class_name']} {match['confidence']:.2f}",
                (int(x1), max(16, int(y1) - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 255),
                2,
            )
        return matches

    def make_match(
        self,
        detection,
        min_point,
        max_point,
        cloud_to_output,
        score,
        iou,
        match_type,
    ):
        corners_cloud = bbox_corners(min_point, max_point)
        corners_output = transform_points(corners_cloud, cloud_to_output)
        output_min = np.min(corners_output, axis=0)
        output_max = np.max(corners_output, axis=0)
        center = (output_min + output_max) * 0.5
        dimensions = np.maximum(output_max - output_min, 0.03)
        return {
            "class_id": int(detection["class_id"]),
            "class_name": detection["class_name"],
            "confidence": float(detection["confidence"]),
            "bbox2d": tuple(float(value) for value in detection["bbox"]),
            "center": center,
            "dimensions": dimensions,
            "min": output_min,
            "max": output_max,
            "score": score,
            "iou": iou,
            "match_type": match_type,
        }

    def project_box(self, min_point, max_point, cloud_to_camera, camera_matrix, width, height):
        corners = bbox_corners(min_point, max_point)
        camera_corners = transform_points(corners, cloud_to_camera)
        optical_corners = self.camera_points_to_optical(camera_corners)
        uv, valid = self.project_points(optical_corners, camera_matrix)
        if uv.shape[0] == 0 or np.count_nonzero(valid) < 4:
            return None

        visible_uv = uv[valid]
        if (
            np.all(visible_uv[:, 0] < 0.0)
            or np.all(visible_uv[:, 0] >= width)
            or np.all(visible_uv[:, 1] < 0.0)
            or np.all(visible_uv[:, 1] >= height)
        ):
            return None

        clipped_x = np.clip(visible_uv[:, 0], 0.0, float(width - 1))
        clipped_y = np.clip(visible_uv[:, 1], 0.0, float(height - 1))

        center_camera = transform_points(
            np.array([(min_point + max_point) * 0.5], dtype=np.float32),
            cloud_to_camera,
        )
        center_optical = self.camera_points_to_optical(center_camera)
        center_uv, center_valid = self.project_points(center_optical, camera_matrix)
        if center_uv.shape[0] == 0 or not bool(center_valid[0]):
            center_pixel = (
                float(np.mean(clipped_x)),
                float(np.mean(clipped_y)),
            )
        else:
            center_pixel = (float(center_uv[0, 0]), float(center_uv[0, 1]))

        return {
            "bbox": (
                float(np.min(clipped_x)),
                float(np.min(clipped_y)),
                float(np.max(clipped_x)),
                float(np.max(clipped_y)),
            ),
            "corners_uv": visible_uv,
            "center_uv": center_pixel,
        }

    def camera_points_to_optical(self, camera_points):
        if self.camera_frame_uses_optical_axes:
            return camera_points
        optical = np.empty_like(camera_points)
        optical[:, 0] = -camera_points[:, 1]
        optical[:, 1] = -camera_points[:, 2]
        optical[:, 2] = camera_points[:, 0]
        return optical

    def project_points(self, optical_points, camera_matrix):
        if optical_points.shape[0] == 0:
            return np.empty((0, 2), dtype=np.float32), np.zeros(0, dtype=bool)
        z = optical_points[:, 2]
        valid = z > 0.05
        uv = np.zeros((optical_points.shape[0], 2), dtype=np.float32)
        safe_z = np.where(valid, z, 1.0)
        uv[:, 0] = camera_matrix[0, 0] * optical_points[:, 0] / safe_z
        uv[:, 0] += camera_matrix[0, 2]
        uv[:, 1] = camera_matrix[1, 1] * optical_points[:, 1] / safe_z
        uv[:, 1] += camera_matrix[1, 2]
        return uv, valid

    def camera_matrix(self, cv_image):
        camera_info = self.latest_camera_info
        if (
            camera_info is not None
            and camera_info.k[0] > 0.0
            and camera_info.k[4] > 0.0
        ):
            return np.array(camera_info.k, dtype=np.float32).reshape((3, 3))

        if not self.warned_fallback_camera_info:
            self.get_logger().warn(
                "CameraInfo is not available yet; using fallback camera intrinsics"
            )
            self.warned_fallback_camera_info = True
        image_height, image_width = cv_image.shape[:2]
        width = image_width if image_width > 0 else self.fallback_image_width
        height = image_height if image_height > 0 else self.fallback_image_height
        fx = width / (2.0 * math.tan(self.fallback_horizontal_fov * 0.5))
        fy = fx
        cx = (width - 1.0) * 0.5
        cy = (height - 1.0) * 0.5
        return np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]], dtype=np.float32)

    def lookup_matrix(
        self,
        target_frame,
        source_frame,
        stamp,
        purpose,
        allow_fallback=False,
        fallback_xyz=None,
    ):
        target_frame = self.message_frame(target_frame, "")
        source_frame = self.message_frame(source_frame, "")
        if not target_frame or not source_frame:
            return None
        if target_frame == source_frame:
            return np.eye(4, dtype=np.float32)
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                Time.from_msg(stamp),
                timeout=Duration(seconds=self.tf_timeout),
            )
            return transform_to_matrix(transform)
        except TransformException as exc:
            if allow_fallback and fallback_xyz is not None:
                warning_key = (target_frame, source_frame, purpose)
                if warning_key not in self.warned_fallbacks:
                    self.get_logger().warn(
                        f"Using fallback transform for {purpose}: "
                        f"{target_frame} <- {source_frame}; TF error: {exc}"
                    )
                    self.warned_fallbacks.add(warning_key)
                matrix = np.eye(4, dtype=np.float32)
                matrix[:3, 3] = fallback_xyz
                return matrix
            self.get_logger().warn(
                f"Skipping {purpose}; missing TF {target_frame} <- {source_frame}: {exc}",
                throttle_duration_sec=2.0,
            )
            return None

    def message_frame(self, frame_id, fallback):
        frame = str(frame_id or "").strip()
        if frame:
            return frame[1:] if frame.startswith("/") else frame
        fallback = str(fallback or "").strip()
        return fallback[1:] if fallback.startswith("/") else fallback

    def resolve_camera_frame(self, image_msg):
        if self.camera_frame:
            return self.message_frame(self.camera_frame, "")
        camera_info = self.latest_camera_info
        if camera_info is not None and camera_info.header.frame_id:
            return self.message_frame(camera_info.header.frame_id, "")
        return self.message_frame(image_msg.header.frame_id, "front_camera_link")

    def make_output_header(self, stamp):
        header = PoseArray().header
        header.stamp = stamp
        header.frame_id = self.output_frame
        return header

    def publish_empty(self, stamp, reason="empty", diagnostics=None):
        header = self.make_output_header(stamp)
        self.marker_pub.publish(self.delete_markers(header))
        pose_array = PoseArray()
        pose_array.header = header
        self.pose_pub.publish(pose_array)
        payload = {
            "stamp": {
                "sec": int(header.stamp.sec),
                "nanosec": int(header.stamp.nanosec),
            },
            "frame_id": self.output_frame,
            "reason": reason,
            "detections": [],
        }
        if diagnostics:
            payload["diagnostics"] = diagnostics
        self.detection_pub.publish(
            String(data=json.dumps(payload, separators=(",", ":")))
        )

    def publish_results(self, header, matches, source_header, time_diff, diagnostics):
        marker_array = self.delete_markers(header)
        pose_array = PoseArray()
        pose_array.header = header
        payload = {
            "stamp": {
                "sec": int(header.stamp.sec),
                "nanosec": int(header.stamp.nanosec),
            },
            "frame_id": header.frame_id,
            "source_frame": self.message_frame(source_header.frame_id, self.pointcloud_frame),
            "sync_dt": float(time_diff),
            "diagnostics": diagnostics,
            "detections": [],
        }

        next_marker_id = 1
        for index, match in enumerate(matches):
            center = match["center"]
            dimensions = match["dimensions"]
            pose = Pose()
            pose.position.x = float(center[0])
            pose.position.y = float(center[1])
            pose.position.z = float(center[2])
            pose.orientation.w = 1.0
            pose_array.poses.append(pose)

            cube, next_marker_id = self.make_cube_marker(
                header, next_marker_id, match
            )
            center_marker, next_marker_id = self.make_center_marker(
                header, next_marker_id, match
            )
            text_marker, next_marker_id = self.make_text_marker(
                header, next_marker_id, match, index
            )
            marker_array.markers.extend([cube, center_marker, text_marker])

            payload["detections"].append(
                {
                    "id": index,
                    "class_id": match["class_id"],
                    "class_name": match["class_name"],
                    "confidence": match["confidence"],
                    "match_type": match["match_type"],
                    "score": match["score"],
                    "iou": match["iou"],
                    "bbox2d": list(match["bbox2d"]),
                    "center": {
                        "x": float(center[0]),
                        "y": float(center[1]),
                        "z": float(center[2]),
                    },
                    "dimensions": {
                        "x": float(dimensions[0]),
                        "y": float(dimensions[1]),
                        "z": float(dimensions[2]),
                    },
                }
            )

        self.marker_pub.publish(marker_array)
        self.pose_pub.publish(pose_array)
        self.detection_pub.publish(
            String(data=json.dumps(payload, separators=(",", ":")))
        )

    def delete_markers(self, header):
        marker = Marker()
        marker.header = header
        marker.ns = "trash_fusion"
        marker.id = 0
        marker.action = Marker.DELETEALL
        marker_array = MarkerArray()
        marker_array.markers.append(marker)
        return marker_array

    def make_cube_marker(self, header, marker_id, match):
        marker = Marker()
        marker.header = header
        marker.ns = "trash_fusion_box"
        marker.id = marker_id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = float(match["center"][0])
        marker.pose.position.y = float(match["center"][1])
        marker.pose.position.z = float(match["center"][2])
        marker.pose.orientation.w = 1.0
        marker.scale.x = max(float(match["dimensions"][0]), 0.06)
        marker.scale.y = max(float(match["dimensions"][1]), 0.06)
        marker.scale.z = max(float(match["dimensions"][2]), 0.06)
        r, g, b = self.class_color(match["class_name"])
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.color.a = 0.35
        marker.lifetime = Duration(seconds=self.marker_lifetime).to_msg()
        return marker, marker_id + 1

    def make_center_marker(self, header, marker_id, match):
        marker = Marker()
        marker.header = header
        marker.ns = "trash_fusion_center"
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = float(match["center"][0])
        marker.pose.position.y = float(match["center"][1])
        marker.pose.position.z = float(match["center"][2])
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.18
        marker.scale.y = 0.18
        marker.scale.z = 0.18
        marker.color.r = 1.0
        marker.color.g = 0.95
        marker.color.b = 0.10
        marker.color.a = 1.0
        marker.lifetime = Duration(seconds=self.marker_lifetime).to_msg()
        return marker, marker_id + 1

    def make_text_marker(self, header, marker_id, match, index):
        marker = Marker()
        marker.header = header
        marker.ns = "trash_fusion_label"
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = float(match["center"][0])
        marker.pose.position.y = float(match["center"][1])
        marker.pose.position.z = float(match["max"][2]) + 0.35
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.32
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        marker.text = (
            f"{index}:{match['class_name']} "
            f"{match['confidence']:.2f} {match['match_type']}"
        )
        marker.lifetime = Duration(seconds=self.marker_lifetime).to_msg()
        return marker, marker_id + 1

    def class_color(self, class_name):
        palette = {
            "trash": (1.0, 0.35, 0.10),
            "cardboard": (0.95, 0.70, 0.25),
            "glass": (0.15, 0.75, 1.0),
            "metal": (0.80, 0.80, 0.85),
            "paper": (0.95, 0.95, 0.90),
            "plastic": (0.20, 1.0, 0.55),
            "bottle": (0.20, 0.75, 1.0),
            "can": (1.0, 0.45, 0.20),
        }
        return palette.get(class_name.lower(), (0.90, 0.20, 0.85))

    def draw_yolo_detections(self, image, detections):
        for detection in detections:
            x1, y1, x2, y2 = detection["bbox"]
            cv2.rectangle(
                image,
                (int(x1), int(y1)),
                (int(x2), int(y2)),
                (255, 80, 0),
                2,
            )
            label = f"{detection['class_name']} {detection['confidence']:.2f}"
            cv2.putText(
                image,
                label,
                (int(x1), max(16, int(y1) - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 80, 0),
                2,
            )

    def draw_projected_box(self, image, projection, color, thickness):
        x1, y1, x2, y2 = projection["bbox"]
        cv2.rectangle(
            image,
            (int(x1), int(y1)),
            (int(x2), int(y2)),
            color,
            thickness,
        )

    def draw_match(self, image, detection, projection, match):
        self.draw_projected_box(image, projection, (0, 255, 0), 2)
        x1, y1, _, _ = detection["bbox"]
        label = (
            f"3D {match['class_name']} {match['confidence']:.2f} "
            f"{match['match_type']}"
        )
        cv2.putText(
            image,
            label,
            (int(x1), max(32, int(y1) + 18)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 0),
            2,
        )

    def draw_status_text(self, image, text):
        cv2.rectangle(image, (6, 6), (min(image.shape[1] - 1, 740), 38), (0, 0, 0), -1)
        cv2.putText(
            image,
            text,
            (14, 29),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.58,
            (255, 255, 255),
            2,
        )

    def publish_debug_image(self, image, header):
        try:
            msg = self.bridge.cv2_to_imgmsg(image, "bgr8")
            msg.header = header
            self.debug_image_pub.publish(msg)
        except CvBridgeError as exc:
            self.get_logger().warn(f"Failed to publish debug image: {exc}")

    def log_diagnostic(
        self,
        reason,
        sync_dt,
        yolo_count,
        cluster_count,
        match_count,
        inference_sec,
        cycle_sec,
    ):
        self.get_logger().info(
            "Trash fusion diagnostic: "
            f"reason={reason}, sync_dt={sync_dt:.3f}s, yolo={yolo_count}, "
            f"clusters={cluster_count}, matches={match_count}, "
            f"inference={inference_sec:.2f}s, cycle={cycle_sec:.2f}s",
            throttle_duration_sec=2.0,
        )

    def log_fps(self):
        self.frames += 1
        if self.frames % 10 != 0:
            return
        elapsed = max(time.monotonic() - self.start_wall_time, 1e-6)
        self.get_logger().info(f"Trash fusion rate: {self.frames / elapsed:.2f} Hz")


def main(args=None):
    rclpy.init(args=args)
    node = TrashFusionDetector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
