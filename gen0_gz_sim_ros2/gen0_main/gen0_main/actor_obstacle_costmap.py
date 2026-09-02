#!/usr/bin/env python3

import math
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


FIELDS_XYZI = [
    PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
]


class ActorTrajectory:
    def __init__(self, name, waypoints, loop=True, delay=0.0):
        self.name = name
        self.waypoints = sorted(waypoints, key=lambda item: item[0])
        self.loop = loop
        self.delay = max(0.0, float(delay))

    def pose_at(self, time_sec):
        if not self.waypoints:
            return None

        t = max(0.0, float(time_sec) - self.delay)
        first_time = self.waypoints[0][0]
        last_time = self.waypoints[-1][0]
        if self.loop and last_time > first_time:
            t = ((t - first_time) % (last_time - first_time)) + first_time

        if t <= first_time:
            return self.waypoints[0][1]
        if t >= last_time:
            return self.waypoints[-1][1]

        for index in range(len(self.waypoints) - 1):
            t0, pose0 = self.waypoints[index]
            t1, pose1 = self.waypoints[index + 1]
            if t0 <= t <= t1:
                if t1 <= t0:
                    return pose1
                ratio = (t - t0) / (t1 - t0)
                return pose0 + (pose1 - pose0) * ratio

        return self.waypoints[-1][1]


class ActorObstacleCostmap(Node):
    def __init__(self):
        super().__init__("actor_obstacle_costmap")

        self.declare_parameter("actors_scenario_path", "")
        self.declare_parameter("actor_pose_topics", "")
        self.declare_parameter("output_topic", "/gen0_mapping/actor_obstacles")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("world_sdf_path", "")
        self.declare_parameter("world_vehicle_name", "gen0_model")
        self.declare_parameter("transform_world_to_output", True)
        self.declare_parameter("output_origin_xy", "0.0,0.0")
        self.declare_parameter("publish_rate", 10.0)
        self.declare_parameter("live_pose_timeout", 1.0)
        self.declare_parameter("actor_radius", 0.45)
        self.declare_parameter("actor_z_min", 0.15)
        self.declare_parameter("actor_z_max", 1.45)
        self.declare_parameter("actor_mark_intensity", 0.6)
        self.declare_parameter("actor_clear_intensity", 0.0)
        self.declare_parameter("actor_radial_samples", 24)
        self.declare_parameter("actor_height_samples", 4)

        self.scenario_path = self.get_parameter("actors_scenario_path").value
        self.actor_pose_topics = self.string_list_parameter("actor_pose_topics")
        self.output_topic = self.get_parameter("output_topic").value
        self.frame_id = self.get_parameter("frame_id").value
        self.world_sdf_path = self.get_parameter("world_sdf_path").value
        self.world_vehicle_name = self.get_parameter("world_vehicle_name").value
        self.transform_world_to_output = bool(
            self.get_parameter("transform_world_to_output").value
        )
        self.output_origin_xy = self.vector2_parameter("output_origin_xy", [0.0, 0.0])
        self.publish_rate = max(0.1, float(self.get_parameter("publish_rate").value))
        self.live_pose_timeout = max(
            0.0, float(self.get_parameter("live_pose_timeout").value)
        )
        self.actor_radius = max(0.05, float(self.get_parameter("actor_radius").value))
        self.actor_z_min = float(self.get_parameter("actor_z_min").value)
        self.actor_z_max = float(self.get_parameter("actor_z_max").value)
        self.actor_mark_intensity = float(
            self.get_parameter("actor_mark_intensity").value
        )
        self.actor_clear_intensity = float(
            self.get_parameter("actor_clear_intensity").value
        )
        self.actor_radial_samples = max(
            8, int(self.get_parameter("actor_radial_samples").value)
        )
        self.actor_height_samples = max(
            1, int(self.get_parameter("actor_height_samples").value)
        )

        self.world_origin_xy = np.zeros(2, dtype=np.float32)
        self.world_origin_yaw = 0.0
        self.load_world_origin()
        self.trajectories = self.load_trajectories(self.scenario_path)
        self.live_positions = {}
        self.previous_centers = np.empty((0, 2), dtype=np.float32)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.actor_subscriptions = [
            self.create_subscription(
                PoseStamped,
                topic,
                lambda msg, actor_topic=topic: self.pose_callback(actor_topic, msg),
                qos,
            )
            for topic in self.actor_pose_topics
        ]
        self.publisher = self.create_publisher(PointCloud2, self.output_topic, 10)
        self.create_timer(1.0 / self.publish_rate, self.publish_obstacles)

        self.get_logger().info(
            f"Publishing actor costmap obstacles on {self.output_topic}, "
            f"scenario_actors={len(self.trajectories)}, live_topics={len(self.actor_pose_topics)}, "
            f"frame={self.frame_id}, radius={self.actor_radius:.2f}, "
            f"z=[{self.actor_z_min:.2f}, {self.actor_z_max:.2f}], "
            f"mark_intensity={self.actor_mark_intensity:.2f}, "
            f"world_to_output={self.transform_world_to_output}, "
            f"world_origin=({self.world_origin_xy[0]:.2f}, {self.world_origin_xy[1]:.2f}, "
            f"yaw={self.world_origin_yaw:.3f}), "
            f"output_origin=({self.output_origin_xy[0]:.2f}, {self.output_origin_xy[1]:.2f})"
        )

    def string_list_parameter(self, name):
        value = self.get_parameter(name).value
        if isinstance(value, str):
            return [item.strip() for item in value.split(",") if item.strip()]
        try:
            return [str(item).strip() for item in value if str(item).strip()]
        except TypeError:
            return []

    def vector2_parameter(self, name, fallback):
        value = self.get_parameter(name).value
        try:
            if isinstance(value, str):
                values = [float(item) for item in value.split(",") if item.strip()]
            else:
                values = [float(item) for item in value]
            values = [float(item) for item in values]
        except (TypeError, ValueError):
            values = list(fallback)
        if len(values) != 2:
            self.get_logger().warn(
                f"Parameter {name} must contain exactly 2 values; using {fallback}"
            )
            values = list(fallback)
        return np.asarray(values, dtype=np.float32)

    def load_world_origin(self):
        if not self.transform_world_to_output or not self.world_sdf_path:
            return

        path = Path(self.world_sdf_path)
        if not path.exists():
            self.get_logger().warn(f"World SDF path does not exist: {path}")
            return

        try:
            root = ET.parse(path).getroot()
        except ET.ParseError as error:
            self.get_logger().warn(f"Could not parse world SDF {path}: {error}")
            return

        model = root.find(f".//model[@name='{self.world_vehicle_name}']")
        if model is None:
            self.get_logger().warn(
                f"Could not find model '{self.world_vehicle_name}' in {path}"
            )
            return

        pose = self.parse_pose(model.findtext("pose", default=""))
        if pose is None:
            self.get_logger().warn(
                f"Could not read pose for model '{self.world_vehicle_name}' in {path}"
            )
            return

        self.world_origin_xy = pose[:2].astype(np.float32, copy=False)
        self.world_origin_yaw = float(pose[5])

    def load_trajectories(self, scenario_path):
        if not scenario_path:
            return {}

        path = Path(scenario_path)
        if not path.exists():
            self.get_logger().warn(f"Actor scenario path does not exist: {path}")
            return {}

        try:
            root = ET.parse(path).getroot()
        except ET.ParseError as error:
            self.get_logger().warn(f"Could not parse actor scenario {path}: {error}")
            return {}

        trajectories = {}
        for actor in root.findall(".//actor"):
            name = actor.get("name", "").strip()
            if not name:
                continue

            script = actor.find("script")
            trajectory = actor.find("./script/trajectory")
            if script is None or trajectory is None:
                continue

            loop = self.text_bool(script.findtext("loop", default="true"))
            delay = self.text_float(script.findtext("delay_start", default="0.0"), 0.0)
            waypoints = []
            for waypoint in trajectory.findall("waypoint"):
                time_sec = self.text_float(waypoint.findtext("time"), None)
                pose_text = waypoint.findtext("pose")
                if time_sec is None or not pose_text:
                    continue
                pose_values = self.parse_pose(pose_text)
                if pose_values is None:
                    continue
                waypoints.append((time_sec, pose_values))

            if waypoints:
                trajectories[name] = ActorTrajectory(name, waypoints, loop, delay)

        return trajectories

    @staticmethod
    def text_bool(value):
        return str(value).strip().lower() not in ("false", "0", "no")

    @staticmethod
    def text_float(value, fallback):
        try:
            return float(value)
        except (TypeError, ValueError):
            return fallback

    @staticmethod
    def parse_pose(value):
        try:
            parts = [float(part) for part in value.split()]
        except ValueError:
            return None
        if len(parts) < 2:
            return None
        while len(parts) < 6:
            parts.append(0.0)
        return np.asarray(parts[:6], dtype=np.float32)

    @staticmethod
    def actor_name_from_topic(topic):
        parts = [part for part in topic.split("/") if part]
        if len(parts) >= 2 and parts[-1] == "pose":
            return parts[-2]
        return topic.strip("/")

    def pose_callback(self, topic, msg):
        actor_name = self.actor_name_from_topic(topic)
        xy = np.asarray([msg.pose.position.x, msg.pose.position.y], dtype=np.float32)
        self.live_positions[actor_name] = (
            self.world_to_output_xy(xy),
            self.now_sec(),
        )

    def now_sec(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def actor_centers(self, now_sec):
        centers = []
        active_live = set()
        for actor_name, (xy, seen_time) in self.live_positions.items():
            if now_sec - seen_time <= self.live_pose_timeout:
                centers.append(xy)
                active_live.add(actor_name)

        for actor_name, trajectory in self.trajectories.items():
            if actor_name in active_live:
                continue
            pose = trajectory.pose_at(now_sec)
            if pose is not None:
                centers.append(self.world_to_output_xy(pose[:2]))

        if not centers:
            return np.empty((0, 2), dtype=np.float32)
        return np.vstack(centers).astype(np.float32, copy=False)

    def world_to_output_xy(self, xy):
        point = np.asarray(xy, dtype=np.float32)
        if not self.transform_world_to_output:
            return point

        delta = point - self.world_origin_xy
        cos_yaw = math.cos(self.world_origin_yaw)
        sin_yaw = math.sin(self.world_origin_yaw)
        return np.asarray(
            [
                cos_yaw * delta[0] + sin_yaw * delta[1] + self.output_origin_xy[0],
                -sin_yaw * delta[0] + cos_yaw * delta[1] + self.output_origin_xy[1],
            ],
            dtype=np.float32,
        )

    def build_actor_points(self, centers, intensity):
        if centers.size == 0:
            return np.empty((0, 4), dtype=np.float32)

        angles = np.linspace(0.0, 2.0 * math.pi, self.actor_radial_samples, endpoint=False)
        ring_unit = np.column_stack((np.cos(angles), np.sin(angles))).astype(np.float32)
        levels = np.linspace(
            self.actor_z_min, self.actor_z_max, self.actor_height_samples, dtype=np.float32
        )
        radii = np.asarray(
            [0.0, self.actor_radius * 0.5, self.actor_radius], dtype=np.float32
        )

        clouds = []
        for center in centers:
            for z in levels:
                for radius in radii:
                    if radius <= 1e-6:
                        points = np.asarray([[center[0], center[1], z]], dtype=np.float32)
                    else:
                        xy = center + ring_unit * radius
                        z_column = np.full((xy.shape[0], 1), z, dtype=np.float32)
                        points = np.hstack((xy, z_column))
                    clouds.append(points)

        xyz = np.vstack(clouds).astype(np.float32, copy=False)
        values = np.full((xyz.shape[0], 1), intensity, dtype=np.float32)
        return np.hstack((xyz, values)).astype(np.float32, copy=False)

    def publish_obstacles(self):
        now = self.now_sec()
        current_centers = self.actor_centers(now)
        mark_points = self.build_actor_points(current_centers, self.actor_mark_intensity)
        clear_points = self.build_actor_points(
            self.previous_centers, self.actor_clear_intensity
        )
        self.previous_centers = current_centers.copy()

        if mark_points.size and clear_points.size:
            points = np.vstack((clear_points, mark_points))
        elif mark_points.size:
            points = mark_points
        elif clear_points.size:
            points = clear_points
        else:
            return

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.frame_id
        self.publisher.publish(self.create_cloud(header, points))

    @staticmethod
    def create_cloud(header, points):
        cloud = np.ascontiguousarray(points.astype(np.float32, copy=False))
        msg = PointCloud2()
        msg.header = header
        msg.height = 1
        msg.width = int(cloud.shape[0])
        msg.fields = FIELDS_XYZI
        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step = msg.point_step * msg.width
        msg.data = cloud.tobytes()
        msg.is_dense = False
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = ActorObstacleCostmap()
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
