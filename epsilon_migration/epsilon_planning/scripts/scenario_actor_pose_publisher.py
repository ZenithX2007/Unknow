#!/usr/bin/env python3

import math
import xml.etree.ElementTree as ET
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy


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
            duration = last_time - first_time
            t = ((t - first_time) % duration) + first_time

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
                return interpolate_pose(pose0, pose1, ratio)

        return self.waypoints[-1][1]


class ScenarioActorPosePublisher(Node):
    def __init__(self):
        super().__init__("epsilon_scenario_actor_pose_publisher")

        self.declare_parameter("actors_scenario_path", "")
        self.declare_parameter("topic_prefix", "/epsilon/scenario_actor")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("publish_rate", 10.0)
        self.declare_parameter("world_sdf_path", "")
        self.declare_parameter("world_vehicle_name", "gen0_model")
        self.declare_parameter("transform_world_to_output", True)
        self.declare_parameter("output_origin_xy", "0.0,0.0")

        self.scenario_path = str(self.get_parameter("actors_scenario_path").value)
        self.topic_prefix = str(self.get_parameter("topic_prefix").value).rstrip("/")
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.publish_rate = max(0.1, float(self.get_parameter("publish_rate").value))
        self.world_sdf_path = str(self.get_parameter("world_sdf_path").value)
        self.world_vehicle_name = str(self.get_parameter("world_vehicle_name").value)
        transform_world_to_output = self.get_parameter("transform_world_to_output").value
        self.transform_world_to_output = (
            text_bool(transform_world_to_output)
            if isinstance(transform_world_to_output, str)
            else bool(transform_world_to_output)
        )
        self.output_origin_xy = parse_vector2(
            self.get_parameter("output_origin_xy").value, (0.0, 0.0)
        )
        self.world_origin_xy = (0.0, 0.0)
        self.world_origin_yaw = 0.0

        self.load_world_origin()
        self.trajectories = self.load_trajectories()
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.actor_publishers = {
            name: self.create_publisher(PoseStamped, self.topic_for_actor(name), qos)
            for name in sorted(self.trajectories)
        }
        self.create_timer(1.0 / self.publish_rate, self.publish_actor_poses)

        self.get_logger().info(
            "scenario actor pose publisher: actors=%d scenario=%s prefix=%s "
            "frame=%s rate=%.1fHz world_to_output=%s origin=(%.2f, %.2f, %.3f)"
            % (
                len(self.trajectories),
                self.scenario_path or "none",
                self.topic_prefix or "/",
                self.frame_id,
                self.publish_rate,
                self.transform_world_to_output,
                self.world_origin_xy[0],
                self.world_origin_xy[1],
                self.world_origin_yaw,
            )
        )

    def topic_for_actor(self, actor_name):
        if self.topic_prefix:
            return "%s/%s/pose" % (self.topic_prefix, actor_name)
        return "/%s/pose" % actor_name

    def load_world_origin(self):
        if not self.transform_world_to_output or not self.world_sdf_path:
            return

        path = Path(self.world_sdf_path)
        if not path.exists():
            self.get_logger().warn("World SDF path does not exist: %s" % path)
            return

        try:
            root = ET.parse(path).getroot()
        except ET.ParseError as exc:
            self.get_logger().warn("Could not parse world SDF %s: %s" % (path, exc))
            return

        model = root.find(".//model[@name='%s']" % self.world_vehicle_name)
        if model is None:
            self.get_logger().warn(
                "Could not find model '%s' in %s" % (self.world_vehicle_name, path)
            )
            return

        pose = parse_pose(model.findtext("pose", default=""))
        if pose is None:
            self.get_logger().warn(
                "Could not read pose for model '%s' in %s"
                % (self.world_vehicle_name, path)
            )
            return

        self.world_origin_xy = (pose[0], pose[1])
        self.world_origin_yaw = pose[5]

    def load_trajectories(self):
        if not self.scenario_path:
            return {}

        path = Path(self.scenario_path)
        if not path.exists():
            self.get_logger().warn("Actor scenario path does not exist: %s" % path)
            return {}

        try:
            root = ET.parse(path).getroot()
        except ET.ParseError as exc:
            self.get_logger().warn("Could not parse actor scenario %s: %s" % (path, exc))
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

            loop = text_bool(script.findtext("loop", default="true"))
            delay = text_float(script.findtext("delay_start", default="0.0"), 0.0)
            waypoints = []
            for waypoint in trajectory.findall("waypoint"):
                time_sec = text_float(waypoint.findtext("time"), None)
                pose = parse_pose(waypoint.findtext("pose", default=""))
                if time_sec is None or pose is None:
                    continue
                waypoints.append((time_sec, pose))

            if waypoints:
                trajectories[name] = ActorTrajectory(name, waypoints, loop, delay)

        return trajectories

    def now_sec(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def world_to_output_pose(self, pose):
        if not self.transform_world_to_output:
            return pose

        dx = pose[0] - self.world_origin_xy[0]
        dy = pose[1] - self.world_origin_xy[1]
        cos_yaw = math.cos(self.world_origin_yaw)
        sin_yaw = math.sin(self.world_origin_yaw)
        x = cos_yaw * dx + sin_yaw * dy + self.output_origin_xy[0]
        y = -sin_yaw * dx + cos_yaw * dy + self.output_origin_xy[1]
        yaw = normalize_angle(pose[5] - self.world_origin_yaw)
        return (x, y, 0.0, pose[3], pose[4], yaw)

    def publish_actor_poses(self):
        now = self.now_sec()
        stamp = self.get_clock().now().to_msg()
        for name, trajectory in self.trajectories.items():
            pose = trajectory.pose_at(now)
            if pose is None:
                continue
            pose = self.world_to_output_pose(pose)

            msg = PoseStamped()
            msg.header.stamp = stamp
            msg.header.frame_id = self.frame_id
            msg.pose.position.x = pose[0]
            msg.pose.position.y = pose[1]
            msg.pose.position.z = pose[2]
            msg.pose.orientation.z = math.sin(pose[5] * 0.5)
            msg.pose.orientation.w = math.cos(pose[5] * 0.5)
            self.actor_publishers[name].publish(msg)


def parse_pose(value):
    try:
        parts = [float(part) for part in str(value).split()]
    except ValueError:
        return None
    if len(parts) < 2:
        return None
    while len(parts) < 6:
        parts.append(0.0)
    return tuple(parts[:6])


def parse_vector2(value, fallback):
    try:
        if isinstance(value, str):
            values = [float(part) for part in value.split(",") if part.strip()]
        else:
            values = [float(part) for part in value]
    except (TypeError, ValueError):
        return fallback
    if len(values) != 2:
        return fallback
    return (values[0], values[1])


def text_bool(value):
    return str(value).strip().lower() not in ("false", "0", "no")


def text_float(value, fallback):
    try:
        return float(value)
    except (TypeError, ValueError):
        return fallback


def normalize_angle(value):
    return math.atan2(math.sin(value), math.cos(value))


def interpolate_angle(start, end, ratio):
    return start + normalize_angle(end - start) * ratio


def interpolate_pose(start, end, ratio):
    return (
        start[0] + (end[0] - start[0]) * ratio,
        start[1] + (end[1] - start[1]) * ratio,
        start[2] + (end[2] - start[2]) * ratio,
        interpolate_angle(start[3], end[3], ratio),
        interpolate_angle(start[4], end[4], ratio),
        interpolate_angle(start[5], end[5], ratio),
    )


def main(args=None):
    rclpy.init(args=args)
    node = ScenarioActorPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
