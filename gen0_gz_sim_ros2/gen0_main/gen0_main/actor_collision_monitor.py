#!/usr/bin/env python3

import json
import math
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import PoseArray, PoseStamped
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


CLEAR = 0
NEAR_MISS = 1
COLLISION = 2


@dataclass
class TrackedActor:
    name: str
    topic: str
    x: float
    y: float
    z: float
    stamp_sec: float


class ActorCollisionMonitor(Node):
    def __init__(self):
        super().__init__("actor_collision_monitor")

        self.declare_parameter("actor_pose_topics", "")
        self.declare_parameter("vehicle_pose_topic", "/gen0_model/links/poses")
        self.declare_parameter("vehicle_pose_index", 15)
        self.declare_parameter("event_topic", "/gen0_validation/actor_collision_events")
        self.declare_parameter("watchdog_rate", 20.0)
        self.declare_parameter("actor_pose_timeout", 1.0)
        self.declare_parameter("vehicle_pose_timeout", 1.0)
        self.declare_parameter("vehicle_length", 4.0)
        self.declare_parameter("vehicle_width", 2.0)
        self.declare_parameter("vehicle_padding", 0.05)
        self.declare_parameter("actor_radius", 0.45)
        self.declare_parameter("near_margin", 0.75)
        self.declare_parameter("collision_margin", 0.05)

        self.actor_pose_topics = self.string_list_parameter("actor_pose_topics")
        self.vehicle_pose_topic = self.get_parameter("vehicle_pose_topic").value
        self.vehicle_pose_index = max(
            0, int(self.get_parameter("vehicle_pose_index").value)
        )
        self.event_topic = self.get_parameter("event_topic").value
        self.watchdog_rate = max(1.0, float(self.get_parameter("watchdog_rate").value))
        self.actor_pose_timeout = max(
            0.0, float(self.get_parameter("actor_pose_timeout").value)
        )
        self.vehicle_pose_timeout = max(
            0.0, float(self.get_parameter("vehicle_pose_timeout").value)
        )
        self.vehicle_length = max(0.1, float(self.get_parameter("vehicle_length").value))
        self.vehicle_width = max(0.1, float(self.get_parameter("vehicle_width").value))
        self.vehicle_padding = max(
            0.0, float(self.get_parameter("vehicle_padding").value)
        )
        self.actor_radius = max(0.05, float(self.get_parameter("actor_radius").value))
        self.collision_margin = max(
            0.0, float(self.get_parameter("collision_margin").value)
        )
        self.near_margin = max(
            self.collision_margin, float(self.get_parameter("near_margin").value)
        )

        self.vehicle_pose = None
        self.vehicle_stamp_sec = None
        self.actors = {}
        self.actor_levels = {}
        self.last_warn_sec = {}

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.vehicle_subscription = self.create_subscription(
            PoseArray, self.vehicle_pose_topic, self.vehicle_callback, qos
        )
        self.actor_subscriptions = [
            self.create_subscription(
                PoseStamped,
                topic,
                lambda msg, actor_topic=topic: self.actor_callback(actor_topic, msg),
                qos,
            )
            for topic in self.actor_pose_topics
        ]
        self.event_publisher = self.create_publisher(String, self.event_topic, 10)
        self.create_timer(1.0 / self.watchdog_rate, self.check_collisions)

        half_l = self.vehicle_length * 0.5 + self.vehicle_padding
        half_w = self.vehicle_width * 0.5 + self.vehicle_padding
        self.get_logger().info(
            f"Monitoring actor collisions on {self.event_topic}, "
            f"actors={len(self.actor_pose_topics)}, vehicle_pose={self.vehicle_pose_topic}"
            f"[{self.vehicle_pose_index}], vehicle_box={half_l * 2.0:.2f}x"
            f"{half_w * 2.0:.2f}m, actor_radius={self.actor_radius:.2f}m, "
            f"near_margin={self.near_margin:.2f}m"
        )
        if not self.actor_pose_topics:
            self.get_logger().info(
                "No actor_pose_topics configured; collision monitor is idle."
            )

    def string_list_parameter(self, name):
        value = self.get_parameter(name).value
        if isinstance(value, str):
            return [item.strip() for item in value.split(",") if item.strip()]
        try:
            return [str(item).strip() for item in value if str(item).strip()]
        except TypeError:
            return []

    def now_sec(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def vehicle_callback(self, msg):
        if self.vehicle_pose_index >= len(msg.poses):
            self.warn_limited(
                "vehicle_index",
                5.0,
                f"{self.vehicle_pose_topic} has {len(msg.poses)} poses, "
                f"but vehicle_pose_index={self.vehicle_pose_index}",
            )
            return
        self.vehicle_pose = msg.poses[self.vehicle_pose_index]
        self.vehicle_stamp_sec = self.now_sec()

    def actor_callback(self, topic, msg):
        position = msg.pose.position
        self.actors[topic] = TrackedActor(
            name=self.actor_name_from_topic(topic),
            topic=topic,
            x=float(position.x),
            y=float(position.y),
            z=float(position.z),
            stamp_sec=self.now_sec(),
        )

    def check_collisions(self):
        if not self.actor_pose_topics:
            return

        now = self.now_sec()
        if self.vehicle_pose is None:
            self.warn_limited(
                "vehicle_missing",
                5.0,
                f"Waiting for vehicle pose on {self.vehicle_pose_topic}",
            )
            return
        if (
            self.vehicle_pose_timeout > 0.0
            and self.vehicle_stamp_sec is not None
            and now - self.vehicle_stamp_sec > self.vehicle_pose_timeout
        ):
            self.warn_limited(
                "vehicle_stale",
                5.0,
                f"Vehicle pose is stale: {now - self.vehicle_stamp_sec:.2f}s",
            )
            return

        vehicle_x = float(self.vehicle_pose.position.x)
        vehicle_y = float(self.vehicle_pose.position.y)
        vehicle_yaw = yaw_from_quaternion(self.vehicle_pose.orientation)
        cos_yaw = math.cos(vehicle_yaw)
        sin_yaw = math.sin(vehicle_yaw)

        for topic, actor in list(self.actors.items()):
            if (
                self.actor_pose_timeout > 0.0
                and now - actor.stamp_sec > self.actor_pose_timeout
            ):
                self.update_actor_level(
                    actor,
                    CLEAR,
                    clearance=None,
                    local_x=None,
                    local_y=None,
                    vehicle_x=vehicle_x,
                    vehicle_y=vehicle_y,
                    reason="stale_actor_pose",
                )
                continue

            rel_x = actor.x - vehicle_x
            rel_y = actor.y - vehicle_y
            local_x = cos_yaw * rel_x + sin_yaw * rel_y
            local_y = -sin_yaw * rel_x + cos_yaw * rel_y
            rect_distance = self.signed_distance_to_vehicle_box(local_x, local_y)
            clearance = rect_distance - self.actor_radius
            level = self.classify_clearance(clearance)
            self.update_actor_level(
                actor,
                level,
                clearance=clearance,
                local_x=local_x,
                local_y=local_y,
                vehicle_x=vehicle_x,
                vehicle_y=vehicle_y,
                reason="geometry_check",
            )

    def signed_distance_to_vehicle_box(self, local_x, local_y):
        half_l = self.vehicle_length * 0.5 + self.vehicle_padding
        half_w = self.vehicle_width * 0.5 + self.vehicle_padding
        inside_x = half_l - abs(local_x)
        inside_y = half_w - abs(local_y)
        if inside_x >= 0.0 and inside_y >= 0.0:
            return -min(inside_x, inside_y)
        outside_x = max(-inside_x, 0.0)
        outside_y = max(-inside_y, 0.0)
        return math.hypot(outside_x, outside_y)

    def classify_clearance(self, clearance):
        if clearance <= self.collision_margin:
            return COLLISION
        if clearance <= self.near_margin:
            return NEAR_MISS
        return CLEAR

    def update_actor_level(
        self,
        actor,
        level,
        clearance,
        local_x,
        local_y,
        vehicle_x,
        vehicle_y,
        reason,
    ):
        previous = self.actor_levels.get(actor.topic, CLEAR)
        if level == previous:
            return
        self.actor_levels[actor.topic] = level

        event = level_to_event(level)
        payload = {
            "stamp": round(self.now_sec(), 3),
            "event": event,
            "actor": actor.name,
            "actor_topic": actor.topic,
            "reason": reason,
            "clearance_m": round(clearance, 3) if clearance is not None else None,
            "actor_xy": [round(actor.x, 3), round(actor.y, 3)],
            "vehicle_xy": [round(vehicle_x, 3), round(vehicle_y, 3)],
            "actor_local_xy": (
                [round(local_x, 3), round(local_y, 3)]
                if local_x is not None and local_y is not None
                else None
            ),
            "vehicle_length_m": self.vehicle_length,
            "vehicle_width_m": self.vehicle_width,
            "vehicle_padding_m": self.vehicle_padding,
            "actor_radius_m": self.actor_radius,
        }
        self.event_publisher.publish(
            String(data=json.dumps(payload, separators=(",", ":")))
        )

        clear_text = (
            "unknown" if clearance is None else f"{clearance:.2f}m"
        )
        local_text = (
            "unknown"
            if local_x is None or local_y is None
            else f"({local_x:.2f}, {local_y:.2f})"
        )
        if level == COLLISION:
            self.get_logger().error(
                f"Actor collision: {actor.name}, clearance={clear_text}, "
                f"actor_local_xy={local_text}"
            )
        elif level == NEAR_MISS:
            self.get_logger().warn(
                f"Actor near miss: {actor.name}, clearance={clear_text}, "
                f"actor_local_xy={local_text}"
            )
        else:
            self.get_logger().info(
                f"Actor clear: {actor.name}, reason={reason}, "
                f"last_clearance={clear_text}"
            )

    def warn_limited(self, key, interval_sec, message):
        now = self.now_sec()
        last = self.last_warn_sec.get(key)
        if last is None or now - last >= interval_sec:
            self.last_warn_sec[key] = now
            self.get_logger().warn(message)

    @staticmethod
    def actor_name_from_topic(topic):
        parts = [part for part in topic.split("/") if part]
        if len(parts) >= 2 and parts[-1] == "pose":
            return parts[-2]
        if parts:
            return parts[-1]
        return topic or "actor"


def level_to_event(level):
    if level == COLLISION:
        return "collision"
    if level == NEAR_MISS:
        return "near_miss"
    return "clear"


def yaw_from_quaternion(quaternion):
    x = float(quaternion.x)
    y = float(quaternion.y)
    z = float(quaternion.z)
    w = float(quaternion.w)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def main(args=None):
    rclpy.init(args=args)
    node = ActorCollisionMonitor()
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
