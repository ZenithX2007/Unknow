#!/usr/bin/env python3
"""Remove visual trash entities when the vehicle drives over them."""

import json
import math
import os
import subprocess
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node


class TrashCleanupNode(Node):
    def __init__(self):
        super().__init__("gen0_trash_cleanup")

        self.declare_parameter("world", "my_map")
        self.declare_parameter("trash_scenario", "small_trash")
        self.declare_parameter("gazebo_world_name", "default")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("cleanup_radius", 0.90)
        self.declare_parameter("retry_period", 1.0)
        self.declare_parameter("service_timeout_ms", 1000)

        self.world = self.get_parameter("world").value
        self.trash_scenario = self.get_parameter("trash_scenario").value
        self.gazebo_world_name = self.get_parameter("gazebo_world_name").value
        self.cleanup_radius = float(self.get_parameter("cleanup_radius").value)
        self.retry_period = float(self.get_parameter("retry_period").value)
        self.service_timeout_ms = int(self.get_parameter("service_timeout_ms").value)
        odom_topic = self.get_parameter("odom_topic").value

        self.remaining = self.load_trash_items()
        self.last_attempt = {}
        self.create_subscription(Odometry, odom_topic, self.odom_callback, 20)

        self.get_logger().info(
            f"Loaded {len(self.remaining)} trash items from "
            f"{self.world}/{self.trash_scenario}; cleanup_radius={self.cleanup_radius:.2f} m"
        )

    def load_trash_items(self):
        package_share = Path(get_package_share_directory("gen0_main"))
        scenario_path = (
            package_share
            / "worlds"
            / "trash_scenarios"
            / self.world
            / f"{self.trash_scenario}.json"
        )
        if not scenario_path.exists():
            self.get_logger().warning(f"Trash scenario not found: {scenario_path}")
            return {}

        with scenario_path.open("r", encoding="utf-8") as scenario_file:
            scenario = json.load(scenario_file)

        items = {}
        for index, item in enumerate(scenario):
            pose = item.get("pose", [])
            if len(pose) < 2:
                self.get_logger().warning(f"Skipping trash item without xy pose: {item}")
                continue

            model = item.get("model", "trash")
            name = item.get("name", f"{model}_{index}")
            items[name] = (float(pose[0]), float(pose[1]))
        return items

    def odom_callback(self, msg):
        if not self.remaining:
            return

        vehicle_x = msg.pose.pose.position.x
        vehicle_y = msg.pose.pose.position.y
        radius_sq = self.cleanup_radius * self.cleanup_radius
        now = self.get_clock().now().nanoseconds * 1e-9

        for name, (trash_x, trash_y) in list(self.remaining.items()):
            dx = vehicle_x - trash_x
            dy = vehicle_y - trash_y
            distance_sq = dx * dx + dy * dy
            if distance_sq > radius_sq:
                continue

            if now - self.last_attempt.get(name, 0.0) < self.retry_period:
                continue
            self.last_attempt[name] = now

            distance = math.sqrt(distance_sq)
            if self.remove_gazebo_entity(name):
                del self.remaining[name]
                self.get_logger().info(
                    f"Removed {name}; distance={distance:.2f} m; "
                    f"remaining={len(self.remaining)}"
                )

    def remove_gazebo_entity(self, name):
        request = f'name: "{name}"\ntype: MODEL\n'
        command = [
            "ign",
            "service",
            "-s",
            f"/world/{self.gazebo_world_name}/remove",
            "--reqtype",
            "ignition.msgs.Entity",
            "--reptype",
            "ignition.msgs.Boolean",
            "--timeout",
            str(self.service_timeout_ms),
            "--req",
            request,
        ]

        try:
            result = subprocess.run(
                command,
                check=False,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                timeout=max(2.0, self.service_timeout_ms / 1000.0 + 1.0),
                env=os.environ.copy(),
            )
        except (OSError, subprocess.TimeoutExpired) as exc:
            self.get_logger().warning(f"Failed to remove {name}: {exc}")
            return False

        output = result.stdout.strip()
        if result.returncode == 0 and "data: true" in output.lower():
            return True

        self.get_logger().warning(
            f"Gazebo did not remove {name}; returncode={result.returncode}; output={output}"
        )
        return False


def main(args=None):
    rclpy.init(args=args)
    node = TrashCleanupNode()
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
