#!/usr/bin/env python3
"""Remove visual trash entities when the vehicle drives over them."""

import json
import math
import os
import subprocess
from dataclasses import dataclass
from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry
import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node

from .trash_model_geometry import load_trash_model_geometry


TRASH_FOOTPRINTS = {
    "trash_coffee_cup_01": (0.1073, 0.1736),
    "trash_can_01": (0.1357, 0.1376),
    "trash_crushed_can_01": (0.1449, 0.1478),
    "trash_food_can_01": (0.0704, 0.0718),
    "trash_paper_crumple_01": (0.1006, 0.1044),
    "trash_small_bottle_01": (0.1374, 0.0781),
}


def float_parameter(node, name, default):
    value = node.get_parameter(name).value
    if value is None:
        return float(default)
    return float(value)


def bool_parameter(node, name, default):
    value = node.get_parameter(name).value
    if value is None:
        return bool(default)
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() not in {"0", "false", "off", "no"}


@dataclass(frozen=True)
class TrashItem:
    name: str
    model: str
    x: float
    y: float
    yaw: float
    length: float
    width: float
    model_x: float
    model_y: float
    visual_offset_x: float
    visual_offset_y: float


class TrashCleanupNode(Node):
    def __init__(self):
        super().__init__("gen0_trash_cleanup")

        self.declare_parameter("world", "my_map")
        self.declare_parameter("trash_scenario", "small_trash")
        self.declare_parameter("gazebo_world_name", "default")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("vehicle_length")
        self.declare_parameter("vehicle_width")
        self.declare_parameter("vehicle_center_offset_x")
        self.declare_parameter("vehicle_center_offset_y")
        self.declare_parameter("coverage_margin")
        self.declare_parameter("use_mesh_visual_center", True)
        self.declare_parameter("debug_item", "")
        self.declare_parameter("debug_period", 1.0)
        self.declare_parameter("retry_period", 1.0)
        self.declare_parameter("service_timeout_ms", 1000)

        self.world = self.get_parameter("world").value
        self.trash_scenario = self.get_parameter("trash_scenario").value
        self.gazebo_world_name = self.get_parameter("gazebo_world_name").value
        self.vehicle_length = float_parameter(self, "vehicle_length", 3.50)
        self.vehicle_width = float_parameter(self, "vehicle_width", 1.80)
        self.vehicle_center_offset_x = float_parameter(
            self, "vehicle_center_offset_x", 0.0
        )
        self.vehicle_center_offset_y = float_parameter(
            self, "vehicle_center_offset_y", 0.0
        )
        self.coverage_margin = float_parameter(self, "coverage_margin", 0.0)
        self.use_mesh_visual_center = bool_parameter(
            self, "use_mesh_visual_center", True
        )
        self.debug_item = str(self.get_parameter("debug_item").value)
        self.debug_period = float(self.get_parameter("debug_period").value)
        self.retry_period = float(self.get_parameter("retry_period").value)
        self.service_timeout_ms = int(self.get_parameter("service_timeout_ms").value)
        odom_topic = self.get_parameter("odom_topic").value

        self.update_vehicle_footprint()
        self.package_share = Path(get_package_share_directory("gen0_main"))
        self.model_geometry_offsets = {}
        self.remaining = self.load_trash_items()
        self.last_attempt = {}
        self.last_debug_time = 0.0
        self.last_odom_msg = None
        self.add_on_set_parameters_callback(self.on_parameter_update)
        self.create_subscription(Odometry, odom_topic, self.odom_callback, 20)

        self.get_logger().info(
            f"Loaded {len(self.remaining)} trash items from "
            f"{self.world}/{self.trash_scenario}; "
            f"vehicle={self.vehicle_length:.2f}x{self.vehicle_width:.2f} m; "
            f"odom_center_offset=({self.vehicle_center_offset_x:.2f}, "
            f"{self.vehicle_center_offset_y:.2f}) m; "
            f"coverage_margin={self.coverage_margin:.2f} m; "
            f"mesh_visual_center={self.use_mesh_visual_center}; "
            f"offset axes: +x forward, +y left"
        )

    def update_vehicle_footprint(self):
        self.vehicle_half_length = max(
            0.0, self.vehicle_length * 0.5 - self.coverage_margin
        )
        self.vehicle_half_width = max(
            0.0, self.vehicle_width * 0.5 - self.coverage_margin
        )

    def on_parameter_update(self, parameters):
        for parameter in parameters:
            if parameter.name == "vehicle_length":
                self.vehicle_length = float(parameter.value)
            elif parameter.name == "vehicle_width":
                self.vehicle_width = float(parameter.value)
            elif parameter.name == "vehicle_center_offset_x":
                self.vehicle_center_offset_x = float(parameter.value)
            elif parameter.name == "vehicle_center_offset_y":
                self.vehicle_center_offset_y = float(parameter.value)
            elif parameter.name == "coverage_margin":
                self.coverage_margin = float(parameter.value)
            elif parameter.name == "use_mesh_visual_center":
                self.get_logger().warning(
                    "use_mesh_visual_center is startup-only; restart the node to change it"
                )
            elif parameter.name == "debug_item":
                self.debug_item = str(parameter.value)
            elif parameter.name == "debug_period":
                self.debug_period = float(parameter.value)
            else:
                continue

        self.update_vehicle_footprint()
        self.get_logger().info(
            f"Updated trash cleanup params: "
            f"vehicle={self.vehicle_length:.2f}x{self.vehicle_width:.2f} m, "
            f"offset=({self.vehicle_center_offset_x:.2f}, "
            f"{self.vehicle_center_offset_y:.2f}) m, "
            f"coverage_margin={self.coverage_margin:.2f} m, "
            f"debug_item={self.debug_item or '<none>'}"
        )
        if self.last_odom_msg is not None:
            self.evaluate_odom(
                self.last_odom_msg,
                force_attempt=True,
                force_debug=True,
            )
        return SetParametersResult(successful=True)

    def load_trash_items(self):
        scenario_path = (
            self.package_share
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
            footprint = item.get(
                "footprint", TRASH_FOOTPRINTS.get(model, (0.20, 0.20))
            )
            if len(footprint) < 2:
                self.get_logger().warning(
                    f"Skipping trash item without valid footprint: {item}"
                )
                continue
            model_x = float(pose[0])
            model_y = float(pose[1])
            yaw = float(pose[5]) if len(pose) >= 6 else 0.0
            visual_offset_x, visual_offset_y = self.get_visual_offset(model)
            visual_x, visual_y = offset_pose_xy(
                model_x,
                model_y,
                yaw,
                visual_offset_x,
                visual_offset_y,
            )
            items[name] = TrashItem(
                name=name,
                model=model,
                x=visual_x,
                y=visual_y,
                yaw=yaw,
                length=float(footprint[0]),
                width=float(footprint[1]),
                model_x=model_x,
                model_y=model_y,
                visual_offset_x=visual_offset_x,
                visual_offset_y=visual_offset_y,
            )
        return items

    def get_visual_offset(self, model):
        if not self.use_mesh_visual_center:
            return 0.0, 0.0
        if model in self.model_geometry_offsets:
            return self.model_geometry_offsets[model]

        try:
            geometry = load_trash_model_geometry(self.package_share, model)
            offset = (geometry.center_offset_x, geometry.center_offset_y)
            self.get_logger().info(
                f"Trash model calibration {model}: "
                f"visual_center_offset=({offset[0]:.3f}, {offset[1]:.3f}) m"
            )
        except (OSError, ValueError, ET.ParseError) as exc:
            self.get_logger().warning(
                f"Could not read visual center for {model}: {exc}; using (0, 0)"
            )
            offset = (0.0, 0.0)

        self.model_geometry_offsets[model] = offset
        return offset

    def odom_callback(self, msg):
        self.last_odom_msg = msg
        self.evaluate_odom(msg)

    def evaluate_odom(self, msg, force_attempt=False, force_debug=False):
        if not self.remaining:
            return

        odom_x = msg.pose.pose.position.x
        odom_y = msg.pose.pose.position.y
        vehicle_yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        vehicle_x, vehicle_y = offset_pose_xy(
            odom_x,
            odom_y,
            vehicle_yaw,
            self.vehicle_center_offset_x,
            self.vehicle_center_offset_y,
        )
        now = self.get_clock().now().nanoseconds * 1e-9

        for name, item in list(self.remaining.items()):
            covered = self.vehicle_fully_covers_trash(
                vehicle_x, vehicle_y, vehicle_yaw, item
            )
            self.log_debug_item(
                name,
                item,
                covered,
                odom_x,
                odom_y,
                vehicle_x,
                vehicle_y,
                vehicle_yaw,
                now,
                force_debug,
            )
            if not covered:
                continue

            if (
                not force_attempt
                and now - self.last_attempt.get(name, 0.0) < self.retry_period
            ):
                continue
            self.last_attempt[name] = now

            if self.remove_gazebo_entity(name):
                bounds = trash_local_bounds(vehicle_x, vehicle_y, vehicle_yaw, item)
                del self.remaining[name]
                self.get_logger().info(
                    f"Removed {name}; fully covered by vehicle footprint; "
                    f"offset=({self.vehicle_center_offset_x:.2f}, "
                    f"{self.vehicle_center_offset_y:.2f}) m; "
                    f"trash_local_x=[{bounds[0]:.2f}, {bounds[1]:.2f}], "
                    f"trash_local_y=[{bounds[2]:.2f}, {bounds[3]:.2f}], "
                    f"edge_clearance=(front={self.vehicle_half_length - bounds[1]:.2f}, "
                    f"rear={bounds[0] + self.vehicle_half_length:.2f}, "
                    f"left={self.vehicle_half_width - bounds[3]:.2f}, "
                    f"right={bounds[2] + self.vehicle_half_width:.2f}) m; "
                    f"remaining={len(self.remaining)}"
                )

    def vehicle_fully_covers_trash(self, vehicle_x, vehicle_y, vehicle_yaw, item):
        for corner_x, corner_y in trash_corners(item):
            local_x, local_y = world_to_local(
                corner_x,
                corner_y,
                vehicle_x,
                vehicle_y,
                vehicle_yaw,
            )
            if abs(local_x) > self.vehicle_half_length:
                return False
            if abs(local_y) > self.vehicle_half_width:
                return False
        return True

    def log_debug_item(
        self,
        name,
        item,
        covered,
        odom_x,
        odom_y,
        vehicle_x,
        vehicle_y,
        vehicle_yaw,
        now,
        force=False,
    ):
        if name != self.debug_item:
            return
        if not force and now - self.last_debug_time < self.debug_period:
            return
        self.last_debug_time = now

        corner_locals = [
            world_to_local(corner_x, corner_y, vehicle_x, vehicle_y, vehicle_yaw)
            for corner_x, corner_y in trash_corners(item)
        ]
        local_x_values = [point[0] for point in corner_locals]
        local_y_values = [point[1] for point in corner_locals]
        center_local_x, center_local_y = world_to_local(
            item.x, item.y, vehicle_x, vehicle_y, vehicle_yaw
        )

        center_offset_x = self.vehicle_center_offset_x + center_local_x
        center_offset_y = self.vehicle_center_offset_y + center_local_y
        front_right_offset_x = (
            self.vehicle_center_offset_x
            + max(local_x_values)
            - self.vehicle_half_length
        )
        front_right_offset_y = (
            self.vehicle_center_offset_y
            + min(local_y_values)
            + self.vehicle_half_width
        )
        front_left_offset_x = front_right_offset_x
        front_left_offset_y = (
            self.vehicle_center_offset_y
            + max(local_y_values)
            - self.vehicle_half_width
        )
        (
            covering_offset_min_x,
            covering_offset_max_x,
            covering_offset_min_y,
            covering_offset_max_y,
        ) = covering_offset_bounds(
            self.vehicle_center_offset_x,
            self.vehicle_center_offset_y,
            local_x_values,
            local_y_values,
            self.vehicle_half_length,
            self.vehicle_half_width,
        )
        nearest_covering_offset = nearest_offset_in_bounds(
            self.vehicle_center_offset_x,
            self.vehicle_center_offset_y,
            covering_offset_min_x,
            covering_offset_max_x,
            covering_offset_min_y,
            covering_offset_max_y,
        )
        if nearest_covering_offset is None:
            nearest_covering_text = "none"
        else:
            nearest_covering_text = (
                f"({nearest_covering_offset[0]:.2f}, "
                f"{nearest_covering_offset[1]:.2f})"
            )

        self.get_logger().info(
            f"debug {name}: covered={covered}; "
            f"odom=({odom_x:.2f}, {odom_y:.2f}); "
            f"cleanup_center=({vehicle_x:.2f}, {vehicle_y:.2f}); "
            f"yaw={vehicle_yaw:.3f}; "
            f"trash_model_pose=({item.model_x:.2f}, {item.model_y:.2f}); "
            f"visual_offset=({item.visual_offset_x:.2f}, "
            f"{item.visual_offset_y:.2f}); "
            f"trash_center=({item.x:.2f}, {item.y:.2f}); "
            f"trash_local_center=({center_local_x:.2f}, {center_local_y:.2f}); "
            f"trash_local_x=[{min(local_x_values):.2f}, {max(local_x_values):.2f}], "
            f"trash_local_y=[{min(local_y_values):.2f}, {max(local_y_values):.2f}]; "
            f"vehicle_half=({self.vehicle_half_length:.2f}, "
            f"{self.vehicle_half_width:.2f}); "
            f"suggest_center_offset=({center_offset_x:.2f}, {center_offset_y:.2f}); "
            f"suggest_front_left_offset=({front_left_offset_x:.2f}, "
            f"{front_left_offset_y:.2f}); "
            f"suggest_front_right_offset=({front_right_offset_x:.2f}, "
            f"{front_right_offset_y:.2f}); "
            f"covering_offset_x=[{covering_offset_min_x:.2f}, "
            f"{covering_offset_max_x:.2f}], "
            f"covering_offset_y=[{covering_offset_min_y:.2f}, "
            f"{covering_offset_max_y:.2f}], "
            f"nearest_covering_offset={nearest_covering_text}"
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


def yaw_from_quaternion(quaternion):
    siny_cosp = 2.0 * (
        quaternion.w * quaternion.z + quaternion.x * quaternion.y
    )
    cosy_cosp = 1.0 - 2.0 * (
        quaternion.y * quaternion.y + quaternion.z * quaternion.z
    )
    return math.atan2(siny_cosp, cosy_cosp)


def offset_pose_xy(x, y, yaw, offset_x, offset_y):
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    return (
        x + cos_yaw * offset_x - sin_yaw * offset_y,
        y + sin_yaw * offset_x + cos_yaw * offset_y,
    )


def world_to_local(point_x, point_y, origin_x, origin_y, yaw):
    dx = point_x - origin_x
    dy = point_y - origin_y
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    return (
        cos_yaw * dx + sin_yaw * dy,
        -sin_yaw * dx + cos_yaw * dy,
    )


def trash_corners(item):
    half_length = item.length * 0.5
    half_width = item.width * 0.5
    cos_yaw = math.cos(item.yaw)
    sin_yaw = math.sin(item.yaw)
    for local_x in (-half_length, half_length):
        for local_y in (-half_width, half_width):
            yield (
                item.x + cos_yaw * local_x - sin_yaw * local_y,
                item.y + sin_yaw * local_x + cos_yaw * local_y,
            )


def trash_local_bounds(vehicle_x, vehicle_y, vehicle_yaw, item):
    corner_locals = [
        world_to_local(corner_x, corner_y, vehicle_x, vehicle_y, vehicle_yaw)
        for corner_x, corner_y in trash_corners(item)
    ]
    local_x_values = [point[0] for point in corner_locals]
    local_y_values = [point[1] for point in corner_locals]
    return (
        min(local_x_values),
        max(local_x_values),
        min(local_y_values),
        max(local_y_values),
    )


def covering_offset_bounds(
    current_offset_x,
    current_offset_y,
    local_x_values,
    local_y_values,
    vehicle_half_length,
    vehicle_half_width,
):
    return (
        current_offset_x + max(local_x_values) - vehicle_half_length,
        current_offset_x + min(local_x_values) + vehicle_half_length,
        current_offset_y + max(local_y_values) - vehicle_half_width,
        current_offset_y + min(local_y_values) + vehicle_half_width,
    )


def nearest_offset_in_bounds(
    current_offset_x,
    current_offset_y,
    min_x,
    max_x,
    min_y,
    max_y,
):
    if min_x > max_x or min_y > max_y:
        return None
    return (
        min(max(current_offset_x, min_x), max_x),
        min(max(current_offset_y, min_y), max_y),
    )


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
