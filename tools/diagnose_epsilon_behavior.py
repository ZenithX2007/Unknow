#!/usr/bin/env python3
"""Diagnose the EPSILON behavior-layer inputs from live ROS 2 topics."""

from __future__ import annotations

import argparse
import math
from pathlib import Path
from typing import Iterable

import rclpy
from nav_msgs.msg import Path as NavPath
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from std_msgs.msg import String
from vehicle_msgs.msg import ArenaInfoDynamic
from vehicle_msgs.msg import ArenaInfoStatic


def stamp_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def point_xy(point) -> tuple[float, float]:
    return float(point.x), float(point.y)


def point_distance(a, b) -> float:
    ax, ay = point_xy(a)
    bx, by = point_xy(b)
    return math.hypot(ax - bx, ay - by)


def point_to_segment_distance(point, start, end) -> tuple[float, float]:
    px, py = point_xy(point)
    sx, sy = point_xy(start)
    ex, ey = point_xy(end)
    vx = ex - sx
    vy = ey - sy
    denom = vx * vx + vy * vy
    if denom <= 1e-12:
        return math.hypot(px - sx, py - sy), 0.0
    t = max(0.0, min(1.0, ((px - sx) * vx + (py - sy) * vy) / denom))
    proj_x = sx + t * vx
    proj_y = sy + t * vy
    return math.hypot(px - proj_x, py - proj_y), t


def segment_heading(start, end) -> float:
    sx, sy = point_xy(start)
    ex, ey = point_xy(end)
    return math.atan2(ey - sy, ex - sx)


def angle_difference(lhs: float, rhs: float) -> float:
    return abs(math.atan2(math.sin(lhs - rhs), math.cos(lhs - rhs)))


def lane_length(points: Iterable) -> float:
    points = list(points)
    return sum(point_distance(points[index - 1], points[index]) for index in range(1, len(points)))


def nearest_lane(ego_point, lanes) -> tuple[float, int | None, int, float]:
    best_distance = math.inf
    best_lane_id: int | None = None
    best_segment = -1
    best_segment_t = 0.0
    for lane in lanes:
        if len(lane.points) == 1:
            distance = point_distance(ego_point, lane.points[0])
            if distance < best_distance:
                best_distance = distance
                best_lane_id = int(lane.id)
                best_segment = 0
                best_segment_t = 0.0
            continue
        for index in range(1, len(lane.points)):
            distance, segment_t = point_to_segment_distance(
                ego_point, lane.points[index - 1], lane.points[index]
            )
            if distance < best_distance:
                best_distance = distance
                best_lane_id = int(lane.id)
                best_segment = index - 1
                best_segment_t = segment_t
    return best_distance, best_lane_id, best_segment, best_segment_t


def read_last_matches(path: Path, patterns: list[str], limit: int = 8) -> list[str]:
    if not path.exists():
        return []
    matches: list[str] = []
    try:
        with path.open("r", errors="replace") as handle:
            for line in handle:
                if any(pattern in line for pattern in patterns):
                    matches.append(line.rstrip())
    except OSError:
        return []
    return matches[-limit:]


class DiagnosticNode(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("epsilon_behavior_diagnostic_node")
        self.static_scene: ArenaInfoStatic | None = None
        self.dynamic_scene: ArenaInfoDynamic | None = None
        self.path: NavPath | None = None
        self.status: String | None = None
        self.selected_source: String | None = None

        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        path_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        default_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.create_subscription(
            ArenaInfoStatic,
            args.static_topic,
            self._static_callback,
            map_qos,
        )
        self.create_subscription(
            ArenaInfoDynamic,
            args.dynamic_topic,
            self._dynamic_callback,
            sensor_qos,
        )
        self.create_subscription(NavPath, args.path_topic, self._path_callback, path_qos)
        self.create_subscription(String, args.status_topic, self._status_callback, default_qos)
        self.create_subscription(
            String,
            args.selected_source_topic,
            self._selected_source_callback,
            default_qos,
        )

    def _static_callback(self, msg: ArenaInfoStatic) -> None:
        self.static_scene = msg

    def _dynamic_callback(self, msg: ArenaInfoDynamic) -> None:
        self.dynamic_scene = msg

    def _path_callback(self, msg: NavPath) -> None:
        self.path = msg

    def _status_callback(self, msg: String) -> None:
        self.status = msg

    def _selected_source_callback(self, msg: String) -> None:
        self.selected_source = msg


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Diagnose EPSILON LaneNet/EUDM/SSC live inputs."
    )
    parser.add_argument("--timeout", type=float, default=8.0)
    parser.add_argument("--ego-id", type=int, default=0)
    parser.add_argument("--nearest-lane-max-distance", type=float, default=2.0)
    parser.add_argument("--min-lane-length", type=float, default=8.0)
    parser.add_argument("--static-topic", default="/epsilon/arena_info_static")
    parser.add_argument("--dynamic-topic", default="/epsilon/arena_info_dynamic")
    parser.add_argument("--path-topic", default="/plan_smoothed")
    parser.add_argument("--status-topic", default="/epsilon/status")
    parser.add_argument(
        "--selected-source-topic", default="/epsilon/selected_control_source"
    )
    parser.add_argument("--log-dir", default="runtime_logs")
    parser.add_argument(
        "--strict",
        action="store_true",
        help="Exit non-zero if static scene, ego, or nearest-lane checks fail.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    rclpy.init()
    node = DiagnosticNode(args)
    deadline = node.get_clock().now().nanoseconds / 1e9 + args.timeout
    while rclpy.ok() and node.get_clock().now().nanoseconds / 1e9 < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.static_scene is not None and node.dynamic_scene is not None:
            if node.status is not None and node.selected_source is not None:
                break

    failures: list[str] = []
    print("EPSILON behavior diagnostic")

    if node.path is None:
        print(f"path: no live sample on {args.path_topic}")
    else:
        print(
            "path: "
            f"frame={node.path.header.frame_id or '<empty>'} "
            f"poses={len(node.path.poses)} stamp={stamp_seconds(node.path.header.stamp):.3f}"
        )

    if node.static_scene is None:
        print(f"static: no sample on {args.static_topic}")
        failures.append("missing static scene")
        lanes = []
    else:
        lanes = list(node.static_scene.lane_net.lanes)
        obstacle_set = node.static_scene.obstacle_set
        print(
            "static: "
            f"frame={node.static_scene.header.frame_id or '<empty>'} "
            f"lanes={len(lanes)} "
            f"circle_obstacles={len(obstacle_set.obs_circle)} "
            f"polygon_obstacles={len(obstacle_set.obs_polygon)}"
        )
        for lane in lanes[:6]:
            start = lane.points[0] if lane.points else lane.start_point
            final = lane.points[-1] if lane.points else lane.final_point
            print(
                "  lane "
                f"id={lane.id} points={len(lane.points)} "
                f"length={lane_length(lane.points):.2f} "
                f"l={lane.l_lane_id}:{lane.l_change_avbl} "
                f"r={lane.r_lane_id}:{lane.r_change_avbl} "
                f"child={list(lane.child_id)} father={list(lane.father_id)} "
                f"start=({start.x:.2f},{start.y:.2f}) "
                f"end=({final.x:.2f},{final.y:.2f})"
            )
        if not lanes:
            failures.append("lane net is empty")
        else:
            max_lane_length = max(lane_length(lane.points) for lane in lanes)
            print(
                "  lane_length_check: "
                f"max={max_lane_length:.2f}m threshold={args.min_lane_length:.2f}m"
            )
            if max_lane_length < args.min_lane_length:
                failures.append("LaneNet is too short for behavior planning")

    ego_vehicle = None
    actor_vehicles = []
    if node.dynamic_scene is None:
        print(f"dynamic: no sample on {args.dynamic_topic}")
        failures.append("missing dynamic scene")
    else:
        vehicles = list(node.dynamic_scene.vehicle_set.vehicles)
        for vehicle in vehicles:
            vehicle_id = int(vehicle.id.data)
            if vehicle_id == args.ego_id:
                ego_vehicle = vehicle
            else:
                actor_vehicles.append(vehicle)
        print(
            "dynamic: "
            f"frame={node.dynamic_scene.header.frame_id or '<empty>'} "
            f"vehicles={len(vehicles)} ego_present={ego_vehicle is not None} "
            f"actors={len(actor_vehicles)}"
        )
        if ego_vehicle is None:
            failures.append("ego vehicle is missing")
        else:
            ego_state = ego_vehicle.state
            pos = ego_state.vec_position
            print(
                "  ego: "
                f"id={ego_vehicle.id.data} "
                f"pos=({pos.x:.2f},{pos.y:.2f}) "
                f"yaw={ego_state.angle:.3f} "
                f"v={ego_state.velocity:.3f} "
                f"size={ego_vehicle.param.length:.2f}x{ego_vehicle.param.width:.2f}"
            )
            if lanes:
                distance, lane_id, segment, segment_t = nearest_lane(pos, lanes)
                lane_heading_text = ""
                if lane_id is not None:
                    nearest = next((lane for lane in lanes if int(lane.id) == lane_id), None)
                    if nearest is not None and len(nearest.points) > 1:
                        heading_segment = max(0, min(segment, len(nearest.points) - 2))
                        heading = segment_heading(
                            nearest.points[heading_segment],
                            nearest.points[heading_segment + 1],
                        )
                        lane_heading_text = (
                            f" lane_heading={heading:.3f} "
                            f"yaw_diff={angle_difference(ego_state.angle, heading):.3f}"
                        )
                print(
                    "  nearest_lane: "
                    f"id={lane_id} distance={distance:.3f}m "
                    f"segment={segment} t={segment_t:.2f} "
                    f"threshold={args.nearest_lane_max_distance:.3f}m"
                    f"{lane_heading_text}"
                )
                if distance > args.nearest_lane_max_distance:
                    failures.append("ego is too far from LaneNet")

        if ego_vehicle is not None and actor_vehicles:
            ego_pos = ego_vehicle.state.vec_position
            actor_distances = sorted(
                (
                    point_distance(ego_pos, vehicle.state.vec_position),
                    int(vehicle.id.data),
                    vehicle.state.vec_position.x,
                    vehicle.state.vec_position.y,
                    vehicle.state.vec_position,
                )
                for vehicle in actor_vehicles
            )
            for distance, vehicle_id, x, y, actor_pos in actor_distances[:5]:
                lane_text = ""
                if lanes:
                    lane_distance, lane_id, _, _ = nearest_lane(actor_pos, lanes)
                    lane_text = f" nearest_lane={lane_id} lane_distance={lane_distance:.2f}"
                print(
                    "  actor: "
                    f"id={vehicle_id} distance={distance:.2f} "
                    f"pos=({x:.2f},{y:.2f}){lane_text}"
                )

    if node.status is None:
        print(f"status: no sample on {args.status_topic}")
    else:
        print(f"status: {node.status.data}")

    if node.selected_source is None:
        print(f"selected_source: no sample on {args.selected_source_topic}")
    else:
        print(f"selected_source: {node.selected_source.data}")

    log_dir = Path(args.log_dir)
    epsilon_matches = read_last_matches(
        log_dir / "epsilon.log",
        [
            "No nearest lane found",
            "Fail to find any valid behavior",
            "GetBezierSplineUsingCorridor",
            "Solver error",
            "degraded",
            "ssc failed",
            "eudm failed",
        ],
    )
    print(f"epsilon_log_matches: {len(epsilon_matches)} recent")
    for line in epsilon_matches:
        print(f"  {line}")

    gazebo_matches = read_last_matches(
        log_dir / "gazebo.log",
        [
            "Actor soft-stop enabled",
            "Actor soft-stop holding",
            "Actor soft-stop released",
            "front_obstacle",
            "vehicle+front_obstacle",
        ],
    )
    print(f"gazebo_actor_soft_stop_matches: {len(gazebo_matches)} recent")
    for line in gazebo_matches:
        print(f"  {line}")

    node.destroy_node()
    rclpy.shutdown()

    if failures:
        print("diagnostic_failures: " + ", ".join(failures))
        return 1 if args.strict else 0
    print("diagnostic_failures: none")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
