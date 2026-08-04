#!/usr/bin/env python3

from array import array
from math import atan2, cos, exp, floor, hypot, sin
import time

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


class ProjectedTerrainMap(Node):
    def __init__(self):
        super().__init__("projected_terrain_map")

        self.declare_parameter("input_topic", "/gen0_mapping/terrain_map_ext")
        self.declare_parameter("odom_topic", "/gen0_mapping/fast_lio/odom")
        self.declare_parameter("map_topic", "/projected_map")
        self.declare_parameter("costmap_topic", "/projected_costmap")
        self.declare_parameter("frame_id", "odom")
        self.declare_parameter("resolution", 0.10)
        self.declare_parameter("publish_period", 1.0)
        self.declare_parameter("republish_unchanged", True)
        self.declare_parameter("accumulate_history", False)
        self.declare_parameter("max_points_per_update", 90000)
        self.declare_parameter("free_intensity_threshold", 0.10)
        self.declare_parameter("occupied_intensity_threshold", 0.15)
        self.declare_parameter("occupied_cost_intensity", 0.45)
        self.declare_parameter("hit_log_odds", 0.85)
        self.declare_parameter("miss_log_odds", 0.20)
        self.declare_parameter("min_log_odds", -2.0)
        self.declare_parameter("max_log_odds", 3.5)
        self.declare_parameter("occupied_log_odds_threshold", 0.0)
        self.declare_parameter("free_log_odds_threshold", -0.2)
        self.declare_parameter("mark_low_intensity_free", True)
        self.declare_parameter("ground_clears_occupied", False)
        self.declare_parameter("raytrace_free_space", True)
        self.declare_parameter("raytrace_clears_occupied", True)
        self.declare_parameter("occupied_clear_log_odds_threshold", 1.7)
        self.declare_parameter("raytrace_max_range", 22.0)
        self.declare_parameter("max_raytrace_cells_per_update", 5000)
        self.declare_parameter("occupied_padding_radius", 0.0)
        self.declare_parameter("filter_speckles", False)
        self.declare_parameter("min_occupied_component_cells", 1)
        self.declare_parameter("min_occupied_component_span_cells", 1)
        self.declare_parameter("occupied_gap_bridge_cells", 0)
        self.declare_parameter("remove_dense_blob_components", False)
        self.declare_parameter("dense_blob_min_cells", 12)
        self.declare_parameter("dense_blob_min_short_span_cells", 5)
        self.declare_parameter("dense_blob_min_density", 0.45)
        self.declare_parameter("reference_odom_topic", "")
        self.declare_parameter("max_reference_odom_error", 0.0)
        self.declare_parameter("max_reference_yaw_error", 0.0)
        self.declare_parameter("reference_odom_timeout", 2.0)
        self.declare_parameter("reference_odom_warn_period", 2.0)
        self.declare_parameter("robot_clear_radius", 0.8)
        self.declare_parameter("robot_clear_length", 0.0)
        self.declare_parameter("robot_clear_width", 0.0)
        self.declare_parameter("robot_clear_margin", 0.0)
        self.declare_parameter("publish_padding_cells", 25)
        self.declare_parameter("max_side_cells", 1800)
        self.declare_parameter("inflation_radius", 0.7)
        self.declare_parameter("inflation_cost_scaling", 3.0)
        self.declare_parameter("max_inflation_sources", 12000)

        self.input_topic = self.get_parameter("input_topic").value
        self.odom_topic = self.get_parameter("odom_topic").value
        self.map_topic = self.get_parameter("map_topic").value
        self.costmap_topic = self.get_parameter("costmap_topic").value
        self.frame_id = self.get_parameter("frame_id").value
        self.resolution = float(self.get_parameter("resolution").value)
        publish_period = float(self.get_parameter("publish_period").value)
        self.republish_unchanged = bool(
            self.get_parameter("republish_unchanged").value
        )
        self.accumulate_history = bool(
            self.get_parameter("accumulate_history").value
        )
        self.max_points_per_update = int(
            self.get_parameter("max_points_per_update").value
        )
        self.free_threshold = float(
            self.get_parameter("free_intensity_threshold").value
        )
        self.occupied_threshold = float(
            self.get_parameter("occupied_intensity_threshold").value
        )
        self.occupied_cost_intensity = max(
            self.occupied_threshold,
            float(self.get_parameter("occupied_cost_intensity").value),
        )
        self.hit_log_odds = float(self.get_parameter("hit_log_odds").value)
        self.miss_log_odds = float(self.get_parameter("miss_log_odds").value)
        self.min_log_odds = float(self.get_parameter("min_log_odds").value)
        self.max_log_odds = float(self.get_parameter("max_log_odds").value)
        self.occupied_log_odds_threshold = float(
            self.get_parameter("occupied_log_odds_threshold").value
        )
        self.free_log_odds_threshold = float(
            self.get_parameter("free_log_odds_threshold").value
        )
        self.mark_low_intensity_free = bool(
            self.get_parameter("mark_low_intensity_free").value
        )
        self.ground_clears_occupied = bool(
            self.get_parameter("ground_clears_occupied").value
        )
        self.raytrace_free_space = bool(
            self.get_parameter("raytrace_free_space").value
        )
        self.raytrace_clears_occupied = bool(
            self.get_parameter("raytrace_clears_occupied").value
        )
        self.occupied_clear_log_odds_threshold = float(
            self.get_parameter("occupied_clear_log_odds_threshold").value
        )
        self.raytrace_max_range = float(self.get_parameter("raytrace_max_range").value)
        self.max_raytrace_cells = int(
            self.get_parameter("max_raytrace_cells_per_update").value
        )
        self.occupied_padding_radius = float(
            self.get_parameter("occupied_padding_radius").value
        )
        self.filter_speckles = bool(self.get_parameter("filter_speckles").value)
        self.min_occupied_component_cells = max(
            1, int(self.get_parameter("min_occupied_component_cells").value)
        )
        self.min_occupied_component_span_cells = max(
            1, int(self.get_parameter("min_occupied_component_span_cells").value)
        )
        self.occupied_gap_bridge_cells = max(
            0, int(self.get_parameter("occupied_gap_bridge_cells").value)
        )
        self.remove_dense_blob_components = bool(
            self.get_parameter("remove_dense_blob_components").value
        )
        self.dense_blob_min_cells = max(
            1, int(self.get_parameter("dense_blob_min_cells").value)
        )
        self.dense_blob_min_short_span_cells = max(
            1, int(self.get_parameter("dense_blob_min_short_span_cells").value)
        )
        self.dense_blob_min_density = max(
            0.0, float(self.get_parameter("dense_blob_min_density").value)
        )
        self.reference_odom_topic = str(
            self.get_parameter("reference_odom_topic").value
        )
        self.max_reference_odom_error = float(
            self.get_parameter("max_reference_odom_error").value
        )
        self.max_reference_yaw_error = float(
            self.get_parameter("max_reference_yaw_error").value
        )
        self.reference_odom_timeout = float(
            self.get_parameter("reference_odom_timeout").value
        )
        self.reference_odom_warn_period = float(
            self.get_parameter("reference_odom_warn_period").value
        )
        self.reference_odom_enabled = (
            bool(self.reference_odom_topic)
            and self.reference_odom_topic != self.odom_topic
            and (
                self.max_reference_odom_error > 0.0
                or self.max_reference_yaw_error > 0.0
            )
        )
        self.robot_clear_radius = float(self.get_parameter("robot_clear_radius").value)
        self.robot_clear_length = float(self.get_parameter("robot_clear_length").value)
        self.robot_clear_width = float(self.get_parameter("robot_clear_width").value)
        self.robot_clear_margin = float(self.get_parameter("robot_clear_margin").value)
        self.publish_padding_cells = int(
            self.get_parameter("publish_padding_cells").value
        )
        self.max_side_cells = int(self.get_parameter("max_side_cells").value)
        self.inflation_radius = float(self.get_parameter("inflation_radius").value)
        self.inflation_cost_scaling = float(
            self.get_parameter("inflation_cost_scaling").value
        )
        self.max_inflation_sources = int(
            self.get_parameter("max_inflation_sources").value
        )
        self.hit_offsets = self.make_disk_offsets(self.occupied_padding_radius)
        self.inflation_offsets = self.make_inflation_offsets()

        self.log_odds = {}
        self.costs = {}
        self.robot_xy = None
        self.robot_yaw = 0.0
        self.reference_xy = None
        self.reference_yaw = 0.0
        self.reference_odom_received_monotonic = 0.0
        self.odom_guard_robot_start_xy = None
        self.odom_guard_reference_start_xy = None
        self.odom_guard_yaw_offset = 0.0
        self.last_odom_guard_reason = None
        self.last_odom_guard_warn_monotonic = 0.0
        self.latest_stamp = None
        self.dirty = False
        self.last_map_msg = None
        self.last_costmap_msg = None

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.create_subscription(PointCloud2, self.input_topic, self.cloud_callback, sensor_qos)
        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)
        if self.reference_odom_enabled:
            self.create_subscription(
                Odometry,
                self.reference_odom_topic,
                self.reference_odom_callback,
                10,
            )
        self.map_pub = self.create_publisher(OccupancyGrid, self.map_topic, map_qos)
        self.costmap_pub = self.create_publisher(OccupancyGrid, self.costmap_topic, map_qos)
        self.create_timer(max(publish_period, 0.2), self.publish_maps)

        self.get_logger().info(
            f"Projecting {self.input_topic} -> {self.map_topic}, "
            f"cost overlay={self.costmap_topic}, resolution={self.resolution:.2f}, "
            f"accumulate_history={self.accumulate_history}, "
            f"free<={self.free_threshold:.2f}, occupied>={self.occupied_threshold:.2f}, "
            f"hit/miss={self.hit_log_odds:.2f}/{self.miss_log_odds:.2f}, "
            f"protected occupied>={self.occupied_clear_log_odds_threshold:.2f}, "
            f"speckle_filter={self.filter_speckles}:"
            f"{self.min_occupied_component_cells} cells/"
            f"{self.min_occupied_component_span_cells} span, "
            f"gap_bridge={self.occupied_gap_bridge_cells} cells, "
            f"dense_blob_filter={self.remove_dense_blob_components}:"
            f"{self.dense_blob_min_cells} cells/"
            f"{self.dense_blob_min_short_span_cells} short_span/"
            f"{self.dense_blob_min_density:.2f} density, "
            f"ground_clears={self.ground_clears_occupied}"
        )
        if self.reference_odom_enabled:
            self.get_logger().info(
                "Projected-map odom guard enabled: comparing "
                f"relative motion from {self.odom_topic} to "
                f"{self.reference_odom_topic}; "
                f"max_xy_error={self.max_reference_odom_error:.2f} m, "
                f"max_yaw_error={self.max_reference_yaw_error:.2f} rad."
            )

    def odom_callback(self, msg):
        self.robot_xy = (
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
        )
        self.robot_yaw = self.yaw_from_odom(msg)

    def reference_odom_callback(self, msg):
        self.reference_xy = (
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
        )
        self.reference_yaw = self.yaw_from_odom(msg)
        self.reference_odom_received_monotonic = time.monotonic()

    def cloud_callback(self, msg):
        odom_valid, odom_reason = self.odom_guard_is_valid()
        if not odom_valid:
            self.warn_odom_guard(odom_reason)
            return

        points = self.read_points(msg)
        if points.size == 0:
            return

        points = self.filter_points(points)
        if points.size == 0:
            return

        if not self.accumulate_history:
            self.log_odds.clear()
            self.costs.clear()

        cell_xy = np.floor(points[:, 0:2] / self.resolution).astype(np.int32)
        unique_cells, inverse = np.unique(cell_xy, axis=0, return_inverse=True)
        max_intensity = np.full(unique_cells.shape[0], -np.inf, dtype=np.float32)
        min_intensity = np.full(unique_cells.shape[0], np.inf, dtype=np.float32)
        np.maximum.at(max_intensity, inverse, points[:, 3])
        np.minimum.at(min_intensity, inverse, points[:, 3])

        hit_cells = []
        raytrace_hit_cells = []
        free_cells = []
        for idx, (cell_x, cell_y) in enumerate(unique_cells):
            key = (int(cell_x), int(cell_y))
            intensity = float(max_intensity[idx])
            if intensity >= self.occupied_threshold:
                raytrace_hit_cells.append(key)
                cost = min(
                    100,
                    max(1, int(100.0 * intensity / self.occupied_cost_intensity)),
                )
                for hit_key in self.expand_hit_cell(key):
                    self.add_log_odds(hit_key, self.hit_log_odds)
                    self.costs[hit_key] = max(self.costs.get(hit_key, 0), cost)
                    hit_cells.append(hit_key)
            elif self.mark_low_intensity_free and float(min_intensity[idx]) <= self.free_threshold:
                free_cells.append(key)

        hit_set = set(hit_cells)
        if self.robot_xy is not None:
            if self.raytrace_free_space:
                self.mark_raytrace_free_space(raytrace_hit_cells, hit_set)
            self.clear_robot_footprint(hit_set)

        for key in free_cells:
            if key in hit_set:
                continue
            if self.should_keep_occupied(key, self.ground_clears_occupied):
                continue
            self.add_log_odds(key, -self.miss_log_odds)

        self.latest_stamp = msg.header.stamp
        self.dirty = True

    def read_points(self, msg):
        total_points = int(msg.width * msg.height)
        if total_points <= 0:
            return np.empty((0, 4), dtype=np.float32)

        uvs = None
        if self.max_points_per_update > 0 and total_points > self.max_points_per_update:
            uvs = np.linspace(
                0, total_points - 1, self.max_points_per_update, dtype=np.int64
            )

        available_fields = {field.name for field in msg.fields}
        if not {"x", "y", "z"}.issubset(available_fields):
            return np.empty((0, 4), dtype=np.float32)

        field_names = ["x", "y", "z"]
        has_intensity = "intensity" in available_fields
        if has_intensity:
            field_names.append("intensity")

        try:
            structured = point_cloud2.read_points(
                msg, field_names=field_names, skip_nans=True, uvs=uvs
            )
        except (AssertionError, ValueError, KeyError):
            return np.empty((0, 4), dtype=np.float32)

        if structured.size == 0:
            return np.empty((0, 4), dtype=np.float32)

        xyz = np.column_stack(
            (structured["x"], structured["y"], structured["z"])
        ).astype(np.float32, copy=False)
        if has_intensity:
            intensity = structured["intensity"].astype(np.float32, copy=False)
        else:
            intensity = np.zeros(xyz.shape[0], dtype=np.float32)

        finite = np.isfinite(xyz).all(axis=1) & np.isfinite(intensity)
        if not finite.any():
            return np.empty((0, 4), dtype=np.float32)

        return np.column_stack((xyz[finite], intensity[finite])).astype(
            np.float32, copy=False
        )

    def filter_points(self, points):
        if self.robot_xy is None or self.raytrace_max_range <= 0.0:
            return points

        dx = points[:, 0] - self.robot_xy[0]
        dy = points[:, 1] - self.robot_xy[1]
        mask = (dx * dx + dy * dy) <= self.raytrace_max_range * self.raytrace_max_range
        return points[mask]

    def clear_robot_footprint(self, protected_cells=None):
        if self.robot_clear_length > 0.0 and self.robot_clear_width > 0.0:
            self.clear_robot_rectangle(protected_cells)
            return

        robot_cell = self.world_to_cell(*self.robot_xy)
        radius_cells = max(1, int(self.robot_clear_radius / self.resolution))
        for dx in range(-radius_cells, radius_cells + 1):
            for dy in range(-radius_cells, radius_cells + 1):
                key = (robot_cell[0] + dx, robot_cell[1] + dy)
                if protected_cells is not None and key in protected_cells:
                    continue
                if hypot(dx, dy) * self.resolution <= self.robot_clear_radius:
                    self.clear_cell(key)

    def clear_robot_rectangle(self, protected_cells=None):
        robot_cell = self.world_to_cell(*self.robot_xy)
        half_length = max(0.0, self.robot_clear_length * 0.5 + self.robot_clear_margin)
        half_width = max(0.0, self.robot_clear_width * 0.5 + self.robot_clear_margin)
        radius_cells = max(
            1,
            int(hypot(half_length, half_width) / self.resolution) + 1,
        )
        yaw_cos = cos(self.robot_yaw)
        yaw_sin = sin(self.robot_yaw)

        for dx in range(-radius_cells, radius_cells + 1):
            for dy in range(-radius_cells, radius_cells + 1):
                cell_x = robot_cell[0] + dx
                cell_y = robot_cell[1] + dy
                world_x = (cell_x + 0.5) * self.resolution
                world_y = (cell_y + 0.5) * self.resolution
                rel_x = world_x - self.robot_xy[0]
                rel_y = world_y - self.robot_xy[1]
                key = (cell_x, cell_y)
                if protected_cells is not None and key in protected_cells:
                    continue
                body_x = rel_x * yaw_cos + rel_y * yaw_sin
                body_y = -rel_x * yaw_sin + rel_y * yaw_cos
                if abs(body_x) <= half_length and abs(body_y) <= half_width:
                    self.clear_cell(key)

    def clear_cell(self, key):
        self.log_odds[key] = self.min_log_odds
        self.costs[key] = 0

    def mark_raytrace_free_space(self, observed_cells, protected_cells):
        robot_cell = self.world_to_cell(*self.robot_xy)
        if not observed_cells:
            return

        observed_cells = np.array(list(set(observed_cells)), dtype=np.int32)
        max_cells = max(1, self.max_raytrace_cells)
        if observed_cells.shape[0] > max_cells:
            indices = np.linspace(0, observed_cells.shape[0] - 1, max_cells, dtype=np.int64)
            observed_cells = observed_cells[indices]

        for cell_x, cell_y in observed_cells:
            for key in self.bresenham(robot_cell[0], robot_cell[1], int(cell_x), int(cell_y)):
                if key == (int(cell_x), int(cell_y)):
                    break
                if key not in protected_cells:
                    if self.should_keep_occupied(key, self.raytrace_clears_occupied):
                        continue
                    self.add_log_odds(key, -self.miss_log_odds)

    def add_log_odds(self, key, delta):
        value = self.log_odds.get(key, 0.0) + delta
        self.log_odds[key] = min(self.max_log_odds, max(self.min_log_odds, value))

    def should_keep_occupied(self, key, clearing_enabled):
        odds = self.log_odds.get(key, 0.0)
        if odds < self.occupied_log_odds_threshold:
            return False
        if not clearing_enabled:
            return True
        return odds >= self.occupied_clear_log_odds_threshold

    def expand_hit_cell(self, key):
        cell_x, cell_y = key
        for dx, dy, _ in self.hit_offsets:
            yield (cell_x + dx, cell_y + dy)

    def world_to_cell(self, x, y):
        return (int(floor(x / self.resolution)), int(floor(y / self.resolution)))

    @staticmethod
    def bresenham(x0, y0, x1, y1):
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        x = x0
        y = y0

        while True:
            yield (x, y)
            if x == x1 and y == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy

    def publish_maps(self):
        if not self.log_odds:
            return

        if self.dirty:
            self.last_map_msg, self.last_costmap_msg = self.create_grid_messages()
            self.dirty = False
        elif not self.republish_unchanged or self.last_map_msg is None:
            return

        stamp = self.get_clock().now().to_msg()
        self.last_map_msg.header.stamp = stamp
        self.last_map_msg.info.map_load_time = stamp
        self.last_costmap_msg.header.stamp = stamp
        self.last_costmap_msg.info.map_load_time = stamp
        self.map_pub.publish(self.last_map_msg)
        self.costmap_pub.publish(self.last_costmap_msg)

    def create_grid_messages(self):
        keys = np.array(list(self.log_odds.keys()), dtype=np.int32)
        min_x = int(keys[:, 0].min()) - self.publish_padding_cells
        max_x = int(keys[:, 0].max()) + self.publish_padding_cells
        min_y = int(keys[:, 1].min()) - self.publish_padding_cells
        max_y = int(keys[:, 1].max()) + self.publish_padding_cells

        min_x, max_x = self.crop_axis(min_x, max_x, axis=0)
        min_y, max_y = self.crop_axis(min_y, max_y, axis=1)

        width = max_x - min_x + 1
        height = max_y - min_y + 1
        occupancy = np.full((height, width), -1, dtype=np.int8)
        cost = np.full((height, width), -1, dtype=np.int8)
        odds_grid = np.full((height, width), -np.inf, dtype=np.float32)

        for (cell_x, cell_y), odds in self.log_odds.items():
            if cell_x < min_x or cell_x > max_x or cell_y < min_y or cell_y > max_y:
                continue
            x = cell_x - min_x
            y = cell_y - min_y
            odds_grid[y, x] = np.float32(odds)
            if odds >= self.occupied_log_odds_threshold:
                occupancy[y, x] = np.int8(100)
                cell_cost = max(100, self.costs.get((cell_x, cell_y), 100))
                cost[y, x] = np.int8(min(100, cell_cost))
            elif odds <= self.free_log_odds_threshold:
                occupancy[y, x] = np.int8(0)
                cost[y, x] = np.int8(0)

        self.bridge_occupied_gaps(occupancy, cost)
        self.filter_occupied_components(occupancy, cost, odds_grid)
        self.apply_inflation(cost)

        map_msg = self.make_occupancy_grid(occupancy, min_x, min_y)
        costmap_msg = self.make_occupancy_grid(cost, min_x, min_y)
        return map_msg, costmap_msg

    def crop_axis(self, min_cell, max_cell, axis):
        side = max_cell - min_cell + 1
        if side <= self.max_side_cells or self.max_side_cells <= 0:
            return min_cell, max_cell

        if self.robot_xy is not None:
            center = self.world_to_cell(*self.robot_xy)[axis]
        else:
            center = (min_cell + max_cell) // 2

        half = self.max_side_cells // 2
        return center - half, center - half + self.max_side_cells - 1

    def make_disk_offsets(self, radius):
        if radius <= 0.0:
            return [(0, 0, 100)]

        radius_cells = max(1, int(radius / self.resolution))
        offsets = []
        for dx in range(-radius_cells, radius_cells + 1):
            for dy in range(-radius_cells, radius_cells + 1):
                if hypot(dx, dy) * self.resolution <= radius + 1e-6:
                    offsets.append((dx, dy, 100))
        return offsets

    def filter_occupied_components(self, occupancy, cost, odds_grid=None):
        occupied = occupancy >= 100
        if not occupied.any():
            return

        min_cells = self.min_occupied_component_cells
        if self.filter_speckles:
            min_cells = max(min_cells, 2)
        if min_cells <= 1:
            return

        height, width = occupancy.shape
        visited = np.zeros((height, width), dtype=bool)
        remove = np.zeros((height, width), dtype=bool)
        neighbors = (
            (-1, -1), (0, -1), (1, -1),
            (-1, 0), (1, 0),
            (-1, 1), (0, 1), (1, 1),
        )

        for start_y, start_x in np.argwhere(occupied):
            if visited[start_y, start_x]:
                continue

            stack = [(int(start_x), int(start_y))]
            component = []
            visited[start_y, start_x] = True

            while stack:
                cell_x, cell_y = stack.pop()
                component.append((cell_x, cell_y))
                for offset_x, offset_y in neighbors:
                    next_x = cell_x + offset_x
                    next_y = cell_y + offset_y
                    if (
                        next_x < 0
                        or next_x >= width
                        or next_y < 0
                        or next_y >= height
                        or visited[next_y, next_x]
                        or not occupied[next_y, next_x]
                    ):
                        continue
                    visited[next_y, next_x] = True
                    stack.append((next_x, next_y))

            if self.should_keep_occupied_component(component, min_cells, odds_grid):
                continue
            for cell_x, cell_y in component:
                remove[cell_y, cell_x] = True

        occupancy[remove] = np.int8(-1)
        cost[remove] = np.int8(-1)

    def should_keep_occupied_component(self, component, min_cells, odds_grid=None):
        xs = [cell_x for cell_x, _ in component]
        ys = [cell_y for _, cell_y in component]
        span_x = max(xs) - min(xs) + 1
        span_y = max(ys) - min(ys) + 1
        if (
            len(component) < min_cells
            and max(span_x, span_y) < self.min_occupied_component_span_cells
        ):
            return False

        if self.should_remove_dense_blob_component(
            component, span_x, span_y, odds_grid
        ):
            return False

        return True

    def should_remove_dense_blob_component(
        self, component, span_x, span_y, odds_grid=None
    ):
        if not self.remove_dense_blob_components:
            return False
        if len(component) < self.dense_blob_min_cells:
            return False

        short_span = min(span_x, span_y)
        if short_span < self.dense_blob_min_short_span_cells:
            return False

        density = len(component) / float(max(1, span_x * span_y))
        if density < self.dense_blob_min_density:
            return False

        return True

    def bridge_occupied_gaps(self, occupancy, cost):
        if self.occupied_gap_bridge_cells <= 0:
            return

        for _ in range(self.occupied_gap_bridge_cells):
            occupied = occupancy >= 100
            if not occupied.any():
                return

            bridge = np.zeros_like(occupied, dtype=bool)
            bridge[:, 1:-1] |= occupied[:, :-2] & occupied[:, 2:]
            bridge[1:-1, :] |= occupied[:-2, :] & occupied[2:, :]
            bridge[1:-1, 1:-1] |= occupied[:-2, :-2] & occupied[2:, 2:]
            bridge[1:-1, 1:-1] |= occupied[2:, :-2] & occupied[:-2, 2:]
            bridge &= ~occupied
            if not bridge.any():
                return

            occupancy[bridge] = np.int8(100)
            cost[bridge] = np.int8(100)

    def make_inflation_offsets(self):
        if self.inflation_radius <= 0.0:
            return []

        radius_cells = max(1, int(self.inflation_radius / self.resolution))
        offsets = []
        for dx in range(-radius_cells, radius_cells + 1):
            for dy in range(-radius_cells, radius_cells + 1):
                distance = hypot(dx, dy) * self.resolution
                if distance > self.inflation_radius:
                    continue
                inflated_cost = int(98.0 * exp(-self.inflation_cost_scaling * distance))
                offsets.append((dx, dy, max(1, min(98, inflated_cost))))
        return offsets

    def apply_inflation(self, cost):
        if not self.inflation_offsets:
            return

        height, width = cost.shape
        occupied = cost >= 100
        if not occupied.any():
            return

        inflated = cost.astype(np.int16, copy=True)
        for dx, dy, inflated_cost in self.inflation_offsets:
            if abs(dx) >= width or abs(dy) >= height:
                continue

            if dx >= 0:
                src_x = slice(0, width - dx)
                dst_x = slice(dx, width)
            else:
                src_x = slice(-dx, width)
                dst_x = slice(0, width + dx)

            if dy >= 0:
                src_y = slice(0, height - dy)
                dst_y = slice(dy, height)
            else:
                src_y = slice(-dy, height)
                dst_y = slice(0, height + dy)

            source = occupied[src_y, src_x]
            if not source.any():
                continue

            target = inflated[dst_y, dst_x]
            target[source] = np.maximum(target[source], inflated_cost)

        cost[:, :] = np.clip(inflated, -1, 100).astype(np.int8)

    def make_occupancy_grid(self, data, min_x, min_y):
        msg = OccupancyGrid()
        msg.header.stamp = self.latest_stamp if self.latest_stamp is not None else self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.info.map_load_time = msg.header.stamp
        msg.info.resolution = self.resolution
        msg.info.width = int(data.shape[1])
        msg.info.height = int(data.shape[0])
        msg.info.origin.position.x = float(min_x * self.resolution)
        msg.info.origin.position.y = float(min_y * self.resolution)
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0
        msg.data = array("b", data.ravel(order="C"))
        return msg

    def odom_guard_is_valid(self):
        if not self.reference_odom_enabled:
            return True, ""
        if self.robot_xy is None:
            return False, f"waiting for odometry on {self.odom_topic}"
        if self.reference_xy is None:
            return False, f"waiting for reference odometry on {self.reference_odom_topic}"

        age = time.monotonic() - self.reference_odom_received_monotonic
        if self.reference_odom_timeout > 0.0 and age > self.reference_odom_timeout:
            return False, (
                f"reference odometry {self.reference_odom_topic} is stale "
                f"({age:.2f}s > {self.reference_odom_timeout:.2f}s)"
            )

        self.ensure_odom_guard_baseline()
        expected_xy = self.expected_reference_xy_in_odom_frame()
        xy_error = hypot(
            self.robot_xy[0] - expected_xy[0],
            self.robot_xy[1] - expected_xy[1],
        )
        if (
            self.max_reference_odom_error > 0.0
            and xy_error > self.max_reference_odom_error
        ):
            return False, (
                f"{self.odom_topic} relative drift from "
                f"{self.reference_odom_topic} is {xy_error:.2f} m; "
                "freezing projected-map integration"
            )

        yaw_error = abs(
            self.normalize_angle(
                self.robot_yaw
                - self.reference_yaw
                - self.odom_guard_yaw_offset
            )
        )
        if (
            self.max_reference_yaw_error > 0.0
            and yaw_error > self.max_reference_yaw_error
        ):
            return False, (
                f"{self.odom_topic} relative yaw drift from "
                f"{self.reference_odom_topic} is {yaw_error:.2f} rad; "
                "freezing projected-map integration"
            )

        if self.last_odom_guard_reason is not None:
            self.get_logger().info("Projected-map odom guard cleared; accepting point clouds.")
            self.last_odom_guard_reason = None
        return True, ""

    def warn_odom_guard(self, reason):
        now = time.monotonic()
        if (
            reason == self.last_odom_guard_reason
            and now - self.last_odom_guard_warn_monotonic
            < self.reference_odom_warn_period
        ):
            return

        self.last_odom_guard_reason = reason
        self.last_odom_guard_warn_monotonic = now
        self.get_logger().warn(reason)

    def ensure_odom_guard_baseline(self):
        if self.odom_guard_robot_start_xy is not None:
            return

        self.odom_guard_robot_start_xy = self.robot_xy
        self.odom_guard_reference_start_xy = self.reference_xy
        self.odom_guard_yaw_offset = self.normalize_angle(
            self.robot_yaw - self.reference_yaw
        )
        initial_xy_offset = hypot(
            self.robot_xy[0] - self.reference_xy[0],
            self.robot_xy[1] - self.reference_xy[1],
        )
        self.get_logger().info(
            "Projected-map odom guard baseline set: "
            f"initial_xy_offset={initial_xy_offset:.2f} m, "
            f"initial_yaw_offset={self.odom_guard_yaw_offset:.2f} rad."
        )

    def expected_reference_xy_in_odom_frame(self):
        reference_dx = self.reference_xy[0] - self.odom_guard_reference_start_xy[0]
        reference_dy = self.reference_xy[1] - self.odom_guard_reference_start_xy[1]
        yaw_cos = cos(self.odom_guard_yaw_offset)
        yaw_sin = sin(self.odom_guard_yaw_offset)
        return (
            self.odom_guard_robot_start_xy[0]
            + reference_dx * yaw_cos
            - reference_dy * yaw_sin,
            self.odom_guard_robot_start_xy[1]
            + reference_dx * yaw_sin
            + reference_dy * yaw_cos,
        )

    @staticmethod
    def yaw_from_odom(msg):
        orientation = msg.pose.pose.orientation
        siny_cosp = 2.0 * (
            orientation.w * orientation.z + orientation.x * orientation.y
        )
        cosy_cosp = 1.0 - 2.0 * (
            orientation.y * orientation.y + orientation.z * orientation.z
        )
        return atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def normalize_angle(angle):
        while angle > np.pi:
            angle -= 2.0 * np.pi
        while angle < -np.pi:
            angle += 2.0 * np.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    node = ProjectedTerrainMap()
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
