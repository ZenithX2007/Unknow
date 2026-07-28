#!/usr/bin/env python3

from array import array
from math import exp, floor, hypot

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
        self.declare_parameter("robot_clear_radius", 0.8)
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
        self.robot_clear_radius = float(self.get_parameter("robot_clear_radius").value)
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
        self.map_pub = self.create_publisher(OccupancyGrid, self.map_topic, map_qos)
        self.costmap_pub = self.create_publisher(OccupancyGrid, self.costmap_topic, map_qos)
        self.create_timer(max(publish_period, 0.2), self.publish_maps)

        self.get_logger().info(
            f"Projecting {self.input_topic} -> {self.map_topic}, "
            f"cost overlay={self.costmap_topic}, resolution={self.resolution:.2f}, "
            f"free<={self.free_threshold:.2f}, occupied>={self.occupied_threshold:.2f}, "
            f"hit/miss={self.hit_log_odds:.2f}/{self.miss_log_odds:.2f}, "
            f"protected occupied>={self.occupied_clear_log_odds_threshold:.2f}"
        )

    def odom_callback(self, msg):
        self.robot_xy = (
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
        )

    def cloud_callback(self, msg):
        points = self.read_points(msg)
        if points.size == 0:
            return

        points = self.filter_points(points)
        if points.size == 0:
            return

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
            self.clear_robot_footprint()

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

    def clear_robot_footprint(self):
        robot_cell = self.world_to_cell(*self.robot_xy)
        radius_cells = max(1, int(self.robot_clear_radius / self.resolution))
        for dx in range(-radius_cells, radius_cells + 1):
            for dy in range(-radius_cells, radius_cells + 1):
                if hypot(dx, dy) * self.resolution <= self.robot_clear_radius:
                    key = (robot_cell[0] + dx, robot_cell[1] + dy)
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

        for (cell_x, cell_y), odds in self.log_odds.items():
            if cell_x < min_x or cell_x > max_x or cell_y < min_y or cell_y > max_y:
                continue
            x = cell_x - min_x
            y = cell_y - min_y
            if odds >= self.occupied_log_odds_threshold:
                occupancy[y, x] = np.int8(100)
                cell_cost = max(100, self.costs.get((cell_x, cell_y), 100))
                cost[y, x] = np.int8(min(100, cell_cost))
            elif odds <= self.free_log_odds_threshold:
                occupancy[y, x] = np.int8(0)
                cost[y, x] = np.int8(0)

        if self.filter_speckles:
            self.filter_occupied_speckles(occupancy, cost)
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

    def filter_occupied_speckles(self, occupancy, cost):
        occupied = occupancy >= 100
        if not occupied.any():
            return

        height, width = occupancy.shape
        neighbor_count = np.zeros((height, width), dtype=np.uint8)
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                if dx == 0 and dy == 0:
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
                neighbor_count[dst_y, dst_x] += occupied[src_y, src_x]

        speckles = occupied & (neighbor_count == 0)
        occupancy[speckles] = np.int8(-1)
        cost[speckles] = np.int8(-1)

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
