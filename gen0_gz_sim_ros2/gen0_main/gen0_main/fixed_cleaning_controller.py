#!/usr/bin/env python3
"""Deterministic cleaning-route controller independent of Nav2.

Supports a fixed pose route and optional lane-change-style obstacle avoidance
using the front LaserScan range data. Route points are tuples of:
- (x, y) for normal mode
- (x, y, yaw_radians) for explicit yaw target at that point
"""

import math
import ast
import time
import rclpy
from geometry_msgs.msg import PoseArray, PoseStamped, Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from rclpy.node import Node


def yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y*q.y + q.z*q.z))


def normalize_angle(angle):
    """Normalize angle to [-pi, pi)."""
    return math.atan2(math.sin(angle), math.cos(angle))


def _interpolate_angle(a, b, alpha):
    """Shortest-path interpolation between two angles."""
    return a + normalize_angle(b - a) * alpha


class FixedCleaningController(Node):
    def __init__(self):
        super().__init__('fixed_cleaning_controller')
        self.declare_parameter('pose_topic', '/gen0_model/links/poses')
        self.declare_parameter('pose_index', 15)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('speed', 1.0)
        self.declare_parameter('goal_tolerance', 0.7)
        self.declare_parameter('front_offset', 1.75)
        self.declare_parameter('front_center_tolerance', 0.1)
        self.declare_parameter('front_center_waypoint_advance', False)
        self.declare_parameter('strict_waypoint_mode', False)
        self.declare_parameter(
            'route_points',
            '[{"x":17.27,"y":-44.68,"yaw":-0.52},'
            '{"x":25.27,"y":-46.68,"yaw":-0.52},'
            '{"x":29.27,"y":-44.68,"yaw":1.05},'
            '{"x":38.82,"y":-30.68,"yaw":2.62},'
            '{"x":35.82,"y":-21.68,"yaw":2.62},'
            '{"x":-16.20,"y":8.22,"yaw":2.62}]'
        )
        # Obstacle-aware lane-change (front obstacle -> side shift, then recover).
        # This is the stable coordinate-route mode.  Traffic/lidar avoidance
        # is intentionally disabled; the route itself must remain unchanged.
        self.declare_parameter('enable_obstacle_avoidance', False)
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('front_scan_distance', 3.0)
        self.declare_parameter('front_scan_angle_deg', 20.0)
        # The nominal route must not drift because of walls/curbs.  This is
        # only the maximum temporary offset used while yielding to traffic.
        self.declare_parameter('lane_offset_max', 0.8)
        self.declare_parameter('lane_offset_step', 0.15)
        self.declare_parameter('lane_return_step', 0.12)
        self.declare_parameter('obstacle_lost_cycles', 20)
        self.declare_parameter('avoid_side', 'left')
        self.declare_parameter('yaw_gain', 1.4)
        self.declare_parameter('yaw_limit', 0.45)
        self.declare_parameter('yaw_to_path_blend', 1.0)
        self.declare_parameter('lane_center_yaw_index', 3)
        self.declare_parameter('post_turn_lane_offset', 0.0)
        self.declare_parameter('car009_yield_enabled', True)
        self.declare_parameter('car009_pose_topic', '/car/car_009/pose')
        self.declare_parameter('car009_approach_distance', 10.0)
        self.declare_parameter('car009_yaw_band', 1.5)
        self.declare_parameter('car009_yield_lateral_tolerance', 2.2)
        self.declare_parameter('car009_yield_offset', 1.0)
        self.declare_parameter('car009_yield_clear_time', 1.2)
        self.declare_parameter('car008_yield_enabled', True)
        self.declare_parameter('car008_pose_topic', '/car/car_008/pose')
        self.declare_parameter('car008_approach_distance', 10.0)
        self.declare_parameter('car008_yaw_band', 1.5)
        self.declare_parameter('car008_yield_lateral_tolerance', 2.2)
        self.declare_parameter('car008_yield_offset', 1.0)
        self.declare_parameter('car008_yield_clear_time', 1.2)
        self.declare_parameter('anchor_to_current_pose', False)
        self.declare_parameter('anchor_route_rotation', False)

        self.route = self._parse_route(self.get_parameter('route_points').value)
        if len(self.route) < 2:
            # Fallback to a minimal safe default route.
            self.route = self._default_route()
        self._raw_route = list(self.route)
        self._route_anchored = False

        self._obstacle_avoidance = bool(self.get_parameter('enable_obstacle_avoidance').value)
        self._avoid_side = 1.0 if self.get_parameter('avoid_side').value == 'left' else -1.0
        self._anchor_to_current_pose = bool(self.get_parameter('anchor_to_current_pose').value)
        self._anchor_route_rotation = bool(self.get_parameter('anchor_route_rotation').value)
        self._lane_center_yaw_index = int(self.get_parameter('lane_center_yaw_index').value)
        self._post_turn_lane_offset = float(self.get_parameter('post_turn_lane_offset').value)
        self._car009_yield_enabled = bool(self.get_parameter('car009_yield_enabled').value)
        self._car009_pose_topic = str(self.get_parameter('car009_pose_topic').value)
        self._car009_approach_distance = float(self.get_parameter('car009_approach_distance').value)
        self._car009_yaw_band = float(self.get_parameter('car009_yaw_band').value)
        self._car009_yield_lateral_tolerance = float(self.get_parameter('car009_yield_lateral_tolerance').value)
        self._car009_yield_offset = float(self.get_parameter('car009_yield_offset').value)
        self._car009_yield_clear_time = max(0.1, float(self.get_parameter('car009_yield_clear_time').value))
        self._car008_yield_enabled = bool(self.get_parameter('car008_yield_enabled').value)
        self._car008_pose_topic = str(self.get_parameter('car008_pose_topic').value)
        self._car008_approach_distance = float(self.get_parameter('car008_approach_distance').value)
        self._car008_yaw_band = float(self.get_parameter('car008_yaw_band').value)
        self._car008_yield_lateral_tolerance = float(self.get_parameter('car008_yield_lateral_tolerance').value)
        self._car008_yield_offset = float(self.get_parameter('car008_yield_offset').value)
        self._car008_yield_clear_time = max(0.1, float(self.get_parameter('car008_yield_clear_time').value))
        self._front_obstacle_seen = None
        self._lane_offset = 0.0
        self._car009_yield_active = False
        self._car009_seen_monotonic = 0.0
        self._car008_yield_active = False
        self._car008_seen_monotonic = 0.0
        self._front_center_waypoint_advance = bool(self.get_parameter('front_center_waypoint_advance').value)
        self._strict_waypoint_mode = bool(self.get_parameter('strict_waypoint_mode').value)
        self._yaw_to_path_blend = float(self.get_parameter('yaw_to_path_blend').value)

        self.pose = None
        self.index = 0
        self.car009_pose = None
        self.car009_pose_monotonic = 0.0
        self.car008_pose = None
        self.car008_pose_monotonic = 0.0
        self.create_subscription(PoseArray, self.get_parameter('pose_topic').value, self.pose_cb, 10)
        self.pub = self.create_publisher(Twist, self.get_parameter('cmd_vel_topic').value, 10)
        self.yield_state_pub = self.create_publisher(String, '/gen0_model/traffic_yield_state', 10)
        if self._obstacle_avoidance and not self._strict_waypoint_mode:
            self.create_subscription(LaserScan, self.get_parameter('scan_topic').value, self.scan_cb, 10)
        if self._obstacle_avoidance and self._car009_yield_enabled and self._car009_pose_topic and not self._strict_waypoint_mode:
            self.create_subscription(PoseStamped, self._car009_pose_topic, self.car009_pose_cb, 10)
        if self._obstacle_avoidance and self._car008_yield_enabled and self._car008_pose_topic and not self._strict_waypoint_mode:
            self.create_subscription(PoseStamped, self._car008_pose_topic, self.car008_pose_cb, 10)
        self.timer = self.create_timer(0.05, self.control)

    def _default_route(self):
        return [
            (17.27, -44.68, -0.52),
            (25.27, -46.68, -0.52),
            (29.27, -44.68, 1.05),
            (38.82, -30.68, 2.62),
            (35.82, -21.68, 2.62),
            (-16.20, 8.22, 2.62),
        ]

    def _parse_route(self, raw_route):
        """Parse user route definitions from parameter input."""
        points = []
        if isinstance(raw_route, list):
            for item in raw_route:
                parsed = self._coerce_route_item(item)
                if parsed is not None:
                    points.append(parsed)
            return points
        if isinstance(raw_route, str):
            raw_route = raw_route.strip()
            if not raw_route:
                return []
            try:
                loaded = ast.literal_eval(raw_route)
            except Exception:
                self.get_logger().error(
                    f'Invalid route_points syntax: {raw_route!r}. Fallback to default route.'
                )
                return []
            if isinstance(loaded, list):
                for item in loaded:
                    parsed = self._coerce_route_item(item)
                    if parsed is not None:
                        points.append(parsed)
        return points

    @staticmethod
    def _coerce_route_item(item):
        if isinstance(item, dict):
            if 'x' in item and 'y' in item:
                try:
                    x = float(item['x'])
                    y = float(item['y'])
                    yaw = float(item['yaw']) if 'yaw' in item else None
                    return (x, y, yaw)
                except (TypeError, ValueError):
                    return None
        elif isinstance(item, (list, tuple)):
            if len(item) >= 2:
                try:
                    x = float(item[0]); y = float(item[1])
                    yaw = float(item[2]) if len(item) >= 3 else None
                    return (x, y, yaw)
                except (TypeError, ValueError):
                    return None
        return None

    def scan_cb(self, msg):
        half_angle = math.radians(float(self.get_parameter('front_scan_angle_deg').value))
        obstacle_distance = float(self.get_parameter('front_scan_distance').value)
        if not msg.ranges:
            return
        angle_min = msg.angle_min
        angle_inc = msg.angle_increment if msg.angle_increment != 0.0 else 0.01
        start_idx = int(max(0, math.floor((-half_angle - angle_min) / angle_inc)))
        end_idx = int(min(len(msg.ranges), math.ceil((half_angle - angle_min) / angle_inc)))
        if start_idx < 0:
            start_idx = 0
        if end_idx < start_idx:
            end_idx = len(msg.ranges)
        front_obstacle = False
        for r in msg.ranges[start_idx:end_idx]:
            if msg.range_min <= r <= msg.range_max and r < obstacle_distance:
                front_obstacle = True
                break
        if front_obstacle:
            self._front_obstacle_seen = 0
        else:
            if self._front_obstacle_seen is not None:
                self._front_obstacle_seen += 1

    def _update_lane_offset(self, obstacle_in_front):
        max_offset = float(self.get_parameter('lane_offset_max').value)
        step = float(self.get_parameter('lane_offset_step').value)
        recover = float(self.get_parameter('lane_return_step').value)
        if obstacle_in_front:
            self._lane_offset = min(max_offset, self._lane_offset + step)
        else:
            self._lane_offset = max(0.0, self._lane_offset - recover)

    def _obstacle_active(self):
        if self._front_obstacle_seen is None:
            return False
        max_lost_cycles = int(self.get_parameter('obstacle_lost_cycles').value)
        return self._front_obstacle_seen <= max_lost_cycles

    def pose_cb(self, msg):
        i = int(self.get_parameter('pose_index').value)
        if i < len(msg.poses): self.pose = msg.poses[i]

    def car009_pose_cb(self, msg):
        if not self._car009_yield_enabled:
            return
        self.car009_pose = msg.pose
        self.car009_pose_monotonic = time.monotonic()

    def car008_pose_cb(self, msg):
        if not self._car008_yield_enabled:
            return
        self.car008_pose = msg.pose
        self.car008_pose_monotonic = time.monotonic()

    def _apply_pose_anchor_if_needed(self):
        if self._route_anchored:
            return
        if not self._anchor_to_current_pose or not self.route:
            self._route_anchored = True
            return
        if self.pose is None:
            return

        raw_start_x, raw_start_y, raw_start_yaw = self._raw_route[0]
        if raw_start_yaw is None:
            raw_start_yaw = 0.0

        current_x = self.pose.position.x
        current_y = self.pose.position.y
        current_yaw = yaw(self.pose.orientation)
        front_offset = float(self.get_parameter('front_offset').value)
        current_front_x = current_x + front_offset * math.cos(current_yaw)
        current_front_y = current_y + front_offset * math.sin(current_yaw)
        anchor_base_x = current_front_x - raw_start_x
        anchor_base_y = current_front_y - raw_start_y

        if self._anchor_route_rotation:
            yaw_delta = normalize_angle(current_yaw - raw_start_yaw)
            cos_delta = math.cos(yaw_delta)
            sin_delta = math.sin(yaw_delta)
            anchored_route = []
            for x, y, point_yaw in self._raw_route:
                rel_x = x - raw_start_x
                rel_y = y - raw_start_y
                anchored_x = current_front_x + rel_x * cos_delta - rel_y * sin_delta
                anchored_y = current_front_y + rel_x * sin_delta + rel_y * cos_delta
                if point_yaw is None:
                    anchored_yaw = None
                else:
                    anchored_yaw = normalize_angle(point_yaw + yaw_delta)
                anchored_route.append((anchored_x, anchored_y, anchored_yaw))
            self.route = anchored_route
        else:
            self.route = [
                (x + anchor_base_x, y + anchor_base_y, point_yaw)
                for x, y, point_yaw in self._raw_route
            ]

        self._route_anchored = True
        self.get_logger().info(
            f'Anchored fixed route to current pose '
            f'first=( {self.route[0][0]:.3f}, {self.route[0][1]:.3f}, {self.route[0][2]} ) -> '
            f'current=({current_x:.3f}, {current_y:.3f}, {current_yaw:.3f}), '
            f'rotation_anchor={self._anchor_route_rotation}'
        )

    def control(self):
        out = Twist()
        if self.pose is None:
            self.pub.publish(out); return
        self._apply_pose_anchor_if_needed()
        x, y = self.pose.position.x, self.pose.position.y
        final_x, final_y, _ = self.route[-1]
        final_distance = math.hypot(x - final_x, y - final_y)
        goal_tolerance = float(self.get_parameter('goal_tolerance').value)
        if self.index == len(self.route) - 1 and final_distance <= goal_tolerance:
            self.pub.publish(out)
            return

        heading = yaw(self.pose.orientation)
        # LaserScan is an emergency brake only.  In particular, it must never
        # steer around a curb or a wall, since those are what caused the old
        # controller to accumulate a lateral error and hit the road edge.
        obstacle_in_front = self._obstacle_active() if not self._strict_waypoint_mode else False
        if self._obstacle_avoidance and not self._strict_waypoint_mode:
            self._update_car009_yield_state(x, y, heading)
            self._update_car008_yield_state(x, y, heading)
            traffic_avoidance = (
                self._car009_yield_active or self._car008_yield_active
            )
            # Only traffic activates a lateral avoidance offset.  A scan hit
            # remains a stop condition and cannot alter the nominal route.
            self._update_lane_offset(traffic_avoidance)
        else:
            self._lane_offset = 0.0
            self._car009_yield_active = False
            self._car008_yield_active = False

        yield_active = self._car009_yield_active or self._car008_yield_active
        yield_msg = String()
        if self._strict_waypoint_mode:
            yield_msg.data = 'normal'
        else:
            if yield_active:
                if self._car009_yield_active and self._car008_yield_active:
                    yield_msg.data = 'avoiding_both'
                elif self._car009_yield_active:
                    yield_msg.data = 'avoiding_car_009'
                else:
                    yield_msg.data = 'avoiding_car_008'
            else:
                yield_msg.data = 'normal'
        self.yield_state_pub.publish(yield_msg)

        # For each trash target, advance only after the midpoint of the front
        # bumper physically reaches the trash center.
        while self.index < len(self.route) - 1:
            reached = (
                self.front_center_reached_target(x, y, self.index)
                if self._front_center_waypoint_advance
                else self.body_reached_target(x, y, self.index)
            )
            if not reached:
                break
            self.index += 1
        tx, ty, target_yaw = self.route[self.index]
        if self.index < len(self.route) - 1:
            dx_path, dy_path = tx - x, ty - y
            path_length = math.hypot(dx_path, dy_path)
            if path_length > 1e-6:
                path_heading = math.atan2(dy_path, dx_path)
            else:
                path_heading = heading

            # Keep the previously stable route exactly.  A lateral offset is
            # added only while the explicit traffic-yield mode is active.
            lane_offset = 0.0
            if self._lane_offset > 1e-3:
                lane_offset += self._lane_offset
            if lane_offset > 1e-3:
                tx += lane_offset * self._avoid_side * (-math.sin(path_heading))
                ty += lane_offset * self._avoid_side * (math.cos(path_heading))

            front_offset = float(self.get_parameter('front_offset').value)
            tx -= front_offset * math.cos(path_heading)
            ty -= front_offset * math.sin(path_heading)
        dx, dy = tx-x, ty-y
        target_heading = math.atan2(dy, dx)

        # Keep moving toward the current waypoint by position first, then
        # progressively follow the configured yaw profile as we approach the
        # next waypoint.
        if len(self.route) > 1 and self.index < len(self.route) - 1:
            next_x, next_y, next_yaw = self.route[self.index + 1]
            segment_dx = next_x - tx
            segment_dy = next_y - ty
            segment_length = math.hypot(segment_dx, segment_dy)
            if segment_length > 1e-6:
                dist_to_next = math.hypot(x - next_x, y - next_y)
                # Blend yaw only within one segment-length of the next point so
                # this does not block initial convergence when starting far
                # from the route.
                route_alpha = 1.0 - min(dist_to_next, segment_length) / segment_length
                if route_alpha > 0.0:
                    route_alpha = max(0.0, min(1.0, route_alpha))
                    yaw_from_waypoints = target_yaw if target_yaw is not None else self.pose_to_heading()
                    if next_yaw is not None:
                        yaw_from_waypoints = _interpolate_angle(
                            yaw_from_waypoints,
                            next_yaw,
                            route_alpha
                        )
                    target_heading = _interpolate_angle(
                        target_heading,
                        yaw_from_waypoints,
                        min(1.0, route_alpha * self._yaw_to_path_blend)
                    )
        else:
            if target_yaw is not None:
                target_heading = target_yaw
        error = math.atan2(math.sin(target_heading-heading), math.cos(target_heading-heading))
        speed = min(float(self.get_parameter('speed').value), 1.2)
        if obstacle_in_front:
            speed = 0.0
        if self.index == len(self.route) - 1:
            speed = min(speed, max(0.15, final_distance * 0.8))
        yaw_gain = float(self.get_parameter('yaw_gain').value)
        yaw_limit = float(self.get_parameter('yaw_limit').value)
        out.linear.x = speed
        out.angular.z = max(-yaw_limit, min(yaw_limit, yaw_gain * error))
        self.pub.publish(out)

    def front_center_reached_target(self, x, y, target_index):
        """Return true once the front-bumper midpoint reaches the trash center."""
        tx, ty, _ = self.route[target_index]
        front_offset = float(self.get_parameter('front_offset').value)
        heading = yaw(self.pose.orientation)
        fx = x + front_offset * math.cos(heading)
        fy = y + front_offset * math.sin(heading)
        tolerance = max(
            float(self.get_parameter('front_center_tolerance').value),
            float(self.get_parameter('goal_tolerance').value) * 0.3
        )
        return math.hypot(fx - tx, fy - ty) <= tolerance

    def body_reached_target(self, x, y, target_index):
        """Return true once the vehicle body reaches the target point."""
        tx, ty, _ = self.route[target_index]
        return math.hypot(x - tx, y - ty) <= float(self.get_parameter('goal_tolerance').value)

    def pose_to_heading(self):
        if self.pose is None:
            return 0.0
        return yaw(self.pose.orientation)

    def _is_post_turn_lane_mode(self):
        """True once the vehicle should hold a lane-center offset."""
        if self._strict_waypoint_mode:
            return False
        if len(self.route) <= 1:
            return False
        if self._lane_center_yaw_index >= len(self.route):
            return self.index >= len(self.route) - 1
        if self.index >= self._lane_center_yaw_index:
            return True
        target_yaw = self.route[self._lane_center_yaw_index][2]
        if target_yaw is None:
            return False
        current_yaw = yaw(self.pose.orientation)
        return abs(normalize_angle(current_yaw - target_yaw)) <= self._car009_yaw_band

    def _update_car009_yield_state(self, x, y, vehicle_yaw):
        if self._strict_waypoint_mode:
            self._car009_yield_active = False
            return
        if not self._car009_yield_enabled or self.car009_pose is None:
            self._car009_yield_active = False
            return
        if (time.monotonic() - self.car009_pose_monotonic) > 1.2:
            self._car009_yield_active = False
            return

        rel_x = self.car009_pose.position.x - x
        rel_y = self.car009_pose.position.y - y
        cos_yaw = math.cos(vehicle_yaw)
        sin_yaw = math.sin(vehicle_yaw)
        car_local_x = cos_yaw * rel_x + sin_yaw * rel_y
        car_local_y = -sin_yaw * rel_x + cos_yaw * rel_y
        approaching = (
            0.5 < car_local_x <= self._car009_approach_distance
            and abs(car_local_y) <= self._car009_yield_lateral_tolerance
        )
        if approaching:
            self._car009_seen_monotonic = time.monotonic()
            self._car009_yield_active = True
            return

        if self._car009_yield_active and (time.monotonic() - self._car009_seen_monotonic) > self._car009_yield_clear_time:
            self._car009_yield_active = False

    def _update_car008_yield_state(self, x, y, vehicle_yaw):
        if self._strict_waypoint_mode:
            self._car008_yield_active = False
            return
        if not self._car008_yield_enabled or self.car008_pose is None:
            self._car008_yield_active = False
            return
        if (time.monotonic() - self.car008_pose_monotonic) > 1.2:
            self._car008_yield_active = False
            return

        rel_x = self.car008_pose.position.x - x
        rel_y = self.car008_pose.position.y - y
        cos_yaw = math.cos(vehicle_yaw)
        sin_yaw = math.sin(vehicle_yaw)
        car_local_x = cos_yaw * rel_x + sin_yaw * rel_y
        car_local_y = -sin_yaw * rel_x + cos_yaw * rel_y
        approaching = (
            0.5 < car_local_x <= self._car008_approach_distance
            and abs(car_local_y) <= self._car008_yield_lateral_tolerance
        )
        if approaching:
            self._car008_seen_monotonic = time.monotonic()
            self._car008_yield_active = True
            return

        if self._car008_yield_active and (time.monotonic() - self._car008_seen_monotonic) > self._car008_yield_clear_time:
            self._car008_yield_active = False


def main(args=None):
    rclpy.init(args=args); node = FixedCleaningController()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__': main()
