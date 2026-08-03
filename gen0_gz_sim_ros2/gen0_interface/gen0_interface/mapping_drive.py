#!/usr/bin/env python3

import math

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String


class MappingDrive(Node):
    def __init__(self):
        super().__init__("gen0_mapping_drive")

        self.declare_parameter("enabled", False)
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("fl_scan_topic", "/gen0_model/fl/lidar/scan")
        self.declare_parameter("fr_scan_topic", "/gen0_model/fr/lidar/scan")
        self.declare_parameter("front3d_topic", "/gen0_model/front3d/lidar/points")
        self.declare_parameter("safety_source", "front3d")
        self.declare_parameter("drive_speed", 0.35)
        self.declare_parameter("turn_angular_z", 0.04)
        self.declare_parameter("straight_duration", 10.0)
        self.declare_parameter("turn_duration", 5.0)
        self.declare_parameter("front_arc_degrees", 45.0)
        self.declare_parameter("stop_distance", 2.5)
        self.declare_parameter("slow_distance", 5.0)
        self.declare_parameter("scan_timeout", 2.0)
        self.declare_parameter("ignore_scan_min_margin", 0.08)
        self.declare_parameter("front3d_timeout", 2.0)
        self.declare_parameter("front3d_min_x", 0.8)
        self.declare_parameter("front3d_max_x", 18.0)
        self.declare_parameter("front3d_half_width", 1.8)
        self.declare_parameter("front3d_min_z", -1.2)
        self.declare_parameter("front3d_max_z", 1.5)
        self.declare_parameter("publish_rate", 10.0)

        self.enabled = bool(self.get_parameter("enabled").value)
        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self.safety_source = self.get_parameter("safety_source").value
        self.drive_speed = float(self.get_parameter("drive_speed").value)
        self.turn_angular_z = float(self.get_parameter("turn_angular_z").value)
        self.straight_duration = float(self.get_parameter("straight_duration").value)
        self.turn_duration = float(self.get_parameter("turn_duration").value)
        self.front_arc = math.radians(float(self.get_parameter("front_arc_degrees").value))
        self.stop_distance = float(self.get_parameter("stop_distance").value)
        self.slow_distance = float(self.get_parameter("slow_distance").value)
        self.scan_timeout = float(self.get_parameter("scan_timeout").value)
        self.ignore_scan_min_margin = float(self.get_parameter("ignore_scan_min_margin").value)
        self.front3d_timeout = float(self.get_parameter("front3d_timeout").value)
        self.front3d_min_x = float(self.get_parameter("front3d_min_x").value)
        self.front3d_max_x = float(self.get_parameter("front3d_max_x").value)
        self.front3d_half_width = float(self.get_parameter("front3d_half_width").value)
        self.front3d_min_z = float(self.get_parameter("front3d_min_z").value)
        self.front3d_max_z = float(self.get_parameter("front3d_max_z").value)
        publish_rate = float(self.get_parameter("publish_rate").value)

        self.fl_scan = None
        self.fr_scan = None
        self.fl_scan_time = None
        self.fr_scan_time = None
        self.front3d_cloud = None
        self.front3d_time = None
        self.start_time = self.get_clock().now()
        self.turn_sign = 1.0
        self.last_status = None

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.status_pub = self.create_publisher(String, "/gen0_mapping/drive_status", 10)
        self.create_subscription(
            LaserScan,
            self.get_parameter("fl_scan_topic").value,
            self.fl_scan_callback,
            10,
        )
        self.create_subscription(
            LaserScan,
            self.get_parameter("fr_scan_topic").value,
            self.fr_scan_callback,
            10,
        )
        self.create_subscription(
            PointCloud2,
            self.get_parameter("front3d_topic").value,
            self.front3d_callback,
            10,
        )

        timer_period = 1.0 / max(publish_rate, 1.0)
        self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info(
            f"Mapping drive enabled={self.enabled}, safety_source={self.safety_source}, "
            f"publishing {self.cmd_vel_topic}"
        )

    def fl_scan_callback(self, msg):
        self.fl_scan = msg
        self.fl_scan_time = self.get_clock().now()

    def fr_scan_callback(self, msg):
        self.fr_scan = msg
        self.fr_scan_time = self.get_clock().now()

    def front3d_callback(self, msg):
        self.front3d_cloud = msg
        self.front3d_time = self.get_clock().now()

    def timer_callback(self):
        if not self.enabled:
            self.publish_status("disabled")
            return

        front_min, source_status = self.front_clearance()
        if source_status is not None:
            self.publish_stop(source_status)
            return

        if math.isfinite(front_min) and front_min <= self.stop_distance:
            self.publish_stop(f"blocked_front_min={front_min:.2f}m")
            return

        speed_scale = 1.0
        if math.isfinite(front_min) and front_min < self.slow_distance:
            speed_scale = max(0.2, (front_min - self.stop_distance) / (self.slow_distance - self.stop_distance))

        cmd = Twist()
        cmd.linear.x = self.drive_speed * speed_scale
        cmd.angular.z = self.current_angular_command()
        self.cmd_pub.publish(cmd)
        front_text = f"{front_min:.2f}m" if math.isfinite(front_min) else "clear"
        self.publish_status(
            f"driving v={cmd.linear.x:.2f} wz={cmd.angular.z:.2f} front={front_text}"
        )

    def front_clearance(self):
        if self.safety_source == "front3d":
            if not self.front3d_is_fresh():
                return float("inf"), "waiting_for_fresh_front3d"
            return self.front3d_min_front(), None

        if self.safety_source == "scan":
            if not self.scans_are_fresh():
                return float("inf"), "waiting_for_fresh_scans"
            front_min = min(self.scan_min_front(self.fl_scan), self.scan_min_front(self.fr_scan))
            if not math.isfinite(front_min):
                return float("inf"), "no_valid_front_ranges"
            return front_min, None

        return float("inf"), f"unknown_safety_source={self.safety_source}"

    def scans_are_fresh(self):
        now = self.get_clock().now()
        timeout = Duration(seconds=self.scan_timeout)
        return (
            self.fl_scan is not None
            and self.fr_scan is not None
            and self.fl_scan_time is not None
            and self.fr_scan_time is not None
            and now - self.fl_scan_time <= timeout
            and now - self.fr_scan_time <= timeout
        )

    def front3d_is_fresh(self):
        if self.front3d_cloud is None or self.front3d_time is None:
            return False
        return self.get_clock().now() - self.front3d_time <= Duration(seconds=self.front3d_timeout)

    def scan_min_front(self, scan):
        values = []
        angle = scan.angle_min
        for distance in scan.ranges:
            if -self.front_arc <= angle <= self.front_arc and math.isfinite(distance):
                near_min = scan.range_min + self.ignore_scan_min_margin
                if near_min < distance <= scan.range_max:
                    values.append(distance)
            angle += scan.angle_increment
        return min(values) if values else float("inf")

    def front3d_min_front(self):
        try:
            structured = point_cloud2.read_points(
                self.front3d_cloud,
                field_names=["x", "y", "z"],
                skip_nans=True,
            )
            points = np.column_stack(
                (structured["x"], structured["y"], structured["z"])
            ).astype(np.float32, copy=False)
        except (AssertionError, ValueError, KeyError):
            return float("inf")

        if points.size == 0:
            return float("inf")

        mask = (
            (points[:, 0] >= self.front3d_min_x)
            & (points[:, 0] <= self.front3d_max_x)
            & (np.abs(points[:, 1]) <= self.front3d_half_width)
            & (points[:, 2] >= self.front3d_min_z)
            & (points[:, 2] <= self.front3d_max_z)
        )
        if not np.any(mask):
            return float("inf")

        return float(np.min(points[mask, 0]))

    def current_angular_command(self):
        cycle = self.straight_duration + self.turn_duration
        if cycle <= 0.0:
            return 0.0

        elapsed = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
        cycle_index = int(elapsed // cycle)
        phase_time = elapsed - cycle_index * cycle
        self.turn_sign = 1.0 if cycle_index % 2 == 0 else -1.0

        if phase_time < self.straight_duration:
            return 0.0
        return self.turn_sign * self.turn_angular_z

    def publish_stop(self, reason):
        self.cmd_pub.publish(Twist())
        self.publish_status(reason)

    def publish_status(self, status):
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)
        if status != self.last_status:
            self.get_logger().info(status)
            self.last_status = status


def main(args=None):
    rclpy.init(args=args)
    node = MappingDrive()
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
