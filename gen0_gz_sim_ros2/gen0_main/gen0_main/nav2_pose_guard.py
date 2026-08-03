#!/usr/bin/env python3
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


class Nav2PoseGuard(Node):
    def __init__(self):
        super().__init__('nav2_pose_guard')

        self.declare_parameter('input_cmd_vel_topic', '/control/cmd_vel_raw')
        self.declare_parameter('output_cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('bounds_margin', 5.0)
        self.declare_parameter('max_abs_z', 20.0)
        self.declare_parameter('stop_publish_period', 0.1)
        self.declare_parameter('warn_period', 1.0)
        self.declare_parameter('min_turning_radius', 0.0)
        self.declare_parameter('curvature_warn_period', 2.0)
        self.declare_parameter('curvature_warn_min_delta', 0.03)
        self.declare_parameter('curvature_warn_min_abs_wz', 0.08)
        self.declare_parameter('max_pose_jump', 3.0)

        self.input_cmd_vel_topic = self.get_parameter('input_cmd_vel_topic').value
        self.output_cmd_vel_topic = self.get_parameter('output_cmd_vel_topic').value
        self.map_topic = self.get_parameter('map_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.bounds_margin = float(self.get_parameter('bounds_margin').value)
        self.max_abs_z = float(self.get_parameter('max_abs_z').value)
        self.stop_publish_period = float(self.get_parameter('stop_publish_period').value)
        self.warn_period = float(self.get_parameter('warn_period').value)
        self.min_turning_radius = float(self.get_parameter('min_turning_radius').value)
        self.curvature_warn_period = float(self.get_parameter('curvature_warn_period').value)
        self.curvature_warn_min_delta = float(
            self.get_parameter('curvature_warn_min_delta').value
        )
        self.curvature_warn_min_abs_wz = float(
            self.get_parameter('curvature_warn_min_abs_wz').value
        )
        self.max_pose_jump = float(self.get_parameter('max_pose_jump').value)

        self.map_bounds = None
        self.last_odom = None
        self.last_pose_xy = None
        self.blocked_reason = None
        self.last_warn_monotonic = 0.0
        self.last_curvature_warn_monotonic = 0.0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self.cmd_pub = self.create_publisher(Twist, self.output_cmd_vel_topic, 10)
        self.create_subscription(Twist, self.input_cmd_vel_topic, self.cmd_callback, 10)
        self.create_subscription(OccupancyGrid, self.map_topic, self.map_callback, map_qos)
        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, qos_profile_sensor_data)
        self.create_timer(self.stop_publish_period, self.timer_callback)

        self.get_logger().info(
            'Gating Nav2 cmd_vel from '
            f'{self.input_cmd_vel_topic} to {self.output_cmd_vel_topic}; '
            f'pose must stay inside {self.map_topic} bounds in '
            f'{self.map_frame}->{self.base_frame}.'
        )
        if self.min_turning_radius > 0.0:
            self.get_logger().info(
                'Ackermann curvature limiting enabled: '
                f'|angular.z| <= |linear.x| / {self.min_turning_radius:.2f}.'
            )

    def map_callback(self, msg):
        resolution = float(msg.info.resolution)
        width = int(msg.info.width)
        height = int(msg.info.height)
        if resolution <= 0.0 or width <= 0 or height <= 0:
            self.map_bounds = None
            return

        origin_x = float(msg.info.origin.position.x)
        origin_y = float(msg.info.origin.position.y)
        end_x = origin_x + width * resolution
        end_y = origin_y + height * resolution
        self.map_bounds = (
            min(origin_x, end_x),
            max(origin_x, end_x),
            min(origin_y, end_y),
            max(origin_y, end_y),
        )

    def odom_callback(self, msg):
        self.last_odom = msg

    def cmd_callback(self, msg):
        valid, reason = self.pose_is_valid()
        if valid:
            self.cmd_pub.publish(self.limit_ackermann_curvature(msg))
            return

        self.publish_zero(reason)

    def timer_callback(self):
        valid, reason = self.pose_is_valid()
        if valid:
            if self.blocked_reason is not None:
                self.get_logger().info('Pose guard cleared; forwarding Nav2 velocity commands.')
                self.blocked_reason = None
            return

        self.publish_zero(reason)

    def pose_is_valid(self):
        if self.map_bounds is None:
            return False, f'waiting for occupancy grid on {self.map_topic}'
        if self.last_odom is None:
            return False, f'waiting for odometry on {self.odom_topic}'

        odom_position = self.last_odom.pose.pose.position
        if not self.values_are_finite(odom_position.x, odom_position.y, odom_position.z):
            return False, (
                'non-finite odometry pose: '
                f'x={odom_position.x}, y={odom_position.y}, z={odom_position.z}'
            )

        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                Time(),
            )
        except TransformException as exc:
            return False, f'missing TF {self.map_frame}->{self.base_frame}: {exc}'

        translation = transform.transform.translation
        if not self.values_are_finite(translation.x, translation.y, translation.z):
            return False, (
                f'non-finite TF {self.map_frame}->{self.base_frame}: '
                f'x={translation.x}, y={translation.y}, z={translation.z}'
            )

        if abs(translation.z) > self.max_abs_z:
            return False, (
                f'{self.map_frame}->{self.base_frame} z={translation.z:.2f} exceeds '
                f'max_abs_z={self.max_abs_z:.2f}'
            )

        current_xy = (float(translation.x), float(translation.y))
        if self.last_pose_xy is not None and self.max_pose_jump > 0.0:
            pose_jump = math.hypot(
                current_xy[0] - self.last_pose_xy[0],
                current_xy[1] - self.last_pose_xy[1],
            )
            if pose_jump > self.max_pose_jump:
                self.last_pose_xy = current_xy
                return False, (
                    f'{self.map_frame}->{self.base_frame} pose jumped '
                    f'{pose_jump:.2f} m between samples; '
                    f'max_pose_jump={self.max_pose_jump:.2f}'
                )
        self.last_pose_xy = current_xy

        min_x, max_x, min_y, max_y = self.map_bounds
        margin = self.bounds_margin
        if not (min_x - margin <= translation.x <= max_x + margin):
            return False, (
                f'{self.map_frame}->{self.base_frame} x={translation.x:.2f} is outside '
                f'map bounds [{min_x:.2f}, {max_x:.2f}] with margin {margin:.2f}'
            )
        if not (min_y - margin <= translation.y <= max_y + margin):
            return False, (
                f'{self.map_frame}->{self.base_frame} y={translation.y:.2f} is outside '
                f'map bounds [{min_y:.2f}, {max_y:.2f}] with margin {margin:.2f}'
            )

        return True, ''

    def publish_zero(self, reason):
        self.cmd_pub.publish(Twist())
        self.warn_blocked(reason)

    def limit_ackermann_curvature(self, msg):
        if self.min_turning_radius <= 0.0:
            return msg

        limited = Twist()
        limited.linear.x = msg.linear.x
        limited.linear.y = msg.linear.y
        limited.linear.z = msg.linear.z
        limited.angular.x = msg.angular.x
        limited.angular.y = msg.angular.y
        limited.angular.z = msg.angular.z

        vx = float(limited.linear.x)
        wz = float(limited.angular.z)
        if not self.values_are_finite(vx, wz):
            self.warn_curvature_limited('non-finite cmd_vel received; publishing zero')
            return Twist()

        max_wz = abs(vx) / self.min_turning_radius
        if abs(wz) <= max_wz:
            return limited

        limited.angular.z = math.copysign(max_wz, wz) if max_wz > 0.0 else 0.0
        if (
            abs(wz - limited.angular.z) >= self.curvature_warn_min_delta
            and abs(wz) >= self.curvature_warn_min_abs_wz
        ):
            self.warn_curvature_limited(
                'clamped cmd_vel curvature: '
                f'linear.x={vx:.3f}, angular.z={wz:.3f} -> {limited.angular.z:.3f}'
            )
        return limited

    def warn_blocked(self, reason):
        now = time.monotonic()
        if reason == self.blocked_reason and now - self.last_warn_monotonic < self.warn_period:
            return

        self.blocked_reason = reason
        self.last_warn_monotonic = now
        self.get_logger().warn(f'Blocking Nav2 velocity output; publishing zero cmd_vel: {reason}')

    def warn_curvature_limited(self, reason):
        now = time.monotonic()
        if now - self.last_curvature_warn_monotonic < self.curvature_warn_period:
            return

        self.last_curvature_warn_monotonic = now
        self.get_logger().warn(reason)

    @staticmethod
    def values_are_finite(*values):
        return all(math.isfinite(float(value)) for value in values)


def main(args=None):
    rclpy.init(args=args)
    node = Nav2PoseGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except KeyboardInterrupt:
                pass


if __name__ == '__main__':
    main()
