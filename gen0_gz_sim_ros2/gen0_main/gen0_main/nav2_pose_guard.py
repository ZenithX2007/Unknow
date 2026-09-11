#!/usr/bin/env python3
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformException, TransformListener


class Nav2PoseGuard(Node):
    def __init__(self):
        super().__init__('nav2_pose_guard')

        self.declare_parameter('input_cmd_vel_topic', '/control/cmd_vel_raw')
        self.declare_parameter('output_cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('reference_odom_topic', '')
        self.declare_parameter('max_reference_odom_error', 0.0)
        self.declare_parameter('max_reference_yaw_error', 0.0)
        self.declare_parameter('reference_odom_timeout', 2.0)
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
        self.declare_parameter('actor_obstacle_topic', '/gen0_mapping/actor_obstacles')
        self.declare_parameter('actor_obstacle_timeout', 0.6)
        self.declare_parameter('actor_vehicle_length', 4.0)
        self.declare_parameter('actor_vehicle_width', 2.0)
        self.declare_parameter('actor_radius', 0.45)
        self.declare_parameter('actor_safety_margin', 0.35)
        self.declare_parameter('actor_forward_buffer', 0.8)
        self.declare_parameter('actor_reverse_buffer', 0.35)
        self.declare_parameter('actor_forward_lateral_clearance', 0.75)

        self.input_cmd_vel_topic = self.get_parameter('input_cmd_vel_topic').value
        self.output_cmd_vel_topic = self.get_parameter('output_cmd_vel_topic').value
        self.map_topic = self.get_parameter('map_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.reference_odom_topic = self.get_parameter('reference_odom_topic').value
        self.max_reference_odom_error = float(
            self.get_parameter('max_reference_odom_error').value
        )
        self.max_reference_yaw_error = float(
            self.get_parameter('max_reference_yaw_error').value
        )
        self.reference_odom_timeout = float(
            self.get_parameter('reference_odom_timeout').value
        )
        self.reference_odom_enabled = (
            bool(self.reference_odom_topic)
            and self.reference_odom_topic != self.odom_topic
            and (
                self.max_reference_odom_error > 0.0
                or self.max_reference_yaw_error > 0.0
            )
        )
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
        self.actor_obstacle_topic = self.get_parameter('actor_obstacle_topic').value
        self.actor_obstacle_timeout = float(
            self.get_parameter('actor_obstacle_timeout').value
        )
        self.actor_vehicle_length = max(
            0.1, float(self.get_parameter('actor_vehicle_length').value)
        )
        self.actor_vehicle_width = max(
            0.1, float(self.get_parameter('actor_vehicle_width').value)
        )
        self.actor_radius = max(0.05, float(self.get_parameter('actor_radius').value))
        self.actor_safety_margin = max(
            0.0, float(self.get_parameter('actor_safety_margin').value)
        )
        self.actor_forward_buffer = max(
            0.0, float(self.get_parameter('actor_forward_buffer').value)
        )
        self.actor_reverse_buffer = max(
            0.0, float(self.get_parameter('actor_reverse_buffer').value)
        )
        self.actor_forward_lateral_clearance = max(
            0.05,
            float(self.get_parameter('actor_forward_lateral_clearance').value),
        )

        self.map_bounds = None
        self.last_odom = None
        self.last_reference_odom = None
        self.last_reference_odom_monotonic = 0.0
        self.odom_guard_start_xy = None
        self.reference_odom_guard_start_xy = None
        self.reference_odom_guard_yaw_offset = 0.0
        self.last_pose_xy = None
        self.blocked_reason = None
        self.last_warn_monotonic = 0.0
        self.last_curvature_warn_monotonic = 0.0
        self.actor_points_map = []
        self.actor_points_monotonic = 0.0
        self.actor_cloud_frame = None

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
        self.create_subscription(
            PointCloud2,
            self.actor_obstacle_topic,
            self.actor_obstacle_callback,
            qos_profile_sensor_data,
        )
        if self.reference_odom_enabled:
            self.create_subscription(
                Odometry,
                self.reference_odom_topic,
                self.reference_odom_callback,
                qos_profile_sensor_data,
            )
        self.create_timer(self.stop_publish_period, self.timer_callback)

        self.get_logger().info(
            'Gating Nav2 cmd_vel from '
            f'{self.input_cmd_vel_topic} to {self.output_cmd_vel_topic}; '
            f'pose must stay inside {self.map_topic} bounds in '
            f'{self.map_frame}->{self.base_frame}.'
        )
        self.get_logger().info(
            'Actor collision guard enabled: '
            f'{self.actor_obstacle_topic}, footprint='
            f'{self.actor_vehicle_length:.2f}x{self.actor_vehicle_width:.2f}m, '
            f'margin={self.actor_safety_margin:.2f}m, '
            f'forward_buffer={self.actor_forward_buffer:.2f}m, '
            f'forward_lateral_clearance={self.actor_forward_lateral_clearance:.2f}m.'
        )
        if self.min_turning_radius > 0.0:
            self.get_logger().info(
                'Ackermann curvature limiting enabled: '
                f'|angular.z| <= |linear.x| / {self.min_turning_radius:.2f}.'
            )
        if self.reference_odom_enabled:
            self.get_logger().info(
                'FAST-LIO odom health guard enabled: comparing '
                f'relative motion from {self.odom_topic} to '
                f'{self.reference_odom_topic}; '
                f'max_xy_error={self.max_reference_odom_error:.2f} m, '
                f'max_yaw_error={self.max_reference_yaw_error:.2f} rad.'
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

    def reference_odom_callback(self, msg):
        self.last_reference_odom = msg
        self.last_reference_odom_monotonic = time.monotonic()

    def actor_obstacle_callback(self, msg):
        if not msg.header.frame_id:
            return

        transform = None
        if msg.header.frame_id != self.map_frame:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.map_frame,
                    msg.header.frame_id,
                    Time.from_msg(msg.header.stamp),
                )
            except TransformException as exc:
                self.get_logger().warn(
                    f'Ignoring actor obstacles: missing TF '
                    f'{self.map_frame}->{msg.header.frame_id}: {exc}'
                )
                return

        points = []
        try:
            for point in point_cloud2.read_points(
                msg,
                field_names=('x', 'y', 'intensity'),
                skip_nans=True,
            ):
                if len(point) < 3 or float(point[2]) < 0.3:
                    continue
                x = float(point[0])
                y = float(point[1])
                if transform is not None:
                    x, y = self.transform_xy(x, y, transform)
                points.append((x, y))
        except (TypeError, ValueError, RuntimeError) as exc:
            self.get_logger().warn(f'Could not read actor obstacle cloud: {exc}')
            return

        self.actor_points_map = points
        self.actor_points_monotonic = time.monotonic()
        self.actor_cloud_frame = msg.header.frame_id

    @staticmethod
    def transform_xy(x, y, transform):
        rotation = transform.transform.rotation
        siny_cosp = 2.0 * (rotation.w * rotation.z + rotation.x * rotation.y)
        cosy_cosp = 1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        translation = transform.transform.translation
        return (
            cos_yaw * x - sin_yaw * y + translation.x,
            sin_yaw * x + cos_yaw * y + translation.y,
        )

    def cmd_callback(self, msg):
        valid, reason = self.pose_is_valid()
        if valid:
            actor_blocked, actor_reason = self.actor_collision_guard(msg)
            if actor_blocked:
                self.publish_zero(actor_reason)
                return
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

    def actor_collision_guard(self, msg):
        if not self.actor_points_map:
            return False, ''
        if (
            self.actor_obstacle_timeout > 0.0
            and time.monotonic() - self.actor_points_monotonic
            > self.actor_obstacle_timeout
        ):
            return False, ''

        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                Time(),
            )
        except TransformException:
            return False, ''

        vehicle_x = transform.transform.translation.x
        vehicle_y = transform.transform.translation.y
        vehicle_yaw = self.yaw_from_transform(transform)

        # This is the final emergency gate, not the local planner. Let MPPI
        # turn around a detected actor or back away from it. A hard stop is
        # needed only for a forward command aimed at an actor in the narrow
        # center corridor; otherwise this gate prevents the planned detour.
        linear_x = float(msg.linear.x)
        angular_z = float(msg.angular.z)
        if not math.isfinite(linear_x) or not math.isfinite(angular_z):
            return True, 'non-finite command while actor collision guard is active'
        if linear_x <= 0.02 or abs(angular_z) >= 0.05:
            return False, ''

        cos_yaw = math.cos(vehicle_yaw)
        sin_yaw = math.sin(vehicle_yaw)
        half_length = self.actor_vehicle_length * 0.5
        half_width = self.actor_vehicle_width * 0.5
        margin = self.actor_safety_margin + self.actor_radius
        min_x = -margin
        max_x = half_length + margin + self.actor_forward_buffer
        lateral_limit = min(
            half_width + margin,
            self.actor_forward_lateral_clearance,
        )
        for actor_x, actor_y in self.actor_points_map:
            rel_x = actor_x - vehicle_x
            rel_y = actor_y - vehicle_y
            local_x = cos_yaw * rel_x + sin_yaw * rel_y
            local_y = -sin_yaw * rel_x + cos_yaw * rel_y
            if min_x <= local_x <= max_x and abs(local_y) <= lateral_limit:
                return True, (
                    f'forward actor corridor occupied: '
                    f'local=({local_x:.2f},{local_y:.2f}) '
                    f'cloud={self.actor_cloud_frame or "unknown"}'
                )
        return False, ''

    @staticmethod
    def yaw_from_transform(transform):
        rotation = transform.transform.rotation
        siny_cosp = 2.0 * (rotation.w * rotation.z + rotation.x * rotation.y)
        cosy_cosp = 1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z)
        return math.atan2(siny_cosp, cosy_cosp)

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
        reference_valid, reference_reason = self.reference_odom_is_valid()
        if not reference_valid:
            return False, reference_reason

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

    def reference_odom_is_valid(self):
        if not self.reference_odom_enabled:
            return True, ''
        if self.last_reference_odom is None:
            return False, f'waiting for reference odometry on {self.reference_odom_topic}'

        age = time.monotonic() - self.last_reference_odom_monotonic
        if self.reference_odom_timeout > 0.0 and age > self.reference_odom_timeout:
            return False, (
                f'reference odometry {self.reference_odom_topic} is stale '
                f'({age:.2f}s > {self.reference_odom_timeout:.2f}s)'
            )

        odom_position = self.last_odom.pose.pose.position
        reference_position = self.last_reference_odom.pose.pose.position
        if not self.values_are_finite(
            reference_position.x,
            reference_position.y,
            reference_position.z,
        ):
            return False, (
                f'non-finite reference odometry pose on {self.reference_odom_topic}: '
                f'x={reference_position.x}, y={reference_position.y}, '
                f'z={reference_position.z}'
            )

        self.ensure_reference_odom_baseline()
        expected_xy = self.expected_reference_xy_in_odom_frame()
        xy_error = math.hypot(
            float(odom_position.x) - expected_xy[0],
            float(odom_position.y) - expected_xy[1],
        )
        if (
            self.max_reference_odom_error > 0.0
            and xy_error > self.max_reference_odom_error
        ):
            return False, (
                f'{self.odom_topic} relative drift from '
                f'{self.reference_odom_topic} is {xy_error:.2f} m; '
                'FAST-LIO/Nav2 odom chain is unhealthy'
            )

        yaw_error = abs(
            self.normalize_angle(
                self.yaw_from_odom(self.last_odom)
                - self.yaw_from_odom(self.last_reference_odom)
                - self.reference_odom_guard_yaw_offset
            )
        )
        if (
            self.max_reference_yaw_error > 0.0
            and yaw_error > self.max_reference_yaw_error
        ):
            return False, (
                f'{self.odom_topic} relative yaw drift from '
                f'{self.reference_odom_topic} is {yaw_error:.2f} rad; '
                'FAST-LIO/Nav2 odom chain is unhealthy'
            )

        return True, ''

    def ensure_reference_odom_baseline(self):
        if self.odom_guard_start_xy is not None:
            return

        odom_position = self.last_odom.pose.pose.position
        reference_position = self.last_reference_odom.pose.pose.position
        self.odom_guard_start_xy = (
            float(odom_position.x),
            float(odom_position.y),
        )
        self.reference_odom_guard_start_xy = (
            float(reference_position.x),
            float(reference_position.y),
        )
        self.reference_odom_guard_yaw_offset = self.normalize_angle(
            self.yaw_from_odom(self.last_odom)
            - self.yaw_from_odom(self.last_reference_odom)
        )
        initial_xy_offset = math.hypot(
            self.odom_guard_start_xy[0] - self.reference_odom_guard_start_xy[0],
            self.odom_guard_start_xy[1] - self.reference_odom_guard_start_xy[1],
        )
        self.get_logger().info(
            'FAST-LIO odom health guard baseline set: '
            f'initial_xy_offset={initial_xy_offset:.2f} m, '
            f'initial_yaw_offset={self.reference_odom_guard_yaw_offset:.2f} rad.'
        )

    def expected_reference_xy_in_odom_frame(self):
        reference_position = self.last_reference_odom.pose.pose.position
        reference_dx = (
            float(reference_position.x) - self.reference_odom_guard_start_xy[0]
        )
        reference_dy = (
            float(reference_position.y) - self.reference_odom_guard_start_xy[1]
        )
        yaw_cos = math.cos(self.reference_odom_guard_yaw_offset)
        yaw_sin = math.sin(self.reference_odom_guard_yaw_offset)
        return (
            self.odom_guard_start_xy[0]
            + reference_dx * yaw_cos
            - reference_dy * yaw_sin,
            self.odom_guard_start_xy[1]
            + reference_dx * yaw_sin
            + reference_dy * yaw_cos,
        )

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

    @staticmethod
    def yaw_from_odom(msg):
        orientation = msg.pose.pose.orientation
        siny_cosp = 2.0 * (
            orientation.w * orientation.z + orientation.x * orientation.y
        )
        cosy_cosp = 1.0 - 2.0 * (
            orientation.y * orientation.y + orientation.z * orientation.z
        )
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def normalize_angle(angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


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
