#!/usr/bin/env python3
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class VehicleMovementInterface(Node):
    def __init__(self):
        super().__init__('vehicle_movement_interface')

        self.declare_parameter('cmd_vel_topic', '/control/cmd_vel')
        self.declare_parameter('cmd_vel_timeout', 0.5)
        self.declare_parameter('angular_z_sign', 1.0)
        self.declare_parameter('max_forward_speed', 0.65)
        self.declare_parameter('max_reverse_speed', 0.25)
        self.declare_parameter('max_angular_z', 0.12)
        self.declare_parameter('angular_deadband', 0.005)
        self.declare_parameter('steering_rate_limit', 0.45)
        self.declare_parameter('steering_sign_flip_deadband', 0.015)
        self.declare_parameter('warn_on_curvature_clamp', True)
        self.declare_parameter('curvature_clamp_log_period', 2.0)
        self.declare_parameter('front_stop_enabled', False)
        self.declare_parameter('fl_scan_topic', '/gen0_model/fl/lidar/scan')
        self.declare_parameter('fr_scan_topic', '/gen0_model/fr/lidar/scan')
        self.declare_parameter('front_arc_degrees', 35.0)
        self.declare_parameter('front_stop_distance', 0.65)
        self.declare_parameter('front_slow_distance', 1.5)
        self.declare_parameter('scan_timeout', 0.6)
        self.declare_parameter('ignore_scan_min_margin', 0.08)
        self.declare_parameter('require_fresh_scans', False)
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        self.cmd_vel_timeout = float(self.get_parameter('cmd_vel_timeout').value)
        self.angular_z_sign = float(self.get_parameter('angular_z_sign').value)
        self.max_forward_speed = float(self.get_parameter('max_forward_speed').value)
        self.max_reverse_speed = float(self.get_parameter('max_reverse_speed').value)
        self.max_angular_z = float(self.get_parameter('max_angular_z').value)
        self.angular_deadband = max(0.0, float(self.get_parameter('angular_deadband').value))
        self.steering_rate_limit = max(
            0.0,
            float(self.get_parameter('steering_rate_limit').value),
        )
        self.steering_sign_flip_deadband = max(
            0.0,
            float(self.get_parameter('steering_sign_flip_deadband').value),
        )
        self.warn_on_curvature_clamp = bool(
            self.get_parameter('warn_on_curvature_clamp').value
        )
        self.curvature_clamp_log_period = float(
            self.get_parameter('curvature_clamp_log_period').value
        )
        self.front_stop_enabled = bool(self.get_parameter('front_stop_enabled').value)
        self.front_arc = math.radians(float(self.get_parameter('front_arc_degrees').value))
        self.front_stop_distance = float(self.get_parameter('front_stop_distance').value)
        self.front_slow_distance = max(
            self.front_stop_distance,
            float(self.get_parameter('front_slow_distance').value),
        )
        self.scan_timeout = float(self.get_parameter('scan_timeout').value)
        self.ignore_scan_min_margin = float(self.get_parameter('ignore_scan_min_margin').value)
        self.require_fresh_scans = bool(self.get_parameter('require_fresh_scans').value)

        self.wheel_base= 2.8
        self.wheel_track= 1.385
        self.wheel_radius= 0.33
        self.max_velocity_forward = 5.6
        self.min_velocity_forward = 0.3
        self.max_steering = 0.31 # rad
        self.min_steering = -0.31
        self.min_turning_radius = self.wheel_base / (2 * math.tan(self.max_steering))
        self.delta_f = 0.0
        self.delta_r = 0.0
        self.back_right_joint_speed = 0.0
        self.back_left_joint_speed = 0.0

        self.subscription = self.create_subscription(
            Twist,
            cmd_vel_topic,
            self.cmd_callback,
            10
        )
        self.get_logger().info(
            f'Listening for Twist commands on {cmd_vel_topic}, '
            f'angular_z_sign={self.angular_z_sign:.1f}, '
            f'max_forward={self.max_forward_speed:.2f}m/s, '
            f'angular_deadband={self.angular_deadband:.3f}rad/s, '
            f'steering_rate_limit={self.steering_rate_limit:.2f}rad/s, '
            f'front_stop={self.front_stop_enabled}, '
            f'stop_distance={self.front_stop_distance:.2f}m, '
            f'slow_distance={self.front_slow_distance:.2f}m'
        )

        self.publisher_steering_front_left = self.create_publisher(Float64, '/gen0_model/front_left_steering', 10)
        self.publisher_steering_front_right = self.create_publisher(Float64, '/gen0_model/front_right_steering', 10)
        self.publisher_steering_back_left = self.create_publisher(Float64, '/gen0_model/back_left_steering', 10)
        self.publisher_steering_back_right = self.create_publisher(Float64, '/gen0_model/back_right_steering', 10)
        self.publisher_speed_back_left = self.create_publisher(Float64, '/gen0_model/speed_back_left', 10)
        self.publisher_speed_back_right = self.create_publisher(Float64, '/gen0_model/speed_back_right', 10)

        self.front_left_steering_msg = Float64()
        self.front_right_steering_msg = Float64()
        self.back_left_steering_msg = Float64()
        self.back_right_steering_msg = Float64()
        self.speed_back_left_msg = Float64()
        self.speed_back_right_msg = Float64()
        self.last_cmd_time = None
        self.timed_out = False
        self.fl_scan = None
        self.fr_scan = None
        self.fl_scan_time = None
        self.fr_scan_time = None
        self.last_safety_status = None
        self.last_curvature_clamp_log_time = None
        self.last_steering_update_time = None

        if self.front_stop_enabled:
            self.create_subscription(
                LaserScan,
                self.get_parameter('fl_scan_topic').value,
                self.fl_scan_callback,
                10,
            )
            self.create_subscription(
                LaserScan,
                self.get_parameter('fr_scan_topic').value,
                self.fr_scan_callback,
                10,
            )

        # Timer to publish latest values continuously at 50 Hz
        self.timer = self.create_timer(0.02, self.publish_latest_values)

    def cmd_callback(self, msg):
        linear_velocity = self.clamp_linear_velocity(msg.linear.x)
        angular_velocity = self.angular_z_sign * self.clamp(
            msg.angular.z,
            -self.max_angular_z,
            self.max_angular_z,
        )
        if abs(angular_velocity) < self.angular_deadband:
            angular_velocity = 0.0
        linear_velocity, angular_velocity = self.apply_front_safety(
            linear_velocity,
            angular_velocity,
        )

        # Steering angle calculations
        if abs(angular_velocity) > 1e-6:
            if abs(linear_velocity) <= 1e-6:
                turning_radius = math.copysign(self.min_turning_radius, angular_velocity)
                self.maybe_log_curvature_clamp(
                    linear_velocity,
                    angular_velocity,
                    float('inf'),
                )
            else:
                turning_radius = linear_velocity / angular_velocity
                requested_radius = abs(turning_radius)
                if requested_radius < self.min_turning_radius:
                    self.maybe_log_curvature_clamp(
                        linear_velocity,
                        angular_velocity,
                        requested_radius,
                    )
                turning_radius = max(abs(turning_radius), self.min_turning_radius) * math.copysign(1, turning_radius)
        else:
            turning_radius = float('inf')

        target_delta_f = math.atan(self.wheel_base / (2 * turning_radius))
        target_delta_f = self.apply_steering_hysteresis(target_delta_f)
        self.delta_f = self.limit_steering_rate(target_delta_f)
        self.delta_r = -self.delta_f  # rear always the opposite sign but same value

        # Joint velocity calculations
        phi = math.atan(self.wheel_base / turning_radius)
        self.back_right_joint_speed = (linear_velocity * (1.0 + (self.wheel_track * math.tan(phi)) / (2 * self.wheel_base))) / self.wheel_radius
        self.back_left_joint_speed = (linear_velocity * (1.0 - (self.wheel_track * math.tan(phi)) / (2 * self.wheel_base))) / self.wheel_radius

        # Set the data for each message
        self.front_left_steering_msg.data = self.delta_f
        self.front_right_steering_msg.data = self.delta_f
        self.back_left_steering_msg.data = self.delta_r
        self.back_right_steering_msg.data = self.delta_r
        self.speed_back_left_msg.data = self.back_left_joint_speed
        self.speed_back_right_msg.data = self.back_right_joint_speed
        self.last_cmd_time = self.get_clock().now()
        self.timed_out = False

    def apply_steering_hysteresis(self, target_delta_f):
        if (
            self.delta_f * target_delta_f < 0.0
            and abs(target_delta_f) < self.steering_sign_flip_deadband
        ):
            return 0.0
        return target_delta_f

    def limit_steering_rate(self, target_delta_f):
        now = self.get_clock().now()
        if self.last_steering_update_time is None:
            dt = 0.02
        else:
            dt = (now - self.last_steering_update_time).nanoseconds * 1e-9
            dt = min(max(dt, 0.0), 0.1)
        self.last_steering_update_time = now

        if self.steering_rate_limit <= 0.0:
            return target_delta_f

        max_step = self.steering_rate_limit * dt
        delta = self.clamp(
            target_delta_f - self.delta_f,
            -max_step,
            max_step,
        )
        return self.delta_f + delta

    def fl_scan_callback(self, msg):
        self.fl_scan = msg
        self.fl_scan_time = self.get_clock().now()

    def fr_scan_callback(self, msg):
        self.fr_scan = msg
        self.fr_scan_time = self.get_clock().now()

    def clamp_linear_velocity(self, velocity):
        if velocity >= 0.0:
            return min(velocity, self.max_forward_speed)
        return max(velocity, -self.max_reverse_speed)

    @staticmethod
    def clamp(value, lower, upper):
        return max(lower, min(upper, value))

    def apply_front_safety(self, linear_velocity, angular_velocity):
        if not self.front_stop_enabled or linear_velocity <= 0.0:
            self.update_safety_status(None)
            return linear_velocity, angular_velocity

        if not self.scans_are_fresh():
            if self.require_fresh_scans:
                self.update_safety_status('waiting_for_fresh_front_scans')
                return 0.0, 0.0
            return linear_velocity, angular_velocity

        front_min = min(
            self.scan_min_front(self.fl_scan),
            self.scan_min_front(self.fr_scan),
        )
        if not math.isfinite(front_min):
            self.update_safety_status(None)
            return linear_velocity, angular_velocity

        if front_min <= self.front_stop_distance:
            self.update_safety_status(f'front_stop {front_min:.2f}m')
            return 0.0, 0.0

        if front_min < self.front_slow_distance:
            span = self.front_slow_distance - self.front_stop_distance
            scale = 0.2 if span <= 0.0 else max(
                0.2,
                (front_min - self.front_stop_distance) / span,
            )
            self.update_safety_status(f'front_slow {front_min:.2f}m scale={scale:.2f}')
            return linear_velocity * scale, angular_velocity

        self.update_safety_status(None)
        return linear_velocity, angular_velocity

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

    def scan_min_front(self, scan):
        if scan is None:
            return float('inf')

        values = []
        angle = scan.angle_min
        near_min = scan.range_min + self.ignore_scan_min_margin
        for distance in scan.ranges:
            if -self.front_arc <= angle <= self.front_arc and math.isfinite(distance):
                if near_min < distance <= scan.range_max:
                    values.append(distance)
            angle += scan.angle_increment
        return min(values) if values else float('inf')

    def update_safety_status(self, status):
        if status == self.last_safety_status:
            return

        if status is None and self.last_safety_status is not None:
            self.get_logger().info('Front safety clear')
        elif status is not None:
            self.get_logger().warn(status)
        self.last_safety_status = status

    def maybe_log_curvature_clamp(self, linear_velocity, angular_velocity, requested_radius):
        if not self.warn_on_curvature_clamp:
            return

        should_log = True
        if self.curvature_clamp_log_period > 0.0:
            now = self.get_clock().now()
            if self.last_curvature_clamp_log_time is not None:
                elapsed = now - self.last_curvature_clamp_log_time
                should_log = elapsed >= Duration(seconds=self.curvature_clamp_log_period)
            if should_log:
                self.last_curvature_clamp_log_time = now

        if not should_log:
            return

        achievable_wz = 0.0
        if abs(linear_velocity) > 1e-6:
            achievable_wz = abs(linear_velocity) / self.min_turning_radius

        radius_text = "inf" if math.isinf(requested_radius) else f"{requested_radius:.2f}"
        self.get_logger().warn(
            "Requested curvature exceeds Gen0 steering geometry: "
            f"v={linear_velocity:.2f}m/s wz={angular_velocity:.2f}rad/s "
            f"radius={radius_text}m min_radius={self.min_turning_radius:.2f}m "
            f"max_wz_at_v={achievable_wz:.2f}rad/s; steering was clamped"
        )

    def publish_latest_values(self):
        if self.command_timed_out():
            self.stop_vehicle()

        self.publish_joint_values()

    def publish_joint_values(self):
        # Publish all values continuously
        self.publisher_steering_front_left.publish(self.front_left_steering_msg)
        self.publisher_steering_front_right.publish(self.front_right_steering_msg)
        self.publisher_steering_back_left.publish(self.back_left_steering_msg)
        self.publisher_steering_back_right.publish(self.back_right_steering_msg)
        self.publisher_speed_back_left.publish(self.speed_back_left_msg)
        self.publisher_speed_back_right.publish(self.speed_back_right_msg)

    def command_timed_out(self):
        if self.cmd_vel_timeout <= 0.0 or self.last_cmd_time is None:
            return False

        elapsed = self.get_clock().now() - self.last_cmd_time
        return elapsed > Duration(seconds=self.cmd_vel_timeout)

    def stop_vehicle(self, log=True):
        self.delta_f = 0.0
        self.delta_r = 0.0
        self.front_left_steering_msg.data = 0.0
        self.front_right_steering_msg.data = 0.0
        self.back_left_steering_msg.data = 0.0
        self.back_right_steering_msg.data = 0.0
        self.speed_back_left_msg.data = 0.0
        self.speed_back_right_msg.data = 0.0
        self.last_steering_update_time = None
        if log and not self.timed_out:
            self.get_logger().warn(
                f'No Twist received for {self.cmd_vel_timeout:.2f}s; stopping vehicle'
            )
            self.timed_out = True

    def publish_zero_now(self, count=5):
        self.stop_vehicle(log=False)
        for _ in range(count):
            self.publish_joint_values()
            time.sleep(0.02)

def main(args=None):
    rclpy.init(args=args)
    node = VehicleMovementInterface()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.publish_zero_now()
        except Exception:
            pass
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except (Exception, KeyboardInterrupt):
            pass

if __name__ == '__main__':
    main()
