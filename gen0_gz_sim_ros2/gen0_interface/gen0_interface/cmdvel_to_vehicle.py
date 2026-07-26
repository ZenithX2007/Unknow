#!/usr/bin/env python3
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

class VehicleMovementInterface(Node):
    def __init__(self):
        super().__init__('vehicle_movement_interface')

        self.declare_parameter('cmd_vel_topic', '/control/cmd_vel')
        self.declare_parameter('cmd_vel_timeout', 0.5)
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        self.cmd_vel_timeout = float(self.get_parameter('cmd_vel_timeout').value)

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
        self.get_logger().info(f'Listening for Twist commands on {cmd_vel_topic}')

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

        # Timer to publish latest values continuously at 50 Hz
        self.timer = self.create_timer(0.02, self.publish_latest_values)

    def cmd_callback(self, msg):
        linear_velocity = msg.linear.x
        angular_velocity = msg.angular.z

        # Steering angle calculations
        if angular_velocity != 0:
            if linear_velocity == 0:
                turning_radius = math.copysign(self.min_turning_radius, angular_velocity)
            else:
                turning_radius = linear_velocity / angular_velocity
                turning_radius = max(abs(turning_radius), self.min_turning_radius) * math.copysign(1, turning_radius)
        else:
            turning_radius = float('inf')
        
        self.delta_f = math.atan(self.wheel_base / (2 * turning_radius))
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
        self.front_left_steering_msg.data = 0.0
        self.front_right_steering_msg.data = 0.0
        self.back_left_steering_msg.data = 0.0
        self.back_right_steering_msg.data = 0.0
        self.speed_back_left_msg.data = 0.0
        self.speed_back_right_msg.data = 0.0
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
