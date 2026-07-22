#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

class VehicleMovementInterface(Node):
    def __init__(self):
        super().__init__('vehicle_movement_interface')

        self.wheel_base= 2.8
        self.wheel_track= 1.385
        self.wheel_radius= 0.33
        self.max_velocity_forward = 5.6
        self.min_velocity_forward = 0.3
        self.max_steering = 0.31 # rad
        self.min_steering = -0.31
        self.min_turning_radius = self.wheel_base / (2 * math.tan(self.max_steering))
        self.declare_parameter('command_timeout', 0.5)
        self.command_timeout = float(self.get_parameter('command_timeout').value)
        self.last_command_time = None
        self.delta_f = 0.0
        self.delta_r = 0.0
        self.back_right_joint_speed = 0.0
        self.back_left_joint_speed = 0.0

        self.subscription = self.create_subscription(
            Twist,
            '/control/cmd_vel',
            self.cmd_callback,
            10
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

        # Timer to publish latest values continuously at 50 Hz
        self.timer = self.create_timer(0.02, self.publish_latest_values)

    def cmd_callback(self, msg):
        self.last_command_time = self.get_clock().now()
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

    def stop_vehicle(self):
        self.front_left_steering_msg.data = 0.0
        self.front_right_steering_msg.data = 0.0
        self.back_left_steering_msg.data = 0.0
        self.back_right_steering_msg.data = 0.0
        self.speed_back_left_msg.data = 0.0
        self.speed_back_right_msg.data = 0.0

    def publish_latest_values(self):
        if self.last_command_time is None:
            self.stop_vehicle()
        else:
            age = (self.get_clock().now() - self.last_command_time).nanoseconds * 1e-9
            if age > self.command_timeout:
                self.stop_vehicle()

        # Publish all values continuously
        self.publisher_steering_front_left.publish(self.front_left_steering_msg)
        self.publisher_steering_front_right.publish(self.front_right_steering_msg)
        self.publisher_steering_back_left.publish(self.back_left_steering_msg)
        self.publisher_steering_back_right.publish(self.back_right_steering_msg)
        self.publisher_speed_back_left.publish(self.speed_back_left_msg)
        self.publisher_speed_back_right.publish(self.speed_back_right_msg)

def main(args=None):
    rclpy.init(args=args)
    node = VehicleMovementInterface()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
