#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Vector3, Quaternion, PoseArray
from nav_msgs.msg import Odometry
import tf2_ros
import time
import math

class GroundTruthPublisher(Node):
    def __init__(self):
        super().__init__('pose_publisher')
        self.subscription = self.create_subscription(PoseArray, '/gen0_model/links/poses', self.pose_callback, 10)
        self.publisher = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.location = Odometry()
        self.last_position = None
        self.last_time = None

    def pose_callback(self, msg):
        current_time = self.get_clock().now()
        current_position = msg.poses[14].position

        # Compute elapsed time
        if self.last_time is not None:
            elapsed_time = (current_time.nanoseconds - self.last_time.nanoseconds) * 1e-9  # Convert nanoseconds to seconds
            if elapsed_time > 0:
                # Calculate linear velocity
                dx = current_position.x - self.last_position.x if self.last_position else 0.0
                dy = current_position.y - self.last_position.y if self.last_position else 0.0

                # Calculate linear speed
                linear_speed = math.sqrt(dx**2 + dy**2) / elapsed_time
                self.location.twist.twist.linear.x = linear_speed

                # Calculate angular velocity (assuming heading is represented by quaternion)
                current_orientation = msg.poses[14].orientation
                if self.last_position is not None:
                    last_orientation = self.location.pose.pose.orientation
                    # Calculate angular velocity based on orientation change
                    angular_velocity = self.calculate_angular_velocity(last_orientation, current_orientation, elapsed_time)
                    self.location.twist.twist.angular.z = angular_velocity

        # Update location
        self.location.header.stamp = current_time.to_msg()  # Convert to ROS2 message format
        self.location.header.frame_id, self.location.child_frame_id = "map", "odom"
        self.location.pose.pose.position = current_position
        self.location.pose.pose.orientation = msg.poses[14].orientation

        # Update last position and time
        self.last_position = current_position
        self.last_time = current_time

        self.tf_broadcaster.sendTransform(self.pose_to_transform(self.location))
        self.publisher.publish(self.location)


    def pose_to_transform(self, location_msg):
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = "map"
        transform.child_frame_id = "odom"
        translation = location_msg.pose.pose.position
        rotation = location_msg.pose.pose.orientation
        transform.transform.translation = Vector3()
        transform.transform.translation.x = translation.x
        transform.transform.translation.y = translation.y
        transform.transform.translation.z = translation.z
        transform.transform.rotation = Quaternion()
        transform.transform.rotation.x = rotation.x
        transform.transform.rotation.y = rotation.y
        transform.transform.rotation.z = rotation.z 
        transform.transform.rotation.w = rotation.w
        return transform

    def calculate_angular_velocity(self, last_orientation, current_orientation, elapsed_time):
        # Convert quaternions to Euler angles (roll, pitch, yaw)
        last_yaw = self.quaternion_to_yaw(last_orientation)
        current_yaw = self.quaternion_to_yaw(current_orientation)

        # Calculate the change in yaw
        delta_yaw = current_yaw - last_yaw

        # Normalize the angle to the range [-pi, pi]
        delta_yaw = (delta_yaw + math.pi) % (2 * math.pi) - math.pi

        # Calculate angular velocity
        angular_velocity = delta_yaw / elapsed_time
        return angular_velocity

    def quaternion_to_yaw(self, quaternion):
        # Convert a quaternion to yaw (rotation around the z-axis)
        x, y, z, w = quaternion.x, quaternion.y, quaternion.z, quaternion.w
        return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

def main(args=None):
    rclpy.init(args=args)
    pose_publisher = GroundTruthPublisher()
    rclpy.spin(pose_publisher)
    pose_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
