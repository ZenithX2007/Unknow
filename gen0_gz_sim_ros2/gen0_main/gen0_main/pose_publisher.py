#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Vector3, Quaternion, PoseArray, PoseStamped, Twist
from nav_msgs.msg import Odometry
import tf2_ros
from geometry_msgs.msg import Quaternion
from tf_transformations import euler_from_quaternion, quaternion_from_euler
import math

class GroundTruthPublisher(Node):
    def __init__(self):
        super().__init__('pose_publisher')
        self.subscription = self.create_subscription(PoseArray, '/gen0_model/links/poses', self.pose_callback, 10)
        self.publisher = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # Odometry and timing
        self.location = Odometry()
        self.last_position = None
        self.last_orientation = None
        self.last_time = None

    def pose_callback(self, msg):
        # Retrieve current time
        current_time = self.get_clock().now()
        
        # Update location message
        self.location.header.stamp = current_time.to_msg()
        self.location.header.frame_id = "map"
        self.location.child_frame_id = "odom"
        self.location.pose.pose.position = msg.poses[15].position
        self.location.pose.pose.orientation = msg.poses[15].orientation

        # Calculate twist (linear x and angular z) if last pose is available
        if self.last_position and self.last_orientation and self.last_time:
            time_delta = (current_time - self.last_time).nanoseconds * 1e-9  # Convert to seconds
            
            if time_delta > 0:
                # Calculate linear x velocity
                dx = msg.poses[15].position.x - self.last_position.x
                dy = msg.poses[15].position.y - self.last_position.y
                linear_x = math.sqrt(dx ** 2 + dy ** 2) / time_delta

                # Calculate angular z velocity
                yaw = self.get_yaw_from_orientation(msg.poses[15].orientation)
                last_yaw = self.get_yaw_from_orientation(self.last_orientation)
                angular_z = (yaw - last_yaw) / time_delta

                # Update the twist in odometry message
                self.location.twist.twist.linear.x = linear_x
                self.location.twist.twist.angular.z = angular_z

        # Publish the odometry and broadcast the transform
        self.tf_broadcaster.sendTransform(self.pose_to_transform(self.location))
        self.publisher.publish(self.location)

        # Update last pose and time
        self.last_position = msg.poses[15].position
        self.last_orientation = msg.poses[15].orientation
        self.last_time = current_time

    def pose_to_transform(self, location_msg):
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = "map"
        transform.child_frame_id = "odom"
        
        translation = location_msg.pose.pose.position
        rotation = location_msg.pose.pose.orientation
        
        transform.transform.translation.x = translation.x
        transform.transform.translation.y = translation.y
        transform.transform.translation.z = translation.z
        transform.transform.rotation.x = rotation.x
        transform.transform.rotation.y = rotation.y
        transform.transform.rotation.z = rotation.z
        transform.transform.rotation.w = rotation.w

        return transform

    def get_yaw_from_orientation(self, orientation):
        """Converts a Quaternion into a yaw angle in radians."""
        _, _, yaw = euler_from_quaternion([
            orientation.x, 
            orientation.y, 
            orientation.z, 
            orientation.w
        ])
        return yaw

def main(args=None):
    rclpy.init(args=args)
    pose_publisher = GroundTruthPublisher()
    rclpy.spin(pose_publisher)
    pose_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
