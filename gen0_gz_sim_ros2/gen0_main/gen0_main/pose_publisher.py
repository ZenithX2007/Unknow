#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Vector3, Quaternion, PoseArray, PoseStamped
from nav_msgs.msg import Odometry
import tf2_ros
from geometry_msgs.msg import Quaternion
import tf2_geometry_msgs

class GroundTruthPublisher(Node):
    def __init__(self):
        super().__init__('pose_publisher')
        self.subscription = self.create_subscription(PoseArray, '/gen0_model/links/poses', self.pose_callback, 10)
        self.publisher = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.location= Odometry()

    def pose_callback(self, msg):
        # Create an odom message of location relative to map frame
        self.location.header.stamp= self.get_clock().now().to_msg()
        self.location.header.frame_id, self.location.child_frame_id = "map", "odom"

        self.location.pose.pose.position= msg.poses[13].position
        self.location.pose.pose.orientation= msg.poses[13].orientation

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

def main(args=None):
    rclpy.init(args=args)
    pose_publisher = GroundTruthPublisher()
    rclpy.spin(pose_publisher)
    pose_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
