#!/usr/bin/env python3
import os
import xml.etree.ElementTree as ET
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from vision_msgs.msg import Detection3D, Detection3DArray
from std_msgs.msg import Header


class ActorsLoader(Node):
    def __init__(self):
        super().__init__('actors_loader')
        self.declare_parameter('actors_scenario', " ")
        self.declare_parameter('world', " ")
        self.actors_scenario = self.get_parameter('actors_scenario').value
        self.world = self.get_parameter('world').value
        self.package_directory = get_package_share_directory('gen0_main')
        self.actor_topics = []  # List of all topics of actors for poses
        self.actors_subscriptions = []  # List of all subscriptions to actors
        self.actor_positions = {}  # Dictionary to store latest positions of actors

        # Changed publisher to use Detection3DArray instead of PoseArray
        self.publisher = self.create_publisher(Detection3DArray, '/actors/detections', 10)

        self.add_scenario()
        self.subscribe_to_actors()

    def add_scenario(self):
        # Load the world file
        world_file_path = self.package_directory + '/worlds/' + self.world + '/' + self.world + '.sdf'
        world_tree = ET.parse(world_file_path)
        world_root = world_tree.getroot()
        world_element = world_root.find("world")
        if world_element is None:
            print("Error: No <world> element found in the base world file.")
            return

        # Remove existing actors
        actors = world_element.findall('actor')
        for actor in actors:
            world_element.remove(actor)

        # Load the actors scenario file
        actors_scenario_path = self.package_directory + '/worlds/scenarios/' + self.world + '/' + self.actors_scenario + '.sdf'
        if os.path.exists(actors_scenario_path):
            actors_scenario_tree = ET.parse(actors_scenario_path)
            actors_scenario_root = actors_scenario_tree.getroot()
            
            # Assuming your actors are directly under the root in the actors_scenario.sdf
            for actor in actors_scenario_root.findall('actor'):
                actor_string = ET.tostring(actor, encoding='unicode')
                new_actor_element = ET.fromstring(actor_string)
                world_element.append(new_actor_element)
                self.actor_topics.append("/actor/" + actor.get('name') + "/pose")
                
        # Write the modified world file back
        world_tree.write(world_file_path)

    def subscribe_to_actors(self):
        self.get_logger().info("Available actor topics: " + str(self.actor_topics))
        for topic in self.actor_topics:
            subscription = self.create_subscription(
                PoseStamped,
                topic,
                lambda msg, actor_name=topic[1:]: self.actor_position_callback(msg, actor_name),
                10)
            self.actors_subscriptions.append(subscription)

    def actor_position_callback(self, msg, actor_name):
        # Set the frame_id if it's missing
        msg.header.frame_id = 'map' if not msg.header.frame_id else msg.header.frame_id

        # Store the position
        self.actor_positions[actor_name] = msg

        # Create Detection3DArray message
        detection_array_msg = Detection3DArray()
        detection_array_msg.header = Header()
        detection_array_msg.header.frame_id = 'map'
        detection_array_msg.header.stamp = self.get_clock().now().to_msg()

        # Add each actor as a Detection3D
        for actor_name, position in self.actor_positions.items():
            detection = Detection3D()
            detection.header = position.header
            detection.bbox.center.position = position.pose.position
            detection.bbox.center.orientation = position.pose.orientation

            detection_array_msg.detections.append(detection)

        # Publish the detection array
        self.publisher.publish(detection_array_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ActorsLoader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()