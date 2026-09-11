#!/usr/bin/env python3
"""Deterministic cleaning-route controller independent of Nav2."""
import math
import rclpy
from geometry_msgs.msg import PoseArray, Twist
from rclpy.node import Node


def yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y*q.y + q.z*q.z))


class FixedCleaningController(Node):
    def __init__(self):
        super().__init__('fixed_cleaning_controller')
        self.declare_parameter('pose_topic', '/gen0_model/links/poses')
        self.declare_parameter('pose_index', 15)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('speed', 1.0)
        self.declare_parameter('goal_tolerance', 0.7)
        self.declare_parameter('front_offset', 1.75)
        self.declare_parameter('front_center_tolerance', 0.1)
        self.route_start = (-20.6991, -22.4324)
        # All main-road trash except the explicitly excluded items. The final
        # point is a parking goal, not a trash target.
        self.route = [
            (-12.357, -27.965), (-10.762, -28.222), (-7.524, -29.184),
            (-7.212, -31.053), (-5.617, -31.310), (-0.473, -34.398),
            (0.726, -36.182), (4.549, -39.835), (9.847, -42.583),
            (16.95, -43.85), (18.3, -44.65), (19.47, -45.30),
        ]
        self.pose = None
        self.index = 0
        self.create_subscription(PoseArray, self.get_parameter('pose_topic').value, self.pose_cb, 10)
        self.pub = self.create_publisher(Twist, self.get_parameter('cmd_vel_topic').value, 10)
        self.timer = self.create_timer(0.05, self.control)

    def pose_cb(self, msg):
        i = int(self.get_parameter('pose_index').value)
        if i < len(msg.poses): self.pose = msg.poses[i]

    def control(self):
        out = Twist()
        if self.pose is None:
            self.pub.publish(out); return
        x, y = self.pose.position.x, self.pose.position.y
        final_x, final_y = self.route[-1]
        final_distance = math.hypot(x - final_x, y - final_y)
        goal_tolerance = float(self.get_parameter('goal_tolerance').value)
        if self.index == len(self.route) - 1 and final_distance <= goal_tolerance:
            self.pub.publish(out)
            return

        heading = yaw(self.pose.orientation)
        # For each trash target, advance only after the midpoint of the front
        # bumper physically reaches the trash center.
        while self.index < len(self.route) - 1:
            if not self.front_center_reached_target(x, y, self.index):
                break
            self.index += 1
        tx, ty = self.route[self.index]
        if self.index < len(self.route) - 1:
            dx_path, dy_path = tx - x, ty - y
            path_length = math.hypot(dx_path, dy_path)
            if path_length > 1e-6:
                path_heading = math.atan2(dy_path, dx_path)
            else:
                path_heading = heading
            front_offset = float(self.get_parameter('front_offset').value)
            tx -= front_offset * math.cos(path_heading)
            ty -= front_offset * math.sin(path_heading)
        dx, dy = tx-x, ty-y
        target_heading = math.atan2(dy, dx)
        error = math.atan2(math.sin(target_heading-heading), math.cos(target_heading-heading))
        speed = min(float(self.get_parameter('speed').value), 1.2)
        if self.index == len(self.route) - 1:
            speed = min(speed, max(0.15, final_distance * 0.8))
        out.linear.x = speed
        out.angular.z = max(-0.45, min(0.45, 1.4*error))
        self.pub.publish(out)

    def front_center_reached_target(self, x, y, target_index):
        """Return true once the front-bumper midpoint reaches the trash center."""
        tx, ty = self.route[target_index]
        front_offset = float(self.get_parameter('front_offset').value)
        heading = yaw(self.pose.orientation)
        fx = x + front_offset * math.cos(heading)
        fy = y + front_offset * math.sin(heading)
        tolerance = float(self.get_parameter('front_center_tolerance').value)
        return math.hypot(fx - tx, fy - ty) <= tolerance


def main(args=None):
    rclpy.init(args=args); node = FixedCleaningController()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__': main()
