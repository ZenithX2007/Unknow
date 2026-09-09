#!/usr/bin/env python3
"""Deterministic cleaning-route controller independent of Nav2."""
import math
import rclpy
from geometry_msgs.msg import PoseArray, PoseStamped, Twist
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
        self.declare_parameter('pedestrian_stop_distance', 2.8)
        self.declare_parameter('car009_rear_trigger', 10.0)
        self.declare_parameter('car009_lane_tolerance', 1.4)
        self.route = [
            (-17.50, -24.88), (-15.91, -25.13), (-14.44, -25.61),
            (-4.15, -31.78), (-2.07, -34.14), (6.14, -37.96),
            (9.82, -40.57), (19.47, -45.30),
        ]
        self.pose = None
        self.car009 = None
        self.pedestrians = {}
        self.index = 0
        self.finished = False
        self.yield_until = 0.0
        self.create_subscription(PoseArray, self.get_parameter('pose_topic').value, self.pose_cb, 10)
        self.create_subscription(PoseStamped, '/car/car_009/pose', self.car_cb, 10)
        for i in (4, 5, 8, 9):
            self.create_subscription(PoseStamped, f'/actor/pedestrian_{i}/pose',
                                     lambda m, i=i: self.ped_cb(i, m), 10)
        self.pub = self.create_publisher(Twist, self.get_parameter('cmd_vel_topic').value, 10)
        self.timer = self.create_timer(0.05, self.control)

    def pose_cb(self, msg):
        i = int(self.get_parameter('pose_index').value)
        if i < len(msg.poses): self.pose = msg.poses[i]

    def car_cb(self, msg): self.car009 = msg.pose
    def ped_cb(self, i, msg): self.pedestrians[i] = msg.pose

    def control(self):
        out = Twist()
        if self.pose is None:
            self.pub.publish(out); return
        x, y = self.pose.position.x, self.pose.position.y
        final_x, final_y = self.route[-1]
        final_distance = math.hypot(x - final_x, y - final_y)
        goal_tolerance = float(self.get_parameter('goal_tolerance').value)
        if self.finished or (self.index == len(self.route) - 1 and final_distance <= goal_tolerance):
            self.finished = True
            self.pub.publish(out)
            return
        heading = yaw(self.pose.orientation)
        while self.index < len(self.route)-1 and math.hypot(x-self.route[self.index][0], y-self.route[self.index][1]) < goal_tolerance:
            self.index += 1
        tx, ty = self.route[self.index]
        dx, dy = tx-x, ty-y
        distance = math.hypot(dx, dy)
        target_heading = math.atan2(dy, dx)
        error = math.atan2(math.sin(target_heading-heading), math.cos(target_heading-heading))
        # Stop for pedestrians in the forward corridor.
        for p in self.pedestrians.values():
            px, py = p.position.x-x, p.position.y-y
            forward = math.cos(heading)*px + math.sin(heading)*py
            lateral = -math.sin(heading)*px + math.cos(heading)*py
            if 0.0 < forward < float(self.get_parameter('pedestrian_stop_distance').value) and abs(lateral) < 1.2:
                self.pub.publish(out); return
        # Let car_009 pass: steer left briefly, then return to route heading.
        if self.car009 is not None:
            px, py = self.car009.position.x-x, self.car009.position.y-y
            forward = math.cos(heading)*px + math.sin(heading)*py
            lateral = -math.sin(heading)*px + math.cos(heading)*py
            if -float(self.get_parameter('car009_rear_trigger').value) < forward < -2.0 and abs(lateral) < float(self.get_parameter('car009_lane_tolerance').value):
                self.yield_until = self.get_clock().now().nanoseconds*1e-9 + 2.5
            if self.get_clock().now().nanoseconds*1e-9 < self.yield_until:
                out.linear.x = min(float(self.get_parameter('speed').value), 1.2)
                out.angular.z = 0.28
                self.pub.publish(out); return
        speed = min(float(self.get_parameter('speed').value), 1.2)
        if self.index == len(self.route) - 1:
            speed = min(speed, max(0.15, final_distance * 0.8))
        out.linear.x = speed
        out.angular.z = max(-0.45, min(0.45, 1.4*error))
        if distance <= goal_tolerance and self.index == len(self.route)-1:
            self.finished = True
            out = Twist()
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args); node = FixedCleaningController()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__': main()
