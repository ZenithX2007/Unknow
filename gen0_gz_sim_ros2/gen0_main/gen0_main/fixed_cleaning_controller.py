#!/usr/bin/env python3
"""Execute the validated one-round-trip cleaning trajectory in Gazebo world."""
import json
import math
import os
from pathlib import Path
import signal
import time
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import numpy as np
import rclpy
from geometry_msgs.msg import PoseArray, PoseStamped, Twist
from nav_msgs.msg import Path as PathMessage
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

from .cleaning_trajectory import Tracker, direction, generate_plan
from .gazebo_pose_selection import select_world_pose


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y*q.y + q.z*q.z))


class FixedCleaningController(Node):
    def __init__(self):
        super().__init__('fixed_cleaning_controller')
        defaults = {
            'pose_topic': '/gen0_model/links/poses', 'pose_index': -1,
            'cmd_vel_topic': '/cmd_vel', 'speed': .45, 'front_center_tolerance': .10,
            'scenario': os.environ.get('GEN0_TRASH_SCENARIO', 'small_trash_dense'), 'route_config': '',
            'require_cleanup_confirmation': os.environ.get('GEN0_TRASH_CLEANUP', 'false').lower() == 'true',
            'pose_timeout': 2.0,
            'actor_pose_topics': '',
            'actor_scenario': os.environ.get('GEN0_ACTORS_SCENARIO', 'walking_actors3'),
            'trajectory_frame': 'gen0_world',
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.pub = self.create_publisher(Twist, self.get_parameter('cmd_vel_topic').value, 10)
        retained = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(String, '/gen0_cleaning/status', retained)
        self.path_pub = self.create_publisher(PathMessage, '/gen0_cleaning/reference_path', retained)
        self.pose = None
        self.selected_pose_index = None
        self.pose_received = 0.
        self.pose_stamp = None
        self.last_stamp = None
        self.started = False
        self.cleanup = None
        self.cleanup_received = 0.
        self.actors = {}
        share = Path(get_package_share_directory('gen0_main'))
        self.actor_topics = [s.strip() for s in self.get_parameter('actor_pose_topics').value.split(',') if s.strip()]
        actor_scenario = self.get_parameter('actor_scenario').value
        if actor_scenario and not self.actor_topics:
            scenario = ET.parse(share / f'worlds/scenarios/my_map/{actor_scenario}.sdf')
            self.actor_topics = [f"/actor/{actor.get('name')}/pose" for actor in scenario.findall('.//actor')]
        self.actor_subscriptions = [self.create_subscription(PoseStamped, topic,
                                    lambda msg, key=topic: self.actor_cb(key, msg), 10)
                                    for topic in self.actor_topics]
        self.create_subscription(PoseArray, self.get_parameter('pose_topic').value, self.pose_cb, 10)
        self.create_subscription(String, '/gen0_cleaning/cleanup_status', self.cleanup_cb, retained)
        self.last_status = None
        self.get_logger().info('Generating and checking the complete Ackermann cleaning route before enabling motion')
        self.plan = generate_plan(share, self.get_parameter('scenario').value,
                                  self.get_parameter('route_config').value or None)
        requested_speed = float(self.get_parameter('speed').value)
        # The unchanged launcher passes its historical 1.0 m/s default.
        # Enforce the validated tracking speed inside the backend.
        tracking_speed = min(requested_speed, .45)
        if requested_speed > tracking_speed:
            self.get_logger().info(f'Limiting requested speed {requested_speed:.2f} to {tracking_speed:.2f} m/s')
        self.tracker = Tracker(self.plan, tracking_speed,
                               float(self.get_parameter('front_center_tolerance').value))
        self.publish_path()
        self.get_logger().info(f'Validated {len(self.plan.targets)} targets; conservative sampled static clearance '
                               f'{self.plan.minimum_clearance:.3f} m; waiting for start pose')
        self.timer = self.create_timer(.02, self.control)

    def publish_path(self):
        path = PathMessage()
        path.header.frame_id = self.get_parameter('trajectory_frame').value
        for state in self.plan.states[::5]:
            point = PoseStamped()
            point.header.frame_id = path.header.frame_id
            base = state[:2] + self.plan.geometry.rear_offset * direction(state[2])
            point.pose.position.x, point.pose.position.y = map(float, base)
            point.pose.position.z = 2.85
            point.pose.orientation.z = math.sin(state[2] / 2)
            point.pose.orientation.w = math.cos(state[2] / 2)
            path.poses.append(point)
        self.path_pub.publish(path)

    def pose_cb(self, msg):
        index = int(self.get_parameter('pose_index').value)
        if index < 0:
            if self.selected_pose_index is None:
                state = self.plan.states[0]
                start = np.r_[state[:2] + self.plan.geometry.rear_offset * direction(state[2]), state[2]]
                self.selected_pose_index = select_world_pose(msg.poses, start)
                if self.selected_pose_index is not None:
                    self.get_logger().info(f'Resolved Gazebo world vehicle pose at array index {self.selected_pose_index}')
            index = self.selected_pose_index
        if index is None:
            return
        if index < 0 or index >= len(msg.poses):
            return
        self.pose = msg.poses[index]
        self.pose_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.pose_received = time.monotonic()
        if hasattr(self, 'tracker'):
            self.control()  # process simulation updates even when simulation runs faster than wall time

    def actor_cb(self, topic, msg):
        self.actors[topic] = (np.array([msg.pose.position.x, msg.pose.position.y]), time.monotonic())

    def cleanup_cb(self, msg):
        try:
            data = json.loads(msg.data)
            if data.get('scenario') != self.get_parameter('scenario').value:
                return
            if set(data.get('targets', [])) != set(self.plan.target_names):
                return
            self.cleanup = data
            self.cleanup_received = time.monotonic()
        except (ValueError, TypeError, AttributeError):
            self.get_logger().warning('Ignoring malformed cleanup confirmation')

    def report(self, state, reason=''):
        data = {'state': state, 'reason': reason, 'covered': len(self.tracker.covered),
                'total': len(self.plan.targets), 'progress_m': round(self.tracker.progress, 2),
                'remaining': [name for i, name in enumerate(self.plan.target_names) if i not in self.tracker.covered],
                'cleanup_remaining': None if self.cleanup is None else self.cleanup['remaining']}
        self.status_pub.publish(String(data=json.dumps(data)))
        key = (state, reason, len(self.tracker.covered))
        if key != self.last_status:
            self.get_logger().info(json.dumps(data))
            self.last_status = key

    def stop(self, state, reason):
        self.pub.publish(Twist())
        self.tracker.last_speed = 0.
        self.report(state, reason)

    def actor_block(self, base):
        heading = direction(base[2])
        for topic in self.actor_topics:
            if topic not in self.actors or time.monotonic() - self.actors[topic][1] > 3.:
                return f'Waiting for fresh pedestrian pose: {topic}'
            delta = self.actors[topic][0] - base[:2]
            x = float(np.dot(delta, heading))
            y = float(heading[0] * delta[1] - heading[1] * delta[0])
            braking = self.tracker.last_speed ** 2 / (.6)
            if -.5 < x < self.plan.geometry.length / 2 + .85 + braking and abs(y) < self.plan.geometry.width / 2 + .75:
                return f'Pedestrian occupies the stopping corridor: {topic}'
        return ''

    def control(self):
        now = time.monotonic()
        if self.count_publishers(self.get_parameter('cmd_vel_topic').value) > 1:
            self.tracker.fail('Another controller is publishing to the vehicle command topic')
            self.stop('failed', self.tracker.reason)
            return
        if self.pose is None or now - self.pose_received > float(self.get_parameter('pose_timeout').value):
            self.stop('waiting_pose', 'No fresh Gazebo vehicle pose; command is zero')
            return
        if self.pose_stamp == self.last_stamp:
            return  # paused simulation must not advance the reference
        dt = .02 if self.last_stamp is None else self.pose_stamp - self.last_stamp
        self.last_stamp = self.pose_stamp
        if not 0 < dt <= .25:
            # Gazebo can pause/reset its clock while cleanup services remove a
            # model.  Do not invalidate the route on that single discontinuity:
            # discard this sample, clear the spatial finite-difference
            # reference, and establish a fresh time base on the next pose.
            self.tracker.previous_front = None
            self.tracker.last_speed = 0.
            self.get_logger().warning(
                f'Simulation time discontinuity (dt={dt:.3f}s); '
                'pausing one control sample and resynchronizing')
            self.stop('waiting_time_sync', '仿真时间跳变，正在重新同步')
            return
        require = bool(self.get_parameter('require_cleanup_confirmation').value)
        if require and (self.cleanup is None or now - self.cleanup_received > 3.):
            self.stop('waiting_cleanup', 'Waiting for matching trash cleanup node confirmation')
            return
        base = np.array([self.pose.position.x, self.pose.position.y, yaw(self.pose.orientation)])
        blocked = self.actor_block(base)
        if blocked:
            self.stop('waiting_obstacle', blocked)
            return
        velocity, omega = self.tracker.step(base, dt)
        if self.tracker.state == 'completed':
            if require and self.cleanup['remaining']:
                self.stop('incomplete_cleanup', 'Route covered, but Gazebo has not confirmed removal of every target')
            else:
                self.stop('completed' if require else 'coverage_completed',
                          'All targets covered and removal confirmed' if require else 'Coverage only; removal confirmation disabled')
            return
        if self.tracker.state == 'failed':
            self.stop('failed', self.tracker.reason)
            return
        command = Twist()
        command.linear.x, command.angular.z = float(velocity), float(omega)
        self.pub.publish(command)
        self.report('tracking')


def main(args=None):
    rclpy.init(args=args)
    node = None
    # SIGTERM from the supervising shell follows the same zero-command cleanup.
    def terminate(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, terminate)
    signal.signal(signal.SIGINT, terminate)
    try:
        node = FixedCleaningController()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            if rclpy.ok():
                node.pub.publish(Twist())
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
