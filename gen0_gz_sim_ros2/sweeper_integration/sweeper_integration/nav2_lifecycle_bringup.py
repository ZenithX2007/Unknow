#!/usr/bin/env python3

import sys
import time

from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.node import Node


class Nav2LifecycleBringup(Node):
    def __init__(self):
        super().__init__('nav2_lifecycle_bringup')

        self.declare_parameter('node_names', [
            'controller_server',
            'smoother_server',
            'planner_server',
            'behavior_server',
            'bt_navigator',
            'waypoint_follower',
            'velocity_smoother',
        ])
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('map_wait_timeout', 60.0)
        self.declare_parameter('service_wait_timeout', 90.0)
        self.declare_parameter('transition_timeout', 45.0)
        self.declare_parameter('retry_delay', 2.0)

        self.node_names = [
            self.normalize_node_name(name)
            for name in self.get_parameter('node_names').value
        ]
        self.map_topic = self.get_parameter('map_topic').value
        self.map_wait_timeout = float(
            self.get_parameter('map_wait_timeout').value)
        self.service_wait_timeout = float(
            self.get_parameter('service_wait_timeout').value)
        self.transition_timeout = float(
            self.get_parameter('transition_timeout').value)
        self.retry_delay = float(self.get_parameter('retry_delay').value)

        self.map_ready = False
        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self.map_callback,
            1,
        )

    @staticmethod
    def normalize_node_name(node_name):
        if node_name.startswith('/'):
            return node_name
        return f'/{node_name}'

    def map_callback(self, msg):
        if msg.info.width > 0 and msg.info.height > 0:
            self.map_ready = True

    def wait_for_map(self):
        if not self.map_topic:
            return True

        self.get_logger().info(f'Waiting for non-empty map on {self.map_topic}')
        deadline = time.monotonic() + self.map_wait_timeout
        while rclpy.ok() and time.monotonic() < deadline:
            if self.map_ready:
                self.get_logger().info(f'Map is available on {self.map_topic}')
                return True
            rclpy.spin_once(self, timeout_sec=0.2)

        self.get_logger().error(
            f'Timed out waiting for non-empty map on {self.map_topic}')
        return False

    def wait_for_client(self, client, service_name):
        deadline = time.monotonic() + self.service_wait_timeout
        while rclpy.ok() and time.monotonic() < deadline:
            if client.wait_for_service(timeout_sec=1.0):
                return True
            self.get_logger().info(f'Waiting for {service_name}')
        return False

    def call_service(self, client, request, service_name):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(
            self,
            future,
            timeout_sec=self.transition_timeout,
        )
        if not future.done():
            self.get_logger().error(f'Timed out calling {service_name}')
            return None
        if future.exception() is not None:
            self.get_logger().error(
                f'{service_name} failed: {future.exception()}')
            return None
        return future.result()

    def get_state(self, node_name):
        service_name = f'{node_name}/get_state'
        client = self.create_client(GetState, service_name)
        if not self.wait_for_client(client, service_name):
            self.get_logger().error(f'Service unavailable: {service_name}')
            return None

        response = self.call_service(client, GetState.Request(), service_name)
        if response is None:
            return None
        return response.current_state.id

    def change_state(self, node_name, transition_id, label):
        service_name = f'{node_name}/change_state'
        client = self.create_client(ChangeState, service_name)
        if not self.wait_for_client(client, service_name):
            self.get_logger().error(f'Service unavailable: {service_name}')
            return False

        request = ChangeState.Request()
        request.transition.id = transition_id
        response = self.call_service(client, request, service_name)
        if response is None or not response.success:
            self.get_logger().error(f'Failed to {label} {node_name}')
            return False

        self.get_logger().info(f'{node_name}: {label} successful')
        return True

    def configure_node(self, node_name):
        state = self.get_state(node_name)
        if state is None:
            return False
        if state in (State.PRIMARY_STATE_INACTIVE, State.PRIMARY_STATE_ACTIVE):
            return True
        if state != State.PRIMARY_STATE_UNCONFIGURED:
            self.get_logger().error(
                f'{node_name}: cannot configure from lifecycle state {state}')
            return False
        return self.change_state(
            node_name,
            Transition.TRANSITION_CONFIGURE,
            'configure',
        )

    def activate_node(self, node_name):
        state = self.get_state(node_name)
        if state is None:
            return False
        if state == State.PRIMARY_STATE_ACTIVE:
            return True
        if state == State.PRIMARY_STATE_UNCONFIGURED:
            if not self.configure_node(node_name):
                return False
            state = self.get_state(node_name)
        if state != State.PRIMARY_STATE_INACTIVE:
            self.get_logger().error(
                f'{node_name}: cannot activate from lifecycle state {state}')
            return False
        return self.change_state(
            node_name,
            Transition.TRANSITION_ACTIVATE,
            'activate',
        )

    def bringup(self):
        if not self.wait_for_map():
            return False

        self.get_logger().info('Configuring Nav2 lifecycle nodes')
        for node_name in self.node_names:
            if not self.configure_node(node_name):
                return False
            time.sleep(self.retry_delay)

        self.get_logger().info('Activating Nav2 lifecycle nodes')
        for node_name in self.node_names:
            if not self.activate_node(node_name):
                return False
            time.sleep(self.retry_delay)

        self.get_logger().info('Nav2 lifecycle nodes are active')
        return True


def main(args=None):
    rclpy.init(args=args)
    node = Nav2LifecycleBringup()
    exit_code = 0
    try:
        if not node.bringup():
            exit_code = 1
    except KeyboardInterrupt:
        exit_code = 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
