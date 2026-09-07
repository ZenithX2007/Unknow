import json
import time
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from vision_msgs.msg import Detection3DArray

from .safety_guard import SafetyGuard, SafetyViolation
from .skill_registry import SkillRegistry, SkillStatus
from .skills.navigate import NavigateSkill
from .skills.search_object import SearchObjectSkill
from .skills.stop import StopSkill
from .task_executor import TaskExecutor
from .task_planner import PlanValidationError, create_planner
from .world_model import WorldModel


class Nav2Adapter:
    def __init__(self, node, action_name, frame_id, control_mode_topic):
        self.node, self.frame_id = node, frame_id
        self.client = ActionClient(node, NavigateToPose, action_name)
        self.control_mode_pub = node.create_publisher(String, control_mode_topic, 10)
        self.goal_future = self.goal_handle = self.result_future = None

    def start(self, x, y):
        if not self.client.wait_for_server(timeout_sec=1.0):
            return False
        self.control_mode_pub.publish(String(data='auto'))
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.frame_id
        goal.pose.header.stamp = self.node.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = x, y
        goal.pose.pose.orientation.w = 1.0
        self.goal_future = self.client.send_goal_async(goal)
        self.goal_handle = self.result_future = None
        return True

    def poll(self):
        if self.goal_handle is None:
            if self.goal_future is None or not self.goal_future.done():
                return SkillStatus.RUNNING
            self.goal_handle = self.goal_future.result()
            if self.goal_handle is None or not self.goal_handle.accepted:
                return SkillStatus.FAILURE
            self.result_future = self.goal_handle.get_result_async()
        if not self.result_future.done():
            return SkillStatus.RUNNING
        status = self.result_future.result().status
        return SkillStatus.SUCCESS if status == GoalStatus.STATUS_SUCCEEDED else SkillStatus.FAILURE

    def cancel(self):
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()


class StopAdapter:
    """Request the existing velocity mux's highest-priority stop mode."""
    def __init__(self, node, mode_topic, repeats, latch_seconds):
        self.publisher = node.create_publisher(String, mode_topic, 10)
        self.repeats = repeats
        self.latch_seconds = latch_seconds
        self.stop_until = 0.0
        self.stop_mode = String(data='stop')
        node.create_timer(0.05, self._publish_while_latched)

    def stop(self):
        self.stop_until = max(self.stop_until, time.monotonic() + self.latch_seconds)
        for _ in range(self.repeats):
            self.publisher.publish(self.stop_mode)

    def _publish_while_latched(self):
        if time.monotonic() < self.stop_until:
            self.publisher.publish(self.stop_mode)


class LlmAgentNode(Node):
    def __init__(self):
        super().__init__('gen0_llm_agent')
        self.declare_parameter('provider', 'mock')
        self.declare_parameter('command_topic', '/llm_agent/command')
        self.declare_parameter('status_topic', '/llm_agent/status')
        self.declare_parameter('detections_topic', '/yolo/detections')
        self.declare_parameter('actor_detections_topic', '/actors/detections')
        self.declare_parameter('navigate_action', '/navigate_to_pose')
        self.declare_parameter('stop_mode_topic', '/epsilon/control_mode')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('stop_repeats', 5)
        self.declare_parameter('stop_latch_seconds', 1.0)
        self.declare_parameter('tick_period', 0.1)

        self.world = WorldModel()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        frame = self.get_parameter('map_frame').value
        nav_adapter = Nav2Adapter(
            self,
            self.get_parameter('navigate_action').value,
            frame,
            self.get_parameter('stop_mode_topic').value,
        )
        stop_adapter = StopAdapter(self, self.get_parameter('stop_mode_topic').value,
                                   int(self.get_parameter('stop_repeats').value),
                                   float(self.get_parameter('stop_latch_seconds').value))
        self.registry = SkillRegistry()
        self.registry.register('navigate', NavigateSkill(nav_adapter, self.world))
        self.registry.register('search_object', SearchObjectSkill(self.world))
        self.registry.register('stop', StopSkill(stop_adapter))
        self.task_executor = TaskExecutor(
            self.registry, self.world, SafetyGuard(self.registry, self.world)
        )

        prompt_path = Path(get_package_share_directory('gen0_llm_agent')) / 'prompts' / 'task_planner.txt'
        prompt = prompt_path.read_text(encoding='utf-8') if prompt_path.exists() else ''
        self.planner = create_planner(self.get_parameter('provider').value, prompt)
        self.status_pub = self.create_publisher(String, self.get_parameter('status_topic').value, 10)
        self.create_subscription(String, self.get_parameter('command_topic').value, self.command_callback, 10)
        self.create_subscription(String, self.get_parameter('detections_topic').value, self.detection_callback, 10)
        self.create_subscription(
            Detection3DArray,
            self.get_parameter('actor_detections_topic').value,
            self.actor_detection_callback,
            10,
        )
        self.create_service(Trigger, '/llm_agent/stop', self.stop_callback)
        self.create_timer(float(self.get_parameter('tick_period').value), self.tick)
        self.get_logger().info('Gen0 LLM agent ready; accepted skills: navigate, search_object, stop')

    def command_callback(self, msg):
        try:
            plan = self.planner.plan(msg.data)
            if plan['tasks'][0]['skill'] == 'stop':
                self.registry.get('navigate').cancel()
            self.task_executor.start(plan)
            self.publish_status('plan_accepted', plan=plan)
        except (PlanValidationError, SafetyViolation, RuntimeError, ValueError) as exc:
            self.publish_status('rejected', error=str(exc))

    def detection_callback(self, msg):
        try:
            data = json.loads(msg.data)
            for item in data.get('detections', []):
                if isinstance(item, dict) and isinstance(item.get('class'), str):
                    self.world.detect(item['class'])
        except (json.JSONDecodeError, AttributeError):
            self.get_logger().warning('Ignoring malformed /yolo/detections message')

    def actor_detection_callback(self, msg):
        """Expose simulated pedestrians to the same world-model safety event."""
        if any(str(detection.id).startswith('actor_') for detection in msg.detections):
            self.world.detect('person')

    def stop_callback(self, request, response):
        nav = self.registry.get('navigate')
        nav.cancel()
        self.registry.get('stop').start({})
        self.world.status = 'stopped_by_service'
        response.success, response.message = True, 'navigation canceled and stop issued'
        return response

    def tick(self):
        try:
            self.tf_buffer.lookup_transform(self.get_parameter('map_frame').value,
                                            self.get_parameter('base_frame').value, Time())
            self.world.localization_ready = True
        except TransformException:
            self.world.localization_ready = False
        if self.task_executor.plan is not None and self.world.status == 'running':
            try:
                result = self.task_executor.tick()
                self.publish_status(result.value)
            except (SafetyViolation, KeyError) as exc:
                self.world.status = 'failure'
                self.publish_status('failure', error=str(exc))

    def publish_status(self, event, **extra):
        payload = {'event': event, 'world': self.world.as_dict(), **extra}
        msg = String(); msg.data = json.dumps(payload, ensure_ascii=False)
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LlmAgentNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__': main()
