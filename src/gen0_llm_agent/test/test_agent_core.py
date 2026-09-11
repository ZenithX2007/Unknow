import json
import unittest
from unittest import mock

from gen0_llm_agent.safety_guard import SafetyGuard, SafetyViolation
from gen0_llm_agent.skill_registry import SkillRegistry, SkillResult, SkillStatus
from gen0_llm_agent.skills.navigate import NavigateSkill
from gen0_llm_agent.skills.forward import ForwardSkill
from gen0_llm_agent.skills.rotate import RotateSkill
from gen0_llm_agent.skills.search_object import SearchObjectSkill
from gen0_llm_agent.skills.stop import StopSkill
from gen0_llm_agent.task_executor import TaskExecutor
from gen0_llm_agent.task_planner import (MockPlanner, OllamaPlanner,
                                         PlanValidationError, create_planner,
                                         validate_plan)
from gen0_llm_agent.world_model import WorldModel


class FakeNav:
    def __init__(self): self.started = None; self.cancelled = False; self.status = SkillStatus.RUNNING
    def start(self, x, y, yaw=0.0): self.started = (x, y, yaw); return True
    def poll(self): return self.status
    def cancel(self): self.cancelled = True


class FakeStop:
    def __init__(self): self.calls = 0
    def stop(self): self.calls += 1


class FakeMotion:
    def __init__(self):
        self.forward = None
        self.turn = None
        self.stopped = False
        self.status = SkillStatus.RUNNING
    def start_forward(self, distance_m): self.forward = distance_m; return True
    def start_turn(self, direction, degrees): self.turn = (direction, degrees); return True
    def poll(self): return self.status
    def stop(self): self.stopped = True


def valid_plan():
    return {'goal': 'go and stop for person', 'tasks': [
        {'id': 'task_1', 'skill': 'navigate', 'parameters': {'x': 10.0, 'y': 5.0}},
        {'id': 'task_2', 'skill': 'search_object', 'parameters': {'object': 'person'}},
        {'id': 'task_3', 'skill': 'stop', 'parameters': {}},
    ]}


class PlanValidationTests(unittest.TestCase):
    def test_valid_schema(self): self.assertEqual(validate_plan(valid_plan())['tasks'][0]['skill'], 'navigate')
    def test_unknown_skill_rejected(self):
        plan = valid_plan(); plan['tasks'][0]['skill'] = 'publish_cmd_vel'
        with self.assertRaises(PlanValidationError): validate_plan(plan)
    def test_malformed_output_rejected(self):
        with self.assertRaises((json.JSONDecodeError, PlanValidationError)):
            validate_plan(json.loads('{"goal":'))
    def test_unknown_parameter_rejected(self):
        plan = valid_plan(); plan['tasks'][0]['parameters']['topic'] = '/cmd_vel'
        with self.assertRaises(PlanValidationError): validate_plan(plan)
    def test_non_finite_coordinate_rejected(self):
        plan = valid_plan(); plan['tasks'][0]['parameters']['x'] = float('nan')
        with self.assertRaises(PlanValidationError): validate_plan(plan)
    def test_mock_natural_language(self):
        plan = MockPlanner().plan('去坐标 (10, 5)，途中如果发现行人就停车。')
        self.assertEqual([t['skill'] for t in plan['tasks']], ['navigate', 'search_object', 'stop'])
    def test_mock_relative_motion_commands(self):
        plan = MockPlanner().plan('前进 5m，然后左旋转 90度')
        self.assertEqual([t['skill'] for t in plan['tasks']], ['forward', 'rotate'])
        self.assertEqual(plan['tasks'][0]['parameters'], {'distance_m': 5.0})
        self.assertEqual(plan['tasks'][1]['parameters'], {'direction': 'left', 'degrees': 90.0})
    def test_voice_style_chinese_distance_is_relative_motion(self):
        plan = MockPlanner().plan('前进五米')
        self.assertEqual(plan['tasks'], [
            {'id': 'task_1', 'skill': 'forward', 'parameters': {'distance_m': 5.0}},
        ])
    def test_relative_motion_limits_are_enforced(self):
        plan = {'goal': '前进太远', 'tasks': [
            {'id': 'task_1', 'skill': 'forward', 'parameters': {'distance_m': 20.1}},
        ]}
        with self.assertRaises(PlanValidationError): validate_plan(plan)
    def test_ollama_provider_uses_local_defaults(self):
        with mock.patch.dict('os.environ', {'GEN0_OLLAMA_MODEL': 'qwen2.5:3b'}, clear=True):
            planner = create_planner('ollama', 'test prompt')
        self.assertIsInstance(planner, OllamaPlanner)
        self.assertEqual(planner.base_url, 'http://127.0.0.1:11434')
    def test_ollama_normalizes_only_non_string_goal(self):
        response = {'message': {'content': json.dumps({
            'goal': {'x': 10, 'y': 5}, 'tasks': valid_plan()['tasks'],
        })}}
        planner = OllamaPlanner('http://example.invalid', 'test', 'prompt')
        fake_response = mock.MagicMock()
        fake_response.__enter__.return_value.read.return_value = json.dumps(response).encode()
        with mock.patch('gen0_llm_agent.task_planner.request.urlopen', return_value=fake_response):
            self.assertEqual(planner.plan('去坐标（10，5）')['goal'], '去坐标（10，5）')
    def test_ollama_cannot_reinterpret_direct_motion_as_absolute_navigation(self):
        planner = OllamaPlanner('http://example.invalid', 'test', 'prompt')
        with mock.patch('gen0_llm_agent.task_planner.request.urlopen') as request_mock:
            plan = planner.plan('前进5m')
        request_mock.assert_not_called()
        self.assertEqual(plan['tasks'][0], {
            'id': 'task_1', 'skill': 'forward', 'parameters': {'distance_m': 5.0},
        })


class ExecutorTests(unittest.TestCase):
    def setUp(self):
        self.world = WorldModel(localization_ready=True)
        self.nav, self.stop = FakeNav(), FakeStop()
        self.registry = SkillRegistry()
        self.registry.register('navigate', NavigateSkill(self.nav, self.world))
        self.registry.register('search_object', SearchObjectSkill(self.world))
        self.registry.register('stop', StopSkill(self.stop))
        self.executor = TaskExecutor(self.registry, self.world, SafetyGuard(self.registry, self.world))

    def test_navigate_task_dispatch(self):
        self.executor.start(valid_plan())
        self.assertEqual(self.executor.tick(), SkillStatus.RUNNING)
        self.assertEqual(self.nav.started, (10.0, 5.0, 0.0))
        self.assertTrue(self.world.navigation_active)

    def test_person_cancels_navigation_and_stops(self):
        self.executor.start(valid_plan()); self.executor.tick()
        self.world.detect('person')
        self.assertEqual(self.executor.tick(), SkillStatus.SUCCESS)
        self.assertTrue(self.nav.cancelled)
        self.assertEqual(self.stop.calls, 1)
        self.assertEqual(self.world.status, 'stopped_on_person')

    def test_navigate_rejected_without_localization(self):
        self.world.localization_ready = False
        self.executor.start(valid_plan())
        with self.assertRaises(SafetyViolation):
            self.executor.tick()

    def test_arrival_without_person_does_not_wait_forever(self):
        self.executor.start(valid_plan()); self.executor.tick()
        self.nav.status = SkillStatus.SUCCESS
        self.assertEqual(self.executor.tick(), SkillStatus.SUCCESS)
        self.assertEqual(self.stop.calls, 0)


class RelativeMotionSkillTests(unittest.TestCase):
    def setUp(self):
        self.world = WorldModel(localization_ready=True)
        self.motion = FakeMotion()

    def test_forward_uses_relative_distance(self):
        skill = ForwardSkill(self.motion, self.world)
        self.assertEqual(skill.start({'distance_m': 5.0}).status, SkillStatus.RUNNING)
        self.assertEqual(self.motion.forward, 5.0)
        skill.cancel()
        self.assertTrue(self.motion.stopped)

    def test_left_turn_uses_bounded_driving_arc(self):
        skill = RotateSkill(self.motion, self.world)
        self.assertEqual(skill.start({'direction': 'left', 'degrees': 90.0}).status, SkillStatus.RUNNING)
        self.assertEqual(self.motion.turn, ('left', 90.0))


if __name__ == '__main__': unittest.main()
