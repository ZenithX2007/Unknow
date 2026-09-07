import json
import unittest

from gen0_llm_agent.safety_guard import SafetyGuard, SafetyViolation
from gen0_llm_agent.skill_registry import SkillRegistry, SkillResult, SkillStatus
from gen0_llm_agent.skills.navigate import NavigateSkill
from gen0_llm_agent.skills.search_object import SearchObjectSkill
from gen0_llm_agent.skills.stop import StopSkill
from gen0_llm_agent.task_executor import TaskExecutor
from gen0_llm_agent.task_planner import MockPlanner, PlanValidationError, validate_plan
from gen0_llm_agent.world_model import WorldModel


class FakeNav:
    def __init__(self): self.started = None; self.cancelled = False; self.status = SkillStatus.RUNNING
    def start(self, x, y): self.started = (x, y); return True
    def poll(self): return self.status
    def cancel(self): self.cancelled = True


class FakeStop:
    def __init__(self): self.calls = 0
    def stop(self): self.calls += 1


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
        self.assertEqual(self.nav.started, (10.0, 5.0))
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


if __name__ == '__main__': unittest.main()
