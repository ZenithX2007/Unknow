from .skill_registry import SkillStatus


class TaskExecutor:
    def __init__(self, registry, world_model, safety_guard):
        self.registry, self.world, self.guard = registry, world_model, safety_guard
        self.plan = None
        self.index = 0
        self.active_skill = None
        self.stop_on_object = None

    def start(self, plan):
        self.plan, self.index, self.active_skill = plan, 0, None
        self.world.status = 'running'
        self.stop_on_object = self._navigation_interrupt_object(plan['tasks'])

    @staticmethod
    def _navigation_interrupt_object(tasks):
        for i, task in enumerate(tasks[:-1]):
            if task['skill'] == 'search_object' and tasks[i + 1]['skill'] == 'stop':
                return task['parameters']['object']
        return None

    def tick(self):
        if self.plan is None or self.index >= len(self.plan['tasks']):
            return SkillStatus.SUCCESS
        task = self.plan['tasks'][self.index]
        if (self.world.navigation_active and self.stop_on_object and
                self.world.is_detected(self.stop_on_object)):
            if self.active_skill is not None:
                self.active_skill.cancel()
            self.registry.get('stop').start({})
            self.world.navigation_active = False
            self.world.status = 'stopped_on_' + self.stop_on_object
            self.index = len(self.plan['tasks'])
            return SkillStatus.SUCCESS
        if self.active_skill is None:
            self.guard.validate_dispatch(task)
            self.active_skill = self.registry.get(task['skill'])
            self.world.current_task = task['id']
            result = self.active_skill.start(task['parameters'])
        else:
            result = self.active_skill.poll()
        if result.status == SkillStatus.RUNNING:
            return result.status
        if result.status == SkillStatus.FAILURE:
            self.world.status = 'failure'
            return result.status
        if task['skill'] in ('navigate', 'forward', 'rotate') and self.stop_on_object:
            # The following search_object + stop pair is an en-route monitor.
            # Reaching the destination without the event completes the plan.
            self.index = len(self.plan['tasks'])
            self.active_skill = None
            self.world.current_task = None
            self.world.status = 'success'
            return SkillStatus.SUCCESS
        self.index += 1
        self.active_skill = None
        if self.index >= len(self.plan['tasks']):
            self.world.current_task = None
            self.world.status = 'success'
            return SkillStatus.SUCCESS
        return SkillStatus.RUNNING
