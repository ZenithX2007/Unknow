from ..skill_registry import SkillResult, SkillStatus


class ForwardSkill:
    """Execute a bounded forward distance relative to the current vehicle pose."""

    def __init__(self, motion_adapter, world_model):
        self.motion_adapter = motion_adapter
        self.world = world_model

    def start(self, parameters):
        distance = float(parameters['distance_m'])
        if not self.motion_adapter.start_forward(distance):
            return SkillResult(SkillStatus.FAILURE, 'forward command rejected')
        self.world.navigation_active = True
        return SkillResult(SkillStatus.RUNNING, 'forward goal accepted')

    def poll(self):
        status = self.motion_adapter.poll()
        if status != SkillStatus.RUNNING:
            self.world.navigation_active = False
        return SkillResult(status)

    def cancel(self):
        self.motion_adapter.stop()
        self.world.navigation_active = False
