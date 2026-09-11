from ..skill_registry import SkillResult, SkillStatus


class RotateSkill:
    """Execute a bounded Ackermann-compatible driving arc."""

    def __init__(self, motion_adapter, world_model):
        self.motion_adapter = motion_adapter
        self.world = world_model

    def start(self, parameters):
        degrees = float(parameters['degrees'])
        if not self.motion_adapter.start_turn(parameters['direction'], degrees):
            return SkillResult(SkillStatus.FAILURE, 'turn command rejected')
        self.world.navigation_active = True
        return SkillResult(SkillStatus.RUNNING, 'turn command accepted')

    def poll(self):
        status = self.motion_adapter.poll()
        if status != SkillStatus.RUNNING:
            self.world.navigation_active = False
        return SkillResult(status)

    def cancel(self):
        self.motion_adapter.stop()
        self.world.navigation_active = False
