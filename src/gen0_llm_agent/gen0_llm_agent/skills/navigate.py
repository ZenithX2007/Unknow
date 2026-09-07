from ..skill_registry import SkillResult, SkillStatus


class NavigateSkill:
    def __init__(self, adapter, world_model): self.adapter, self.world = adapter, world_model
    def start(self, parameters):
        if not self.adapter.start(float(parameters['x']), float(parameters['y'])):
            return SkillResult(SkillStatus.FAILURE, 'Nav2 goal rejected')
        self.world.navigation_active = True
        return SkillResult(SkillStatus.RUNNING, 'goal accepted')
    def poll(self):
        status = self.adapter.poll()
        if status != SkillStatus.RUNNING: self.world.navigation_active = False
        return SkillResult(status)
    def cancel(self): self.adapter.cancel(); self.world.navigation_active = False
