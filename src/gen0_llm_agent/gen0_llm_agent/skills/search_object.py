from ..skill_registry import SkillResult, SkillStatus


class SearchObjectSkill:
    def __init__(self, world_model): self.world = world_model; self.target = None
    def start(self, parameters): self.target = parameters['object']; return self.poll()
    def poll(self):
        found = self.world.is_detected(self.target)
        return SkillResult(SkillStatus.SUCCESS if found else SkillStatus.RUNNING,
                           'detected' if found else 'searching')
