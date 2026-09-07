from ..skill_registry import SkillResult, SkillStatus


class StopSkill:
    def __init__(self, stop_adapter): self.stop_adapter = stop_adapter
    def start(self, parameters): self.stop_adapter.stop(); return SkillResult(SkillStatus.SUCCESS, 'stopped')
    def poll(self): return SkillResult(SkillStatus.SUCCESS, 'stopped')
