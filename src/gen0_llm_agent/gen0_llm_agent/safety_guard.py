class SafetyViolation(ValueError):
    pass


class SafetyGuard:
    def __init__(self, registry, world_model):
        self.registry = registry
        self.world_model = world_model

    def validate_dispatch(self, task):
        skill = task['skill']
        if skill not in self.registry.names:
            raise SafetyViolation(f'unregistered skill: {skill}')
        if skill == 'navigate' and not self.world_model.localization_ready:
            raise SafetyViolation('navigate rejected: localization is not ready')

    @staticmethod
    def priority(skill_name):
        return 100 if skill_name == 'stop' else 10
