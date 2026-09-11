from dataclasses import dataclass
from enum import Enum
from typing import Any, Dict


class SkillStatus(str, Enum):
    RUNNING = 'RUNNING'
    SUCCESS = 'SUCCESS'
    FAILURE = 'FAILURE'


@dataclass
class SkillResult:
    status: SkillStatus
    message: str = ''


class SkillRegistry:
    def __init__(self):
        self._skills: Dict[str, Any] = {}

    def register(self, name: str, skill: Any) -> None:
        if name in self._skills:
            raise ValueError(f'duplicate skill: {name}')
        self._skills[name] = skill

    def get(self, name: str) -> Any:
        if name not in self._skills:
            raise KeyError(f'unregistered skill: {name}')
        return self._skills[name]

    @property
    def names(self):
        return frozenset(self._skills)
