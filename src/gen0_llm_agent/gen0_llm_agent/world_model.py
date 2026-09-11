from dataclasses import dataclass, field
from typing import List, Optional
import time


@dataclass
class WorldModel:
    localization_ready: bool = False
    navigation_active: bool = False
    detected_objects: List[str] = field(default_factory=list)
    current_task: Optional[str] = None
    status: str = 'idle'
    detection_times: dict = field(default_factory=dict, repr=False)

    def detect(self, object_name: str) -> None:
        if object_name not in self.detected_objects:
            self.detected_objects.append(object_name)
        self.detection_times[object_name] = time.monotonic()

    def is_detected(self, object_name: str, max_age: float = 1.0) -> bool:
        seen = self.detection_times.get(object_name)
        return seen is not None and time.monotonic() - seen <= max_age

    def clear_detections(self) -> None:
        self.detected_objects.clear()
        self.detection_times.clear()

    def as_dict(self):
        return {
            'robot': {
                'localization_ready': self.localization_ready,
                'navigation_active': self.navigation_active,
            },
            'perception': {'detected_objects': list(self.detected_objects)},
            'task': {'current_task': self.current_task, 'status': self.status},
        }
