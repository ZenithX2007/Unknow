import json
import math
import os
import re
from urllib import request


ALLOWED_PARAMETERS = {
    'navigate': {'x': (int, float), 'y': (int, float)},
    'search_object': {'object': str},
    'stop': {},
}

TASK_PLAN_SCHEMA = {
    'type': 'object', 'additionalProperties': False,
    'required': ['goal', 'tasks'],
    'properties': {
        'goal': {'type': 'string', 'minLength': 1},
        'tasks': {
            'type': 'array', 'minItems': 1,
            'items': {
                'type': 'object', 'additionalProperties': False,
                'required': ['id', 'skill', 'parameters'],
                'properties': {
                    'id': {'type': 'string', 'pattern': '^task_[1-9][0-9]*$'},
                    'skill': {'enum': ['navigate', 'search_object', 'stop']},
                    'parameters': {'type': 'object'},
                },
                'oneOf': [
                    {'properties': {'skill': {'const': 'navigate'}, 'parameters': {
                        'type': 'object', 'additionalProperties': False,
                        'required': ['x', 'y'],
                        'properties': {'x': {'type': 'number'}, 'y': {'type': 'number'}},
                    }}},
                    {'properties': {'skill': {'const': 'search_object'}, 'parameters': {
                        'type': 'object', 'additionalProperties': False,
                        'required': ['object'],
                        'properties': {'object': {'type': 'string', 'minLength': 1}},
                    }}},
                    {'properties': {'skill': {'const': 'stop'}, 'parameters': {
                        'type': 'object', 'additionalProperties': False, 'maxProperties': 0,
                    }}},
                ],
            },
        },
    },
}


class PlanValidationError(ValueError):
    pass


def validate_plan(plan):
    # Production installs jsonschema. The explicit checks below remain as a
    # dependency-free fail-closed validator for minimal CI environments.
    try:
        import jsonschema
        jsonschema.validate(plan, TASK_PLAN_SCHEMA)
    except ImportError:
        pass
    except Exception as exc:
        raise PlanValidationError(f'JSON Schema validation failed: {exc}') from exc
    if not isinstance(plan, dict) or set(plan) != {'goal', 'tasks'}:
        raise PlanValidationError('plan must contain only goal and tasks')
    if not isinstance(plan['goal'], str) or not plan['goal'].strip():
        raise PlanValidationError('goal must be a non-empty string')
    if not isinstance(plan['tasks'], list) or not plan['tasks']:
        raise PlanValidationError('tasks must be a non-empty array')
    ids = set()
    for position, task in enumerate(plan['tasks'], start=1):
        if not isinstance(task, dict) or set(task) != {'id', 'skill', 'parameters'}:
            raise PlanValidationError('each task must contain only id, skill, parameters')
        task_id = task['id']
        if not isinstance(task_id, str) or not re.fullmatch(r'task_[1-9][0-9]*', task_id):
            raise PlanValidationError(f'invalid task id: {task_id!r}')
        if task_id in ids:
            raise PlanValidationError(f'duplicate task id: {task_id}')
        if task_id != f'task_{position}':
            raise PlanValidationError('task IDs must be sequential from task_1')
        ids.add(task_id)
        skill = task['skill']
        if skill not in ALLOWED_PARAMETERS:
            raise PlanValidationError(f'unknown skill: {skill!r}')
        params = task['parameters']
        expected = ALLOWED_PARAMETERS[skill]
        if not isinstance(params, dict) or set(params) != set(expected):
            raise PlanValidationError(f'invalid parameters for {skill}')
        for key, expected_type in expected.items():
            value = params[key]
            if isinstance(value, bool) or not isinstance(value, expected_type):
                raise PlanValidationError(f'{skill}.{key} has invalid type')
            if skill == 'navigate' and not math.isfinite(float(value)):
                raise PlanValidationError(f'{skill}.{key} must be finite')
        if skill == 'search_object' and not params['object'].strip():
            raise PlanValidationError('search_object.object cannot be empty')
    return plan


class MockPlanner:
    COORDINATES = re.compile(r'\(?\s*(-?\d+(?:\.\d+)?)\s*[,，]\s*(-?\d+(?:\.\d+)?)\s*\)?')

    def plan(self, command):
        tasks = []
        match = self.COORDINATES.search(command)
        if match and any(word in command.lower() for word in ('去', '前往', 'navigate', 'go')):
            tasks.append({'id': 'task_1', 'skill': 'navigate',
                          'parameters': {'x': float(match.group(1)), 'y': float(match.group(2))}})
        person_requested = '行人' in command or 'person' in command.lower()
        if person_requested:
            tasks.append({'id': f'task_{len(tasks)+1}', 'skill': 'search_object',
                          'parameters': {'object': 'person'}})
        if any(word in command.lower() for word in ('停车', '停止', 'stop')):
            tasks.append({'id': f'task_{len(tasks)+1}', 'skill': 'stop', 'parameters': {}})
        if not tasks:
            raise PlanValidationError('mock planner cannot map command to registered skills')
        return validate_plan({'goal': command.strip(), 'tasks': tasks})


class OpenAICompatiblePlanner:
    def __init__(self, api_key, base_url, model, prompt):
        self.api_key, self.base_url, self.model, self.prompt = api_key, base_url, model, prompt

    def plan(self, command):
        body = json.dumps({
            'model': self.model,
            'messages': [{'role': 'system', 'content': self.prompt},
                         {'role': 'user', 'content': command}],
            'response_format': {'type': 'json_object'}, 'temperature': 0,
        }).encode()
        req = request.Request(self.base_url.rstrip('/') + '/chat/completions', body,
                              {'Authorization': 'Bearer ' + self.api_key, 'Content-Type': 'application/json'})
        with request.urlopen(req, timeout=30) as response:
            payload = json.load(response)
        return validate_plan(json.loads(payload['choices'][0]['message']['content']))


def create_planner(provider, prompt):
    if provider == 'mock':
        return MockPlanner()
    key = os.environ.get('GEN0_LLM_API_KEY', '')
    if not key:
        raise RuntimeError('GEN0_LLM_API_KEY is required for a non-mock provider')
    return OpenAICompatiblePlanner(key, os.environ.get('GEN0_LLM_BASE_URL', 'https://api.openai.com/v1'),
                                   os.environ.get('GEN0_LLM_MODEL', 'gpt-4o-mini'), prompt)
