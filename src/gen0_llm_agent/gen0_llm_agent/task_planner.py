import json
import math
import os
import re
from urllib import request


ALLOWED_PARAMETERS = {
    'navigate': {'x': (int, float), 'y': (int, float)},
    'forward': {'distance_m': (int, float)},
    'rotate': {'direction': str, 'degrees': (int, float)},
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
                    'skill': {'enum': ['navigate', 'forward', 'rotate', 'search_object', 'stop']},
                    'parameters': {'type': 'object'},
                },
                'oneOf': [
                    {'properties': {'skill': {'const': 'navigate'}, 'parameters': {
                        'type': 'object', 'additionalProperties': False,
                        'required': ['x', 'y'],
                        'properties': {'x': {'type': 'number'}, 'y': {'type': 'number'}},
                    }}},
                    {'properties': {'skill': {'const': 'forward'}, 'parameters': {
                        'type': 'object', 'additionalProperties': False,
                        'required': ['distance_m'],
                        'properties': {'distance_m': {'type': 'number', 'minimum': 0.1, 'maximum': 20.0}},
                    }}},
                    {'properties': {'skill': {'const': 'rotate'}, 'parameters': {
                        'type': 'object', 'additionalProperties': False,
                        'required': ['direction', 'degrees'],
                        'properties': {
                            'direction': {'enum': ['left', 'right']},
                            'degrees': {'type': 'number', 'minimum': 1.0, 'maximum': 180.0},
                        },
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


MOTION_NUMBER = r'(?:\d+(?:\.\d+)?|[零一二三四五六七八九十百两]+)'
FORWARD_COMMAND = re.compile(
    rf'(?:前进|向前|forward)\s*({MOTION_NUMBER})\s*(?:m|米)?', re.IGNORECASE)
ROTATE_COMMAND = re.compile(
    rf'(左|右)\s*(?:旋转|转)\s*({MOTION_NUMBER})\s*(?:度|°)?')
CHINESE_DIGITS = {
    '零': 0, '一': 1, '二': 2, '两': 2, '三': 3, '四': 4,
    '五': 5, '六': 6, '七': 7, '八': 8, '九': 9,
}


def parse_motion_number(value):
    try:
        return float(value)
    except ValueError:
        pass
    total = 0
    current = 0
    for character in value:
        if character in CHINESE_DIGITS:
            current = CHINESE_DIGITS[character]
        elif character == '十':
            total += (current or 1) * 10
            current = 0
        elif character == '百':
            total += (current or 1) * 100
            current = 0
        else:
            raise PlanValidationError(f'unsupported motion number: {value!r}')
    return float(total + current)


def parse_direct_motion(command):
    """Parse explicit relative motion before an LLM can reinterpret it as a map goal."""
    matches = []
    for match in FORWARD_COMMAND.finditer(command):
        matches.append((match.start(), 'forward', {'distance_m': parse_motion_number(match.group(1))}))
    for match in ROTATE_COMMAND.finditer(command):
        matches.append((match.start(), 'rotate', {
            'direction': 'left' if match.group(1) == '左' else 'right',
            'degrees': parse_motion_number(match.group(2)),
        }))
    if not matches:
        return None
    tasks = [
        {'id': f'task_{index}', 'skill': skill, 'parameters': parameters}
        for index, (_, skill, parameters) in enumerate(sorted(matches), start=1)
    ]
    if any(word in command.lower() for word in ('停车', '停止', 'stop')):
        tasks.append({'id': f'task_{len(tasks) + 1}', 'skill': 'stop', 'parameters': {}})
    return validate_plan({'goal': command.strip(), 'tasks': tasks})


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
            if skill in ('navigate', 'forward', 'rotate') and key != 'direction' and not math.isfinite(float(value)):
                raise PlanValidationError(f'{skill}.{key} must be finite')
        if skill == 'forward' and not 0.1 <= float(params['distance_m']) <= 20.0:
            raise PlanValidationError('forward.distance_m must be within [0.1, 20.0]')
        if skill == 'rotate':
            if params['direction'] not in ('left', 'right'):
                raise PlanValidationError('rotate.direction must be left or right')
            if not 1.0 <= float(params['degrees']) <= 180.0:
                raise PlanValidationError('rotate.degrees must be within [1.0, 180.0]')
        if skill == 'search_object' and not params['object'].strip():
            raise PlanValidationError('search_object.object cannot be empty')
    return plan


class MockPlanner:
    COORDINATES = re.compile(r'\(?\s*(-?\d+(?:\.\d+)?)\s*[,，]\s*(-?\d+(?:\.\d+)?)\s*\)?')

    def plan(self, command):
        direct_motion = parse_direct_motion(command)
        if direct_motion is not None:
            return direct_motion
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
        direct_motion = parse_direct_motion(command)
        if direct_motion is not None:
            return direct_motion
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


class OllamaPlanner:
    """Use a local Ollama server without sending tasks to an external API."""
    def __init__(self, base_url, model, prompt):
        self.base_url, self.model, self.prompt = base_url, model, prompt

    def plan(self, command):
        direct_motion = parse_direct_motion(command)
        if direct_motion is not None:
            return direct_motion
        body = json.dumps({
            'model': self.model,
            'messages': [{'role': 'system', 'content': self.prompt},
                         {'role': 'user', 'content': command}],
            'stream': False,
            'format': 'json',
            'options': {'temperature': 0},
        }).encode()
        req = request.Request(self.base_url.rstrip('/') + '/api/chat', body,
                              {'Content-Type': 'application/json'})
        try:
            with request.urlopen(req, timeout=60) as response:
                payload = json.load(response)
            plan = json.loads(payload['message']['content'])
            # Small local models occasionally emit coordinates in the
            # descriptive goal field. Goal is never executed; retain the
            # original request while the complete skill plan stays fail-closed
            # under validate_plan below.
            if isinstance(plan, dict) and not isinstance(plan.get('goal'), str):
                plan = {**plan, 'goal': command.strip()}
            return validate_plan(plan)
        except (KeyError, OSError, ValueError, json.JSONDecodeError) as exc:
            raise RuntimeError(f'Ollama request failed: {exc}') from exc


def create_planner(provider, prompt):
    if provider == 'mock':
        return MockPlanner()
    if provider == 'ollama':
        model = os.environ.get('GEN0_OLLAMA_MODEL', '')
        if not model:
            raise RuntimeError('GEN0_OLLAMA_MODEL is required for provider=ollama')
        return OllamaPlanner(
            os.environ.get('GEN0_OLLAMA_BASE_URL', 'http://127.0.0.1:11434'),
            model, prompt)
    if provider != 'openai_compatible':
        raise RuntimeError(f'unsupported LLM provider: {provider}')
    key = os.environ.get('GEN0_LLM_API_KEY', '')
    if not key:
        raise RuntimeError('GEN0_LLM_API_KEY is required for a non-mock provider')
    return OpenAICompatiblePlanner(key, os.environ.get('GEN0_LLM_BASE_URL', 'https://api.openai.com/v1'),
                                   os.environ.get('GEN0_LLM_MODEL', 'gpt-4o-mini'), prompt)
