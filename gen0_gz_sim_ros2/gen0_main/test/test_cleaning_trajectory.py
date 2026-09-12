"""Task-level regressions: all targets, realistic actuator lag, and fault stops."""
import json
import math
from pathlib import Path

import numpy as np
import pytest

from gen0_main.cleaning_trajectory import StaticBoundary, Tracker, direction, generate_plan

SHARE = Path(__file__).resolve().parents[1]


@pytest.fixture(scope='module')
def plan():
    return generate_plan(SHARE)


def simulate(plan, lag=0., noisy=False):
    tracker = Tracker(plan)
    rear = plan.states[0].copy()
    speed, steering = 0., 0.
    rng = np.random.default_rng(9)
    trace = []
    minimum_errors = np.full(len(plan.targets), np.inf)
    for step in range(90000):
        dt = (.02, .03, .04)[step % 3]
        base = np.r_[rear[:2] + plan.geometry.rear_offset * direction(rear[2]), rear[2]]
        measured = base + (rng.normal(0, [.003, .003, .0015]) if noisy else 0)
        velocity, omega = tracker.step(measured, dt)
        assert abs(omega) <= plan.geometry.max_curvature * velocity + 1e-9
        # The plant has finite steering slew and first-order response; it is
        # not integrated with the controller's front-point feedback equations.
        desired_steering = math.atan(plan.geometry.wheelbase * omega / max(velocity, 1e-9))
        alpha = 1. if lag == 0 else dt / (dt + lag)
        speed += alpha * (velocity - speed)
        steering += np.clip(alpha * (desired_steering - steering), -.35 * dt, .35 * dt)
        curvature = math.tan(steering) / plan.geometry.wheelbase
        rear[:2] += speed * dt * direction(rear[2] + speed * curvature * dt / 2)
        rear[2] += speed * curvature * dt
        physical_front = rear[:2] + plan.geometry.arm * direction(rear[2])
        minimum_errors = np.minimum(minimum_errors, np.linalg.norm(plan.targets - physical_front, axis=1))
        if step % 5 == 0:
            trace.append(rear.copy())
        if tracker.state in ('failed', 'completed'):
            break
    return tracker, np.array(trace), minimum_errors


@pytest.mark.parametrize('lag,noisy', [(0., False), (.3, True)])
def test_entire_round_trip_completes_without_missing_trash(plan, lag, noisy):
    tracker, trace, errors = simulate(plan, lag, noisy)
    assert tracker.state == 'completed', (tracker.reason, tracker.progress, tracker.covered)
    assert len(tracker.covered) == 18
    assert errors.max() < .10  # independently measured physical front coverage
    assert StaticBoundary(SHARE / 'worlds/my_map/my_map.obj').clearance(trace, plan.geometry) > .15


def test_task_order_and_initial_state(plan):
    config = json.loads((SHARE / 'config/fixed_cleaning_route.json').read_text())
    assert plan.target_names == config['outbound'] + config['inbound']
    assert len(config['outbound']) == 13 and len(config['inbound']) == 5
    assert np.all(np.diff(plan.target_distances) > 0)
    assert plan.minimum_clearance >= config['minimum_clearance']
    rear_step = np.diff(plan.states[:, :2], axis=0)
    assert np.all(np.sum(rear_step * direction(plan.states[:-1, 2]), axis=1) > 0)


def test_wrong_start_and_time_reset_stop(plan):
    tracker = Tracker(plan)
    assert tracker.step([5.67, -36.51, -.54], .02) == (0., 0.)
    assert tracker.state == 'failed'
    tracker = Tracker(plan)
    assert tracker.step([-20.6991, -22.4324, -.5406], -.1) == (0., 0.)
    assert tracker.state == 'failed'


def test_reference_progress_cannot_confirm_trash(plan):
    tracker = Tracker(plan)
    tracker.progress = plan.target_distances[0] + .4
    i = np.searchsorted(plan.distance, tracker.progress)
    state = plan.states[i]
    base = np.r_[state[:2] + plan.geometry.rear_offset * direction(state[2]), state[2]]
    tracker.previous_front = base[:2] + plan.geometry.front_offset * direction(state[2])
    assert tracker.step(base, .02) == (0., 0.)
    assert tracker.state == 'failed'
    assert not tracker.covered


def test_unlisted_trash_rejected(tmp_path):
    config = json.loads((SHARE / 'config/fixed_cleaning_route.json').read_text())
    config['outbound'].pop()
    path = tmp_path / 'route.json'
    path.write_text(json.dumps(config))
    with pytest.raises(ValueError, match='every non-leaf'):
        generate_plan(SHARE, config_path=path)


def test_pose_selection_does_not_assume_array_index():
    from types import SimpleNamespace
    from gen0_main.gazebo_pose_selection import select_world_pose
    def pose(x, y, heading):
        return SimpleNamespace(position=SimpleNamespace(x=x, y=y),
                               orientation=SimpleNamespace(x=0., y=0., z=math.sin(heading / 2), w=math.cos(heading / 2)))
    poses = [pose(0, 0, 0) for _ in range(17)]
    poses[15] = pose(.85, 0, 0)  # a lidar link's local pose, observed in Gazebo
    poses[3] = pose(-20.6991, -22.4324, -.5406)
    assert select_world_pose(poses, (-20.6991, -22.4324, -.5406)) == 3
    assert select_world_pose(poses, (5.67, -36.51, -.54)) is None


def test_recorded_turn_stall_recovers_and_covers_return_targets(plan):
    # Real Gazebo sample: almost purely lateral 8 cm error; old tracker
    # commanded about 0.0035 m/s and stayed near progress=73.24 m.
    base = np.array([27.28151206964646, -55.09207893898632, 2.779989841828947])
    tracker = Tracker(plan)
    tracker.progress = 73.24
    tracker.covered = set(range(13))
    tracker.previous_front = base[:2] + plan.geometry.front_offset * direction(base[2])
    rear = np.r_[base[:2] - plan.geometry.rear_offset * direction(base[2]), base[2]]
    speed, steering = .003476, -math.atan(plan.geometry.wheelbase * plan.geometry.max_curvature)
    trace = []
    errors = np.full(5, np.inf)
    for step in range(40000):
        dt = .03
        base = np.r_[rear[:2] + plan.geometry.rear_offset * direction(rear[2]), rear[2]]
        velocity, omega = tracker.step(base, dt)
        requested = math.atan(plan.geometry.wheelbase * omega / max(velocity, 1e-9))
        speed += dt / (.3 + dt) * (velocity - speed)
        steering += np.clip(dt / (.3 + dt) * (requested - steering), -.35 * dt, .35 * dt)
        curvature = math.tan(steering) / plan.geometry.wheelbase
        rear[:2] += speed * dt * direction(rear[2] + speed * curvature * dt / 2)
        rear[2] += speed * curvature * dt
        front = rear[:2] + plan.geometry.arm * direction(rear[2])
        errors = np.minimum(errors, np.linalg.norm(plan.targets[13:] - front, axis=1))
        if step % 5 == 0:
            trace.append(rear.copy())
        if step == 333:
            assert tracker.progress > 74.0, 'Turn recovery must make measurable progress within 10 s'
        if tracker.state in ('failed', 'completed'):
            break
    assert tracker.state == 'completed', (tracker.reason, tracker.progress)
    assert len(tracker.covered) == 18
    assert errors.max() < .1
    assert StaticBoundary(SHARE / 'worlds/my_map/my_map.obj').clearance(np.array(trace), plan.geometry) > .15
