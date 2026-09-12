"""Offline Ackermann route construction and ROS-independent tracking.

Coordinates are Gazebo world coordinates. States refer to the REAR AXLE;
the tracked cleaning point is ahead of base_link, not the rear axle.
"""

from dataclasses import dataclass
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from scipy.integrate import solve_ivp
from scipy.interpolate import CubicSpline
from scipy.optimize import least_squares
from scipy.spatial import cKDTree


def direction(theta):
    return np.stack((np.cos(theta), np.sin(theta)), axis=-1)


@dataclass
class Geometry:
    wheelbase: float
    track: float
    steering_limit: float
    rear_offset: float
    front_offset: float
    length: float
    width: float

    @property
    def arm(self):
        return self.rear_offset + self.front_offset

    @property
    def max_curvature(self):
        return 1.0 / (self.wheelbase / math.tan(self.steering_limit) + self.track / 2)

    @classmethod
    def read(cls, sdf, front_offset):
        model = ET.parse(sdf).getroot().find('model')
        plugin = next(p for p in model.findall('plugin')
                      if 'ackermann-steering' in p.get('filename', ''))
        rear = model.find("link[@name='back_left_steering_link']/pose")
        limits = [float(model.find(f"joint[@name='{side}_steering_joint']/axis/limit/upper").text)
                  for side in ('front_left', 'front_right')]
        box = model.find("link[@name='base_link']/collision/geometry/box/size")
        length, width, _ = map(float, box.text.split())
        return cls(float(plugin.findtext('wheel_base')), float(plugin.findtext('kingpin_width')),
                   min(*limits, float(plugin.findtext('steering_limit'))),
                   -float(rear.text.split()[0]), front_offset, length, width)


def sweep(points, heading, geometry):
    """Interpolate the CLEANING point, then integrate the no-slip constraint."""
    points = np.asarray(points)
    distances = np.linalg.norm(np.diff(points, axis=0), axis=1)
    if np.any(distances < 0.05):
        raise ValueError('Cleaning targets must be distinct and at least 5 cm apart')
    knots = np.r_[0., np.cumsum(distances)]
    curve = CubicSpline(knots, points, bc_type=((1, direction(heading)), 'natural'))

    def rhs(t, theta):
        dx, dy = curve(t, 1)
        return (np.cos(theta) * dy - np.sin(theta) * dx) / geometry.arm

    solution = solve_ivp(rhs, (0, knots[-1]), [heading], rtol=1e-9,
                         atol=1e-10, max_step=0.03, dense_output=True)
    if not solution.success:
        raise ValueError('Failed to integrate cleaning trajectory')
    samples = np.unique(np.r_[np.linspace(0, knots[-1], int(knots[-1] / .02) + 2), knots])
    theta = solution.sol(samples)[0]
    velocity = curve(samples, 1)
    forward = np.sum(velocity * direction(theta), axis=1)
    curvature = (velocity[:, 1] * np.cos(theta) - velocity[:, 0] * np.sin(theta)) / geometry.arm / forward
    if np.min(forward) <= 0 or np.max(abs(curvature)) > geometry.max_curvature:
        raise ValueError('Trash sequence requires reverse motion or excessive steering')
    rear = curve(samples) - geometry.arm * direction(theta)
    return np.c_[rear, theta]


def arcs(start, curvatures, lengths, sample=True):
    x, y, theta = start
    result = [np.array(start)]
    for curvature, length in zip(curvatures, lengths):
        count = max(1, int(length / .02) + 1) if sample else 1
        for _ in range(count):
            ds = length / count
            if abs(curvature) < 1e-10:
                x += ds * np.cos(theta)
                y += ds * np.sin(theta)
            else:
                x += (np.sin(theta + curvature * ds) - np.sin(theta)) / curvature
                y += (np.cos(theta) - np.cos(theta + curvature * ds)) / curvature
            theta += curvature * ds
            result.append(np.array([x, y, theta]))
    return np.array(result)


class StaticBoundary:
    """Conservative 2-D wall samples from several world-mesh height sections.

    Not a full 3-D or dynamic collision checker. Sampling uncertainty is
    subtracted from the clearance returned to the planner.
    """

    def __init__(self, obj_path):
        vertices, faces = [], []
        with open(obj_path) as stream:
            for line in stream:
                if line.startswith('v '):
                    x, y, z = map(float, line.split()[1:4])
                    vertices.append((x, -z, y))  # my_map SDF roll = pi/2
                elif line.startswith('f '):
                    indices = [int(v.split('/')[0]) - 1 for v in line.split()[1:]]
                    faces.extend((indices[0], indices[i], indices[i + 1])
                                 for i in range(1, len(indices) - 1))
        triangles = np.asarray(vertices)[np.asarray(faces)]
        # This profile is deliberately restricted to the validated road region.
        keep = ((triangles[:, :, 0].max(1) > -32) & (triangles[:, :, 0].min(1) < 65)
                & (triangles[:, :, 1].max(1) > -85) & (triangles[:, :, 1].min(1) < -15))
        triangles = triangles[keep]
        segments = []
        for height in (2.86, 2.90, 3., 3.1, 3.2, 3.3, 3.4, 3.5):
            intersecting = triangles[(triangles[:, :, 2].min(1) < height)
                                     & (triangles[:, :, 2].max(1) > height)]
            for triangle in intersecting:
                points = []
                for i, j in ((0, 1), (1, 2), (2, 0)):
                    a, b = triangle[i], triangle[j]
                    if (a[2] - height) * (b[2] - height) < 0:
                        points.append((a + (b - a) * (height - a[2]) / (b[2] - a[2]))[:2])
                if len(points) == 2:
                    segments.append(points)
        if not segments:
            raise ValueError('No static world boundaries found')
        self.points = np.concatenate([
            a + np.linspace(0, 1, max(2, int(np.linalg.norm(b - a) / .03) + 2))[:, None] * (b - a)
            for a, b in np.asarray(segments)])
        self.tree = cKDTree(self.points)

    def clearance(self, states, geometry):
        minimum = float('inf')
        radius = math.hypot(geometry.length / 2, geometry.width / 2) + 1.
        for x, y, theta in states:
            center = np.array([x, y]) + geometry.rear_offset * direction(theta)
            ids = self.tree.query_ball_point(center, radius)
            if not ids:
                continue
            d = self.points[ids] - center
            c, s = np.cos(theta), np.sin(theta)
            local = np.c_[c * d[:, 0] + s * d[:, 1], -s * d[:, 0] + c * d[:, 1]]
            q = abs(local) - [geometry.length / 2, geometry.width / 2]
            distance = np.linalg.norm(np.maximum(q, 0), axis=1) + np.minimum(q.max(axis=1), 0)
            minimum = min(minimum, float(distance.min()))
        return minimum - .05  # wall discretization plus inter-pose motion


def connect(start, end, radius, boundary, geometry, clearance):
    candidates = []
    for signs in ((1, 0, 1), (-1, 0, -1), (1, 0, -1), (-1, 0, 1)):
        curvatures = np.array(signs) / radius
        solution = least_squares(
            lambda lengths: arcs(start, curvatures, lengths, False)[-1] - end,
            [1., np.linalg.norm(end[:2] - start[:2]), 1.],
            bounds=(0, 2 * np.pi * radius), max_nfev=120)
        if np.linalg.norm(solution.fun) > 1e-6:
            continue
        states = arcs(start, curvatures, solution.x)
        if boundary.clearance(states, geometry) >= clearance:
            candidates.append((sum(solution.x), states))
    if not candidates:
        raise ValueError('No collision-free forward connection to the turnaround; do not drive')
    return min(candidates, key=lambda item: item[0])[1]


@dataclass
class Plan:
    states: np.ndarray
    front: np.ndarray
    distance: np.ndarray
    target_names: list
    targets: np.ndarray
    target_distances: np.ndarray
    geometry: Geometry
    minimum_clearance: float
    turn_interval: tuple


def generate_plan(share, scenario='small_trash_dense', config_path=None):
    share = Path(share)
    config = json.loads(Path(config_path or share / 'config/fixed_cleaning_route.json').read_text())
    if config['world'] != 'my_map':
        raise ValueError('Only the validated my_map road profile is supported')
    geometry = Geometry.read(share / 'urdf/gen0_model.sdf', config['front_offset'])
    world = ET.parse(share / 'worlds/my_map/my_map.sdf').getroot().find('world')
    mesh_pose = list(map(float, world.find("model[@name='my_map']/pose").text.split()))
    if not np.allclose(mesh_pose, [0, 0, 0, np.pi / 2, 0, 0], atol=1e-7):
        raise ValueError('World mesh transform changed; profile must be revalidated')
    spawn = list(map(float, world.find("model[@name='gen0_model']/pose").text.split()))
    if not np.allclose([spawn[0], spawn[1], spawn[5]], config['start'], atol=.001):
        raise ValueError('Vehicle spawn differs from the cleaning profile')
    items = json.loads((share / f'worlds/trash_scenarios/my_map/{scenario}.json').read_text())
    items = {item['name']: item for item in items if not item['model'].startswith('trash_leaf')}
    names = config['outbound'] + config['inbound']
    if len(set(names)) != len(names) or set(names) != set(items):
        raise ValueError('Route must include every non-leaf trash entity exactly once')
    targets = np.array([items[name]['pose'][:2] for name in names])
    start = np.array(config['start'])
    initial_front = start[:2] + geometry.front_offset * direction(start[2])
    count = len(config['outbound'])
    outbound = sweep(np.vstack([initial_front, targets[:count]]), start[2], geometry)
    boundary = StaticBoundary(share / 'worlds/my_map/my_map.obj')
    radius, heading = config['turn_radius'], config['turn_heading']
    if 1 / radius > geometry.max_curvature:
        raise ValueError('Turn radius exceeds actual front-wheel steering limits')
    arc_start = np.r_[np.array(config['turn_center']) - radius * np.array([np.sin(heading), -np.cos(heading)]), heading]
    turn = arcs(arc_start, [-1 / radius], [np.pi * radius])
    return_heading = heading - np.pi
    return_start = np.r_[targets[count] - geometry.arm * direction(return_heading), return_heading]
    minimum = float(config['minimum_clearance'])
    approach = connect(outbound[-1], turn[0], radius, boundary, geometry, minimum)
    departure = connect(turn[-1], return_start, radius, boundary, geometry, minimum)
    finish = np.array(config['finish']) + geometry.front_offset * direction(return_heading)
    inbound = sweep(np.vstack([targets[count:], finish]), return_heading, geometry)
    states = np.vstack([outbound, approach[1:], turn[1:], departure[1:], inbound[1:]])
    front = states[:, :2] + geometry.arm * direction(states[:, 2])
    keep = np.r_[True, np.linalg.norm(np.diff(front, axis=0), axis=1) > 1e-8]
    states, front = states[keep], front[keep]
    clearance = boundary.clearance(states, geometry)
    if clearance < minimum:
        raise ValueError(f'Insufficient static clearance: {clearance:.3f} m < {minimum:.3f} m')
    distance = np.r_[0., np.cumsum(np.linalg.norm(np.diff(front, axis=0), axis=1))]
    indices = [int(np.argmin(np.linalg.norm(front - target, axis=1))) for target in targets]
    if np.any(np.diff(indices) <= 0) or max(np.linalg.norm(front[i] - q) for i, q in zip(indices, targets)) > .001:
        raise ValueError('Trajectory does not cover all trash centers in the requested order')
    # Keep the original precise target tracker around both boundary trash items.
    turn_interval = (float(distance[indices[count - 1]] + .30),
                     float(distance[indices[count]] - .80))
    return Plan(states, front, distance, names, targets, distance[indices], geometry, clearance,
                turn_interval)


class Tracker:
    """Front-point feedback linearization with bounded Ackermann curvature.

    Reference progress uses simulation elapsed time, never wall-clock time.
    Physical swept front-center segments confirm targets; reference progress
    alone can never mark a target covered.
    """

    def __init__(self, plan, speed=.45, tolerance=.10):
        if not 0 < speed <= .65 or not 0 < tolerance <= .10:
            raise ValueError('Speed must be in (0, .65], tolerance in (0, .10]')
        self.plan, self.speed, self.tolerance = plan, speed, tolerance
        self.progress = 0.
        self.covered = set()
        self.previous_front = None
        self.state = 'ready'
        self.reason = ''
        self.last_speed = 0.

    def fail(self, reason):
        self.state, self.reason, self.last_speed = 'failed', reason, 0.
        return 0., 0.

    def step(self, base_pose, dt):
        if self.state in ('failed', 'completed'):
            return 0., 0.
        if not np.all(np.isfinite(base_pose)) or not 0 < dt <= .25:
            return self.fail('Invalid pose or discontinuous simulation timestamp')
        g, plan = self.plan.geometry, self.plan
        theta = base_pose[2]
        h = direction(theta)
        front = np.array(base_pose[:2]) + g.front_offset * h
        if self.previous_front is None:
            yaw_error = math.atan2(math.sin(theta - plan.states[0, 2]), math.cos(theta - plan.states[0, 2]))
            if np.linalg.norm(front - plan.front[0]) > .15 or abs(yaw_error) > .10:
                return self.fail(f'Vehicle is not at the validated start pose: '
                                 f'x={base_pose[0]:.3f}, y={base_pose[1]:.3f}, yaw={theta:.3f}; '
                                 'reset before starting or check pose_index')
        else:
            delta = front - self.previous_front
            if np.linalg.norm(delta) > max(.15, dt * 2):
                return self.fail('Vehicle pose jumped; tracking stopped')
            for i, target in enumerate(plan.targets):
                if i in self.covered or abs(plan.target_distances[i] - self.progress) > .5:
                    continue
                fraction = np.clip(np.dot(target - self.previous_front, delta) / max(np.dot(delta, delta), 1e-12), 0, 1)
                if np.linalg.norm(target - self.previous_front - fraction * delta) <= self.tolerance:
                    self.covered.add(i)
        self.previous_front = front
        turning = plan.turn_interval[0] <= self.progress < plan.turn_interval[1]
        if turning:
            # Project only onto the nearby part of the turn. A global nearest
            # point could jump across the adjacent outbound/return lanes.
            lo = max(0, np.searchsorted(plan.distance, self.progress - .35) - 1)
            hi = min(len(plan.distance) - 1, np.searchsorted(plan.distance, self.progress + .35))
            starts = plan.front[lo:hi]
            segments = plan.front[lo + 1:hi + 1] - starts
            fractions = np.clip(np.sum((front - starts) * segments, axis=1)
                                / np.sum(segments * segments, axis=1), 0., 1.)
            projections = starts + fractions[:, None] * segments
            nearest = int(np.argmin(np.linalg.norm(projections - front, axis=1)))
            along = plan.distance[lo + nearest] + fractions[nearest] * (
                plan.distance[lo + nearest + 1] - plan.distance[lo + nearest])
            self.progress = float(np.clip(along, plan.turn_interval[0], plan.turn_interval[1]))
        reference = np.array([np.interp(self.progress, plan.distance, plan.front[:, j]) for j in range(2)])
        error = reference - front
        if np.linalg.norm(error) > .20:
            return self.fail('Front tracking error exceeds 0.20 m; no unchecked detour is permitted')
        missed = [i for i, s in enumerate(plan.target_distances) if s + .25 < self.progress and i not in self.covered]
        if missed:
            return self.fail(f'Missed trash target: {plan.target_names[missed[0]]}')
        remaining = plan.distance[-1] - self.progress
        if remaining < .001 and np.linalg.norm(error) < .08:
            if len(self.covered) != len(plan.targets):
                return self.fail('Route ended without covering every trash target')
            self.state, self.last_speed = 'completed', 0.
            return 0., 0.
        index = min(np.searchsorted(plan.distance, self.progress, side='right'), len(plan.distance) - 1)
        tangent = plan.front[index] - plan.front[index - 1]
        tangent /= np.linalg.norm(tangent)
        rear_ds = np.linalg.norm(plan.states[index, :2] - plan.states[index - 1, :2])
        curvature = abs(plan.states[index, 2] - plan.states[index - 1, 2]) / max(rear_ds, 1e-9)
        pace = min(self.speed, .20 if curvature > .16 else self.speed)
        if np.min(abs(plan.target_distances - self.progress)) < .8:
            pace = min(pace, .20)
        pace = min(pace, max(0., remaining * 1.5))
        if turning:
            # Ackermann cannot remove lateral error at zero forward velocity.
            # Follow the checked turn at low speed instead of freezing its
            # reference point. The 0.20 m error stop and curvature bound remain.
            pace = min(pace, .20)
        elif np.linalg.norm(error) > .08:
            # Keep a small positive crawl on the whole route.  Ackermann
            # steering cannot reduce lateral error while stationary; stopping
            # here can therefore create a deadlock before a turn.  A severe
            # error is still rejected by the safety bound above.
            pace = min(pace, .08)
        desired = pace * tangent + 1.5 * error
        velocity = max(0., float(np.dot(desired, h)))
        if turning:
            # Lateral feedback can cancel the forward component near the
            # hairpin. Ackermann needs a small positive longitudinal motion to
            # rotate, so retain a crawl speed while the checked error bound
            # and curvature limit still hold.
            velocity = max(velocity, min(.12, pace))
        omega = float(h[0] * desired[1] - h[1] * desired[0]) / g.arm
        requested = min(self.speed, velocity)
        # Never let ordinary tracking feedback collapse longitudinal motion
        # to zero: retain a crawl speed so the vehicle can steer its way back
        # onto the checked path.  Explicit stop/failure/completion paths return
        # before this point, and obstacle stopping is handled by the controller.
        if not turning and remaining > .001 and np.linalg.norm(error) <= .20:
            requested = max(requested, min(.08, pace))
        velocity = min(requested, self.last_speed + .30 * dt)
        if requested > 1e-8:
            omega *= velocity / requested
        omega = float(np.clip(omega, -g.max_curvature * velocity, g.max_curvature * velocity))
        self.last_speed = velocity
        progress_limit = plan.turn_interval[1] if turning else plan.distance[-1]
        self.progress = min(progress_limit, self.progress + pace * dt)
        self.state = 'tracking'
        return velocity, omega
