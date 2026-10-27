"""Deterministic static fixtures -> one neutral geometry + replay manifest.

No ROS/Gazebo imports. Structural acceptance is not motion feasibility.
The initial robot collision check is performed against live MoveIt at startup.
"""

import hashlib
import json
import math
from pathlib import Path
import random
import xml.etree.ElementTree as ET

import yaml

from pick_place_scene.static_geometry import Box, load_geometry

VERSION = "rm65b-static-obstacles/1"
NEGATIVE = {"blocked_pick", "blocked_place"}


def rect(box):
    return tuple((p - s/2, p + s/2) for p, s in zip(box.pose[:2], box.size[:2]))


def gap(a, b):
    """Euclidean separation of axis-aligned XY footprints; zero for touching."""
    return math.hypot(*(max(0., x[0]-y[1], y[0]-x[1]) for x, y in zip(a, b)))


def config_from(path, **overrides):
    config = yaml.safe_load(Path(path).read_text())
    keys = {'obstacle_mode', 'obstacle_seed', 'obstacle_count', 'max_attempts',
            'obstacle_min_height', 'obstacle_max_height', 'obstacle_clearance',
            'protected_margin', 'support_margin', 'robot_keepout_radius',
            'benchmark', 'footprints', 'fixtures'}
    if not isinstance(config, dict) or set(config) != keys or set(overrides)-keys:
        raise ValueError('obstacle config has missing or unknown keys')
    config.update({k: v for k, v in overrides.items() if v is not None})
    if config['obstacle_mode'] not in ('fixed', 'random', 'benchmark'):
        raise ValueError('obstacle_mode must be fixed, random or benchmark')
    for key in ('obstacle_seed', 'obstacle_count', 'max_attempts'):
        if type(config[key]) is not int or config[key] < 0:
            raise ValueError(f'{key} must be a nonnegative integer')
    if not 1 <= config['max_attempts'] <= 100000 or config['obstacle_count'] > 100:
        raise ValueError('generation budget/count out of range')
    for key in ('obstacle_min_height', 'obstacle_max_height', 'obstacle_clearance',
                'protected_margin', 'support_margin', 'robot_keepout_radius'):
        value = config[key]
        if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
            raise ValueError(f'{key} must be finite and positive')
    if config['obstacle_min_height'] > config['obstacle_max_height']:
        raise ValueError('minimum height exceeds maximum')
    if config['benchmark'] not in config['fixtures']:
        raise ValueError('unknown benchmark')
    for name, size in config['footprints'].items():
        if (not name.isidentifier() or not isinstance(size, list) or len(size) != 2
                or any(type(x) not in (int, float) or not math.isfinite(x) or x <= 0 for x in size)):
            raise ValueError('invalid footprint')
    if not config['footprints']:
        raise ValueError('empty footprints')
    for name, fixtures in config['fixtures'].items():
        if not isinstance(name, str) or not isinstance(fixtures, list):
            raise ValueError('invalid fixture list')
        for item in fixtures:
            if (not isinstance(item, list) or len(item) != 4 or item[0] not in config['footprints']
                    or any(type(x) not in (int, float) or not math.isfinite(x) for x in item[1:])
                    or item[3] <= 0):
                raise ValueError('invalid fixture box')
    return config


def generate_scene(workbench, task, mount, config, destination):
    """Write a NEW evidence directory. No silent overwrite or fallback scene."""
    frame, supports = load_geometry(workbench)
    if (frame != 'base_link' or {b.id for b in supports} !=
            {'pick_table', 'left_table', 'right_table'} or any(any(b.pose[3:]) for b in supports)):
        raise ValueError('generator requires the axis-aligned base_link U workbench')
    joint = ET.parse(mount).find("joint[@name='world_to_rm65b']")
    if (joint is None or joint.find('parent').get('link') != 'world'
            or joint.find('child').get('link') != frame):
        raise ValueError('explicit world-to-base mount required')
    origin = joint.find('origin')
    if any(map(float, origin.get('rpy', '0 0 0').split())):
        raise ValueError('rotated mount unsupported')
    offset = tuple(map(float, origin.get('xyz').split()))
    if len(offset) != 3 or not all(math.isfinite(x) for x in offset):
        raise ValueError('robot mount translation requires three finite numbers')
    task_data = yaml.safe_load(Path(task).read_text())['known_pick_place_scene']['ros__parameters']
    if task_data['frame_id'] != 'world' or task_data['object_id'] != 'pick_object':
        raise ValueError('task requires world frame and pick_object ID')
    for key, count in (('object_size', 3), ('object_pose', 6), ('place_pose', 6)):
        vector = task_data[key]
        if (not isinstance(vector, list) or len(vector) != count
                or any(type(x) not in (int, float) or not math.isfinite(x) for x in vector)
                or (key == 'object_size' and any(x <= 0 for x in vector))
                or (key != 'object_size' and any(vector[3:]))):
            raise ValueError('generator requires finite, axis-aligned task geometry')
    margin = config['protected_margin']
    zones = []
    for key in ('object_pose', 'place_pose'):
        pose = task_data[key]
        bottom = pose[2]-offset[2]-task_data['object_size'][2]/2
        footprint = tuple((pose[i]-offset[i]-task_data['object_size'][i]/2,
                           pose[i]-offset[i]+task_data['object_size'][i]/2) for i in range(2))
        if not any(abs(bottom-(s.pose[2]+s.size[2]/2)) < 1e-8 and all(
                lo >= sl and hi <= sh for (lo, hi), (sl, sh) in zip(footprint, rect(s))) for s in supports):
            raise ValueError(f'{key} is not supported by the configured workbench')
        zones.append(tuple((pose[i]-offset[i]-task_data['object_size'][i]/2-margin,
                            pose[i]-offset[i]+task_data['object_size'][i]/2+margin) for i in range(2)))

    def violations(box, others):
        bounds = rect(box)
        containers = [s for s in supports if all(
            lo >= sl+config['support_margin']-1e-9 and hi <= sh-config['support_margin']+1e-9
            for (lo, hi), (sl, sh) in zip(bounds, rect(s))) and
            abs(box.pose[2]-box.size[2]/2-(s.pose[2]+s.size[2]/2)) < 1e-8]
        errors = [] if containers else ['outside_support']
        if any(gap(bounds, zone) <= 1e-9 for zone in zones):
            errors.append('occupied_pick_or_place_zone')
        origin_rect = ((0., 0.), (0., 0.))
        if gap(bounds, origin_rect) < config['robot_keepout_radius']:
            errors.append('robot_base_keepout')
        if any(gap(bounds, rect(b)) < config['obstacle_clearance']-1e-9 for b in others):
            errors.append('overlap_or_insufficient_clearance')
        return errors

    obstacles, rejected, deliberate = [], 0, []
    mode, name = config['obstacle_mode'], config['benchmark']
    expected = 'blocked' if mode == 'benchmark' and name in NEGATIVE else 'candidate'
    if mode == 'fixed' and name in NEGATIVE:
        raise ValueError('negative fixtures require explicit benchmark mode')
    rng = random.Random(config['obstacle_seed'])
    fixtures = config['fixtures'][name] if mode != 'random' else None
    budget = len(fixtures) if fixtures is not None else config['max_attempts']
    for attempt in range(budget):
        if fixtures is None:
            if len(obstacles) == config['obstacle_count']:
                break
            shape = rng.choice(sorted(config['footprints']))
            support = rng.choice(supports)
            x = rng.uniform(*rect(support)[0])
            y = rng.uniform(*rect(support)[1])
            height = rng.uniform(config['obstacle_min_height'], config['obstacle_max_height'])
        else:
            shape, x, y, height = fixtures[attempt]
            support = next((s for s in supports if all(lo <= v <= hi for v, (lo, hi)
                           in zip((x, y), rect(s)))), supports[0])
        size = tuple(config['footprints'][shape]) + (height,)
        box = Box(f'obstacle_{shape}_{len(obstacles):02d}', size,
                  (x, y, support.pose[2]+support.size[2]/2+height/2, 0., 0., 0.))
        errors = violations(box, obstacles)
        if errors and expected != 'blocked':
            if fixtures is not None:
                raise ValueError(f'{name}: {errors}')
            rejected += 1
            continue
        deliberate.extend(errors)
        obstacles.append(box)
    if fixtures is None and len(obstacles) != config['obstacle_count']:
        raise ValueError(f'generation exhausted: placed {len(obstacles)}/{config["obstacle_count"]}')
    if expected == 'blocked' and 'occupied_pick_or_place_zone' not in deliberate:
        raise ValueError('negative fixture must actually occupy the protected task zone')
    geometry = {'frame_id': frame, 'boxes': [
        {'id': b.id, 'size': list(b.size), 'pose': list(b.pose)} for b in (*supports, *obstacles)]}
    # Reuse the strict public loader before making the archive usable.
    destination = Path(destination)
    destination.mkdir(parents=True, exist_ok=False)
    snapshot = destination / 'geometry.yaml'
    snapshot.write_text(yaml.safe_dump(geometry, sort_keys=False))
    load_geometry(snapshot)
    manifest = {
        'generator_version': VERSION, 'config': config, 'expected': expected,
        'reason': sorted(set(deliberate)), 'rejected_samples': rejected,
        'geometry_sha256': hashlib.sha256(snapshot.read_bytes()).hexdigest(),
        'geometry': geometry, 'task': task_data, 'world_to_base_translation': offset,
        'protected_zones_xy': zones, 'initial_robot_collision': 'requires_live_preflight',
        'planning_feasibility': 'not_inferred_from_geometry',
    }
    (destination / 'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    Path(destination / 'task.yaml').write_bytes(Path(task).read_bytes())
    return snapshot, manifest
