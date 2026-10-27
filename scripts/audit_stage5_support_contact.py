#!/usr/bin/env python3
"""Audit recorded support contact cells without ROS nodes or scene writes.

Each --view is a config/one-bundle bag pair. The grid/task/support parameters
come from --geometry-config. This is an offline geometry audit, not a live
synchronized multi-camera or execution acceptance.
"""
import argparse
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import time

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import yaml

from pick_place_perception.obstacle_geometry import (
    decode_depth, estimate, matrix, stamp_ns, validate_bundle, validate_config)
from pick_place_perception.geometry_monitor import occupancy
from pick_place_perception.self_filter import load_model
from pick_place_scene.uncertainty import load_budget


def params(path):
    return yaml.safe_load(Path(path).read_text())['depth_obstacles']['ros__parameters']


def reference_contact(depth, k, camera, cell_lower, cell_upper, c):
    """Independent full-image ray oracle; intentionally slower than production.

    No rectangle batching, eroded minima or incremental candidate masks are
    used. It is limited to the contact cells actually relevant to the target.
    """
    v, u = np.mgrid[:depth.shape[0], :depth.shape[1]]
    optical = np.stack(((u-k[0, 2])/k[0, 0], (v-k[1, 2])/k[1, 1], np.ones_like(u)), axis=-1)
    directions = optical @ camera[:3, :3].T
    origin = camera[:3, 3]
    rounding = c.get('visibility_roundoff_m', 0.)
    with np.errstate(divide='ignore', invalid='ignore'):
        plane = (c['support_surface_height']-origin[2])/directions[:, :, 2]
        hit = origin+plane[:, :, None]*directions
    plane_witness = (np.isfinite(depth) & np.isfinite(plane) & (plane > c['depth_min'])
                     & (directions[:, :, 2] < -1e-12)
                     & (np.abs((depth-plane)*directions[:, :, 2])
                        <= c['support_contact_height_tolerance']+rounding)
                     & ((hit[:, :, :2] >= np.array(c['support_surface_xy_min'])-1e-9)
                        & (hit[:, :, :2] <= np.array(c['support_surface_xy_max'])+1e-9)).all(axis=-1))
    usable = np.where(np.isposinf(depth), c['depth_max'], depth) if c.get('far_no_return') else depth
    result = []
    for lower, upper in zip(cell_lower, cell_upper):
        contact = (lower[2] <= c['support_surface_height']+c['support_contact_height_tolerance']+1e-9
                   and upper[2] >= c['support_surface_height']-c['support_contact_height_tolerance']-1e-9
                   and np.all(lower[:2] >= np.array(c['support_surface_xy_min'])-1e-9)
                   and np.all(upper[:2] <= np.array(c['support_surface_xy_max'])+1e-9))
        vertices = np.array([[x, y, z] for x in (lower[0], upper[0])
                              for y in (lower[1], upper[1]) for z in (lower[2], upper[2])])
        optical_vertices = (vertices-origin) @ camera[:3, :3]
        with np.errstate(divide='ignore', invalid='ignore'):
            px = optical_vertices[:, 0]/optical_vertices[:, 2]*k[0, 0]+k[0, 2]
            py = optical_vertices[:, 1]/optical_vertices[:, 2]*k[1, 1]+k[1, 2]
        in_frame = (optical_vertices[:, 2].min() > c['depth_min'] and np.isfinite(px).all()
                    and np.isfinite(py).all() and np.floor(px.min())-1 >= 0
                    and np.floor(py.min())-1 >= 0 and np.ceil(px.max())+1 < depth.shape[1]
                    and np.ceil(py.max())+1 < depth.shape[0])
        if not contact or not in_frame:
            result.append(False)
            continue
        enter, leave = np.full(depth.shape, -np.inf), np.full(depth.shape, np.inf)
        outside = np.zeros(depth.shape, bool)
        for axis in range(3):
            ray = directions[:, :, axis]
            parallel = np.abs(ray) < 1e-12
            with np.errstate(divide='ignore', invalid='ignore'):
                a, b = (lower[axis]-origin[axis])/ray, (upper[axis]-origin[axis])/ray
            enter = np.maximum(enter, np.where(parallel, -np.inf, np.minimum(a, b)))
            leave = np.minimum(leave, np.where(parallel, np.inf, np.maximum(a, b)))
            outside |= parallel & ((origin[axis] < lower[axis]) | (origin[axis] > upper[axis]))
        intersects = ~outside & (leave >= enter) & (leave > c['depth_min'])
        free = np.isfinite(usable) & (usable+rounding > leave+c['visibility_margin'])
        known_support = plane_witness & (leave <= plane+rounding+1e-12)
        result.append(bool(intersects.any() and (free | known_support)[intersects].all()))
    return np.array(result, dtype=bool)


def read_view(config_path, bag_path, geometry):
    original = params(config_path)
    c = deepcopy(geometry)
    for key in ('camera_frame', 'rgb_image_topic', 'depth_image_topic', 'camera_info_topic', 'depth_info_topic'):
        c[key] = original[key]
    c.pop('additional_perception_configs', None)
    validate_config(c)
    bag = Path(bag_path)
    source = json.loads(Path(str(bag)+'.source.json').read_text())
    hashes = {name: hashlib.sha256((bag/name).read_bytes()).hexdigest() for name in source['sha256']}
    if hashes != source['sha256']:
        raise ValueError('recording differs from its source hashes')
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))
    types = {item.name: get_message(item.type) for item in reader.get_all_topics_and_types()}
    messages = {}
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic in messages:
            raise ValueError('audit requires one recorded bundle per view, not an arbitrary stream')
        messages[topic] = deserialize_message(data, types[topic])
    image = messages[c['depth_image_topic']]
    k = validate_bundle(messages[c['rgb_image_topic']], image,
                        messages[c['camera_info_topic']], messages[c['depth_info_topic']], c)
    inputs = [messages[c[key]] for key in ('rgb_image_topic', 'depth_image_topic', 'camera_info_topic',
                                           'depth_info_topic', 'joint_states_topic', 'target_pose_topic')]
    stamps = [stamp_ns(msg) for msg in inputs]
    if min(stamps) <= 0 or max(stamps)-min(stamps) > c['max_time_skew']*1e9:
        raise ValueError('recorded input bundle violates the time-skew contract')
    transforms = {c['output_frame']: np.eye(4)}
    allowed_stamps = {stamp_ns(image), stamp_ns(messages[c['target_pose_topic']])}
    for tf in messages['/tf'].transforms:
        if tf.header.frame_id != c['output_frame'] or stamp_ns(tf) not in allowed_stamps:
            raise ValueError('audit requires recorded acquisition-time TF flattened to output frame')
        p, q = tf.transform.translation, tf.transform.rotation
        transforms[tf.child_frame_id] = matrix([p.x, p.y, p.z], [q.x, q.y, q.z, q.w])
    target = messages[c['target_pose_topic']]
    p, q = target.pose.position, target.pose.orientation
    target_pose = transforms[target.header.frame_id] @ matrix([p.x, p.y, p.z], [q.x, q.y, q.z, q.w])
    models, joints, digest = load_model(c['self_filter_model'])
    if digest != source['model_sha256'] or not joints <= set(messages[c['joint_states_topic']].name):
        raise ValueError('recorded robot model/state does not match the audit model')
    bodies = [body.at(transforms[body.link]) for body in models]
    depth = decode_depth(image, c['depth_encoding'], c['depth_scale'], c['depth_min'], c['depth_max'],
                         c.get('far_no_return', False))
    return c, depth, k, transforms[c['camera_frame']], bodies, target_pose, {
        'bag': str(bag.resolve()), 'source': source['source'], 'sha256': hashes,
        'stamp_ns': stamp_ns(image), 'model_sha256': digest}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--view', nargs=2, action='append', required=True, metavar=('CONFIG', 'BAG'))
    parser.add_argument('--geometry-config', required=True)
    parser.add_argument('--uncertainty', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--probe', nargs=3, type=float, metavar=('X', 'Y', 'Z'))
    args = parser.parse_args()
    geometry = params(args.geometry_config)
    validate_config(geometry)
    if not geometry.get('support_surface_enabled', False):
        parser.error('audit requires an explicit support contract')
    budget, _, margin, budget_hash = load_budget(args.uncertainty)
    if (abs(geometry['support_uncertainty_budget_m']-margin) > 1e-9
            or geometry['support_contact_height_tolerance'] > budget['depth_bound_m']):
        raise ValueError('support height and inflation budgets do not match')
    before, after, observations, records, targets = [], [], [], [], []
    for config_path, bag_path in args.view:
        c, depth, k, camera, bodies, target, record = read_view(config_path, bag_path, geometry)
        control = {**c, 'support_surface_enabled': False}
        started = time.monotonic()
        seen0, unknown0, _ = estimate(depth, k, camera, bodies, target, control)
        seen1, unknown1, _ = estimate(depth, k, camera, bodies, target, c)
        mask0, mask1 = occupancy(unknown0, c), occupancy(unknown1, c)
        lower, upper = np.array(c['workspace_min']), np.array(c['workspace_max'])
        step = (upper-lower)/np.array(mask0.shape)
        indices = np.argwhere(mask0)
        lo, hi = lower+indices*step, lower+(indices+1)*step
        half = np.array(c['target_size'])/2
        relevant = ((lo-margin < target[:3, 3]+half-1e-9)
                    & (hi+margin > target[:3, 3]-half+1e-9)).all(axis=1)
        indices, lo, hi = indices[relevant], lo[relevant], hi[relevant]
        proven = reference_contact(depth, k, camera, lo, hi, c)
        expected = ~proven
        if not np.array_equal(mask1[tuple(indices.T)], expected):
            raise AssertionError('production support classification differs from full-image ray reference')
        if np.any(mask1 & ~mask0):
            raise AssertionError('support rule unexpectedly introduced unknown cells')
        if len(seen0) != len(seen1) or any(not np.array_equal(a.lower, b.lower)
                or not np.array_equal(a.upper, b.upper) for a, b in zip(seen0, seen1)):
            raise AssertionError('support rule changed independent observed geometry')
        before.append(mask0)
        after.append(mask1)
        observations.extend(seen1)
        targets.append(target)
        records.append({**record, 'camera_frame': c['camera_frame'],
                        'unknown_cells_before': int(mask0.sum()), 'unknown_cells_after': int(mask1.sum()),
                        'witnessed_contact_cells': int((mask0 & ~mask1).sum()),
                        'target_near_reference_cells': len(indices), 'full_image_ray_reference_matches': True,
                        'observed_geometry_unchanged': True, 'two_estimates_wall_sec': time.monotonic()-started})
    if not all(np.allclose(targets[0], t, atol=1e-8, rtol=0) for t in targets):
        raise ValueError('static recordings have different known targets')
    before, after = np.logical_and.reduce(before), np.logical_and.reduce(after)
    lower, upper = np.array(geometry['workspace_min']), np.array(geometry['workspace_max'])
    step = (upper-lower)/np.array(before.shape)
    indices = np.indices(before.shape).reshape(3, -1).T
    lo, hi = lower+indices*step, lower+(indices+1)*step
    # Target is upright in this fixture. Do not silently use an AABB for a
    # tilted or yaw-rotated target in the collision diagnostic.
    if not np.allclose(targets[0][:3, :3], np.eye(3), atol=1e-8, rtol=0):
        raise ValueError('target-intersection audit requires an axis-aligned target')
    half = np.array(geometry['target_size'])/2
    target_lo, target_hi = targets[0][:3, 3]-half, targets[0][:3, 3]+half
    intersects = ((lo-margin < target_hi-1e-9) & (hi+margin > target_lo+1e-9)).all(axis=1)
    before_hits = before.reshape(-1) & intersects
    after_hits = after.reshape(-1) & intersects
    seen_hits = sum(bool(((b.lower-margin < target_hi-1e-9)
                         & (b.upper+margin > target_lo+1e-9)).all()) for b in observations)
    result = {'scope': 'separately recorded static sensor bundles; offline audit, not live execution',
              'views': records, 'uncertainty_sha256': budget_hash, 'per_side_margin_m': margin,
              'support_contact_height_tolerance_m': geometry['support_contact_height_tolerance'],
              'combined_unknown_cells_before': int(before.sum()), 'combined_unknown_cells_after': int(after.sum()),
              'target_intersecting_unknown_cells_before': int(before_hits.sum()),
              'target_intersecting_unknown_cells_after': int(after_hits.sum()),
              'target_intersecting_observed_boxes_after': seen_hits,
              'remaining_target_intersecting_indices': indices[after_hits].tolist()}
    if args.probe is not None:
        point = np.array(args.probe)
        if not np.isfinite(point).all() or not ((point >= lower) & (point < upper)).all():
            raise ValueError('probe must lie inside the configured workspace')
        index = tuple(np.floor((point-lower)/step).astype(int))
        seen = any(((point >= box.lower) & (point <= box.upper)).all() for box in observations)
        result['probe'] = {'point': point.tolist(), 'unknown_before': bool(before[index]),
                           'unknown_after': bool(after[index]), 'observed_after': bool(seen),
                           'collision_covered_without_inflation': bool(after[index] or seen)}
    Path(args.output).write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps({key: val for key, val in result.items() if key != 'views'}, indent=2))


if __name__ == '__main__':
    main()
