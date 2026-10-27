"""Bounded external camera configuration and conservative static fusion.

Observed surfaces are a union. Unknown cells are an intersection: a cell may
leave the exclusion only if at least one complete, valid view proves it free.
All configured views are mandatory; none is silently dropped on failure.
"""
from pathlib import Path
from dataclasses import replace
import signal
import time
import numpy as np
import yaml

from .obstacle_geometry import (boxes_from_occupancy, validate_config, decode_depth, estimate,
                                stamp_ns, visibility_depth)
from .geometry_monitor import occupancy, validate_monitor, quality_check


def initialize_geometry_worker(model_path, expected_hash):
    """Load the exact supplied collision model once in the geometry process."""
    from .self_filter import load_model
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    models, _, digest = load_model(model_path)
    if digest != expected_hash:
        raise ValueError('geometry worker collision model differs from capture model')
    global _worker_models
    _worker_models = tuple(models)


def geometry_payload(packets):
    # Meshes and their search trees are immutable. Only acquisition-time body
    # transforms change; do not serialize the entire robot for every view.
    return [(*packet[:2], tuple(body.inverse for body in packet[2]), *packet[3:])
            for packet in packets]


def calculate_pose_packets(packets, views, frozen_coverage=None):
    expanded = []
    for packet in packets:
        if len(packet[2]) != len(_worker_models):
            raise ValueError('geometry worker body count differs from capture model')
        bodies = [replace(model, inverse=inverse) for model, inverse in zip(_worker_models, packet[2])]
        expanded.append((*packet[:2], bodies, *packet[3:]))
    return calculate_packets(expanded, views, frozen_coverage)


def view_configs(primary):
    paths = primary.get('additional_perception_configs', [])
    if (not isinstance(paths, (list, tuple)) or len(paths) > 4
            or any(not isinstance(p, str) or not Path(p).is_absolute() for p in paths)):
        raise ValueError('additional_perception_configs requires at most four absolute YAML paths')
    configs = [primary]
    for path in paths:
        config = yaml.safe_load(Path(path).read_text())['depth_obstacles']['ros__parameters']
        if config.get('additional_perception_configs'):
            raise ValueError('nested additional views are not supported')
        configs.append(config)
    shared = ('output_frame', 'workspace_min', 'workspace_max', 'voxel_size',
              'target_size', 'target_mask_padding', 'self_filter_model',
              'joint_states_topic', 'target_pose_topic', 'max_input_age',
              'max_time_skew', 'monitor_inputs', 'max_collision_boxes',
              'support_surface_enabled', 'support_surface_height',
              'support_surface_normal', 'support_surface_xy_min',
              'support_surface_xy_max', 'support_contact_height_tolerance',
              'support_uncertainty_budget_m', 'motion_min_valid_depth_fraction',
              'motion_joint_delta', 'motion_hold_sec', 'self_surface_tolerance', 'monitor_robot_occlusion',
              'monitor_target_occlusion', 'monitor_fixed_calibration')
    for config in configs:
        validate_config(config)
        validate_monitor(config)
        for key in shared:
            if config.get(key) != primary.get(key):
                raise ValueError('views must share '+key)
    for key in ('camera_frame', 'rgb_image_topic', 'depth_image_topic', 'camera_info_topic', 'depth_info_topic'):
        if len({c[key] for c in configs}) != len(configs):
            raise ValueError('views must have distinct '+key)
    return configs


def fuse_estimates(estimates, configs):
    if not estimates or len(estimates) != len(configs):
        raise ValueError('every configured view must supply an estimate')
    if len(estimates) == 1:
        return estimates[0]
    masks = [occupancy(result[1], c) for result, c in zip(estimates, configs)]
    common = np.logical_and.reduce(masks)
    config = configs[0]
    unknown = boxes_from_occupancy(common, np.array(config['workspace_min']), np.array(config['workspace_max']))
    # Observed surfaces must remain exclusions even if another view calls the
    # same region free. Cover them outward with the same complete grid cells
    # used by runtime monitoring, so frozen coverage never overstates the
    # geometry actually sent to the planner.
    surfaces = [box for result in estimates for box in result[0]]
    lower, upper = np.array(config['workspace_min']), np.array(config['workspace_max'])
    observed_mask = occupancy(surfaces, config, outward=True)
    observed = boxes_from_occupancy(observed_mask, lower, upper)
    # Minimum fitted box widths can straddle the crop boundary. Keep those
    # original boxes as well; rasterization must not truncate an observation.
    observed.extend(box for box in surfaces if (box.lower < lower).any() or (box.upper > upper).any())
    if len(observed)+len(unknown) > config['max_collision_boxes']:
        raise ValueError('fused collision box budget exceeded; refuse incomplete snapshot')
    quality = {**estimates[0][2], 'observed_boxes': len(observed), 'unknown_boxes': len(unknown),
               'unknown_cells': int(common.sum()), 'total_cells': common.size,
               'views': [{**result[2], 'camera_frame': c['camera_frame']}
                         for result, c in zip(estimates, configs)]}
    for key in ('self_points_removed', 'target_points_removed', 'plane_points_removed', 'rejected_cluster_samples'):
        quality[key] = sum(result[2][key] for result in estimates)
    return observed, unknown, quality

def calculate_packets(packets, views, frozen_coverage=None, *, allow_motion_occlusion=False):
    started = time.monotonic()
    results, decoded_views = [], []
    remaining = None
    for packet, config in zip(packets, views):
        depth, k, bodies, camera, target, _, _, _ = packet
        decoded = decode_depth(depth, config['depth_encoding'], config['depth_scale'],
                               config['depth_min'], config['depth_max'], config.get('far_no_return', False))
        # Once a valid view proves a cell free, later views need only test
        # remaining unknown cells. All observed surfaces are still fitted.
        # The final intersection is identical to full independent grids.
        results.append(estimate(decoded, k, camera, bodies, target, config, remaining))
        results[-1][2]['visibility_candidate_cells'] = (
            int(remaining.sum()) if remaining is not None else results[-1][2]['total_cells'])
        results[-1][2]['unknown_count_scope'] = 'remaining_after_prior_views'
        if len(views) > 1:
            remaining = occupancy(results[-1][1], config)
        decoded_views.append(decoded)
    observed, unknown, quality = fuse_estimates(results, views)
    fractions, spacing = [], []
    for packet, config, decoded in zip(packets, views, decoded_views):
        # The unknown fraction is a property of the joint coverage. Each
        # mandatory view must still meet its own depth/sampling thresholds.
        quality_check(decoded, quality, config, packet[1],
                     allow_motion_occlusion=allow_motion_occlusion)
        usable = visibility_depth(decoded, config)
        fractions.append(float(np.isfinite(usable).mean()))
        if np.isfinite(usable).any():
            spacing.append(float(np.nanmax(usable)*config['sample_stride']/min(packet[1][0, 0], packet[1][1, 1])))
    explained = []
    occlusion_started = time.monotonic()
    if frozen_coverage is not None and views[0].get('monitor_robot_occlusion', False):
        from .robot_occlusion import witnessed_cells, known_occluders
        new_unknown = occupancy(unknown, views[0]) & ~frozen_coverage
        # A positive environment observation cannot be excused as a shadow.
        new_unknown &= ~occupancy(observed, views[0], outward=True)
        indices = np.argwhere(new_unknown)
        # Each view builds one pixel mask, bounded by its acquired image.
        # Check overlapping cell rectangles together so surface distances and
        # image reductions are not repeated for the same pixels in batches.
        # Every cell still needs a complete-view witness under the same lease.
        witnessed = np.zeros(len(indices), dtype=bool)
        for packet, config, decoded in zip(packets, views, decoded_views):
            remaining = np.flatnonzero(~witnessed)
            if len(remaining):
                witnessed[remaining] = witnessed_cells(indices[remaining], decoded, packet[1],
                                                       packet[3], known_occluders(packet[2], packet[4], config), config)
        explained = indices[witnessed].tolist()
    return observed, unknown, quality, {
        'valid_depth_fraction': min(fractions), 'view_count': len(packets),
        'view_stamps_ns': [stamp_ns(p[0]) for p in packets],
        'sample_spacing_upper_bound_m': max(spacing) if spacing else None,
        'motion_occlusion_tolerated': bool(allow_motion_occlusion),
        'robot_occlusion_indices': explained,
        'geometry_compute_sec': time.monotonic()-started,
        'robot_occlusion_compute_sec': time.monotonic()-occlusion_started,
    }
