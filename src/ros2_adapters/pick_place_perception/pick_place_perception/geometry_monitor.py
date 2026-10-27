"""Frozen conservative scene monitoring; never updates an executing scene."""
import math
import numpy as np
from .obstacle_geometry import visibility_depth


def validate_monitor(config):
    if not isinstance(config.get('monitor_inputs', False), bool):
        raise ValueError('monitor_inputs must be boolean')
    if not config.get('monitor_inputs', False):
        return
    if not isinstance(config.get("monitor_fixed_calibration", False), bool):
        raise ValueError("monitor_fixed_calibration must be boolean")
    explain = config.get('monitor_robot_occlusion', False)
    if not isinstance(explain, bool):
        raise ValueError('monitor_robot_occlusion must be boolean')
    if explain and not 0 < config.get('self_surface_tolerance', 0.) <= .003:
        raise ValueError('robot occlusion requires an explicit bounded self_surface_tolerance')
    target = config.get('monitor_target_occlusion', False)
    if not isinstance(target, bool) or (target and not explain):
        raise ValueError('monitor_target_occlusion requires boolean opt-in and robot surface monitoring')
    for name in ('min_valid_depth_fraction', 'max_unknown_fraction'):
        value = config.get(name)
        if (isinstance(value, bool) or not isinstance(value, (int, float))
                or not math.isfinite(value) or not 0 < value <= 1):
            raise ValueError(f'{name} must be explicitly set in (0, 1]')
    relaxed = config.get('motion_min_valid_depth_fraction',
                        config['min_valid_depth_fraction'])
    if (isinstance(relaxed, bool) or not isinstance(relaxed, (int, float))
            or not math.isfinite(relaxed) or not 0 < relaxed <= config['min_valid_depth_fraction']):
        raise ValueError('motion_min_valid_depth_fraction must be in (0, min_valid_depth_fraction]')
    delta = config.get('motion_joint_delta', 0.005)
    hold = config.get('motion_hold_sec', 2.0)
    if (isinstance(delta, bool) or not isinstance(delta, (int, float))
            or not math.isfinite(delta) or delta <= 0):
        raise ValueError('motion_joint_delta must be finite and positive')
    if (isinstance(hold, bool) or not isinstance(hold, (int, float))
            or not math.isfinite(hold) or hold <= 0):
        raise ValueError('motion_hold_sec must be finite and positive')
    bound = config.get('sampling_bound_m')
    if bound is not None and (isinstance(bound, bool) or not isinstance(bound, (int, float)) or not math.isfinite(bound) or bound <= 0):
        raise ValueError('sampling_bound_m must be finite and positive')


def calibration_signature(info):
    """All CameraInfo calibration fields, excluding the acquisition stamp."""
    roi = info.roi
    return (info.header.frame_id, info.width, info.height, info.distortion_model,
            tuple(info.d), tuple(info.k), tuple(info.r), tuple(info.p),
            info.binning_x, info.binning_y, roi.x_offset, roi.y_offset,
            roi.width, roi.height, roi.do_rectify)


def quality_check(decoded, quality, config, k=None, *, allow_motion_occlusion=False):
    if not config.get('monitor_inputs', False):
        return
    usable = visibility_depth(decoded, config)
    valid_fraction = np.isfinite(usable).mean()
    threshold = config['min_valid_depth_fraction']
    if allow_motion_occlusion:
        threshold = config.get('motion_min_valid_depth_fraction', threshold)
    if valid_fraction < threshold:
        raise ValueError('depth quality below configured valid-pixel fraction: '
                         f"view={config.get('camera_frame', 'unspecified')}, "
                         f'measured={valid_fraction:.6f}, required={threshold:.6f}')
    if quality['unknown_cells']/quality['total_cells'] > config['max_unknown_fraction']:
        raise ValueError('unknown workspace exceeds configured fraction')
    if k is not None and 'sampling_bound_m' in config and np.isfinite(usable).any():
        spacing = np.nanmax(usable)*config['sample_stride']/min(k[0, 0], k[1, 1])
        if spacing > config['sampling_bound_m']:
            raise ValueError('pixel sampling spacing exceeds collision uncertainty budget')


def joint_motion(previous, current, threshold):
    """Return true only when a complete joint sample changed materially."""
    if previous is None or current is None or previous.keys() != current.keys():
        return False
    return any(abs(float(current[name]) - float(previous[name])) >= threshold
               for name in previous)


def occupancy(boxes, config, *, outward=False):
    """Rasterize grid boxes, or conservatively cover arbitrary observed AABBs."""
    lower, upper = np.array(config['workspace_min']), np.array(config['workspace_max'])
    shape = np.ceil((upper-lower)/config['voxel_size']).astype(int)
    step = (upper-lower)/shape
    mask = np.zeros(tuple(shape), dtype=bool)
    for box in boxes:
        if outward:
            # boxes_from_occupancy() reconstructs these very grid boundaries.
            # A subtract/divide round trip can put an integer one ULP either
            # side of itself, spuriously adding a full 1 cm occupied layer.
            # Snap only arithmetic roundoff, not geometric uncertainty: a
            # genuinely off-grid observation still rounds strictly outward.
            def boundary(value):
                position = (value-lower)/step
                nearest = np.rint(position)
                tolerance = 16*np.finfo(float).eps*np.maximum(1., np.abs(position))
                return np.where(np.abs(position-nearest) <= tolerance, nearest, position)
            lo = np.floor(boundary(box.lower)).astype(int)
            hi = np.ceil(boundary(box.upper)).astype(int)
        else:
            lo = np.rint((box.lower-lower)/step).astype(int)
            hi = np.rint((box.upper-lower)/step).astype(int)
        lo, hi = np.clip(lo, 0, shape), np.clip(hi, 0, shape)
        mask[tuple(slice(int(a), int(b)) for a, b in zip(lo, hi))] = True
    return mask


def check_frozen_coverage(baseline, current):
    newly_excluded = current & ~baseline
    changed = int(np.count_nonzero(newly_excluded))
    if changed:
        samples = np.argwhere(newly_excluded)[:8].tolist()
        raise ValueError(f'environment coverage changed: {changed} previously free cells now occupied/unknown; '
                         f'sample_grid_indices={samples}; recapture and replan')


def check_new_observed_occupancy(baseline, current):
    """Reject newly observed occupied cells, while tolerating self-occlusion.

    During robot motion an external camera can lose a previously visible
    background cell behind the robot. That produces an unknown cell, not
    evidence of an environmental obstacle. Newly observed surfaces remain a
    positive obstacle witness and therefore invalidate the frozen plan.
    """
    changed = int(np.count_nonzero(current & ~baseline))
    if changed:
        raise ValueError(
            f'environment occupancy changed: {changed} newly observed occupied cells; '
            'recapture and replan')
