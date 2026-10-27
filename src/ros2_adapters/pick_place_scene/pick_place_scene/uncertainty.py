"""Explicit worst-case length budget for conservative collision geometry.

This is a sum of supplied bounds, not a statistical confidence estimate. A
single-view observed box needs unknown-volume coverage before inflation.
"""
from copy import deepcopy
import hashlib
import math
from pathlib import Path
import yaml


def load_budget(path):
    raw = Path(path).read_bytes()
    config = yaml.safe_load(raw)
    lengths = ('depth_bound_m', 'extrinsic_translation_bound_m', 'tracking_bound_m',
               'sampling_bound_m', 'safety_margin_m', 'observation_range_m',
               'relative_speed_bound_mps', 'time_skew_bound_sec', 'extrinsic_rotation_bound_rad')
    if not isinstance(config, dict) or set(config) != set(lengths) | {'object_ids', 'basis'}:
        raise ValueError('uncertainty budget requires all bounds, object_ids and basis')
    if (not isinstance(config['object_ids'], list) or not config['object_ids']
            or any(not isinstance(x, str) or not x.strip() for x in config['object_ids'])
            or len(set(config['object_ids'])) != len(config['object_ids'])):
        raise ValueError('uncertainty object_ids must be unique non-empty strings')
    if not isinstance(config['basis'], str) or not config['basis'].strip():
        raise ValueError('uncertainty budget must state its evidence/assumptions')
    for name in lengths:
        value = config[name]
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value < 0:
            raise ValueError(f'{name} must be finite and nonnegative')
    if (not 0 < config['observation_range_m'] <= 100 or config['extrinsic_rotation_bound_rad'] > math.pi
            or config['relative_speed_bound_mps'] > 100 or config['time_skew_bound_sec'] > 10):
        raise ValueError('uncertainty bounds outside supported range')
    terms = {name: config[name] for name in ('depth_bound_m', 'extrinsic_translation_bound_m',
                                            'tracking_bound_m', 'sampling_bound_m', 'safety_margin_m')}
    terms['rotation_displacement_m'] = 2*config['observation_range_m']*math.sin(config['extrinsic_rotation_bound_rad']/2)
    terms['time_displacement_m'] = config['relative_speed_bound_mps']*config['time_skew_bound_sec']
    margin = sum(terms.values())
    if not 0 < margin <= 1:
        raise ValueError('total per-side uncertainty must be in (0, 1] metres')
    return config, terms, margin, hashlib.sha256(raw).hexdigest()


def inflate(obj, budget):
    """Copy and expand every local BOX axis; never touch support/target IDs."""
    if budget is None or obj.id not in budget[0]['object_ids'] or obj.operation != obj.ADD:
        return obj
    result = deepcopy(obj)
    for shape in result.primitives:
        shape.dimensions = [value+2*budget[2] for value in shape.dimensions]
    return result
