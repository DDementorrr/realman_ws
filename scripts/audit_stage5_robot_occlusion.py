#!/usr/bin/env python3
"""Offline monitoring explanation audit; never writes ROS or planning geometry.

Compares the production surface-tree/rectangle implementation with an independent
full-triangle AABB scan and per-pixel reference. Input bags must be actual static
acquisition bundles; separately sampled views do not prove live synchronization.
"""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
from audit_stage5_support_contact import params, read_view
from pick_place_perception.obstacle_geometry import Bounds, CORNERS, estimate
from pick_place_perception.geometry_monitor import occupancy
from pick_place_perception.multi_view import fuse_estimates
from pick_place_perception.robot_occlusion import witnessed_cells, known_occluders


def mesh_surface_reference(point, triangles, tolerance):
    # Inspect ALL triangle AABBs. No KD tree, hull/parity test or production
    # distance routine is called; degenerate triangles retain their edges.
    keep = ((point >= triangles.min(axis=1)-tolerance)
            & (point <= triangles.max(axis=1)+tolerance)).all(axis=1)
    triangles = triangles[keep]
    if not len(triangles):
        return False
    a, b, c = triangles[:, 0], triangles[:, 1], triangles[:, 2]
    normal = np.cross(b-a, c-a)
    length = np.linalg.norm(normal, axis=1)
    valid = length > 1e-12
    unit = np.divide(normal, length[:, None], out=np.zeros_like(normal), where=valid[:, None])
    height = np.sum((point-a)*unit, axis=1)
    projection = point-height[:, None]*unit
    sides = np.column_stack([np.sum(np.cross(end-start, projection-start)*unit, axis=1)
                             for start, end in ((a, b), (b, c), (c, a))])
    if (valid & (np.abs(height) <= tolerance) & (sides >= -1e-14).all(axis=1)).any():
        return True
    for start, end in ((a, b), (b, c), (c, a)):
        edge = end-start
        length2 = np.sum(edge**2, axis=1)
        along = np.divide(np.sum((point-start)*edge, axis=1), length2,
                          out=np.zeros(len(edge)), where=length2 > 0)
        nearest = start+np.clip(along, 0, 1)[:, None]*edge
        if (np.linalg.norm(point-nearest, axis=1) <= tolerance).any():
            return True
    return False


def surface_reference(point, body, tolerance):
    local = body.inverse[:3, :3] @ point+body.inverse[:3, 3]
    if body.kind == 'mesh':
        if not ((local >= body.dimensions[0]-tolerance)
                & (local <= body.dimensions[1]+tolerance)).all():
            return False
        return mesh_surface_reference(local, body.triangles, tolerance)
    if body.kind == 'box':
        half = body.dimensions/2
        nearest = np.clip(local, -half, half)
        if (np.abs(local) <= half).all():
            return bool(np.min(half-np.abs(local)) <= tolerance)
        return bool(np.linalg.norm(local-nearest) <= tolerance)
    if body.kind == 'sphere':
        return bool(abs(np.linalg.norm(local)-body.dimensions[0]) <= tolerance)
    if body.kind == 'cylinder':
        radial, axial = np.linalg.norm(local[:2])-body.dimensions[0], abs(local[2])-body.dimensions[1]/2
        if radial <= 0 and axial <= 0:
            return bool(min(-radial, -axial) <= tolerance)
        return bool(np.hypot(max(radial, 0), max(axial, 0)) <= tolerance)
    raise ValueError('unrecognized supplied body')


def reference_cells(indices, depth, k, camera, bodies, config):
    lower, upper = np.array(config['workspace_min']), np.array(config['workspace_max'])
    shape = np.ceil((upper-lower)/config['voxel_size']).astype(int)
    step = (upper-lower)/shape
    cache, result = {}, []
    for index in indices:
        vertices = lower+(index+CORNERS)*step
        optical = (vertices-camera[:3, 3]) @ camera[:3, :3]
        z = optical[:, 2]
        if z.min() <= config['depth_min']:
            result.append(False);continue
        u, v = optical[:, 0]/z*k[0, 0]+k[0, 2], optical[:, 1]/z*k[1, 1]+k[1, 2]
        x0, x1 = int(np.floor(u.min())-1), int(np.ceil(u.max())+1)
        y0, y1 = int(np.floor(v.min())-1), int(np.ceil(v.max())+1)
        if min(x0, y0) < 0 or x1 >= depth.shape[1] or y1 >= depth.shape[0]:
            result.append(False);continue
        good, blocked = True, False
        for row in range(y0, y1+1):
            for col in range(x0, x1+1):
                d = depth[row, col]
                usable = config['depth_max'] if config.get('far_no_return') and np.isposinf(d) else d
                if np.isfinite(usable) and usable+config.get('visibility_roundoff_m', 0.) > z.max()+config['visibility_margin']:
                    continue
                blocked = True
                if (row, col) not in cache:
                    if not np.isfinite(d):
                        cache[row, col] = False
                    else:
                        point = camera[:3, :3] @ np.array([(col-k[0, 2])*d/k[0, 0],
                                                           (row-k[1, 2])*d/k[1, 1], d])+camera[:3, 3]
                        cache[row, col] = any(surface_reference(point, body, config['self_surface_tolerance']) for body in bodies)
                if not cache[row, col]:
                    good = False;break
            if not good:break
        result.append(good and blocked)
    return np.array(result, dtype=bool), len(cache)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--geometry-config', required=True)
    parser.add_argument('--baseline', required=True, help='JSON list of frozen raw lower/upper AABBs')
    parser.add_argument('--view', nargs=2, action='append', required=True, metavar=('CONFIG', 'BAG'))
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    config = params(args.geometry_config)
    if not 0 < config.get('self_surface_tolerance', 0.) <= .003:
        parser.error('explicit bounded surface tolerance required')
    records = [read_view(path, bag, config) for path, bag in args.view]
    estimates = [estimate(v[1], v[2], v[3], v[4], v[5], v[0]) for v in records]
    observed, unknown, _ = fuse_estimates(estimates, [v[0] for v in records])
    boxes = [Bounds(np.array(b['lower']), np.array(b['upper'])) for b in json.loads(Path(args.baseline).read_text())]
    baseline = occupancy(boxes, config, outward=True)
    positive, raw = occupancy(observed, config, outward=True), occupancy(unknown, config)
    indices = np.argwhere(raw & ~baseline & ~positive)
    proven, audits = np.zeros(len(indices), bool), []
    for c, depth, k, camera, bodies, target, source in records:
        bodies = known_occluders(bodies, target, c)
        production = witnessed_cells(indices, depth, k, camera, bodies, c)
        reference, pixels = reference_cells(indices, depth, k, camera, bodies, c)
        if not np.array_equal(production, reference):
            raise AssertionError('production explanation differs from independent pixel/triangle reference')
        proven |= production
        audits.append({**source, 'view': c['camera_frame'], 'candidate_cells': len(indices),
                       'witnessed_cells': int(production.sum()), 'reference_pixels': pixels,
                       'independent_reference_matches': True})
    if not np.array_equal(raw, occupancy(unknown, config)):
        raise AssertionError('explanation mutated raw unknown geometry')
    result = {'scope': 'offline separately recorded stopped-state views; not live motion proof',
              'baseline_sha256': hashlib.sha256(Path(args.baseline).read_bytes()).hexdigest(),
              'known_target_surface_enabled': config.get('monitor_target_occlusion', False),
              'new_positive_cells': int((positive & ~baseline).sum()), 'new_unknown_cells': len(indices),
              'explained_indices': indices[proven].tolist(), 'unexplained_indices': indices[~proven].tolist(),
              'unknown_geometry_unchanged': True, 'views': audits}
    Path(args.output).write_text(json.dumps(result, indent=2))
    print(f'PASS reference agreement for {len(indices)} cells in {len(records)} views; '
          f'{int(proven.sum())} explained; new positive cells={result["new_positive_cells"]}')

if __name__ == '__main__':main()
