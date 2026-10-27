"""Explain monitoring-only unknown cells using capture-time known surfaces.

No depth is replaced and no planning geometry is removed. Every pixel in a
cell's complete projected rectangle must prove free space or a finite return
on a supplied robot surface. Missing depth, outside-frame and other occluders
cannot establish this explanation. Positive environment observations always
remain exclusions and must be checked separately by the caller.
"""
import numpy as np
from .obstacle_geometry import (transform, visibility_depth, projected_workspace,
                                rectangle_minima)


def known_occluders(bodies, target, config):
    """Optional target uses its supplied pose/size, without mask padding.

    This is valid only for an independently supplied target-pose contract,
    not an assumption that a moving object stays at its initial task pose.
    """
    if not config.get('monitor_target_occlusion', False):
        return bodies
    from .self_filter import Body
    return [*bodies, Body('separately_known_target', np.eye(4), 'box',
                          np.asarray(config['target_size'])).at(target)]


def witnessed_cells(indices, depth, k, camera, bodies, config):
    indices = np.asarray(indices, dtype=int)
    proven = np.zeros(len(indices), dtype=bool)
    if not len(indices):
        return proven
    lower, upper = np.array(config['workspace_min']), np.array(config['workspace_max'])
    shape = np.ceil((upper-lower)/config['voxel_size']).astype(int)
    projected, near, far = projected_workspace(tuple(lower), tuple(upper), config['voxel_size'],
                                               tuple(k.flat), tuple(camera.flat))
    flat = np.ravel_multi_index(indices.T, tuple(shape))
    rectangles, z_min, z_max = projected[flat], near[flat], far[flat]
    valid = ((z_min > config['depth_min']) & np.isfinite(rectangles).all(axis=1)
             & (rectangles[:, 0] >= 0) & (rectangles[:, 2] >= 0)
             & (rectangles[:, 1] < depth.shape[1]) & (rectangles[:, 3] < depth.shape[0]))
    selected = np.flatnonzero(valid)
    usable = visibility_depth(depth, config)
    rounding = config.get('visibility_roundoff_m', 0.)
    pixels = np.zeros(depth.shape, dtype=bool)
    for i in selected:
        x0, x1, y0, y1 = rectangles[i].astype(int)
        crop = np.s_[y0:y1+1, x0:x1+1]
        free = np.isfinite(usable[crop]) & (usable[crop]+rounding > z_max[i]+config['visibility_margin'])
        pixels[crop] |= ~free
    # Only finite acquired returns can be robot-surface witnesses. A far/no-
    # return pixel may prove free within range, but never identifies the robot.
    rows, cols = np.where(pixels & np.isfinite(depth))
    values = depth[rows, cols]
    points = np.column_stack(((cols-k[0, 2])*values/k[0, 0],
                              (rows-k[1, 2])*values/k[1, 1], values))
    points = transform(points, camera)
    matches = np.zeros(len(points), dtype=bool)
    for body in bodies:
        candidates = np.flatnonzero(~matches)
        if len(candidates):
            matches[candidates] |= body.matches_surface(points[candidates], config['self_surface_tolerance'])
    robot = np.zeros(depth.shape, dtype=bool)
    robot[rows, cols] = matches
    if len(selected):
        # A finite sentinel above every cell replaces witnessed robot pixels
        # only for this reduction. Raw depth and planning geometry stay intact.
        # Every remaining pixel must still prove free, and a robot return must
        # explain at least one non-free pixel in the complete rectangle.
        sentinel = max(config['depth_max'], float(z_max[selected].max())+
                       config['visibility_margin'])+abs(rounding)+1.
        non_robot = np.where(robot, sentinel, usable)
        robot_depth = np.where(robot, usable, sentinel)
        rectangles = rectangles[selected].astype(int)
        threshold = z_max[selected]+config['visibility_margin']
        all_witnessed = rectangle_minima(non_robot, rectangles)+rounding > threshold
        robot_required = rectangle_minima(robot_depth, rectangles)+rounding <= threshold
        proven[selected] = all_witnessed & robot_required
    return proven
