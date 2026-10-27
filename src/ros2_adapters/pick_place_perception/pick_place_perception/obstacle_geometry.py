"""Backend-independent depth geometry. All lengths are metres.

Boxes fitted to samples are explicitly partial observations. Unknown workspace
voxels are retained as collision boxes, rather than inferred to be empty.
"""

from dataclasses import dataclass
from functools import lru_cache
import math
import cv2
import numpy as np
from scipy.spatial import cKDTree

from .rgbd_contract import InputContractError


@dataclass(frozen=True)
class Bounds:
    lower: np.ndarray
    upper: np.ndarray

    @property
    def size(self):
        return self.upper - self.lower

    @property
    def center(self):
        return (self.lower + self.upper) / 2


def matrix(translation, quaternion):
    q = np.asarray(quaternion, dtype=float)
    t = np.asarray(translation, dtype=float)
    if q.shape != (4,) or t.shape != (3,) or not np.isfinite(t).all() or not np.isfinite(q).all():
        raise InputContractError('invalid rigid transform')
    if abs(np.linalg.norm(q) - 1) > 1e-5:
        raise InputContractError('transform quaternion must be normalized')
    x, y, z, w = q
    result = np.eye(4)
    result[:3, :3] = [[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]]
    result[:3, 3] = t
    return result


def transform(points, pose):
    return np.asarray(points) @ pose[:3, :3].T + pose[:3, 3]


def stamp_ns(message):
    return message.header.stamp.sec * 10**9 + message.header.stamp.nanosec


def decode_depth(message, encoding, scale, minimum, maximum, far_no_return=False):
    if message.encoding != encoding or encoding not in ('16UC1', '32FC1'):
        raise InputContractError('unexpected depth encoding')
    dtype = np.dtype(('>' if message.is_bigendian else '<') + ('u2' if encoding == '16UC1' else 'f4'))
    if (message.width <= 0 or message.height <= 0 or message.step < message.width*dtype.itemsize
            or len(message.data) < message.step*message.height):
        raise InputContractError('invalid depth dimensions, stride or payload')
    depth = np.ndarray((message.height, message.width), dtype=dtype, buffer=message.data,
                       strides=(message.step, dtype.itemsize)).astype(float) * scale
    far = np.isposinf(depth) if far_no_return and encoding == '32FC1' else None
    depth[~np.isfinite(depth) | (depth <= minimum) | (depth >= maximum)] = np.nan
    if far is not None:
        depth[far] = np.inf
    return depth


def visibility_depth(depth, config):
    """Explicit REP-117/118 no-return contract, bounded by sensor range.

    Keep +Inf out of observed point geometry; only its free-space witness ends
    at depth_max. NaN, -Inf, zero and missing/out-of-frame data remain unknown.
    """
    if config.get('far_no_return', False):
        return np.where(np.isposinf(depth), config['depth_max'], depth)
    return depth


def validate_bundle(rgb, depth, color_info, depth_info, config):
    messages = (rgb, depth, color_info, depth_info)
    times = [stamp_ns(msg) for msg in messages]
    if min(times) <= 0 or max(times)-min(times) > config['max_time_skew']*1e9:
        raise InputContractError('RGB/depth/CameraInfo timestamp mismatch')
    if any(msg.header.frame_id != config['camera_frame'] for msg in messages):
        raise InputContractError('registered RGB/depth optical frames must agree')
    if any((msg.width, msg.height) != (depth.width, depth.height) for msg in messages):
        raise InputContractError('registered RGB/depth resolutions must agree')
    if rgb.encoding != 'rgb8' or rgb.step < rgb.width*3 or len(rgb.data) < rgb.step*rgb.height:
        raise InputContractError('expected a complete rgb8 image')
    k = np.asarray(color_info.k).reshape(3, 3)
    if (not np.isfinite(k).all() or k[0, 0] <= 0 or k[1, 1] <= 0
            or not np.allclose(k, np.asarray(depth_info.k).reshape(3, 3), rtol=0, atol=1e-8)
            or not np.allclose(k[2], [0, 0, 1]) or k[0, 1] != 0 or k[1, 0] != 0):
        raise InputContractError('invalid or unregistered intrinsics')
    for info in (color_info, depth_info):
        if (any(not math.isfinite(x) or abs(x) > 1e-10 for x in info.d)
                or info.binning_x > 1 or info.binning_y > 1
                or info.roi.x_offset or info.roi.y_offset or info.roi.width or info.roi.height):
            raise InputContractError('requires rectified full-frame images without binning/ROI')
    return k


def validate_config(c):
    if c.get('sensor_reliability', 'best_effort') not in ('best_effort', 'reliable'):
        raise InputContractError('sensor_reliability must be best_effort or reliable')
    """Explicit portable configuration; no implicit robot, camera or workspace."""
    required = ('rgb_image_topic', 'depth_image_topic', 'camera_info_topic', 'depth_info_topic',
                'joint_states_topic', 'target_pose_topic', 'camera_frame', 'output_frame',
                'depth_encoding', 'depth_scale', 'max_time_skew', 'max_input_age',
                'depth_min', 'depth_max', 'workspace_min', 'workspace_max', 'target_size',
                'self_filter_model', 'voxel_size', 'sample_stride', 'cluster_distance',
                'min_cluster_points', 'min_box_size', 'max_box_size', 'max_box_volume',
                'plane_threshold', 'plane_min_points', 'plane_iterations',
                'visibility_margin', 'max_collision_boxes')
    required += ('target_mask_padding', 'min_obstacle_height')
    missing = [name for name in required if name not in c or c[name] is None]
    if missing:
        raise InputContractError('missing obstacle parameters: ' + ', '.join(missing))
    for name in ('rgb_image_topic', 'depth_image_topic', 'camera_info_topic', 'depth_info_topic',
                 'joint_states_topic', 'target_pose_topic'):
        if not isinstance(c[name], str) or not c[name].startswith('/') or c[name] == '/':
            raise InputContractError(f'{name} must be an absolute topic')
    for name in ('camera_frame', 'output_frame', 'self_filter_model'):
        if not isinstance(c[name], str) or not c[name].strip():
            raise InputContractError(f'{name} is required')
    if c['camera_frame'].startswith('/') or c['output_frame'].startswith('/'):
        raise InputContractError('TF frames must not start with /')
    if c['depth_encoding'] not in ('16UC1', '32FC1'):
        raise InputContractError('unsupported depth encoding')
    for name in ('depth_scale', 'max_time_skew', 'max_input_age', 'depth_min', 'depth_max',
                 'voxel_size', 'cluster_distance', 'max_box_volume', 'plane_threshold',
                 'min_obstacle_height'):
        if isinstance(c[name], bool) or not isinstance(c[name], (int, float)) or not math.isfinite(c[name]) or c[name] <= 0:
            raise InputContractError(f'{name} must be finite and positive')
    margin = c['visibility_margin']
    rounding = c.get('visibility_roundoff_m', 0.)
    if (isinstance(margin, bool) or not isinstance(margin, (int, float))
            or not math.isfinite(margin) or margin < 0
            or isinstance(rounding, bool) or not isinstance(rounding, (int, float))
            or not math.isfinite(rounding) or not 0 <= rounding <= 1e-6):
        raise InputContractError('visibility margin must be nonnegative; roundoff must be in [0, 1e-6] m')
    if not isinstance(c.get('far_no_return', False), bool) or (c.get('far_no_return') and c['depth_encoding'] != '32FC1'):
        raise InputContractError('far_no_return requires an explicit bool and 32FC1 depth')
    if (isinstance(c['target_mask_padding'], bool) or not isinstance(c['target_mask_padding'], (int, float))
            or not math.isfinite(c['target_mask_padding']) or not 0 <= c['target_mask_padding'] <= .02):
        raise InputContractError('target_mask_padding must be in [0, .02] metres')
    tolerance = c.get('self_surface_tolerance', 0.)
    if (isinstance(tolerance, bool) or not isinstance(tolerance, (int, float))
            or not math.isfinite(tolerance) or not 0 <= tolerance <= .003):
        raise InputContractError('self_surface_tolerance must be in [0, .003] metres')
    for name in ('sample_stride', 'min_cluster_points', 'plane_min_points', 'plane_iterations', 'max_collision_boxes'):
        if isinstance(c[name], bool) or not isinstance(c[name], int) or not 1 <= c[name] <= 100000:
            raise InputContractError(f'{name} must be a bounded positive integer')
    for name in ('workspace_min', 'workspace_max', 'target_size', 'min_box_size', 'max_box_size'):
        a = np.asarray(c[name], dtype=float)
        if a.shape != (3,) or not np.isfinite(a).all():
            raise InputContractError(f'{name} requires three finite lengths')
        if name in ('target_size', 'min_box_size', 'max_box_size') and np.any(a <= 0):
            raise InputContractError(f'{name} requires positive lengths')
    if (c['depth_min'] >= c['depth_max'] or np.any(np.array(c['workspace_max']) <= c['workspace_min'])
            or np.any(np.array(c['max_box_size']) < c['min_box_size'])):
        raise InputContractError('invalid depth, workspace or box bounds')
    support_enabled = c.get('support_surface_enabled', False)
    if not isinstance(support_enabled, bool):
        raise InputContractError('support_surface_enabled must be bool')
    if support_enabled:
        support_required = ('support_surface_height', 'support_surface_normal',
                            'support_surface_xy_min', 'support_surface_xy_max',
                            'support_contact_height_tolerance',
                            'support_uncertainty_budget_m')
        missing = [name for name in support_required if name not in c or c[name] is None]
        if missing:
            raise InputContractError('missing support surface parameters: ' + ', '.join(missing))
        height = c['support_surface_height']
        tolerance = c['support_contact_height_tolerance']
        budget = c['support_uncertainty_budget_m']
        for name, value in (('support_surface_height', height),
                            ('support_contact_height_tolerance', tolerance),
                            ('support_uncertainty_budget_m', budget)):
            if (isinstance(value, bool) or not isinstance(value, (int, float))
                    or not math.isfinite(value) or value < 0):
                raise InputContractError(f'{name} must be finite and nonnegative')
        if budget <= 0 or tolerance > budget:
            raise InputContractError('support contact tolerance must not exceed uncertainty budget')
        normal = np.asarray(c['support_surface_normal'], dtype=float)
        if normal.shape != (3,) or not np.isfinite(normal).all() or abs(np.linalg.norm(normal)-1) > 1e-6:
            raise InputContractError('support_surface_normal must be a unit vector')
        # The current workspace and collision contract support a horizontal plane.
        if np.linalg.norm(normal[:2]) > 1e-6 or normal[2] <= 0:
            raise InputContractError('support surface must use the +Z normal')
        xy_min = np.asarray(c['support_surface_xy_min'], dtype=float)
        xy_max = np.asarray(c['support_surface_xy_max'], dtype=float)
        if (xy_min.shape != (2,) or xy_max.shape != (2,) or not np.isfinite(xy_min).all()
                or not np.isfinite(xy_max).all() or np.any(xy_max <= xy_min)):
            raise InputContractError('support surface XY bounds are invalid')
        workspace_min = np.asarray(c['workspace_min'], dtype=float)
        workspace_max = np.asarray(c['workspace_max'], dtype=float)
        if (height < workspace_min[2] - tolerance - 1e-9
                or height + tolerance > workspace_max[2] + 1e-9
                or np.any(xy_min < workspace_min[:2] - 1e-9)
                or np.any(xy_max > workspace_max[:2] + 1e-9)):
            raise InputContractError('support surface lies outside the configured workspace')
    shape = np.ceil((np.array(c['workspace_max'])-c['workspace_min']) / c['voxel_size'])
    if np.prod(shape) > 250000:
        raise InputContractError('workspace visibility grid exceeds 250000 cells')


def points_from_depth(depth, k, stride):
    v, u = np.mgrid[0:depth.shape[0]:stride, 0:depth.shape[1]:stride]
    z = depth[::stride, ::stride]
    with np.errstate(invalid='ignore'):
        points = np.stack(((u-k[0, 2])*z/k[0, 0], (v-k[1, 2])*z/k[1, 1], z), axis=-1)
    return points[np.isfinite(points).all(axis=-1)]


def remove_plane(points, config):
    """Fit only a horizontal workspace support, never a vertical thin wall."""
    if config.get('support_surface_enabled', False):
        # An explicitly supplied support already fixes its plane. Re-fitting
        # it from a robot-occluded cloud can tilt the estimate and leave
        # support points connected to a wall, changing its observed AABB.
        # Use only the declared XY extent and the tighter existing tolerance;
        # raw depth and the independent unknown-space proof remain untouched.
        tolerance = min(config['plane_threshold'], config['support_contact_height_tolerance'])
        support = ((np.abs(points[:, 2]-config['support_surface_height']) <= tolerance)
                   & (points[:, :2] >= config['support_surface_xy_min']).all(axis=1)
                   & (points[:, :2] <= config['support_surface_xy_max']).all(axis=1))
        return points[~support], int(support.sum())
    floor = config['workspace_min'][2]
    best = np.zeros(len(points), dtype=bool)
    rng = np.random.default_rng(0)
    if len(points) >= 3:
        # Subsample scoring; refit and classify the original cloud afterwards.
        sample = points[rng.choice(len(points), min(len(points), 3000), replace=False)]
        best_count, model = 0, None
        for _ in range(config['plane_iterations']):
            a, b, c = sample[rng.choice(len(sample), 3, replace=False)]
            normal = np.cross(b-a, c-a)
            length = np.linalg.norm(normal)
            if length < 1e-10:
                continue
            normal /= length
            d = -normal @ a
            if abs(normal[2]) < math.cos(math.radians(8)) or abs(-d/normal[2]-floor) > config['plane_threshold']:
                continue
            count = np.count_nonzero(np.abs(sample @ normal + d) <= config['plane_threshold'])
            if count > best_count:
                best_count, model = count, (normal, d)
        if model is not None:
            best = np.abs(points @ model[0] + model[1]) <= config['plane_threshold']
            if best.sum() < config['plane_min_points']:
                best[:] = False
    return points[~best], int(best.sum())


def fit_clusters(points, config):
    if not len(points):
        return [], 0
    # Voxel representatives bound work without linking obstacles through a support plane.
    _, indices = np.unique(np.floor(points / (config['cluster_distance']/3)).astype(int), axis=0, return_index=True)
    sample = points[indices]
    tree = cKDTree(sample)
    visited = np.zeros(len(sample), dtype=bool)
    boxes, rejected = [], 0
    for start in range(len(sample)):
        if visited[start]:
            continue
        component, queue = [], [start]
        visited[start] = True
        while queue:
            i = queue.pop()
            component.append(i)
            for j in tree.query_ball_point(sample[i], config['cluster_distance']):
                if not visited[j]:
                    visited[j] = True
                    queue.append(j)
        cloud = sample[component]
        box = Bounds(cloud.min(axis=0), cloud.max(axis=0))
        if (len(cloud) < config['min_cluster_points'] or np.any(box.size > config['max_box_size'])
                or box.upper[2] < config['workspace_min'][2]+config['min_obstacle_height']
                or np.prod(np.maximum(box.size, config['min_box_size'])) > config['max_box_volume']):
            rejected += len(cloud)
            continue
        # A planar observation still needs a positive collision thickness. This
        # is numerical thickness, not a claim about the hidden physical volume.
        size = np.maximum(box.size, config['min_box_size'])
        boxes.append(Bounds(box.center-size/2, box.center+size/2))
    return sorted(boxes, key=lambda b: tuple(b.center)), rejected


CORNERS = np.array([[x, y, z] for x in (0, 1) for y in (0, 1) for z in (0, 1)])


@lru_cache(maxsize=2)
def workspace_cells(lower, upper, voxel_size):
    """Immutable workspace geometry only; no sensor or robot state is cached."""
    lower, upper = np.asarray(lower), np.asarray(upper)
    shape = np.ceil((upper-lower)/voxel_size).astype(int)
    steps = (upper-lower)/shape
    indices = np.indices(shape).reshape(3, -1).T
    corners = lower+(indices[:, None, :]+CORNERS)*steps
    for array in (shape, steps, indices, corners):
        array.setflags(write=False)
    return shape, steps, indices, corners


@lru_cache(maxsize=8)
def projected_workspace(lower, upper, voxel_size, intrinsics, camera_pose):
    """Reuse projections only for exactly identical supplied calibration/TF."""
    _, _, _, corners = workspace_cells(lower, upper, voxel_size)
    k, pose = np.asarray(intrinsics).reshape(3, 3), np.asarray(camera_pose).reshape(4, 4)
    camera = transform(corners.reshape(-1, 3), np.linalg.inv(pose)).reshape(-1, 8, 3)
    z = camera[:, :, 2]
    with np.errstate(divide='ignore', invalid='ignore'):
        u = camera[:, :, 0]*k[0, 0]/z+k[0, 2]
        v = camera[:, :, 1]*k[1, 1]/z+k[1, 2]
        rectangles = np.column_stack((np.floor(u.min(axis=1))-1, np.ceil(u.max(axis=1))+1,
                                      np.floor(v.min(axis=1))-1, np.ceil(v.max(axis=1))+1))
    result = (rectangles, z.min(axis=1), z.max(axis=1))
    for array in result:
        array.setflags(write=False)
    return result


@lru_cache(maxsize=8)
def camera_rays(image_shape, intrinsics, rotation):
    k, rotation = np.asarray(intrinsics).reshape(3, 3), np.asarray(rotation).reshape(3, 3)
    v, u = np.mgrid[:image_shape[0], :image_shape[1]]
    optical = np.stack(((u-k[0, 2])/k[0, 0], (v-k[1, 2])/k[1, 1], np.ones_like(u)), axis=-1)
    rays = optical @ rotation.T
    rays.setflags(write=False)
    return rays


def rectangle_minima(depth, rectangles):
    """Exact inclusive rectangle minima; nonfinite samples invalidate a ray.

    Four overlapping power-of-two blocks cover each rectangle exactly. Grouped
    OpenCV erosions avoid a Python per-voxel image scan, without weakening the
    full projected rectangle condition.
    """
    x0, x1, y0, y1 = rectangles.T
    widths, heights = x1-x0+1, y1-y0+1
    powers = np.column_stack((np.floor(np.log2(heights)), np.floor(np.log2(widths)))).astype(int)
    source = np.where(np.isfinite(depth), depth, -np.inf).astype(np.float64)
    result = np.empty(len(rectangles), dtype=float)
    for ph, pw in np.unique(powers, axis=0):
        selected = np.flatnonzero((powers == [ph, pw]).all(axis=1))
        height, width = 1 << ph, 1 << pw
        minimum = cv2.erode(source, np.ones((height, width), np.uint8), anchor=(0, 0),
                            borderType=cv2.BORDER_CONSTANT, borderValue=-float('inf'))
        left, right = x0[selected], x1[selected]-width+1
        top, bottom = y0[selected], y1[selected]-height+1
        result[selected] = np.minimum.reduce((minimum[top, left], minimum[top, right],
                                               minimum[bottom, left], minimum[bottom, right]))
    return result


def boxes_from_occupancy(occupied, lower, upper):
    """Greedy rectangular cover of EXACTLY the occupied cells, with no holes.

    Grow X, then Y, then Z; shallow horizontal support-adjacent cells become a
    slab while tall occlusion columns remain excluded. This changes only the
    partition of occupied space, not its extent or classification.
    """
    remaining = occupied.copy()
    shape = np.array(remaining.shape)
    step = (np.asarray(upper)-lower)/shape
    boxes = []
    for x, y, z in np.argwhere(occupied):
        if not remaining[x, y, z]:
            continue
        xe, ye, ze = x+1, y+1, z+1
        while xe < shape[0] and remaining[xe, y, z]:
            xe += 1
        while ye < shape[1] and remaining[x:xe, ye, z].all():
            ye += 1
        while ze < shape[2] and remaining[x:xe, y:ye, ze].all():
            ze += 1
        remaining[x:xe, y:ye, z:ze] = False
        boxes.append(Bounds(lower+np.array([x, y, z])*step, lower+np.array([xe, ye, ze])*step))
    return boxes


def support_contact_cells(indices, lower, steps, config):
    """Select contact-layer candidates; position alone is never a witness."""
    if not config.get('support_surface_enabled', False) or not len(indices):
        return np.zeros(len(indices), dtype=bool)
    cell_lower = lower + indices * steps
    cell_upper = cell_lower + steps
    xy_min = np.asarray(config['support_surface_xy_min'], dtype=float)
    xy_max = np.asarray(config['support_surface_xy_max'], dtype=float)
    height = float(config['support_surface_height'])
    tolerance = float(config['support_contact_height_tolerance'])
    inside_xy = ((cell_lower[:, :2] >= xy_min - 1e-9)
                 & (cell_upper[:, :2] <= xy_max + 1e-9)).all(axis=1)
    # A voxel may straddle the mathematical plane even when its thickness is
    # larger than the measured contact-height tolerance. Clear that one cell,
    # but never cells wholly above the tolerance band.
    in_contact_layer = ((cell_lower[:, 2] <= height + tolerance + 1e-9)
                        & (cell_upper[:, 2] >= height - tolerance - 1e-9))
    return inside_xy & in_contact_layer


@lru_cache(maxsize=8)
def support_rays(image_shape, intrinsics, camera_pose, height, xy_min, xy_max, depth_min):
    """Cache only fixed ray/support geometry, never a depth or free-space claim."""
    pose = np.asarray(camera_pose).reshape(4, 4)
    rays = camera_rays(image_shape, intrinsics, tuple(pose[:3, :3].flat))
    origin = pose[:3, 3]
    with np.errstate(divide='ignore', invalid='ignore'):
        plane = (height-origin[2])/rays[:, :, 2]
        hits = origin+plane[:, :, None]*rays
        eligible = (np.isfinite(plane) & (plane > depth_min) & (rays[:, :, 2] < -1e-12)
                    & ((hits[:, :, :2] >= np.asarray(xy_min)-1e-9)
                       & (hits[:, :, :2] <= np.asarray(xy_max)+1e-9)).all(axis=-1))
    plane.setflags(write=False)
    eligible.setflags(write=False)
    return rays, plane, eligible


def support_free_cells(depth, k, camera_pose, cell_lower, cell_upper, rectangles, config):
    """Require a depth witness for every sampled ray intersecting each cell.

    At a separately modelled support, a return may coincide with the ray's
    exit at the plane instead of lying strictly beyond it. Only finite returns
    within the explicit normal-height tolerance qualify for this contact rule.
    A nearer target/obstacle, missing data or a ray going behind the support
    remains unknown. The caller must model the same support-height envelope
    in PlanningScene; this function never permits a target/obstacle collision.
    """
    result = np.zeros(len(cell_lower), dtype=bool)
    origin = camera_pose[:3, 3]
    height = config['support_surface_height']
    tolerance = config['support_contact_height_tolerance']
    rounding = config.get('visibility_roundoff_m', 0.)
    margin = config['visibility_margin']
    all_rays, all_plane, eligible = support_rays(
        depth.shape, tuple(k.flat), tuple(camera_pose.flat), height,
        tuple(config['support_surface_xy_min']), tuple(config['support_surface_xy_max']), config['depth_min'])
    with np.errstate(divide='ignore', invalid='ignore'):
        plane_valid = (eligible & np.isfinite(depth)
                       & (np.abs((depth-all_plane)*all_rays[:, :, 2]) <= tolerance+rounding))
    # Most support candidates see only the plane. This full-rectangle proof
    # is a strict subset of the ray proof, so skip no evidence when batching it.
    uniform_plane = rectangle_minima(plane_valid.astype(float), rectangles.astype(int)) == 1
    uniform_plane &= cell_lower[:, 2] >= height-1e-12
    result[uniform_plane] = True
    pending = np.flatnonzero(~result)
    cursor = 0
    # Ragged rectangles become bounded batches of (cell, pixel) pairs. All
    # intersecting rays are retained; the batching changes no classification.
    while cursor < len(pending):
        stop, pixels = cursor, 0
        while stop < len(pending):
            rect = rectangles[pending[stop]].astype(int)
            count = (rect[1]-rect[0]+1)*(rect[3]-rect[2]+1)
            if stop > cursor and pixels+count > 65536:
                break
            pixels += count
            stop += 1
        selected = pending[cursor:stop]
        rects = rectangles[selected].astype(int)
        widths, heights = rects[:, 1]-rects[:, 0]+1, rects[:, 3]-rects[:, 2]+1
        counts = widths*heights
        owner = np.repeat(np.arange(len(selected)), counts)
        offsets = np.arange(int(counts.sum()))-np.repeat(np.cumsum(counts)-counts, counts)
        u = rects[owner, 0]+offsets % widths[owner]
        v = rects[owner, 2]+offsets // widths[owner]
        rays = all_rays[v, u]
        lo, hi = cell_lower[selected][owner], cell_upper[selected][owner]
        parallel = np.abs(rays) < 1e-12
        with np.errstate(divide='ignore', invalid='ignore'):
            a, b = (lo-origin)/rays, (hi-origin)/rays
            entry = np.where(parallel, -np.inf, np.minimum(a, b)).max(axis=-1)
            exit = np.where(parallel, np.inf, np.maximum(a, b)).min(axis=-1)
        outside = (parallel & ((origin < lo) | (origin > hi))).any(axis=-1)
        intersects = ~outside & (exit >= entry) & (exit > config['depth_min'])
        usable = visibility_depth(depth[v, u], config)
        free = np.isfinite(usable) & (usable+rounding > exit+margin)
        contact = plane_valid[v, u] & (exit <= all_plane[v, u]+rounding+1e-12)
        bad = np.bincount(owner, weights=(intersects & ~(free | contact)), minlength=len(selected))
        covered = np.bincount(owner, weights=intersects, minlength=len(selected))
        result[selected] = (bad == 0) & (covered > 0)
        cursor = stop
    return result


def unknown_boxes(depth, k, camera_pose, config, known_inside, known_inside_many=None,
                  candidate_occupancy=None):
    """Exclude every voxel not proven free by its full projected pixel rectangle.

    Behind-surface, out-of-view and missing-depth cells remain unknown. A known
    robot/target volume is exempt only if all eight corners lie within ONE body.
    No free-space ray is extended behind a masked robot or target pixel.
    """
    lower, upper = np.array(config['workspace_min']), np.array(config['workspace_max'])
    shape, steps, indices, corners = workspace_cells(tuple(lower), tuple(upper), config['voxel_size'])
    rectangles, z_min, z_max = projected_workspace(tuple(lower), tuple(upper), config['voxel_size'],
                                                 tuple(k.flat), tuple(camera_pose.flat))
    if candidate_occupancy is not None:
        if candidate_occupancy.shape != tuple(shape) or candidate_occupancy.dtype != bool:
            raise ValueError('candidate occupancy must match the workspace grid')
        candidates = candidate_occupancy.reshape(-1)
        indices, corners = indices[candidates], corners[candidates]
        rectangles, z_min, z_max = rectangles[candidates], z_min[candidates], z_max[candidates]
    occupied = np.ones(len(corners), dtype=bool)
    valid = ((z_min > config['depth_min']) & np.isfinite(rectangles).all(axis=1)
             & (rectangles[:, 0] >= 0) & (rectangles[:, 2] >= 0)
             & (rectangles[:, 1] < depth.shape[1]) & (rectangles[:, 3] < depth.shape[0]))
    selected = np.flatnonzero(valid)
    if len(selected):
        minima = rectangle_minima(visibility_depth(depth, config), rectangles[selected].astype(int))
        occupied[selected] = ~(minima+config.get('visibility_roundoff_m', 0.) >
                              z_max[selected]+config['visibility_margin'])
    # A complete free-space witness already excludes these cells. Evaluate
    # known-volume exemptions only for the remaining unknown cells; the
    # result is the same union of the two independent proofs.
    selected = np.flatnonzero(occupied)
    if len(selected):
        pending = corners[selected]
        exempt = (np.array([known_inside(cell) for cell in pending], dtype=bool)
                  if known_inside_many is None else known_inside_many(pending))
        occupied[selected[exempt]] = False
    # A declared support does not make missing or occluded cells free. Replace
    # the unconditional layer carving with per-ray, finite support witnesses.
    support = support_contact_cells(indices, lower, steps, config) & occupied & valid
    selected = np.flatnonzero(support)
    if len(selected):
        cell_lower = lower + indices[selected]*steps
        proven = support_free_cells(depth, k, camera_pose, cell_lower, cell_lower+steps,
                                    rectangles[selected], config)
        occupied[selected[proven]] = False
    if candidate_occupancy is None:
        occupied = occupied.reshape(tuple(shape))
    else:
        selected_occupancy = np.zeros(tuple(shape), dtype=bool)
        selected_occupancy[tuple(indices.T)] = occupied
        occupied = selected_occupancy
    boxes = boxes_from_occupancy(occupied, lower, upper)
    return boxes, int(occupied.sum()), int(np.prod(shape))


def estimate(depth, k, camera_pose, bodies, target_pose, config, candidate_occupancy=None):
    points = transform(points_from_depth(depth, k, config['sample_stride']), camera_pose)
    crop_min = np.array(config['workspace_min'])-np.array([0, 0, config['plane_threshold']])
    mask = ((points >= crop_min) & (points <= config['workspace_max'])).all(axis=1)
    points = points[mask]
    before = len(points)
    for body in bodies:
        points = points[~body.surface_contains(points, config.get('self_surface_tolerance', 0.))]
    target_inverse = np.linalg.inv(target_pose)
    half = np.array(config['target_size'])/2

    def in_target(p, padding=0.):
        return (np.abs(transform(p, target_inverse)) <= half+padding+1e-7).all(axis=1)

    self_removed = before-len(points)
    mask = in_target(points, config['target_mask_padding'])
    target_removed = int(mask.sum())
    points, plane_count = remove_plane(points[~mask], config)
    observed, rejected = fit_clusters(points, config)

    def known_inside(corners):
        # For a concave mesh, eight corners do not establish that the entire
        # cell is inside the body. Carve analytic convex primitives only.
        return in_target(corners).all() or any(body.kind != 'mesh' and body.contains(corners).all() for body in bodies)

    def known_inside_many(corners):
        points = corners.reshape(-1, 3)
        result = in_target(points).reshape(-1, 8).all(axis=1)
        for body in bodies:
            if body.kind != 'mesh':
                result |= body.contains(points).reshape(-1, 8).all(axis=1)
        return result

    unknown, unknown_cells, total_cells = unknown_boxes(
        depth, k, camera_pose, config, known_inside, known_inside_many, candidate_occupancy)
    if len(observed)+len(unknown) > config['max_collision_boxes']:
        raise InputContractError('collision box budget exceeded; refuse incomplete snapshot')
    grid_shape = np.ceil((np.asarray(config['workspace_max'])-
                          np.asarray(config['workspace_min'])) /
                         config['voxel_size']).astype(int)
    grid_indices = np.indices(tuple(grid_shape)).reshape(3, -1).T
    grid_steps = (np.asarray(config['workspace_max'])-np.asarray(config['workspace_min'])) / grid_shape
    support_declared = int(np.count_nonzero(support_contact_cells(
        grid_indices, np.asarray(config['workspace_min']), grid_steps, config)))
    quality = {'observed_boxes': len(observed), 'unknown_boxes': len(unknown),
               'unknown_cells': unknown_cells, 'total_cells': total_cells,
               'self_points_removed': self_removed, 'target_points_removed': target_removed,
               'plane_points_removed': plane_count, 'rejected_cluster_samples': rejected,
               'visible_bounds_complete': False, 'unknown_space_excluded': True,
               'support_surface_enabled': bool(config.get('support_surface_enabled', False)),
               'support_contact_cells_declared': support_declared}
    return observed, unknown, quality
