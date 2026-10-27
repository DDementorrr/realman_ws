#!/usr/bin/env python3
"""Geometry contracts with synthetic depth. Not a recorded-sensor acceptance."""
from copy import deepcopy
from pathlib import Path
import struct
import sys
import tempfile
import unittest
from types import SimpleNamespace as NS

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/ros2_adapters/pick_place_perception'))
from pick_place_perception.obstacle_geometry import (Bounds, decode_depth, estimate,
    fit_clusters, matrix, remove_plane, unknown_boxes, validate_bundle, validate_config)
from pick_place_perception.self_filter import Body, load_model, prepare_rays, triangle_distance_squared
from scipy.spatial import ConvexHull


def config():
    return yaml.safe_load((ROOT/'src/ros2_adapters/pick_place_perception/config/obstacles_template.yaml').read_text())['depth_obstacles']['ros__parameters']


def synthetic_depth(boxes=()):
    """Actual pinhole rays against floor and boxes, with a downward optical axis."""
    k = np.array([[200., 0, 160], [0, 200., 120], [0, 0, 1.]])
    camera = matrix([.52, 0, 1.], [1, 0, 0, 0])
    v, u = np.mgrid[:240, :320]
    directions = np.stack(((u-160)/200, -(v-120)/200, -np.ones_like(u)), axis=-1)
    depth = np.full((240, 320), .74)
    for box in boxes:
        with np.errstate(divide='ignore', invalid='ignore'):
            a, b = (box.lower-camera[:3, 3])/directions, (box.upper-camera[:3, 3])/directions
        near, far = np.minimum(a, b).max(axis=-1), np.maximum(a, b).min(axis=-1)
        depth = np.where((near > 0) & (far >= near), np.minimum(depth, near), depth)
    return depth, k, camera


class GeometryTests(unittest.TestCase):
    def test_declared_support_is_stable_under_occlusion_and_keeps_low_obstacles(self):
        c = config()
        c.update(support_surface_enabled=True, support_surface_height=.26,
                 support_surface_xy_min=[-.1, -.1], support_surface_xy_max=[.1, .1],
                 support_contact_height_tolerance=.003)
        x, y = np.meshgrid(np.linspace(-.09, .09, 20), np.linspace(-.09, .09, 20))
        support = np.column_stack((x.ravel(), y.ravel(), .26+.0029*np.sin(20*x.ravel())))
        # Keep a thin wall, 4/9 mm obstacles, and coplanar points outside the
        # supplied support. None can be deleted by a random tilted plane fit.
        obstacles = np.array([[.08, -.08, .28], [.08, 0., .35], [.08, .08, .4],
                              [0., 0., .264], [.05, 0., .269], [.2, 0., .26]])
        for visible in (support, support[::17], support[:0]):
            points = np.vstack((visible, obstacles))
            retained, removed = remove_plane(points, c)
            np.testing.assert_array_equal(retained, obstacles)
            self.assertEqual(removed, len(visible))

    def test_support_ray_cache_tracks_geometry_and_never_caches_depth_witnesses(self):
        from pick_place_perception.obstacle_geometry import support_rays, support_free_cells
        k = np.array([[100., 0., 40.], [0., 100., 40.], [0., 0., 1.]])
        camera = matrix([0., 0., 1.], [1., 0., 0., 0.])
        for height, shift, focal, bound in ((.26, 0., 100., .2), (.27, .01, 110., .15)):
            camera[0, 3] = shift
            k[0, 0] = focal
            rays, plane, eligible = support_rays((80, 80), tuple(k.flat), tuple(camera.flat),
                                                 height, (-bound, -bound), (bound, bound), .1)
            v, u = np.mgrid[:80, :80]
            reference_rays = np.stack(((u-40)/focal, -(v-40)/100., -np.ones_like(u)), axis=-1)
            reference_plane = np.full((80, 80), 1.-height)
            hits = camera[:3, 3]+reference_plane[:, :, None]*reference_rays
            reference_eligible = ((hits[:, :, :2] >= -bound-1e-9)
                                  & (hits[:, :, :2] <= bound+1e-9)).all(axis=-1)
            np.testing.assert_array_equal(rays, reference_rays)
            np.testing.assert_array_equal(plane, reference_plane)
            np.testing.assert_array_equal(eligible, reference_eligible)
            self.assertFalse(plane.flags.writeable)
            self.assertFalse(eligible.flags.writeable)
        c = config()
        c.update(support_surface_height=.27, support_surface_xy_min=[-.15, -.15],
                 support_surface_xy_max=[.15, .15], support_contact_height_tolerance=.003,
                 visibility_margin=0.)
        lo, hi = np.array([[-.02, -.02, .27]]), np.array([[.02, .02, .29]])
        rectangles = np.array([[36, 44, 36, 44]])
        clear = support_free_cells(np.full((80, 80), .73), k, camera, lo, hi, rectangles, c)
        missing = support_free_cells(np.full((80, 80), np.nan), k, camera, lo, hi, rectangles, c)
        self.assertTrue(clear[0])
        self.assertFalse(missing[0])

    def test_projection_cache_uses_exact_calibration_and_pose_without_caching_depth(self):
        from pick_place_perception.obstacle_geometry import projected_workspace, workspace_cells
        lower, upper, size = (.01, .01, .5), (.05, .05, .54), .02
        shape, steps, indices, corners = workspace_cells(lower, upper, size)
        k = np.array([[100., 0., 40.], [0., 100., 40.], [0., 0., 1.]])
        for shift, focal in ((0., 100.), (.02, 100.), (.02, 110.)):
            camera = np.eye(4);camera[0, 3] = shift;k[0, 0] = focal
            rectangles, near, far = projected_workspace(lower, upper, size, tuple(k.flat), tuple(camera.flat))
            optical = (corners-camera[:3, 3]) @ camera[:3, :3]
            u = optical[:, :, 0]/optical[:, :, 2]*focal+40.
            v = optical[:, :, 1]/optical[:, :, 2]*100.+40.
            reference = np.column_stack((np.floor(u.min(axis=1))-1, np.ceil(u.max(axis=1))+1,
                                         np.floor(v.min(axis=1))-1, np.ceil(v.max(axis=1))+1))
            self.assertTrue(np.array_equal(rectangles, reference))
            self.assertTrue(np.array_equal(near, optical[:, :, 2].min(axis=1)))
            self.assertTrue(np.array_equal(far, optical[:, :, 2].max(axis=1)))
            self.assertFalse(rectangles.flags.writeable)
        c = config()
        c.update(workspace_min=list(lower), workspace_max=list(upper), voxel_size=size,
                 support_surface_enabled=False)
        k[0, 0] = 100.
        clear = unknown_boxes(np.ones((80, 80)), k, np.eye(4), c, lambda p: False)[1]
        missing = unknown_boxes(np.full((80, 80), np.nan), k, np.eye(4), c, lambda p: False)[1]
        self.assertEqual(clear, 0)
        self.assertEqual(missing, int(np.prod(shape)))

    def test_explicit_configuration(self):
        validate_config(config())
        for key, value in [('depth_scale', float('nan')), ('workspace_max', [0, 0, 0]),
                           ('voxel_size', .00001), ('sample_stride', True), ('depth_encoding', 'mono16')]:
            with self.subTest(key=key), self.assertRaises(ValueError):
                validate_config({**config(), key: value})
        c = config()
        del c['self_filter_model']
        with self.assertRaises(ValueError):
            validate_config(c)

    def test_endian_stride_units_invalid(self):
        for encoding, code, scale in [('16UC1', 'H', .001), ('32FC1', 'f', 1.)]:
            for big in (False, True):
                values = [500, 0, 2000] if code == 'H' else [.5, float('nan'), float('inf')]
                raw = struct.pack(('>' if big else '<')+code*3, *values)+b'pad!'
                msg = NS(width=3, height=1, step=len(raw), data=raw, is_bigendian=big, encoding=encoding)
                decoded = decode_depth(msg, encoding, scale, .1, 2.)
                self.assertAlmostEqual(decoded[0, 0], .5)
                self.assertTrue(np.isnan(decoded[0, 1:]).all())
                with self.assertRaises(ValueError):
                    decode_depth(NS(**{**msg.__dict__, 'data': raw[:-1]}), encoding, scale, .1, 2.)

    def test_registration_and_timestamp_contract(self):
        def image():
            return NS(width=4, height=4, header=NS(frame_id='camera_color_optical_frame', stamp=NS(sec=1, nanosec=0)),
                      encoding='rgb8', step=12, data=bytes(48), k=[10., 0, 2, 0, 10., 2, 0, 0, 1.],
                      d=[], binning_x=0, binning_y=0, roi=NS(x_offset=0, y_offset=0, width=0, height=0))
        messages = [image() for _ in range(4)]
        validate_bundle(*messages, config())
        for field, value in [('width', 5), ('k', [11., 0, 2, 0, 10., 2, 0, 0, 1.]), ('d', [.01])]:
            bad = deepcopy(messages)
            setattr(bad[3], field, value)
            with self.assertRaises(ValueError):
                validate_bundle(*bad, config())
        messages[1].header.stamp.sec = 2
        with self.assertRaises(ValueError):
            validate_bundle(*messages, config())

    def test_floor_removal_never_removes_vertical_wall(self):
        x, y = np.meshgrid(np.linspace(.32, .70, 20), np.linspace(-.25, .25, 20))
        floor = np.column_stack((x.ravel(), y.ravel(), np.full(x.size, .26)))
        wall = floor.copy()
        wall[:, 0], wall[:, 2] = .5, np.linspace(.28, .5, len(wall))
        result, removed = remove_plane(np.vstack((floor, wall)), config())
        self.assertEqual(removed, len(floor))
        self.assertTrue(np.allclose(result, wall))

    def test_partial_bounds_and_hidden_volume(self):
        wall = Bounds(np.array([.48, -.12, .26]), np.array([.52, .12, .40]))
        depth, k, camera = synthetic_depth([wall])
        observed, unknown, quality = estimate(depth, k, camera, [], matrix([2, 2, 2], [0, 0, 0, 1]), config())
        self.assertEqual(len(observed), 1)
        self.assertLess(observed[0].size[2], .01)  # only top surface seen
        self.assertFalse(quality['visible_bounds_complete'])
        self.assertGreater(quality['plane_points_removed'], 100)
        self.assertTrue(any(np.all(b.lower <= [.5, 0, .28]) and np.all(b.upper >= [.5, 0, .28]) for b in unknown))

    def test_missing_depth_and_out_of_view_remain_unknown(self):
        depth, k, camera = synthetic_depth()
        clear, _, _ = unknown_boxes(depth, k, camera, config(), lambda corners: False)
        depth[:] = np.nan
        blocked, count, total = unknown_boxes(depth, k, camera, config(), lambda corners: False)
        self.assertEqual(count, total)
        self.assertGreater(sum(np.prod(b.size) for b in blocked), sum(np.prod(b.size) for b in clear))

    def test_separate_fingers_retain_gap_and_near_obstacle(self):
        left = Body('left', np.eye(4), 'box', np.array([.02, .04, .1])).at(matrix([0, -.04, 0], [0, 0, 0, 1]))
        right = Body('right', np.eye(4), 'box', np.array([.02, .04, .1])).at(matrix([0, .04, 0], [0, 0, 0, 1]))
        points = np.array([[0, -.04, 0], [0, .04, 0], [0, 0, 0], [.012, -.04, 0]])
        keep = ~left.contains(points) & ~right.contains(points)
        self.assertEqual(keep.tolist(), [False, False, True, True])

    def test_target_filtered_without_clearing_its_shadow(self):
        target = Bounds(np.array([.405, -.205, .26]), np.array([.455, -.155, .48]))
        depth, k, camera = synthetic_depth([target])
        observed, unknown, quality = estimate(depth, k, camera, [], matrix(target.center, [0, 0, 0, 1]), config())
        self.assertFalse(observed)
        self.assertGreater(quality['target_points_removed'], 0)
        self.assertGreater(len(unknown), 0)  # boundary/hidden space is not carved away

    def test_bad_clusters_not_interpreted_as_free(self):
        c = config()
        boxes, rejected = fit_clusters(np.array([[.5, 0, .5], [.6, .1, .5]]), c)
        self.assertFalse(boxes)
        self.assertEqual(rejected, 2)
        c['max_collision_boxes'] = 1
        depth, k, camera = synthetic_depth([Bounds(np.array([.48, -.12, .26]), np.array([.52, .12, .40]))])
        with self.assertRaises(ValueError):
            estimate(depth, k, camera, [], matrix([2, 2, 2], [0, 0, 0, 1]), c)

    def test_external_model_requires_tool_and_camera(self):
        with tempfile.TemporaryDirectory() as root:
            root = Path(root)
            (root/'robot.urdf').write_text('<robot name="test"><link name="tool"><collision><geometry><box size=".1 .1 .1"/></geometry></collision></link><link name="camera"/></robot>')
            c = {'robot_urdf': 'robot.urdf', 'required_links': ['tool', 'camera'], 'extra_boxes': []}
            (root/'model.yaml').write_text(yaml.safe_dump(c))
            with self.assertRaises(ValueError):
                load_model(root/'model.yaml')
            c['extra_boxes'] = [{'link': 'camera', 'size': [.1, .02, .03], 'xyz': [0, 0, 0], 'rpy': [0, 0, 0]}]
            (root/'model.yaml').write_text(yaml.safe_dump(c))
            bodies, joints, digest = load_model(root/'model.yaml')
            self.assertEqual(len(bodies), 2)
            self.assertEqual(len(digest), 64)
            self.assertFalse(joints)

    def test_mesh_hull_never_erases_a_concave_gap(self):
        triangles = []
        for center in (-.04, .04):
            vertices = np.array([[center+x, y, z] for x in (-.01, .01) for y in (-.02, .02) for z in (-.05, .05)])
            triangles.extend(vertices[ConvexHull(vertices).simplices])
        triangles = np.array(triangles)
        vertices = triangles.reshape(-1, 3)
        ray_data, closed = prepare_rays(triangles)
        body = Body('concave_link', np.eye(4), 'mesh', np.array([vertices.min(axis=0), vertices.max(axis=0)]),
                    ConvexHull(vertices).equations, ray_data=ray_data, closed=closed).at(np.eye(4))
        self.assertEqual(body.contains(np.array([[-.04, 0, 0], [.04, 0, 0], [0, 0, 0], [.052, 0, 0]])).tolist(),
                         [True, True, False, False])
        from scipy.spatial import cKDTree
        body.triangles = triangles
        centers = triangles.mean(axis=1)
        body.surface_tree = cKDTree(centers)
        body.triangle_radius = float(np.linalg.norm(triangles-centers[:, None], axis=2).max())
        posed = body.at(np.eye(4))
        # A real triangle surface shell is not the convex hull of both fingers.
        # The independent point 5 mm outside the right finger stays unfiltered.
        query = np.array([[.052, 0, 0], [.055, 0, 0], [0, 0, 0], [.052, .024, 0]])
        self.assertEqual(posed.surface_contains(query, .003).tolist(), [True, False, False, False])
        self.assertEqual(posed.surface_contains(query, 0.).tolist(), [False]*4)

    def test_triangle_surface_distance_covers_faces_edges_vertices_and_degenerate_faces(self):
        triangles = np.array([[[0., 0, 0], [1., 0, 0], [0, 1., 0]],
                              [[2., 0, 0], [2., 0, 0], [2., 0, 0]]])
        for point, expected in (([.2, .2, .003], .003**2),
                                ([.5, -.004, 0], .004**2),
                                ([-.003, -.004, 0], .005**2),
                                ([2., 0., .003], .003**2)):
            self.assertAlmostEqual(float(triangle_distance_squared(np.array(point), triangles).min()), expected)

    def test_surface_radius_buckets_match_all_triangles_including_long_faces(self):
        rng = np.random.default_rng(7)
        triangles = rng.uniform(-.1, .1, (80, 3, 3))
        triangles[:20] *= .01
        # The near vertex is far from its centroid; nearest-k truncation or a
        # small shared radius would miss this exact surface.
        triangles[0] = [[0., 0., 0.], [4., 0., 0.], [0., .001, 0.]]
        vertices = triangles.reshape(-1, 3)
        body = Body('mesh', np.eye(4), 'mesh',
                    np.array([vertices.min(axis=0), vertices.max(axis=0)]))
        body.index_surfaces(triangles)
        posed = body.at(np.eye(4))
        probes = np.vstack(([[3.9, .00001, .002], [3.9, .00001, .005]],
                            rng.uniform(-.12, .12, (150, 3)), triangles[:, 0]))
        expected = np.array([triangle_distance_squared(point, triangles).min() <= .003**2 for point in probes])
        self.assertTrue(np.array_equal(posed.matches_surface(probes, .003), expected))
        self.assertGreater(len(posed.surface_groups), 1)

    def test_surface_tolerance_is_explicit_bounded_and_does_not_change_unknown_cells(self):
        from copy import deepcopy
        for bad in (-.001, .0031, True, float('nan')):
            c = config()
            c['self_surface_tolerance'] = bad
            with self.assertRaisesRegex(ValueError, 'self_surface_tolerance'):
                validate_config(c)
        depth, k, camera = synthetic_depth()
        c = config()
        _, unknown0, _ = estimate(depth, k, camera, [], np.eye(4), c)
        c['self_surface_tolerance'] = .003
        _, unknown1, _ = estimate(depth, k, camera, [], np.eye(4), c)
        self.assertEqual(len(unknown0), len(unknown1))
        for a, b in zip(unknown0, unknown1):
            self.assertTrue(np.array_equal(a.lower, b.lower) and np.array_equal(a.upper, b.upper))


if __name__ == '__main__':
    unittest.main()
