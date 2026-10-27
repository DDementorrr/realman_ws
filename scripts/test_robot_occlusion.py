#!/usr/bin/env python3
"""Finite robot-surface explanations; never clear the raw unknown geometry."""
from pathlib import Path
import sys
import unittest
import numpy as np
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/ros2_adapters/pick_place_perception'))
from pick_place_perception.robot_occlusion import witnessed_cells, known_occluders
from pick_place_perception.geometry_monitor import validate_monitor
from pick_place_perception.self_filter import Body
from pick_place_perception.obstacle_geometry import matrix, CORNERS, transform, visibility_depth
from test_stage4_geometry import config

class OcclusionTest(unittest.TestCase):
    def setUp(self):
        self.c = config()
        self.c.update(workspace_min=[.01, .01, .5], workspace_max=[.03, .03, .52],
                      voxel_size=.02, visibility_margin=0., self_surface_tolerance=.003)
        self.k = np.array([[100., 0, 40], [0, 100., 40], [0, 0, 1.]])
        self.body = Body('finger', np.eye(4), 'box', np.array([.2, .2, .02])).at(
            matrix([.02, .02, .4], [0, 0, 0, 1]))
        self.depth = np.full((80, 80), .39)
    def witness(self, depth=None, bodies=None):
        return bool(witnessed_cells(np.array([[0, 0, 0]]), self.depth if depth is None else depth,
                    self.k, np.eye(4), [self.body] if bodies is None else bodies, self.c)[0])
    def test_finite_front_surface_explains_blocking_pixels_without_modifying_depth(self):
        before = self.depth.copy()
        self.assertTrue(self.witness())
        self.assertTrue(np.array_equal(self.depth, before))
        self.assertFalse(self.witness(bodies=[]))
    def test_near_obstacle_five_mm_from_surface_and_interior_are_not_surface_witnesses(self):
        # Both a 5 mm external obstacle and an interior depth must be refused.
        self.assertFalse(self.witness(np.full_like(self.depth, .385)))
        self.assertFalse(self.witness(np.full_like(self.depth, .395)))
        self.assertFalse(self.witness(np.full_like(self.depth, .4)))
    def test_one_missing_or_other_obstacle_pixel_keeps_cell_unexplained(self):
        for bad in (float('nan'), float('-inf'), .3):
            depth = self.depth.copy();depth[44, 44] = bad
            self.assertFalse(self.witness(depth))
    def test_mixed_free_and_robot_rays_require_entire_rectangle(self):
        depth = self.depth.copy();depth[:, 45:] = .9
        self.assertTrue(self.witness(depth))
        depth[44, 46] = .3
        self.assertFalse(self.witness(depth))
    def test_no_return_cannot_identify_robot_and_out_of_frame_cannot_explain(self):
        self.assertFalse(self.witness(np.full_like(self.depth, np.inf)))
        self.k[0, 2] = 1000
        self.assertFalse(self.witness())

    def test_batched_witness_matches_independent_pixel_rectangles(self):
        c = {**self.c, 'workspace_min': [-.14, -.14, .48],
             'workspace_max': [.14, .14, .60], 'voxel_size': .04,
             'far_no_return': True, 'visibility_roundoff_m': 1e-6}
        lower, upper = np.array(c['workspace_min']), np.array(c['workspace_max'])
        shape = np.ceil((upper-lower)/c['voxel_size']).astype(int)
        indices = np.indices(shape).reshape(3, -1).T
        rng = np.random.default_rng(73)
        for trial in range(12):
            camera = matrix([.01*trial, 0., 0.], [0., 0., 0., 1.])
            depth = np.full((80, 80), .39)
            depth[:, 45:] = .9
            depth[rng.integers(0, 80, 20), rng.integers(0, 80, 20)] = rng.choice(
                [np.nan, np.inf, -np.inf, .3, .51], 20)
            expected = []
            usable = visibility_depth(depth, c)
            for index in indices:
                corners = lower+(index+CORNERS)*(upper-lower)/shape
                optical = transform(corners, np.linalg.inv(camera))
                z = optical[:, 2]
                u = optical[:, 0]*self.k[0, 0]/z+self.k[0, 2]
                v = optical[:, 1]*self.k[1, 1]/z+self.k[1, 2]
                x0, x1 = int(np.floor(u.min()))-1, int(np.ceil(u.max()))+1
                y0, y1 = int(np.floor(v.min()))-1, int(np.ceil(v.max()))+1
                if z.min() <= c['depth_min'] or x0 < 0 or y0 < 0 or x1 >= 80 or y1 >= 80:
                    expected.append(False)
                    continue
                rows, cols = np.mgrid[y0:y1+1, x0:x1+1]
                acquired = depth[rows, cols]
                finite = np.isfinite(acquired)
                points = np.column_stack(((cols[finite]-self.k[0, 2])*acquired[finite]/self.k[0, 0],
                                          (rows[finite]-self.k[1, 2])*acquired[finite]/self.k[1, 1],
                                          acquired[finite]))
                robot = np.zeros(acquired.shape, bool)
                robot[finite] = self.body.matches_surface(transform(points, camera), .003)
                rays = usable[rows, cols]
                free = np.isfinite(rays) & (rays+1e-6 > z.max()+c['visibility_margin'])
                expected.append(bool((free | robot).all() and (~free & robot).any()))
            actual = witnessed_cells(indices, depth, self.k, camera, [self.body], c)
            np.testing.assert_array_equal(actual, expected)

    def test_separately_known_target_is_opt_in_uses_current_pose_and_no_mask_padding(self):
        target = matrix([.02, .02, .4], [0., 0., 0., 1.])
        self.c.update(target_size=[.2, .2, .02], target_mask_padding=.05)
        bodies = []
        self.assertFalse(self.witness(bodies=known_occluders(bodies, target, self.c)))
        self.c['monitor_target_occlusion'] = True
        self.assertTrue(self.witness(bodies=known_occluders(bodies, target, self.c)))
        self.assertEqual(bodies, [])
        self.assertFalse(self.witness(np.full_like(self.depth, .385),
                                     known_occluders(bodies, target, self.c)))
        target[2, 3] += .01
        self.assertFalse(self.witness(bodies=known_occluders(bodies, target, self.c)))

    def test_target_explanation_requires_explicit_boolean_surface_monitor_opt_in(self):
        c = {**self.c, 'monitor_inputs': True, 'monitor_target_occlusion': True}
        with self.assertRaisesRegex(ValueError, 'monitor_target_occlusion'):
            validate_monitor(c)
        c['monitor_robot_occlusion'] = True
        validate_monitor(c)
        c['monitor_target_occlusion'] = 'true'
        with self.assertRaisesRegex(ValueError, 'monitor_target_occlusion'):
            validate_monitor(c)

if __name__ == '__main__':unittest.main()
