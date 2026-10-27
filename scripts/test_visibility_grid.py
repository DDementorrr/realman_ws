#!/usr/bin/env python3
"""Optimised visibility must preserve the original conservative classification."""
from pathlib import Path
import sys
import unittest
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/ros2_adapters/pick_place_perception'))
from pick_place_perception.obstacle_geometry import (Bounds, CORNERS, boxes_from_occupancy, matrix,
    rectangle_minima, transform, unknown_boxes)
from pick_place_perception.geometry_monitor import occupancy
from test_stage4_geometry import config


class VisibilityTest(unittest.TestCase):
    def test_rectangle_minima_matches_all_pixels_including_invalid(self):
        rng = np.random.default_rng(2)
        depth = rng.uniform(.1, 2., (43, 61))
        depth[10, 10], depth[20, 20], depth[30, 30] = np.nan, np.inf, 0.
        rectangles = []
        for _ in range(200):
            x = sorted(rng.integers(0, depth.shape[1], 2))
            y = sorted(rng.integers(0, depth.shape[0], 2))
            rectangles.append([*x, *y])
        expected = []
        for x0, x1, y0, y1 in rectangles:
            patch = depth[y0:y1+1, x0:x1+1]
            expected.append(patch.min() if np.isfinite(patch).all() else -np.inf)
        self.assertTrue(np.array_equal(rectangle_minima(depth, np.array(rectangles)), expected))

    def test_exact_cell_union_and_inflated_union_preserved(self):
        rng = np.random.default_rng(3)
        c = config()
        c.update(workspace_min=[0., 0., 0.], workspace_max=[.20, .24, .16], voxel_size=.04)
        shape = (5, 6, 4)
        for mask in (rng.random(shape) > .55, np.ones(shape, bool), np.zeros(shape, bool)):
            boxes = boxes_from_occupancy(mask, np.zeros(3), np.array(c['workspace_max']))
            self.assertTrue(np.array_equal(occupancy(boxes, c), mask))
            self.assertTrue(np.array_equal(occupancy(boxes, c, outward=True), mask))
            # Check points inside a sample of raw or inflated occupied boxes.
            samples = rng.uniform(-.02, .26, (1000, 3))
            for margin in (0., .032):
                direct, merged = np.zeros(len(samples), bool), np.zeros(len(samples), bool)
                for idx in np.argwhere(mask):
                    lo, hi = idx*.04-margin, (idx+1)*.04+margin
                    direct |= ((samples >= lo) & (samples <= hi)).all(axis=1)
                for box in boxes:
                    merged |= ((samples >= box.lower-margin) & (samples <= box.upper+margin)).all(axis=1)
                self.assertTrue(np.array_equal(direct, merged))

    def test_outward_grid_round_trip_never_adds_occupied_layers(self):
        c = config()
        c.update(workspace_min=[.305, -.305, .26], workspace_max=[.735, .305, .65],
                 voxel_size=.01)
        lower, upper = np.array(c['workspace_min']), np.array(c['workspace_max'])
        shape = np.ceil((upper-lower)/c['voxel_size']).astype(int)
        mask = np.zeros(tuple(shape), dtype=bool)
        mask[33:38, 15:46, :13] = True  # Actual single-wall grid boundaries.
        rng = np.random.default_rng(17)
        indices = np.column_stack([rng.integers(0, n, 32) for n in shape])
        mask[tuple(indices.T)] = True
        boxes = boxes_from_occupancy(mask, lower, upper)
        for _ in range(3):
            actual = occupancy(boxes, c, outward=True)
            self.assertTrue(np.array_equal(actual, mask))
            boxes = boxes_from_occupancy(actual, lower, upper)

    def test_real_off_grid_extension_still_rounds_outward(self):
        c = config()
        c.update(workspace_min=[.305, -.305, .26], workspace_max=[.735, .305, .65],
                 voxel_size=.01)
        lower, upper = np.array(c['workspace_min']), np.array(c['workspace_max'])
        shape = np.ceil((upper-lower)/c['voxel_size']).astype(int)
        step = (upper-lower)/shape
        lo, hi = lower+np.array([33, 16, 1])*step, lower+np.array([38, 45, 12])*step
        baseline = occupancy([Bounds(lo, hi)], c, outward=True)
        extended = occupancy([Bounds(lo-1e-10, hi+1e-10)], c, outward=True)
        self.assertGreater(int(extended.sum()), int(baseline.sum()))
        self.assertTrue(extended[32, 16, 1] and extended[38, 16, 1])
        self.assertTrue(((extended | baseline) == extended).all())

    def test_vectorised_visibility_equals_scalar_reference_for_rotated_views(self):
        rng = np.random.default_rng(4)
        c = config()
        c.update(workspace_min=[-.15, -.15, .2], workspace_max=[.15, .15, .5], voxel_size=.04)
        k = np.array([[120., 0, 80], [0, 120., 60], [0, 0, 1.]])
        depth = rng.uniform(.6, 1., (120, 160))
        depth[::23, ::17] = np.nan
        for q in ([1., 0, 0, 0], [.9238795325, 0, 0, .3826834324]):
            camera = matrix([0, 0, 1.], q)
            boxes, count, total = unknown_boxes(depth, k, camera, c, lambda p: False)
            actual = occupancy(boxes, c)
            lower, upper = np.array(c['workspace_min']), np.array(c['workspace_max'])
            step = (upper-lower)/actual.shape
            reference = np.ones(actual.shape, bool)
            for idx in np.ndindex(*actual.shape):
                p = transform(lower+(np.array(idx)+CORNERS)*step, np.linalg.inv(camera))
                z = p[:, 2]
                if z.min() <= c['depth_min']:
                    continue
                u, v = p[:, 0]*k[0, 0]/z+k[0, 2], p[:, 1]*k[1, 1]/z+k[1, 2]
                x0, x1 = int(np.floor(u.min()))-1, int(np.ceil(u.max()))+1
                y0, y1 = int(np.floor(v.min()))-1, int(np.ceil(v.max()))+1
                if x0 < 0 or y0 < 0 or x1 >= depth.shape[1] or y1 >= depth.shape[0]:
                    continue
                patch = depth[y0:y1+1, x0:x1+1]
                reference[idx] = not (np.isfinite(patch).all() and patch.min() > z.max()+c['visibility_margin'])
            self.assertTrue(np.array_equal(actual, reference))
            self.assertEqual(count, int(reference.sum()))
            self.assertEqual(total, reference.size)

    def test_batched_known_volume_matches_scalar_without_carving_boundary_cells(self):
        c = config()
        c.update(workspace_min=[0., 0., 0.], workspace_max=[.12, .12, .12], voxel_size=.04)
        depth = np.full((30, 30), np.nan)
        k = np.array([[20., 0, 15], [0, 20., 15], [0, 0, 1.]])
        camera = matrix([0, 0, 1.], [1, 0, 0, 0])
        extent = np.array([.05, .08, .08])
        scalar = lambda corners: bool((corners <= extent).all())
        batch = lambda corners: (corners <= extent).all(axis=(1, 2))
        original, original_count, _ = unknown_boxes(depth, k, camera, c, scalar)
        vector, vector_count, _ = unknown_boxes(depth, k, camera, c, scalar, batch)
        actual = occupancy(vector, c)
        self.assertTrue(np.array_equal(actual, occupancy(original, c)))
        self.assertEqual(vector_count, original_count)
        self.assertFalse(actual[0, 0, 0])  # whole cell lies inside known geometry
        self.assertTrue(actual[1, 0, 0])   # boundary cell extends outside known geometry


if __name__ == '__main__':
    unittest.main()
