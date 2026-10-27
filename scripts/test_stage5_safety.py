#!/usr/bin/env python3
"""Bounds, frozen coverage and execution lease tests; no physical safety claim."""
from copy import deepcopy
from pathlib import Path
import sys
import tempfile
import time
from types import MethodType
from concurrent.futures import Future
from unittest.mock import Mock
import unittest
from types import SimpleNamespace as NS

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
for name in ('pick_place_scene', 'pick_place_runtime', 'pick_place_perception'):
    sys.path.insert(0, str(ROOT/'src/ros2_adapters'/name))
from pick_place_scene.uncertainty import load_budget, inflate
from pick_place_runtime.input_gate import InputGate
from pick_place_runtime import ExecutionCoordinator, ExecutionState, BackendState
from pick_place_perception.geometry_monitor import (validate_monitor, quality_check, occupancy,
                                                     check_frozen_coverage,
                                                     check_new_observed_occupancy, joint_motion)
from pick_place_perception.obstacle_geometry import Bounds
from pick_place_scene.static_geometry import load_geometry, support_height_envelope
from test_execution_protocol import Clock, MockBackend
from test_stage4_geometry import config, synthetic_depth


class SafetyTest(unittest.TestCase):
    def test_known_support_height_envelope_covers_the_same_normal_tolerance(self):
        frame, boxes = load_geometry(ROOT/'src/ros2_adapters/pick_place_scene/config/u_workbench.yaml')
        result = support_height_envelope(frame, boxes, 'pick_table', frame, .26,
                                          [.31, -.30], [.73, .30], .003)
        original, inflated = boxes[0], result[0]
        self.assertAlmostEqual(inflated.pose[2]+inflated.size[2]/2, .263)
        self.assertAlmostEqual(inflated.pose[2]-inflated.size[2]/2,
                               original.pose[2]-original.size[2]/2)
        self.assertEqual(result[1:], boxes[1:])
        self.assertEqual(inflated.size[:2], original.size[:2])
        self.assertAlmostEqual(original.size[2], .06)
        for changes in ({'support_frame': 'world'}, {'height': .261},
                        {'xy_min': [0., -.3]}, {'tolerance': float('nan')},
                        {'support_id': 'missing'}):
            args = dict(frame=frame, boxes=boxes, support_id='pick_table', support_frame=frame,
                        height=.26, xy_min=[.31, -.30], xy_max=[.73, .30], tolerance=.003)
            args.update(changes)
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                support_height_envelope(**args)

    def test_live_monitor_rejects_new_unknown_cells_without_positive_surface(self):
        from pick_place_perception.depth_obstacles import DepthObstacles
        c = config()
        c.update(workspace_min=[0., 0., 0.], workspace_max=[.08, .08, .08], voxel_size=.04)
        depth = NS(header=NS(stamp=NS(sec=2, nanosec=0)))
        new_unknown = Bounds(np.zeros(3), np.full(3, .04))
        node = NS(config=c, ready=True, pending=False, monitor_inputs=True,
                  last_view_stamps=[10**9], last_valid_stamp=10**9,
                  last_valid_wall=time.monotonic(), monitor_count=0, frozen_coverage=np.zeros((2, 2, 2), bool),
                  next_geometry=lambda: ([(depth, None, None, None, None, 0.)],
                                          ([], [new_unknown], {}, {}), time.monotonic()),
                  get_clock=lambda: NS(now=lambda: NS(nanoseconds=2*10**9)),
                  get_logger=lambda: NS(warning=lambda message: None), publish_status=lambda: None)
        node.expire_lease = MethodType(DepthObstacles.expire_lease, node)
        DepthObstacles.tick(node)
        self.assertFalse(node.ready)
        self.assertIn('previously free cells now occupied/unknown', node.last_error)

    def test_inflight_geometry_does_not_renew_or_revive_expired_lease(self):
        from pick_place_perception.depth_obstacles import DepthObstacles, GeometryPending
        future = Future()
        worker = NS(submit=Mock(return_value=future))
        depth = NS(header=NS(stamp=NS(sec=2, nanosec=0)))
        node = NS(config={'max_input_age': 3.}, ready=True, pending=False, monitor_inputs=True,
                  geometry_worker=worker, geometry_job=None, views=[], frozen_coverage=None,
                  last_view_stamps=[10**9],
                  last_valid_stamp=10**9, last_valid_wall=time.monotonic(), monitor_count=0,
                  bundles=lambda: [(depth, None, [], None, None, 0., [], {})], calculate=lambda *a: None,
                  get_clock=lambda: NS(now=lambda: NS(nanoseconds=2*10**9)),
                  publish_status=lambda: None)
        node.expire_lease = MethodType(DepthObstacles.expire_lease, node)
        node.next_geometry = MethodType(DepthObstacles.next_geometry, node)
        for _ in range(3):
            DepthObstacles.tick(node)
        self.assertTrue(node.ready)
        self.assertEqual(node.last_valid_stamp, 10**9)
        self.assertEqual(worker.submit.call_count, 1)
        node.last_valid_wall -= 4.
        future.set_result(([], [], {}, {}))
        # Even a completed result with a newer stamp must not revive the lease.
        DepthObstacles.tick(node)
        self.assertFalse(node.ready)
        self.assertEqual(node.monitor_count, 0)
        self.assertIn('snapshot expired', node.last_error)
        self.assertGreater(node.lease_expiry['steady_age_sec'], 3.)
        self.assertTrue(node.lease_expiry['geometry_job_done'])
        self.assertGreaterEqual(node.lease_expiry['geometry_job_age_sec'], 0.)
        expiry = dict(node.lease_expiry)
        DepthObstacles.tick(node)
        self.assertFalse(node.ready)
        self.assertEqual(node.lease_expiry, expiry)

    def test_worker_failure_invalidates_monitor(self):
        from pick_place_perception.depth_obstacles import DepthObstacles
        future = Future()
        future.set_exception(ValueError('invalid depth quality'))
        node = NS(config={'max_input_age': 3.}, ready=True, pending=False, monitor_inputs=True,
                  geometry_job=(future, [], time.monotonic()), last_valid_stamp=10**9,
                  last_valid_wall=time.monotonic(),
                  get_clock=lambda: NS(now=lambda: NS(nanoseconds=2*10**9)),
                  get_logger=lambda: NS(warning=lambda message: None), publish_status=lambda: None)
        node.expire_lease = MethodType(DepthObstacles.expire_lease, node)
        node.next_geometry = MethodType(DepthObstacles.next_geometry, node)
        DepthObstacles.tick(node)
        self.assertFalse(node.ready)
        self.assertIn('invalid depth quality', node.last_error)

    def test_completed_geometry_keeps_acquisition_age_in_steady_lease(self):
        from pick_place_perception.depth_obstacles import DepthObstacles
        depth = NS(header=NS(stamp=NS(sec=2, nanosec=0)))
        started = time.monotonic()-.8
        c = config()
        c.update(workspace_min=[0., 0., 0.], workspace_max=[.08, .08, .08], voxel_size=.04)
        node = NS(config=c, ready=True, pending=False, monitor_inputs=True,
                  last_valid_stamp=10**9, last_valid_wall=time.monotonic(),
                  last_view_stamps=[10**9], monitor_count=0,
                  frozen_coverage=np.zeros((2, 2, 2), bool),
                  next_geometry=lambda: ([(depth, None, None, None, None, .4)],
                                         ([], [], {}, {}), started),
                  get_clock=lambda: NS(now=lambda: NS(nanoseconds=2500000000)),
                  get_logger=lambda: NS(warning=lambda message: None), publish_status=lambda: None)
        node.expire_lease = MethodType(DepthObstacles.expire_lease, node)
        DepthObstacles.tick(node)
        self.assertTrue(node.ready)
        self.assertEqual(node.monitor_count, 1)
        self.assertAlmostEqual(node.last_valid_wall, started-.4)
        node.last_valid_wall -= 4.
        node.expire_lease()
        self.assertFalse(node.ready)

    def test_robot_shadow_explanation_never_excuses_new_positive_occupancy(self):
        from pick_place_perception.depth_obstacles import DepthObstacles
        depth = NS(header=NS(stamp=NS(sec=2, nanosec=0)))
        c = config()
        c.update(workspace_min=[0., 0., 0.], workspace_max=[.08, .08, .08], voxel_size=.04)
        box = Bounds(np.zeros(3), np.full(3, .04))
        for positive in (False, True):
            node = NS(config=c, ready=True, pending=False, monitor_inputs=True,
                      last_valid_stamp=10**9, last_valid_wall=time.monotonic(),
                      last_view_stamps=[10**9], monitor_count=0,
                      frozen_coverage=np.zeros((2, 2, 2), bool),
                      next_geometry=lambda: ([(depth, None, None, None, None, 0.)],
                          ([box] if positive else [], [box], {}, {'robot_occlusion_indices': [[0, 0, 0]]}), time.monotonic()),
                      get_clock=lambda: NS(now=lambda: NS(nanoseconds=2*10**9)),
                      get_logger=lambda: NS(warning=lambda message: None), publish_status=lambda: None)
            node.expire_lease = MethodType(DepthObstacles.expire_lease, node)
            DepthObstacles.tick(node)
            self.assertEqual(node.ready, not positive)
            # No geometry mutation, even after a successful explanation.
            self.assertTrue(np.array_equal(box.lower, np.zeros(3)))
            self.assertTrue(np.array_equal(box.upper, np.full(3, .04)))

    def budget(self, **changes):
        c = yaml.safe_load((ROOT/'src/ros2_adapters/pick_place_scene/config/sim_uncertainty_example.yaml').read_text())
        c.update(changes)
        with tempfile.TemporaryDirectory() as root:
            path = Path(root)/'budget.yaml'
            path.write_text(yaml.safe_dump(c))
            return load_budget(path)

    def test_all_error_terms_are_added_per_side(self):
        budget = self.budget(extrinsic_translation_bound_m=.002, extrinsic_rotation_bound_rad=.01)
        self.assertAlmostEqual(budget[2], .003+.002+4*np.sin(.005)+.015+.003+.006+.005)
        obj = NS(id='depth_obstacles', ADD=0, operation=0, primitives=[NS(dimensions=[.02, .1, .2]), NS(dimensions=[.04]*3)])
        before = deepcopy(obj)
        padded = inflate(obj, budget)
        for shape, original in zip(padded.primitives, before.primitives):
            self.assertTrue(np.allclose(np.array(shape.dimensions)-original.dimensions, 2*budget[2]))
        self.assertEqual(obj.primitives[0].dimensions, before.primitives[0].dimensions)
        obj.id = 'pick_object'
        self.assertIs(inflate(obj, budget), obj)
        obj.id, obj.operation = 'depth_obstacles', 1
        self.assertIs(inflate(obj, budget), obj)

    def test_bad_or_unexplained_budgets_rejected(self):
        for name, value in [('depth_bound_m', -1), ('tracking_bound_m', float('nan')),
                            ('basis', ''), ('object_ids', ['x', 'x']), ('time_skew_bound_sec', True),
                            ('extrinsic_rotation_bound_rad', 4), ('observation_range_m', 0)]:
            with self.subTest(name=name), self.assertRaises(ValueError):
                self.budget(**{name: value})

    def test_optional_gate_preserves_legacy(self):
        self.assertTrue(InputGate().status()[0])

    def test_gate_requires_live_true_before_begin(self):
        clock = Clock()
        gate = InputGate(True, 1., clock)
        self.assertEqual(gate.begin(), (False, 'perception_input_missing'))
        gate.update(False)
        self.assertFalse(gate.begin()[0])
        gate.update(True)
        self.assertTrue(gate.begin()[0])

    def test_expired_active_lease_cannot_be_revived_by_heartbeat(self):
        clock = Clock()
        gate = InputGate(True, 1., clock)
        gate.update(True)
        gate.begin()
        clock.now = 2.
        gate.update(True)
        self.assertEqual(gate.status(), (False, 'perception_heartbeat_expired'))
        gate.end()
        gate.update(True)
        self.assertFalse(gate.begin()[0])

    def test_false_during_task_latches_even_if_recovered(self):
        gate = InputGate(True, 1.)
        gate.update(True)
        gate.begin()
        gate.update(False)
        gate.update(True)
        self.assertEqual(gate.status(), (False, 'perception_invalid_during_task'))

    def test_missing_producer_cancels_and_waits_for_terminal_ack(self):
        clock, backend = Clock(), MockBackend()
        gate = InputGate(True, .5, clock)
        gate.update(True)
        gate.begin()
        runtime = ExecutionCoordinator(backend, clock=clock, cancel_timeout=1.)
        runtime.start(['one', 'two'], candidate_revision=1, current_revision=1)
        clock.now = .6
        if not gate.status()[0]:
            runtime.cancel()
        self.assertEqual(runtime.state, ExecutionState.STOPPING)
        self.assertEqual(runtime.tick().state, ExecutionState.STOPPING)
        backend.statuses.append(BackendState.CANCELLED)
        self.assertEqual(runtime.tick().code, 'cancelled')
        self.assertEqual(backend.started, ['one'])

    def test_missing_cancel_ack_remains_blocked(self):
        clock, backend = Clock(), MockBackend()
        runtime = ExecutionCoordinator(backend, clock=clock, cancel_timeout=.5)
        runtime.start(['one', 'two'], candidate_revision=1, current_revision=1)
        runtime.cancel()
        clock.now = .6
        self.assertEqual(runtime.tick().state, ExecutionState.BLOCKED)
        self.assertEqual(backend.started, ['one'])

    def test_monitor_requires_quality_thresholds(self):
        validate_monitor({})
        with self.assertRaises(ValueError):
            validate_monitor({'monitor_inputs': True})
        c = {'monitor_inputs': True, 'min_valid_depth_fraction': .8, 'max_unknown_fraction': .5}
        validate_monitor(c)
        with self.assertRaisesRegex(ValueError, 'quality'):
            quality_check(np.array([1., np.nan]), {'unknown_cells': 1, 'total_cells': 10}, c)
        with self.assertRaisesRegex(ValueError, 'unknown'):
            quality_check(np.array([1., 1.]), {'unknown_cells': 8, 'total_cells': 10}, c)
        c.update(sample_stride=2, sampling_bound_m=.006)
        with self.assertRaisesRegex(ValueError, 'sampling spacing'):
            quality_check(np.array([1., 1.]), {'unknown_cells': 1, 'total_cells': 10}, c, np.diag([100., 100., 1.]))

    def test_motion_occlusion_relaxes_only_the_depth_fraction(self):
        c = {'monitor_inputs': True, 'min_valid_depth_fraction': .8,
             'motion_min_valid_depth_fraction': .2, 'max_unknown_fraction': .9}
        quality_check(np.array([1., np.nan, np.nan, np.nan]),
                      {'unknown_cells': 1, 'total_cells': 10}, c,
                      allow_motion_occlusion=True)
        with self.assertRaisesRegex(ValueError, 'quality'):
            quality_check(np.full(4, np.nan), {'unknown_cells': 1, 'total_cells': 10}, c,
                          allow_motion_occlusion=True)
        with self.assertRaisesRegex(ValueError, 'unknown'):
            quality_check(np.array([1., np.nan, np.nan, np.nan]),
                          {'unknown_cells': 10, 'total_cells': 10}, c,
                          allow_motion_occlusion=True)

    def test_motion_evidence_requires_joint_change(self):
        previous = {'j1': 0., 'j2': 1.}
        self.assertFalse(joint_motion(None, previous, .005))
        self.assertFalse(joint_motion(previous, {'j1': .004, 'j2': 1.}, .005))
        self.assertTrue(joint_motion(previous, {'j1': .005, 'j2': 1.}, .005))

    def test_new_unknown_cells_invalidate_coverage(self):
        c = config()
        c.update(workspace_min=[0, 0, 0], workspace_max=[.12, .08, .08], voxel_size=.04)
        baseline = occupancy([Bounds(np.array([0, 0, 0]), np.array([.04, .08, .08]))], c)
        same = baseline.copy()
        same[0, 0, 0] = False  # seeing more free space needs no geometry removal
        check_frozen_coverage(baseline, same)
        same[1, 1, 1] = True
        with self.assertRaisesRegex(ValueError, '1 previously free'):
            check_frozen_coverage(baseline, same)

    def test_robot_self_occlusion_does_not_look_like_new_obstacle(self):
        c = config()
        c.update(workspace_min=[0, 0, 0], workspace_max=[.12, .08, .08], voxel_size=.04)
        baseline = occupancy([Bounds(np.array([0, 0, 0]), np.array([.04, .08, .08]))], c)
        # A surface hidden by the moving robot disappears from the observed
        # mask; only newly observed positive occupancy is safety relevant.
        current = np.zeros_like(baseline)
        check_new_observed_occupancy(baseline, current)
        current[1, 1, 1] = True
        with self.assertRaisesRegex(ValueError, 'newly observed'):
            check_new_observed_occupancy(baseline, current)


if __name__ == '__main__':
    unittest.main()
