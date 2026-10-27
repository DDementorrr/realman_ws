#!/usr/bin/env python3
"""No-return semantics and conservative multi-view fusion, without a simulator."""
from copy import deepcopy
from pathlib import Path
import struct
import sys
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/ros2_adapters/pick_place_perception'))
from pick_place_perception.obstacle_geometry import (Bounds, decode_depth, estimate, matrix,
    points_from_depth, unknown_boxes, validate_config, visibility_depth)
from pick_place_perception.geometry_monitor import occupancy, quality_check, check_frozen_coverage
from pick_place_perception.multi_view import fuse_estimates, view_configs
from test_stage4_geometry import config, synthetic_depth


def result(unknown=(), observed=()):
    return list(observed), list(unknown), {
        'observed_boxes': len(observed), 'unknown_boxes': len(unknown),
        'unknown_cells': 0, 'total_cells': 27, 'self_points_removed': 0,
        'target_points_removed': 0, 'plane_points_removed': 0,
        'rejected_cluster_samples': 0, 'unknown_space_excluded': True,
    }


class MultiViewTest(unittest.TestCase):
    def test_geometry_worker_payload_preserves_capture_pose_without_serializing_mesh(self):
        from pick_place_perception import multi_view
        from pick_place_perception.self_filter import Body
        model = Body('finger', np.eye(4), 'box', np.array([.02, .03, .1]))
        posed = model.at(matrix([.1, .2, .3], [0., 0., 0., 1.]))
        packet = (object(), np.eye(3), [posed], np.eye(4), np.eye(4), .2, [123], {'joint': .4})
        payload = multi_view.geometry_payload([packet])
        self.assertTrue(np.array_equal(payload[0][2][0], posed.inverse))
        self.assertEqual(payload[0][5:], packet[5:])
        baseline = np.zeros((2, 2, 2), bool)
        with patch.object(multi_view, '_worker_models', (model,), create=True), \
                patch.object(multi_view, 'calculate_packets', return_value='result') as calculate:
            self.assertEqual(multi_view.calculate_pose_packets(payload, ['view'], baseline), 'result')
            restored = calculate.call_args.args[0][0][2][0]
            probes = np.array([[.1, .2, .3], [.2, .2, .3]])
            self.assertTrue(np.array_equal(restored.contains(probes), posed.contains(probes)))
            self.assertIs(restored.dimensions, model.dimensions)
            self.assertIs(calculate.call_args.args[2], baseline)
            with self.assertRaisesRegex(ValueError, 'body count'):
                multi_view.calculate_pose_packets([(*packet[:2], [], *packet[3:])], ['view'])

    def test_geometry_worker_refuses_changed_model_hash(self):
        from pick_place_perception.multi_view import initialize_geometry_worker
        with patch('pick_place_perception.self_filter.load_model', return_value=([], set(), 'changed')), \
                patch('signal.signal'):
            with self.assertRaisesRegex(ValueError, 'differs from capture model'):
                initialize_geometry_worker('unused-path', 'expected')

    def test_large_robot_shadow_checks_every_cell_and_keeps_unproven_tail(self):
        from pick_place_perception.multi_view import calculate_packets
        c = config()
        c.update(workspace_min=[0., 0., 0.], workspace_max=[1.1, 1.1, 1.1], voxel_size=.1,
                 monitor_robot_occlusion=True, self_surface_tolerance=.003)
        unknown = Bounds(np.zeros(3), np.full(3, 1.1))
        depth = NS(header=NS(stamp=NS(sec=1, nanosec=0)))
        packet = (depth, np.eye(3), [], np.eye(4), np.eye(4), 0., [10**9], {})
        checked = []
        def witness(indices, *args):
            checked.extend(map(tuple, indices))
            return ~np.all(indices == [10, 10, 10], axis=1)
        with patch('pick_place_perception.multi_view.decode_depth', return_value=np.ones((2, 2))), \
                patch('pick_place_perception.multi_view.estimate', return_value=result(unknown=[unknown])), \
                patch('pick_place_perception.multi_view.quality_check'), \
                patch('pick_place_perception.robot_occlusion.witnessed_cells', side_effect=witness) as prove:
            output = calculate_packets([packet], [c], np.zeros((11, 11, 11), bool))
        self.assertEqual(len(set(checked)), 1331)
        self.assertEqual(len(output[3]['robot_occlusion_indices']), 1330)
        self.assertNotIn([10, 10, 10], output[3]['robot_occlusion_indices'])
        self.assertIs(output[1][0], unknown)

    def test_rgbd_fixture_keeps_all_physical_models_and_rejects_physical_marker(self):
        import xml.etree.ElementTree as ET
        from pick_place_sim.workbench_world import prepare_rgbd_fixture
        source = ROOT/'src/realman_rm65/robot_gazebo/worlds/rm65b_pick_place_fixed.sdf'
        original = source.read_bytes()
        with tempfile.TemporaryDirectory() as directory:
            world = Path(directory)/'world.sdf'
            world.write_bytes(original)
            before = ET.fromstring(original)
            report = prepare_rgbd_fixture(world)
            after = ET.parse(world).getroot()
            self.assertEqual({m.get('name') for m in before.findall('world/model')},
                             {m.get('name') for m in after.findall('world/model')})
            self.assertEqual([ET.tostring(c) for c in before.findall('.//collision')],
                             [ET.tostring(c) for c in after.findall('.//collision')])
            self.assertEqual([m['model'] for m in report['removed_noncollision_visuals']],
                             ['pick_object', 'pick_object', 'place_zone'])
            self.assertFalse(after.findall("world/model[@name='place_zone']//visual"))
            self.assertEqual(len(after.findall("world/model[@name='pick_object']//visual")), 1)
            marker = before.find("world/model[@name='place_zone']/link")
            ET.SubElement(marker, 'collision', name='physical_marker')
            ET.ElementTree(before).write(world)
            saved = world.read_bytes()
            with self.assertRaisesRegex(ValueError, 'visual-only'):
                prepare_rgbd_fixture(world)
            self.assertEqual(world.read_bytes(), saved)
        self.assertEqual(source.read_bytes(), original)

    def test_explicit_positive_infinity_only_and_no_artificial_far_surface(self):
        for big in (False, True):
            values = [.5, float('inf'), float('-inf'), float('nan'), 0., 3.]
            msg = NS(width=6, height=1, step=24, encoding='32FC1', is_bigendian=big,
                     data=struct.pack(('>' if big else '<')+'f'*6, *values))
            old = decode_depth(msg, '32FC1', 1., .1, 2.)
            new = decode_depth(msg, '32FC1', 1., .1, 2., True)
            self.assertTrue(np.isnan(old[0, 1:]).all())
            self.assertTrue(np.isposinf(new[0, 1]))
            self.assertTrue(np.isnan(new[0, 2:]).all())
            self.assertEqual(len(points_from_depth(new, np.eye(3), 1)), 1)
            usable = visibility_depth(new, {'far_no_return': True, 'depth_max': 2.})
            self.assertEqual(usable[0, 1], 2.)
            self.assertTrue(np.isnan(usable[0, 2:]).all())

    def test_far_witness_ends_at_declared_range_and_nan_remains_unknown(self):
        c = config()
        c.update(workspace_min=[-.04, -.04, .3], workspace_max=[.04, .04, .5],
                 voxel_size=.04, far_no_return=True, depth_max=.6, visibility_margin=0.)
        k = np.array([[20., 0, 40], [0, 20., 40], [0, 0, 1.]])
        depth = np.full((80, 80), np.inf)
        boxes, count, total = unknown_boxes(depth, k, np.eye(4), c, lambda p: False)
        self.assertEqual(count, 0)
        self.assertFalse(boxes)
        c['depth_max'] = .4
        _, count, _ = unknown_boxes(depth, k, np.eye(4), c, lambda p: False)
        self.assertGreater(count, 0)
        c['far_no_return'] = False
        _, count, _ = unknown_boxes(depth, k, np.eye(4), c, lambda p: False)
        self.assertEqual(count, total)
        c['far_no_return'] = True
        _, count, _ = unknown_boxes(np.full_like(depth, np.nan), k, np.eye(4), c, lambda p: False)
        self.assertEqual(count, total)

    def test_far_pixels_still_charge_full_sampling_budget(self):
        c = {'monitor_inputs': True, 'far_no_return': True, 'depth_max': 2.,
             'min_valid_depth_fraction': .8, 'max_unknown_fraction': .9,
             'sample_stride': 2, 'sampling_bound_m': .006}
        q = {'unknown_cells': 0, 'total_cells': 10}
        k = np.diag([400., 400., 1.])
        with self.assertRaisesRegex(ValueError, 'sampling'):
            quality_check(np.full((5, 5), np.inf), q, c, k)
        c['sampling_bound_m'] = .011
        quality_check(np.full((5, 5), np.inf), q, c, k)
        with self.assertRaisesRegex(ValueError, 'quality'):
            quality_check(np.full((5, 5), np.nan), q, c, k)

    def test_incremental_visibility_is_exact_intersection_including_empty_mask(self):
        c = config()
        obstacle = Bounds(np.array([.62, -.15, .26]), np.array([.66, .15, .40]))
        depth, k, camera = synthetic_depth([obstacle])
        full, _, total = unknown_boxes(depth, k, camera, c, lambda p: False)
        full_mask = occupancy(full, c)
        rng = np.random.default_rng(51)
        for mask in (np.zeros_like(full_mask), np.ones_like(full_mask), rng.random(full_mask.shape) > .4):
            partial, count, size = unknown_boxes(depth, k, camera, c, lambda p: False,
                                                  candidate_occupancy=mask)
            self.assertTrue(np.array_equal(occupancy(partial, c), full_mask & mask))
            self.assertEqual(count, int((full_mask & mask).sum()))
            self.assertEqual(size, total)

    def test_visibility_roundoff_cannot_be_used_as_a_clearance_knob(self):
        c = config()
        c.update(far_no_return=True, visibility_margin=0., visibility_roundoff_m=1e-6)
        validate_config(c)
        for change in ({'visibility_roundoff_m': .001}, {'visibility_margin': -.001},
                       {'far_no_return': 'true'}, {'depth_encoding': '16UC1'},
                       {'sensor_reliability': 'automatic'}):
            with self.subTest(change=change), self.assertRaises(ValueError):
                validate_config({**c, **change})

    def test_unknown_intersection_but_observed_obstacles_always_retained(self):
        c = config()
        c.update(workspace_min=[0., 0., 0.], workspace_max=[.12, .12, .12], voxel_size=.04)
        first = Bounds(np.array([0., 0., 0.]), np.array([.08, .12, .12]))
        second = Bounds(np.array([.04, 0., 0.]), np.array([.12, .12, .12]))
        nearby_obstacle = Bounds(np.array([.01, .01, .01]), np.array([.02, .02, .02]))
        visible, unknown, _ = fuse_estimates([result([first], [nearby_obstacle]), result([second])], [c, c])
        expected = occupancy([first], c) & occupancy([second], c)
        self.assertTrue(np.array_equal(occupancy(unknown, c), expected))
        self.assertTrue(occupancy(visible, c)[0, 0, 0])
        visible, unknown, _ = fuse_estimates([result([first], [nearby_obstacle]), result()], [c, c])
        self.assertFalse(unknown)
        self.assertTrue(occupancy(visible, c)[0, 0, 0])  # a free witness cannot erase it

    def test_new_observation_invalidates_even_when_another_view_proves_free(self):
        c = config()
        c.update(workspace_min=[0., 0., 0.], workspace_max=[.12, .12, .12], voxel_size=.04)
        old = Bounds(np.array([.01, .01, .01]), np.array([.02, .02, .02]))
        new = Bounds(np.array([.079, .01, .01]), np.array([.081, .02, .02]))
        observed, unknown, _ = fuse_estimates([result(observed=[old]), result()], [c, c])
        baseline = occupancy(observed+unknown, c)
        observed, unknown, _ = fuse_estimates([result(observed=[old, new]), result()], [c, c])
        current = occupancy(observed+unknown, c)
        self.assertTrue(current[1, 0, 0] and current[2, 0, 0])
        with self.assertRaisesRegex(ValueError, 'previously free'):
            check_frozen_coverage(baseline, current)

    def test_observed_raster_clips_bounds_without_wrapping_negative_indices(self):
        c = config()
        c.update(workspace_min=[0., 0., 0.], workspace_max=[.12, .12, .12], voxel_size=.04)
        box = Bounds(np.array([-.02, .01, .01]), np.array([.01, .02, .02]))
        mask = occupancy([box], c, outward=True)
        self.assertEqual(int(mask.sum()), 1)
        self.assertTrue(mask[0, 0, 0])
        seen, _, _ = fuse_estimates([result(observed=[box]), result()], [c, c])
        self.assertTrue(any(b is box for b in seen))

    def test_missing_view_and_compound_budget_fail_closed(self):
        c = config()
        with self.assertRaisesRegex(ValueError, 'every configured'):
            fuse_estimates([result()], [c, c])
        c.update(workspace_min=[0., 0., 0.], workspace_max=[.12, .12, .12], voxel_size=.04)
        c['max_collision_boxes'] = 1
        box = Bounds(np.zeros(3), np.full(3, .02))
        other = Bounds(np.full(3, .09), np.full(3, .11))
        with self.assertRaisesRegex(ValueError, 'budget'):
            fuse_estimates([result(observed=[box]), result(observed=[other])], [c, c])

    def test_global_join_uses_older_common_anchor_but_never_expired_view(self):
        from collections import deque
        from types import MethodType
        from unittest.mock import patch
        from pick_place_perception.depth_obstacles import DepthObstacles
        def message(seconds):
            sec, ns = divmod(round(seconds*1e9), 10**9)
            return NS(header=NS(stamp=NS(sec=sec, nanosec=ns), frame_id='base_link'),
                      name=['j'], position=[0.], pose=NS(position=NS(x=0., y=0., z=0.),
                      orientation=NS(x=0., y=0., z=0., w=1.)))
        def history(stamps):
            return {key: deque(message(s) for s in stamps) for key in
                    ('rgb', 'depth', 'info', 'depth_info', 'joints', 'target')}
        c = config()
        c.update(max_input_age=3., max_time_skew=.15)
        clock = NS(now=lambda: NS(nanoseconds=101100000000))
        node = NS(config=c, views=[c, c], view_histories=[history([100., 101.]), history([100.02])],
                  monitor_inputs=False, models=[], joint_names={'j'}, get_clock=lambda: clock,
                  last_view_stamps=None,
                  lookup=lambda *args: np.eye(4))
        node.bundle = MethodType(DepthObstacles.bundle, node)
        node.paired_inputs = MethodType(DepthObstacles.paired_inputs, node)
        node.packet = MethodType(DepthObstacles.packet, node)
        node.target_pose_at = MethodType(DepthObstacles.target_pose_at, node)
        with patch('pick_place_perception.depth_obstacles.validate_bundle', return_value=np.eye(3)):
            packets = DepthObstacles.bundles(node)
            self.assertEqual(packets[0][0].header.stamp.sec, 100)
            self.assertEqual(len(packets), 2)
            node.last_view_stamps = [100500000000, 100500000000]
            with self.assertRaisesRegex(ValueError, 'no fresh capture-time'):
                DepthObstacles.bundles(node)
            node.last_view_stamps = None
            from tf2_ros import ExtrapolationException
            node.view_histories = [history([100., 101.]), history([100., 101.])]
            node.monitor_inputs = True
            packets = DepthObstacles.bundles(node)
            self.assertTrue(all(p[0].header.stamp.sec == 101 for p in packets))
            def delayed_tf(frame, stamp):
                if stamp.sec == 101:
                    raise ExtrapolationException('new image arrived before acquisition TF')
                return np.eye(4)
            node.lookup = delayed_tf
            packets = DepthObstacles.bundles(node)
            self.assertTrue(all(p[0].header.stamp.sec == 100 for p in packets))
            clock.now = lambda: NS(nanoseconds=104100000000)
            with self.assertRaisesRegex(ValueError, 'no fresh capture-time'):
                DepthObstacles.bundles(node)

    def test_high_rate_target_history_retains_delayed_image_acquisition_pose(self):
        import rclpy
        from rclpy.parameter import Parameter
        from unittest.mock import patch
        from pick_place_perception.depth_obstacles import DepthObstacles
        c = config()
        overrides = [Parameter(key, value=c[key]) for key in (
            'self_filter_model', 'rgb_image_topic', 'depth_image_topic',
            'camera_info_topic', 'depth_info_topic', 'joint_states_topic', 'target_pose_topic')]
        rclpy.init()
        node = None
        try:
            with patch('pick_place_perception.depth_obstacles.view_configs', return_value=[c]), \
                    patch('pick_place_perception.depth_obstacles.load_model', return_value=([], set(), 'test')), \
                    patch('pick_place_perception.depth_obstacles.ProcessPoolExecutor'):
                node = DepthObstacles(parameter_overrides=overrides)
            def message(ns):
                sec, nanosec = divmod(ns, 10**9)
                return NS(header=NS(stamp=NS(sec=sec, nanosec=nanosec)), x=ns*1e-9)
            for key in ('rgb', 'depth', 'info', 'depth_info', 'joints'):
                node.receive(key, message(2*10**9))
            # Simulate a 1 kHz independent target tracker while the image has
            # 200 ms transport/dispatch latency. Its matching pose must survive.
            for ns in range(2*10**9, 2200000001, 1000000):
                node.receive('target', message(ns))
            clock = NS(now=lambda: NS(nanoseconds=2200000000))
            with patch.object(node, 'get_clock', return_value=clock):
                paired, age, stamps = node.paired_inputs(c, node.histories)
            self.assertEqual(paired['target'].x, 2.)
            self.assertAlmostEqual(age, .2)
            self.assertEqual(max(stamps)-min(stamps), 0)
        finally:
            if node is not None:
                node.destroy_node()
            rclpy.shutdown()

    def test_moving_target_uses_bracketed_acquisition_pose_and_full_span(self):
        from collections import deque
        from types import MethodType
        from pick_place_perception.depth_obstacles import DepthObstacles
        from scipy.spatial.transform import Rotation
        def message(ns, x=0., yaw=0.):
            sec, nanosec = divmod(ns, 10**9)
            q = Rotation.from_euler('z', yaw).as_quat()
            return NS(header=NS(stamp=NS(sec=sec, nanosec=nanosec), frame_id='base_link'),
                      pose=NS(position=NS(x=x, y=0., z=0.),
                              orientation=NS(x=q[0], y=q[1], z=q[2], w=q[3])))
        c = {**config(), 'monitor_target_occlusion': True, 'max_input_age': 3., 'max_time_skew': .15}
        history = {key: deque([message(2*10**9)]) for key in ('rgb', 'depth', 'info', 'depth_info', 'joints')}
        # True target is at x=.5, yaw=pi/2 at the depth acquisition. Neither
        # endpoint is a valid substitute even though both pass the skew gate.
        history['target'] = deque([message(1990000000, .49, np.pi/2-.1),
                                   message(2010000000, .51, np.pi/2+.1)])
        node = NS(get_clock=lambda: NS(now=lambda: NS(nanoseconds=2100000000)),
                  lookup=lambda frame, stamp: np.eye(4))
        paired, age, stamps = DepthObstacles.paired_inputs(node, c, history)
        actual = DepthObstacles.target_pose_at(node, paired, 2*10**9)
        self.assertAlmostEqual(actual[0, 3], .5)
        np.testing.assert_allclose(actual[:3, :3], Rotation.from_euler('z', np.pi/2).as_matrix(), atol=1e-15)
        self.assertIn(1990000000, stamps)
        self.assertIn(2010000000, stamps)
        with self.assertRaisesRegex(ValueError, 'extrapolation'):
            DepthObstacles.target_pose_at(node, paired, 2020000000)
        history['target'].pop()
        with self.assertRaisesRegex(ValueError, 'no fresh capture-time'):
            DepthObstacles.paired_inputs(node, c, history)
        history['target'].append(message(2160000000, .66))
        with self.assertRaisesRegex(ValueError, 'no fresh capture-time'):
            DepthObstacles.paired_inputs(node, c, history)

    def test_explicit_view_configs_reject_mixed_grid_duplicate_inputs_and_nesting(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary)/'second.yaml'
            primary, secondary = config(), config()
            for key in ('camera_frame', 'rgb_image_topic', 'depth_image_topic', 'camera_info_topic', 'depth_info_topic'):
                secondary[key] += '_second'
            primary['additional_perception_configs'] = [str(path)]
            def write(value):
                path.write_text(yaml.safe_dump({'depth_obstacles': {'ros__parameters': value}}))
            write(secondary)
            self.assertEqual(len(view_configs(primary)), 2)
            for change in ({'voxel_size': .02}, {'camera_frame': primary['camera_frame']},
                           {'depth_image_topic': primary['depth_image_topic']},
                           {'additional_perception_configs': [str(path)]}):
                write({**secondary, **change})
                with self.subTest(change=change), self.assertRaises(ValueError):
                    view_configs(primary)

    def test_neighbour_obstacle_outside_exact_target_mask_is_preserved(self):
        c = config()
        c.update(far_no_return=True, visibility_margin=0., visibility_roundoff_m=1e-6)
        target = Bounds(np.array([.405, -.205, .26]), np.array([.455, -.155, .48]))
        # Five millimetres from the known target, outside its 3 mm point mask.
        obstacle = Bounds(np.array([.460, -.205, .26]), np.array([.48, -.155, .48]))
        depth, k, camera = synthetic_depth([target, obstacle])
        seen, unknown, _ = estimate(depth, k, camera, [], matrix(target.center, [0, 0, 0, 1]), c)
        point = np.array([.470, -.18, .48])
        self.assertTrue(any(((point >= b.lower-.005) & (point <= b.upper+.005)).all() for b in seen))
        self.assertTrue(unknown)  # no blanket target-neighbourhood exemption

    def test_declared_support_plane_requires_depth_evidence(self):
        c = config()
        c.update(workspace_min=[0., 0., .26], workspace_max=[.12, .12, .29],
                 voxel_size=.01, support_surface_enabled=True,
                 support_surface_height=.26, support_surface_normal=[0., 0., 1.],
                 support_surface_xy_min=[0., 0.], support_surface_xy_max=[.08, .08],
                 support_contact_height_tolerance=.003, support_uncertainty_budget_m=.035)
        depth = np.full((80, 80), np.nan)
        k = np.array([[40., 0., 40.], [0., 40., 40.], [0., 0., 1.]])
        _, count, total = unknown_boxes(depth, k, np.eye(4), c, lambda p: False)
        # A declared plane cannot clear missing depth, including the layer
        # immediately above it. This used to clear 64 cells unconditionally.
        self.assertEqual(count, total)
        camera = matrix([.04, .04, 1.], [1, 0, 0, 0])
        _, witnessed, _ = unknown_boxes(np.full_like(depth, .74), k, camera, c, lambda p: False)
        c['support_surface_enabled'] = False
        _, conservative, _ = unknown_boxes(np.full_like(depth, .74), k, camera, c, lambda p: False)
        self.assertGreater(conservative-witnessed, 0)
        self.assertLessEqual(conservative-witnessed, 64)

    def test_support_rule_retains_target_shadow_and_low_neighbour_obstacle(self):
        c = config()
        c.update(voxel_size=.01, visibility_margin=0., visibility_roundoff_m=1e-6,
                 support_surface_enabled=True, support_surface_height=.26,
                 support_surface_normal=[0., 0., 1.], support_surface_xy_min=[.31, -.30],
                 support_surface_xy_max=[.73, .30], support_contact_height_tolerance=.003,
                 support_uncertainty_budget_m=.035)
        target = Bounds(np.array([.405, -.205, .26]), np.array([.455, -.155, .48]))
        # An independent 9 mm obstacle is below the 15 mm cluster-height
        # threshold. Unknown-space protection must retain it, even only 5 mm
        # from the target and inside the support contact voxel layer.
        obstacle = Bounds(np.array([.460, -.205, .26]), np.array([.48, -.155, .269]))
        depth, k, camera = synthetic_depth([target, obstacle])
        _, unknown, _ = estimate(depth, k, camera, [], matrix(target.center, [0., 0., 0., 1.]), c)
        point = np.array([.47, -.18, .265])
        self.assertTrue(any(((point >= b.lower) & (point <= b.upper)).all() for b in unknown))
        depth[:] = np.nan
        _, unknown, _ = estimate(depth, k, camera, [], matrix(target.center, [0., 0., 0., 1.]), c)
        self.assertTrue(any(((point >= b.lower) & (point <= b.upper)).all() for b in unknown))

    def test_support_contract_is_fail_closed(self):
        c = config()
        c['support_surface_enabled'] = True
        for change in (
                {'support_surface_normal': [1., 0., 0.]},
                {'support_surface_xy_max': [.30, .30]},
                {'support_contact_height_tolerance': .04},
                {'support_uncertainty_budget_m': 0.},
                {'support_surface_height': .1},
                {'support_surface_xy_min': [.2, -.3]}):
            with self.subTest(change=change), self.assertRaises(ValueError):
                validate_config({**c, **change})


if __name__ == '__main__':
    unittest.main()
