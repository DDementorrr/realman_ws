#!/usr/bin/env python3
"""Geometry, real ROS mock confirmation, and legacy-default regressions."""

from copy import deepcopy
from pathlib import Path
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET

import yaml
from rclpy.parameter import Parameter
from std_msgs.msg import Bool

ROOT = Path(__file__).resolve().parents[1]
for package in ('pick_place_scene', 'pick_place_sim', 'pick_place_demo'):
    sys.path.insert(0, str(ROOT / 'src/ros2_adapters' / package))

from pick_place_scene.static_geometry import collision_object, load_geometry
from pick_place_scene.scene_sync import SceneSync
from pick_place_sim.workbench_world import generate_world
from pick_place_demo.known_scene import KnownScene
import test_scene_sync as existing

GEOMETRY = ROOT / 'src/ros2_adapters/pick_place_scene/config/u_workbench.yaml'
GAZEBO = ROOT / 'src/realman_rm65/robot_gazebo'


class GeometryTest(unittest.TestCase):
    def test_shared_conversions_and_template_preserved(self):
        before = (GAZEBO / 'worlds/rm65b_pick_place_fixed.sdf').read_bytes()
        frame, boxes = load_geometry(GEOMETRY)
        self.assertEqual(frame, 'base_link')
        self.assertEqual({b.id for b in boxes}, {'pick_table', 'left_table', 'right_table'})
        with tempfile.TemporaryDirectory() as temporary:
            output = generate_world(GAZEBO / 'worlds/rm65b_pick_place_fixed.sdf', GEOMETRY,
                                    GAZEBO / 'urdf/rm65b_ctm2f110_gz.urdf.xacro',
                                    Path(temporary) / 'world.sdf')
            world = ET.parse(output).find('world')
            self.assertIsNotNone(world.find("model[@name='pick_object']"))
            self.assertIsNotNone(world.find("model[@name='place_zone']"))
            for box in boxes:
                self.assertEqual(len(world.findall(f"model[@name='{box.id}']")), 1)
                model = world.find(f"model[@name='{box.id}']")
                pose = tuple(map(float, model.findtext('pose').split()))
                expected = (box.pose[0], box.pose[1], box.pose[2] + .15) + box.pose[3:]
                for actual, value in zip(pose, expected):
                    self.assertAlmostEqual(actual, value)
                for kind in ('visual', 'collision'):
                    size = tuple(map(float, model.findtext(f'link/{kind}/geometry/box/size').split()))
                    self.assertEqual(size, box.size)
                obj = collision_object(frame, box)
                self.assertEqual(obj.id, box.id)
                self.assertEqual(tuple(obj.primitives[0].dimensions), box.size)
                self.assertEqual(obj.primitive_poses[0].position.z, box.pose[2])
        self.assertEqual((GAZEBO / 'worlds/rm65b_pick_place_fixed.sdf').read_bytes(), before)

    def test_invalid_definitions_rejected(self):
        original = yaml.safe_load(GEOMETRY.read_text())
        variants = []
        for key, value in [('size', [0., .2, .1]), ('size', [-1., .2, .1]),
                           ('size', [True, .2, .1]), ('size', [float('nan'), .2, .1]),
                           ('size', [.2, .1]), ('pose', [0.] * 5),
                           ('pose', [0., 0., float('inf'), 0., 0., 0.]), ('id', '../bad')]:
            variant = deepcopy(original)
            variant['boxes'][0][key] = value
            variants.append(variant)
        duplicate = deepcopy(original)
        duplicate['boxes'][1]['id'] = duplicate['boxes'][0]['id']
        variants.extend([duplicate, {'frame_id':'base_link', 'boxes':[]},
                         {**original, 'sdf':'forbidden'}, {**original, 'frame_id':''}])
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / 'input.yaml'
            for variant in variants:
                with self.subTest(variant=variant):
                    path.write_text(yaml.safe_dump(variant))
                    with self.assertRaises(ValueError):
                        load_geometry(path)

    def test_u_opening_and_legacy_support(self):
        _, boxes = load_geometry(GEOMETRY)
        back, left, right = boxes
        self.assertAlmostEqual(back.pose[0] - back.size[0]/2,
                               left.pose[0] + left.size[0]/2)
        self.assertAlmostEqual(right.pose[0] + right.size[0]/2,
                               back.pose[0] - back.size[0]/2)
        self.assertAlmostEqual(left.pose[1] - left.size[1]/2 -
                               (right.pose[1] + right.size[1]/2), .70)
        for box in boxes:
            self.assertAlmostEqual(box.pose[2] + box.size[2]/2, .26)
        for x, y in [(.43, -.18), (.43, .18)]:
            self.assertLessEqual(abs(x-back.pose[0])+.025, back.size[0]/2)
            self.assertLessEqual(abs(y-back.pose[1])+.025, back.size[1]/2)


class StaticSceneTest(existing.SceneSyncTest):
    # Also run all 18 existing scene-service tests unchanged.
    def restart_static(self):
        self.executor.remove_node(self.node)
        self.node.destroy_node()
        self.node = SceneSync(parameter_overrides=[
            Parameter('get_scene_service', value=self.prefix + '/get'),
            Parameter('apply_service', value=self.prefix + '/apply'),
            Parameter('observation_topic', value=self.prefix + '/observations'),
            Parameter('static_scene_file', value=str(GEOMETRY)),
            Parameter('service_timeout_sec', value=.3),
            Parameter('max_observation_age_sec', value=2.),
        ])
        self.executor.add_node(self.node)

    def test_static_all_parts_confirmed_once(self):
        self.confirm_delay = .1
        self.restart_static()
        self.settle(.05)
        self.assertFalse(self.node.scene_ready)
        self.wait(lambda: self.node.scene_ready)
        self.assertEqual(self.node.confirmed_ids, {'pick_table', 'left_table', 'right_table'})
        self.assertEqual(self.node.scene_revision, 3)
        self.assertEqual(len(self.applies), 3)
        self.settle(.6)
        self.assertEqual(len(self.applies), 3, 'no periodic scene rewrite')
        self.assertEqual(self.node.scene_revision, 3)

    def test_static_missing_confirmation_blocks_start(self):
        self.confirm_mode = 'missing'
        self.restart_static()
        self.wait(lambda: len(self.node.locked_ids) == 3)
        self.assertFalse(self.node.scene_ready)
        self.assertEqual(self.node.scene_revision, 0)

    def test_static_partial_confirmation_never_ready(self):
        self.restart_static()
        self.node.locked_ids.add('right_table')
        self.wait(lambda: len(self.node.confirmed_ids) == 2)
        self.assertFalse(self.node.scene_ready)

    def test_static_legacy_defaults_and_pose_gate(self):
        legacy = KnownScene()
        self.assertTrue(legacy.publish_table)
        self.assertTrue(legacy.environment_ready)
        legacy.destroy_node()
        known = KnownScene(parameter_overrides=[
            Parameter('publish_table', value=False),
            Parameter('environment_ready_topic', value=self.prefix + '/ready')])
        self.executor.add_node(known)
        try:
            poses = []
            from geometry_msgs.msg import PoseStamped
            from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
            qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
            subscription = self.mock.create_subscription(PoseStamped, '/object_pose', poses.append, qos)
            known.publish_poses()
            self.settle(.1)
            self.assertEqual(poses, [])
            known.on_environment_ready(Bool(data=True))
            known.publish_poses()
            self.settle(.1)
            self.assertEqual(poses, [], 'target ADD must also be complete')
            known.apply_client = known.create_client(existing.ApplyPlanningScene, self.prefix + '/apply')
            self.wait(known.apply_client.service_is_ready)
            known._send_scene(add=True)
            self.wait(lambda: known.scene_ready)
            self.assertEqual([o.id for o in self.applies[-1].world.collision_objects], ['pick_object'])
            known.publish_poses()
            self.wait(lambda: bool(poses))
            known.on_environment_ready(Bool(data=False))
            known.publish_poses()
            self.mock.destroy_subscription(subscription)
        finally:
            self.executor.remove_node(known)
            known.destroy_node()


if __name__ == '__main__':
    unittest.main()
