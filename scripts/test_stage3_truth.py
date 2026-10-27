#!/usr/bin/env python3
"""Static live-export parser and public standard-input gate regressions."""

from pathlib import Path
import importlib.util
import json
import sys
import tempfile
import unittest
from unittest.mock import patch, Mock
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
for package in ('pick_place_scene', 'pick_place_sim'):
    sys.path.insert(0, str(ROOT/'src/ros2_adapters'/package))
from pick_place_sim.gazebo_geometry import extract_obstacles
from pick_place_sim.workbench_world import generate_world
from pick_place_sim.obstacle_scene import generate_scene, config_from
from pick_place_scene.static_geometry import Box
from moveit_msgs.srv import GetPlanningScene
from rclpy.parameter import Parameter
from std_msgs.msg import Bool
from std_msgs.msg import String
from std_srvs.srv import Trigger
import rclpy
from pick_place_scene.scene_sync import SceneSync
import test_scene_sync as existing

spec = importlib.util.spec_from_file_location('truth_sync', ROOT/
    'src/ros2_adapters/pick_place_sim/scripts/gazebo_obstacle_state_sync.py')
truth = importlib.util.module_from_spec(spec)
spec.loader.exec_module(truth)


class ExportTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        root = Path(self.temp.name)
        adapters = ROOT/'src/ros2_adapters'
        gazebo = ROOT/'src/realman_rm65/robot_gazebo'
        mount = gazebo/'urdf/rm65b_ctm2f110_gz.urdf.xacro'
        path, _ = generate_scene(adapters/'pick_place_scene/config/u_workbench.yaml',
            adapters/'pick_place_demo/config/known_scene.yaml', mount,
            config_from(adapters/'pick_place_sim/config/obstacles.yaml'), root/'scene')
        generate_world(gazebo/'worlds/rm65b_pick_place_fixed.sdf', path, mount, root/'world.sdf')
        self.tree = ET.parse(root/'world.sdf')
        self.name = 'obstacle_wall_00'
        self.model = self.tree.find(f"world/model[@name='{self.name}']")

    def read(self):
        return extract_obstacles(ET.tostring(self.tree.getroot(), encoding='unicode'),
                                 [self.name], (0., 0., .15))

    def test_actual_pose_size_and_absence(self):
        self.model.find('pose').text = '.68 .02 .50 0 0 .2'
        for kind in ('visual', 'collision'):
            self.model.find(f'link/{kind}/geometry/box/size').text = '.06 .28 .18'
        box = self.read()[self.name]
        self.assertEqual(box.size, (.06, .28, .18))
        for a, b in zip(box.pose, (.68, .02, .35, 0., 0., .2)):
            self.assertAlmostEqual(a, b)
        self.tree.find('world').remove(self.model)
        self.assertIsNone(self.read()[self.name])

    def test_reject_incomplete_or_ambiguous_export(self):
        for transform in ('duplicate', 'dynamic', 'link_offset', 'relative', 'shape_offset',
                          'size_mismatch', 'nan', 'wrong_world', 'mesh'):
            before = ET.tostring(self.tree.getroot())
            with self.subTest(transform=transform):
                if transform == 'duplicate':
                    self.tree.find('world').append(ET.fromstring(ET.tostring(self.model)))
                elif transform == 'dynamic':
                    self.model.find('static').text = 'false'
                elif transform == 'link_offset':
                    ET.SubElement(self.model.find('link'), 'pose').text = '0 0 1 0 0 0'
                elif transform == 'relative':
                    self.model.find('pose').set('relative_to', 'other')
                elif transform == 'shape_offset':
                    ET.SubElement(self.model.find('link/collision'), 'pose').text = '1 0 0 0 0 0'
                elif transform == 'size_mismatch':
                    self.model.find('link/collision/geometry/box/size').text = '1 1 1'
                elif transform == 'nan':
                    self.model.find('pose').text = 'nan 0 0 0 0 0'
                elif transform == 'wrong_world':
                    self.tree.find('world').set('name', 'wrong')
                elif transform == 'mesh':
                    self.model.find('link/collision/geometry/box').tag = 'mesh'
                with self.assertRaises(ValueError):
                    self.read()
            self.tree = ET.ElementTree(ET.fromstring(before))
            self.model = self.tree.find(f"world/model[@name='{self.name}']")
        with self.assertRaises(ET.ParseError):
            extract_obstacles('<sdf>', [self.name], (0., 0., .15))


class TruthFailureTest(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.temp = tempfile.TemporaryDirectory()
        archive = Path(self.temp.name)
        (archive/'manifest.json').write_text(json.dumps({
            'geometry': {'boxes': [{'id': 'obstacle_wall_00'}]},
            'world_to_base_translation': [0., 0., .15]}))
        self.node = truth.TruthSync(parameter_overrides=[Parameter('archive', value=str(archive))])
        self.node.publisher = Mock()
        self.node.ready = Mock()

    def tearDown(self):
        self.node.destroy_node()
        rclpy.try_shutdown()
        self.temp.cleanup()

    def test_missing_truth_never_falls_back_to_config_or_remove(self):
        with patch.object(truth, 'read_world', side_effect=RuntimeError('unavailable')):
            self.node.tick()
            self.node.worker.join(timeout=2)
            self.node.tick()
        self.assertIn('unavailable', self.node.last_error)
        self.node.publisher.publish.assert_not_called()
        self.assertFalse(self.node.ready.publish.call_args.args[0].data)
        response = self.node.refresh(None, Trigger.Response())
        self.assertFalse(response.success)
        self.assertIn('restart', response.message)

    def test_malformed_export_never_publishes_absence(self):
        with patch.object(truth, 'read_world', return_value='<sdf><world name="wrong"/></sdf>'):
            self.node.tick()
            self.node.worker.join(timeout=2)
            self.node.tick()
        self.assertTrue(self.node.last_error)
        self.node.publisher.publish.assert_not_called()

    def test_refresh_refused_after_execution_status(self):
        self.node.on_execution(String(data='waiting for planned solution and detached Gazebo object'))
        self.assertFalse(self.node.execution_seen)
        self.node.on_execution(String(data='executing Pre-grasp and approach'))
        response = self.node.refresh(None, Trigger.Response())
        self.assertFalse(response.success)
        self.assertIn('after execution', response.message)

    def test_differences_are_serial_and_never_republished_while_waiting(self):
        self.node.worker = Mock()
        self.node.worker.is_alive.return_value = False
        self.node.deadline = float('inf')
        self.node.pending = ('<sdf/>', {
            name: Box(name, (.1, .1, .1), (.6, .4, .4, 0., 0., 0.))
            for name in ('obstacle_wall_00', 'obstacle_cube_01')}, None)
        self.node.get_scene = Mock()
        self.node.get_scene.service_is_ready.return_value = True
        self.node.publisher.get_subscription_count.return_value = 1
        self.node.tick()  # First query before publishing.
        for _ in range(4):
            previous = self.node.publisher.publish.call_count
            self.node.future = Mock()
            self.node.future.done.return_value = True
            self.node.future.result.return_value = GetPlanningScene.Response()
            self.node.next_send = 0.
            self.node.tick()
            self.assertLessEqual(self.node.publisher.publish.call_count-previous, 1)
        self.assertEqual([call.args[0].id for call in self.node.publisher.publish.call_args_list],
                         ['obstacle_wall_00', 'obstacle_cube_01'])


class InputGateTest(existing.SceneSyncTest):
    def restart_gated(self):
        self.executor.remove_node(self.node)
        self.node.destroy_node()
        self.node = SceneSync(parameter_overrides=[
            Parameter('get_scene_service', value=self.prefix+'/get'),
            Parameter('apply_service', value=self.prefix+'/apply'),
            Parameter('observation_topic', value=self.prefix+'/observations'),
            Parameter('allowed_ids', value=['pick_object', 'second']),
            Parameter('required_ids', value=['pick_object', 'second']),
            Parameter('input_ready_topic', value=self.prefix+'/source_ready'),
            Parameter('service_timeout_sec', value=.3),
            Parameter('max_observation_age_sec', value=2.),
        ])
        self.executor.add_node(self.node)
        self.wait(lambda: self.node.scene_client.service_is_ready() and self.node.apply_client.service_is_ready())

    def test_required_external_objects_and_upstream_gate(self):
        self.restart_gated()
        self.node.on_input_ready(Bool(data=True))
        self.node.on_observation(self.observation())
        self.wait(lambda: self.node.scene_revision == 1)
        self.assertFalse(self.node.scene_ready)
        self.node.on_observation(self.observation('second'))
        self.wait(lambda: self.node.scene_ready)
        self.node.on_input_ready(Bool(data=False))
        self.assertFalse(self.node.scene_ready)
        self.assertEqual(self.node.scene_revision, 2)
        self.node.on_input_ready(Bool(data=True))
        self.assertTrue(self.node.scene_ready)
        removal = self.observation('second')
        removal.operation = removal.REMOVE
        self.node.on_observation(removal)
        self.wait(lambda: self.node.scene_revision == 3)
        self.assertFalse(self.node.scene_ready)
        self.assertNotIn('second', self.node.confirmed_ids)

    def test_upstream_not_ready_blocks_even_after_all_confirmed(self):
        self.restart_gated()
        for name in ('pick_object', 'second'):
            self.node.on_observation(self.observation(name))
        self.wait(lambda: self.node.scene_revision == 2)
        self.assertFalse(self.node.scene_ready)


if __name__ == '__main__':
    unittest.main()
