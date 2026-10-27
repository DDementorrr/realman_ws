#!/usr/bin/env python3
"""Real ROS service mock tests of the simulation-owned startup gate."""

import importlib.util
import json
from pathlib import Path
import tempfile
import time
import unittest

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from moveit_msgs.msg import CollisionObject, ContactInformation
from moveit_msgs.srv import GetPlanningScene, GetStateValidity
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from sensor_msgs.msg import JointState

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('obstacle_preflight', ROOT /
    'src/ros2_adapters/pick_place_sim/scripts/obstacle_preflight.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class PreflightTest(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.temp = tempfile.TemporaryDirectory()
        self.archive = Path(self.temp.name)
        self.mock = rclpy.create_node('mock_preflight_services')
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.mock)
        self.valid = True
        self.have_state = True
        self.mock.create_service(GetPlanningScene, '/get_planning_scene', self.query)
        self.mock.create_service(GetStateValidity, '/check_state_validity', self.check)
        self.node = None

    def tearDown(self):
        for node in (self.node, self.mock):
            if node:
                self.executor.remove_node(node)
                node.destroy_node()
        self.executor.shutdown()
        rclpy.shutdown()
        self.temp.cleanup()

    def query(self, _request, response):
        if self.have_state:
            response.scene.robot_state.joint_state.name = [f'joint{i}' for i in range(1, 7)]
            response.scene.robot_state.joint_state.position = [0.]*6
            response.scene.world.collision_objects = [CollisionObject(id='pick_object')]
        return response

    def check(self, _request, response):
        response.valid = self.valid
        if not self.valid:
            response.contacts = [ContactInformation(contact_body_1='Link3', contact_body_2='obstacle_wall_00')]
        return response

    def start(self, expected='candidate'):
        (self.archive/'manifest.json').write_text(json.dumps(
            {'expected': expected, 'reason': ['occupied_pick_or_place_zone'] if expected == 'blocked' else []}))
        self.node = module.Preflight(parameter_overrides=[Parameter('archive', value=str(self.archive))])
        self.executor.add_node(self.node)
        positions = {f'joint{i}': 0. for i in range(1, 7)}
        positions.update({name: .3*sign for name, sign in module.MIMICS.items()})
        positions['ctm2f110_finger_joint'] = .3
        self.node.on_joints(JointState(name=list(positions), position=list(positions.values())))

    def wait(self, predicate):
        deadline = time.monotonic()+5
        while not predicate() and time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=.05)
        self.assertTrue(predicate())

    def result(self):
        self.wait(lambda: self.node.done)
        return json.loads((self.archive/'startup.json').read_text())

    def test_valid_state_requires_environment_confirmation(self):
        self.start()
        for _ in range(5):
            self.executor.spin_once(timeout_sec=.05)
        self.assertFalse(self.node.done)
        self.node.on_scene(Bool(data=True))
        self.assertTrue(self.result()['allowed'])

    def test_initial_robot_collision_blocks_planning(self):
        self.valid = False
        self.start()
        self.node.on_scene(Bool(data=True))
        result = self.result()
        self.assertFalse(result['allowed'])
        self.assertEqual(result['reason'], 'initial_robot_collision')
        self.assertIn(['Link3', 'obstacle_wall_00'], result['contacts'])

    def test_missing_state_times_out_without_readiness(self):
        self.have_state = False
        self.start()
        self.node.on_scene(Bool(data=True))
        self.node.started -= 91
        result = self.result()
        self.assertFalse(result['allowed'])
        self.assertIn('timeout', result['reason'])
        self.assertTrue(result['waiting']['scene_confirmed'])
        self.assertTrue(result['waiting']['joint_feedback_valid'])
        self.assertIsNone(result['waiting']['last_scene_object_ids'])

    def test_expected_blocked_never_allows_explicit_execution(self):
        self.start('blocked')
        self.node.on_scene(Bool(data=True))
        self.assertFalse(self.result()['allowed'])

        client = self.mock.create_client(Trigger, '/pick_place/execute')
        self.wait(client.service_is_ready)
        future = client.call_async(Trigger.Request())
        self.wait(future.done)
        self.assertFalse(future.result().success)
        self.assertIn('occupied_pick_or_place_zone', future.result().message)

    def test_timeout_identifies_missing_scene_confirmation(self):
        self.start()
        self.node.started -= 91
        result = self.result()
        self.assertFalse(result['allowed'])
        self.assertFalse(result['waiting']['scene_confirmed'])
        self.assertFalse(result['waiting']['request_pending'])

    def test_partial_or_inconsistent_feedback_cannot_open_gate(self):
        self.start()
        self.node.joints = None
        self.node.on_scene(Bool(data=True))
        self.node.on_joints(JointState(name=['joint1'], position=[0.]))
        self.assertIsNone(self.node.joints)
        positions = {f'joint{i}': 0. for i in range(1, 7)}
        positions.update({name: 0. for name in module.MIMICS})
        positions['ctm2f110_finger_joint'] = .3
        self.node.on_joints(JointState(name=list(positions), position=list(positions.values())))
        self.assertIsNone(self.node.joints)
        self.node.started -= 91
        result = self.result()
        self.assertFalse(result['allowed'])
        self.assertFalse(result['waiting']['joint_feedback_valid'])


if __name__ == '__main__':
    unittest.main()
