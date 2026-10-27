#!/usr/bin/env python3
"""Exercise the public scene node with ROS messages and real mock services."""

from copy import deepcopy
import math
import time
import unittest
import uuid

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject, PlanningSceneComponents
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from rclpy.task import Future
from shape_msgs.msg import SolidPrimitive

from pick_place_scene.scene_sync import SceneSync


class SceneSyncTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.prefix = '/scene_test_' + uuid.uuid4().hex
        self.mock = Node('mock_' + uuid.uuid4().hex)
        self.query_delay = self.apply_delay = self.confirm_delay = 0.0
        self.attached = False
        self.apply_success = True
        self.confirm_mode = 'match'
        self.applies, self.queries, self.world = [], [], []
        self.returned_queries = 0
        self.delay_timers = []
        group = ReentrantCallbackGroup()
        self.mock.create_service(GetPlanningScene, self.prefix + '/get', self.get_scene,
                                 callback_group=group)
        self.mock.create_service(ApplyPlanningScene, self.prefix + '/apply', self.apply_scene,
                                 callback_group=group)
        self.node = SceneSync(parameter_overrides=[
            Parameter('get_scene_service', value=self.prefix + '/get'),
            Parameter('apply_service', value=self.prefix + '/apply'),
            Parameter('observation_topic', value=self.prefix + '/observations'),
            Parameter('allowed_ids', value=['pick_object', 'second']),
            Parameter('service_timeout_sec', value=0.3),
            Parameter('max_observation_age_sec', value=2.0),
        ])
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.mock.create_publisher(CollisionObject, self.prefix + '/observations', qos)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.mock)
        self.executor.add_node(self.node)
        self.wait(lambda: self.node.scene_client.service_is_ready()
                  and self.node.apply_client.service_is_ready()
                  and self.publisher.get_subscription_count() == 1)

    def tearDown(self):
        # Drain delayed mock replies before destroying service handles.
        self.settle(0.75)
        self.executor.remove_node(self.node)
        self.executor.remove_node(self.mock)
        self.executor.shutdown(timeout_sec=3.0)
        self.node.destroy_node()
        self.mock.destroy_node()

    def wait(self, condition, timeout=3.0):
        deadline = time.monotonic() + timeout
        while not condition() and time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.01)
        self.assertTrue(condition(), 'bounded wait expired')

    def settle(self, seconds=0.15):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.01)

    def observation(self, object_id='pick_object'):
        obj = CollisionObject()
        obj.id = object_id
        obj.header.frame_id = 'base_link'
        obj.header.stamp = self.node.get_clock().now().to_msg()
        obj.operation = CollisionObject.ADD
        obj.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[0.05, 0.07, 0.22])]
        pose = Pose()
        pose.position.z = 0.44
        pose.orientation.w = 1.0
        obj.primitive_poses = [pose]
        return obj

    async def delay(self, seconds):
        if seconds <= 0:
            return
        future = Future(executor=self.executor)

        def complete():
            timer.cancel()
            future.set_result(None)

        timer = self.mock.create_timer(seconds, complete)
        self.delay_timers.append(timer)
        await future

    async def get_scene(self, request, response):
        component = request.components.components
        self.queries.append(component)
        if component == PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS:
            await self.delay(self.query_delay)
            if self.attached:
                item = AttachedCollisionObject()
                item.object.id = 'pick_object'
                response.scene.robot_state.attached_collision_objects = [item]
            self.returned_queries += 1
        else:
            await self.delay(self.confirm_delay)
            objects = deepcopy(self.world)
            if self.confirm_mode == 'missing':
                objects = []
            elif self.confirm_mode == 'remove_present' and not objects:
                objects = [self.observation()]
            elif objects and self.confirm_mode == 'size':
                objects[0].primitives[0].dimensions[0] += 0.01
            elif objects and self.confirm_mode == 'pose':
                objects[0].primitive_poses[0].position.x += 0.01
            elif objects and self.confirm_mode == 'frame':
                objects[0].header.frame_id = 'wrong_frame'
            elif objects and self.confirm_mode == 'top_pose':
                objects[0].pose.position.x = 1.0
            elif objects and self.confirm_mode == 'sign':
                objects[0].primitive_poses[0].orientation.w = -1.0
            elif objects and self.confirm_mode == 'canonical':
                objects[0].pose = objects[0].primitive_poses[0]
                objects[0].primitive_poses = [Pose()]
                objects[0].primitive_poses[0].orientation.w = 1.0
            response.scene.world.collision_objects = objects
        return response

    async def apply_scene(self, request, response):
        self.applies.append(deepcopy(request.scene))
        await self.delay(self.apply_delay)
        if self.apply_success:
            for obj in request.scene.world.collision_objects:
                if obj.operation == CollisionObject.REMOVE:
                    self.world = [item for item in self.world if item.id != obj.id]
                else:
                    self.world = [item for item in self.world if item.id != obj.id]
                    self.world.append(deepcopy(obj))
        response.success = self.apply_success
        return response

    def test_success_and_quaternion_sign(self):
        self.confirm_mode = 'sign'
        self.publisher.publish(self.observation())
        self.wait(lambda: 'pick_object' in self.node.confirmed_ids)
        self.assertEqual(len(self.applies), 1)
        self.assertEqual(self.node.scene_revision, 1)
        self.assertTrue(self.node.scene_ready)
        self.assertEqual(self.queries, [PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS,
                                        PlanningSceneComponents.WORLD_OBJECT_GEOMETRY])
        self.assertTrue(self.applies[0].is_diff)
        self.assertTrue(self.applies[0].robot_state.is_diff)

    def test_new_observation_invalidates_ready_until_confirmation(self):
        self.publisher.publish(self.observation())
        self.wait(lambda: self.node.scene_revision == 1)
        self.assertTrue(self.node.scene_ready)
        self.query_delay = 0.2
        self.publisher.publish(self.observation())
        self.wait(lambda: not self.node.scene_ready)
        self.assertFalse(self.node.scene_ready)
        self.wait(lambda: self.node.scene_revision == 2)
        self.assertTrue(self.node.scene_ready)

    def test_missing_services_never_apply_or_confirm(self):
        self.node.scene_client.destroy()
        self.node.scene_client = self.node.create_client(GetPlanningScene, self.prefix + '/missing_get')
        self.publisher.publish(self.observation())
        self.settle()
        self.assertFalse(self.node.transactions)
        self.assertFalse(self.node.confirmed_ids)
        self.assertEqual(self.applies, [])

    def test_missing_apply_service_releases_without_confirmation(self):
        self.node.apply_client.destroy()
        self.node.apply_client = self.node.create_client(ApplyPlanningScene, self.prefix + '/missing_apply')
        self.publisher.publish(self.observation())
        self.wait(lambda: self.returned_queries == 1 and not self.node.transactions)
        self.assertFalse(self.node.confirmed_ids)
        self.assertFalse(self.node.locked_ids)
        self.assertEqual(self.applies, [])

    def test_attached_refuses_apply(self):
        self.attached = True
        self.publisher.publish(self.observation())
        self.wait(lambda: self.returned_queries == 1 and not self.node.transactions)
        self.assertEqual(self.applies, [])

    def test_moveit_canonical_pose_is_geometrically_equivalent(self):
        self.confirm_mode = 'canonical'
        obj = self.observation()
        obj.primitive_poses[0].orientation.z = math.sin(0.3)
        obj.primitive_poses[0].orientation.w = math.cos(0.3)
        self.publisher.publish(obj)
        self.wait(lambda: 'pick_object' in self.node.confirmed_ids)

    def test_remove_success_confirms_absence_and_advances_revision(self):
        self.publisher.publish(self.observation())
        self.wait(lambda: self.node.scene_revision == 1)
        removal = self.observation()
        removal.operation = CollisionObject.REMOVE
        removal.primitives = []
        removal.primitive_poses = []
        self.publisher.publish(removal)
        self.wait(lambda: self.node.scene_revision == 2)
        self.assertNotIn('pick_object', self.node.confirmed_ids)
        self.assertFalse(self.node.scene_ready)
        self.assertEqual(len(self.applies), 2)
        self.assertEqual(self.applies[1].world.collision_objects[0].operation,
                         CollisionObject.REMOVE)
        self.assertFalse(self.world)

    def test_attached_remove_is_rejected_and_preserves_confirmation(self):
        self.publisher.publish(self.observation())
        self.wait(lambda: 'pick_object' in self.node.confirmed_ids)
        self.attached = True
        removal = self.observation()
        removal.operation = CollisionObject.REMOVE
        removal.primitives = []
        removal.primitive_poses = []
        self.publisher.publish(removal)
        self.wait(lambda: not self.node.transactions)
        self.assertTrue(self.node.scene_ready)
        self.assertIn('pick_object', self.node.confirmed_ids)
        self.assertEqual(len(self.applies), 1)

    def test_remove_apply_rejection_locks(self):
        self.publisher.publish(self.observation())
        self.wait(lambda: 'pick_object' in self.node.confirmed_ids)
        self.apply_success = False
        removal = self.observation()
        removal.operation = CollisionObject.REMOVE
        removal.primitives = []
        removal.primitive_poses = []
        self.publisher.publish(removal)
        self.wait(lambda: 'pick_object' in self.node.locked_ids)
        self.assertFalse(self.node.scene_ready)
        self.assertNotIn('pick_object', self.node.confirmed_ids)

    def test_remove_confirmation_mismatch_locks(self):
        self.publisher.publish(self.observation())
        self.wait(lambda: 'pick_object' in self.node.confirmed_ids)
        self.confirm_mode = 'remove_present'
        removal = self.observation()
        removal.operation = CollisionObject.REMOVE
        removal.primitives = []
        removal.primitive_poses = []
        self.publisher.publish(removal)
        self.wait(lambda: 'pick_object' in self.node.locked_ids)
        self.assertFalse(self.node.scene_ready)

    def test_invalid_observations(self):
        cases = []
        for kind in ('stale', 'future', 'nan', 'quaternion', 'extra', 'top_pose', 'zero_stamp'):
            obj = self.observation()
            if kind == 'stale':
                obj.header.stamp.sec -= 5
            elif kind == 'future':
                obj.header.stamp.sec += 5
            elif kind == 'nan':
                obj.primitives[0].dimensions[0] = math.nan
            elif kind == 'quaternion':
                obj.primitive_poses[0].orientation.w = 2.0
            elif kind == 'extra':
                obj.subframe_names = ['unsupported']
                obj.subframe_poses = [Pose()]
            elif kind == 'top_pose':
                obj.pose.position.x = 1.0
            elif kind == 'zero_stamp':
                obj.header.stamp = Time().to_msg()
            cases.append(obj)
        for obj in cases:
            self.publisher.publish(obj)
            self.settle(0.04)
        self.assertEqual(self.queries, [])
        self.assertEqual(self.applies, [])

    def test_apply_rejection_locks(self):
        self.apply_success = False
        self.publisher.publish(self.observation())
        self.wait(lambda: 'pick_object' in self.node.locked_ids)
        self.assertNotIn('pick_object', self.node.confirmed_ids)

    def test_query_timeout_recovery_and_late_response(self):
        self.query_delay = 0.65
        self.publisher.publish(self.observation())
        self.wait(lambda: len(self.queries) == 1)
        self.wait(lambda: not self.node.transactions)
        self.assertFalse(self.node.locked_ids)
        self.query_delay = 0.0
        self.publisher.publish(self.observation())
        self.wait(lambda: 'pick_object' in self.node.confirmed_ids)
        self.wait(lambda: self.returned_queries == 2)
        self.settle()
        self.assertEqual(len(self.applies), 1)
        self.assertFalse(self.node.transactions)

    def test_observation_expires_during_query(self):
        self.node.max_age_ns = 50_000_000
        self.query_delay = 0.12
        self.publisher.publish(self.observation())
        self.wait(lambda: self.returned_queries == 1 and not self.node.transactions)
        self.assertEqual(self.applies, [])

    def test_apply_timeout_locks_only_affected_id(self):
        self.apply_delay = 0.65
        self.publisher.publish(self.observation())
        self.wait(lambda: 'pick_object' in self.node.locked_ids)
        self.apply_delay = 0.0
        self.publisher.publish(self.observation('second'))
        self.wait(lambda: 'second' in self.node.confirmed_ids)
        self.publisher.publish(self.observation())
        self.settle(0.7)
        self.assertEqual(len(self.applies), 2)
        self.assertNotIn('pick_object', self.node.confirmed_ids)

    def test_confirmation_mismatch_locks(self):
        for mode in ('missing', 'size', 'pose', 'frame', 'top_pose'):
            with self.subTest(mode=mode):
                self.confirm_mode = mode
                self.publisher.publish(self.observation())
                self.wait(lambda: 'pick_object' in self.node.locked_ids)
                self.assertNotIn('pick_object', self.node.confirmed_ids)
                # Only reset diagnostic state between independent injected cases.
                self.node.locked_ids.clear()

    def test_compound_snapshot_opt_in_and_all_primitives_confirmed(self):
        obj = self.observation()
        obj.primitives.append(deepcopy(obj.primitives[0]))
        obj.primitive_poses.append(deepcopy(obj.primitive_poses[0]))
        obj.primitive_poses[1].position.x = 0.2
        self.assertFalse(self.node.valid(obj))
        self.node.max_primitives = 2
        self.assertTrue(self.node.valid(obj))
        self.publisher.publish(obj)
        self.wait(lambda: 'pick_object' in self.node.confirmed_ids)
        self.assertEqual(len(self.applies[-1].world.collision_objects[0].primitives), 2)
        bad = deepcopy(obj)
        bad.primitives[1].dimensions[0] += .01
        from pick_place_scene.scene_sync import geometry_matches
        self.assertFalse(geometry_matches(bad, obj))
        bad = deepcopy(obj)
        bad.primitive_poses[1].orientation.w = 0.
        self.assertFalse(self.node.valid(bad))

    def test_uncertainty_applied_once_and_confirmed_at_moveit_boundary(self):
        from pick_place_scene.uncertainty import load_budget
        from pathlib import Path
        self.node.uncertainty = load_budget(Path(__file__).resolve().parents[1]/
            'src/ros2_adapters/pick_place_scene/config/sim_uncertainty_example.yaml')
        # The standard object id is selectable; fixture target demonstrates the
        # same conversion path without changing the production budget allowlist.
        self.node.uncertainty[0]['object_ids'] = ['pick_object']
        obj = self.observation()
        before = list(obj.primitives[0].dimensions)
        self.publisher.publish(obj)
        self.wait(lambda: 'pick_object' in self.node.confirmed_ids)
        dimensions = self.applies[-1].world.collision_objects[0].primitives[0].dimensions
        self.assertTrue(all(abs(a-b-2*self.node.uncertainty[2]) < 1e-10 for a, b in zip(dimensions, before)))
        self.assertEqual(list(obj.primitives[0].dimensions), before)
        self.assertTrue(self.node.scene_ready)

    def test_silent_input_source_expires_scene_in_steady_time(self):
        from std_msgs.msg import Bool
        self.publisher.publish(self.observation())
        self.wait(lambda: self.node.scene_ready)
        self.node.input_timeout_ns = 100_000_000
        self.node.on_input_ready(Bool(data=True))
        self.wait(lambda: not self.node.scene_ready)
        self.assertFalse(self.node.input_ready)
        self.assertEqual(self.node.scene_revision, 1)

    def test_confirm_timeout_locks(self):
        self.confirm_delay = 0.65
        self.publisher.publish(self.observation())
        self.wait(lambda: 'pick_object' in self.node.locked_ids)
        self.settle(0.7)
        self.assertNotIn('pick_object', self.node.confirmed_ids)

    def test_paused_sim_time_does_not_pause_timeout(self):
        self.node.set_parameters([Parameter('use_sim_time', value=True)])
        self.node.get_clock().set_ros_time_override(Time(seconds=10))
        self.query_delay = 0.65
        self.publisher.publish(self.observation())
        self.wait(lambda: bool(self.queries))
        self.wait(lambda: not self.node.transactions)
        self.assertEqual(self.node.get_clock().now().nanoseconds, 10_000_000_000)
        self.assertEqual(self.applies, [])


if __name__ == '__main__':
    unittest.main()
