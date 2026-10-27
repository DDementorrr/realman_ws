#!/usr/bin/env python3
"""One-shot static-scene initial robot collision gate, not runtime monitoring."""

import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from moveit_msgs.msg import PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene, GetStateValidity
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from sensor_msgs.msg import JointState

MIMICS = {'ctm2f110_rightout_joint': 1, 'ctm2f110_left_kckle_joint': 1,
          'ctm2f110_right_kckle_joint': 1, 'ctm2f110_leftinn_joint': -1,
          'ctm2f110_rightinn_joint': -1}


class Preflight(Node):
    def __init__(self, **kwargs):
        super().__init__('obstacle_preflight', **kwargs)
        self.declare_parameter('archive', '')
        self.archive = Path(self.get_parameter('archive').value)
        self.manifest = json.loads((self.archive / 'manifest.json').read_text())
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.ready = self.create_publisher(Bool, '/obstacles/startup_ready', qos)
        self.status = self.create_publisher(String, '/obstacles/startup_status', qos)
        self.create_subscription(Bool, '/perception/scene_ready', self.on_scene, qos)
        self.create_subscription(JointState, '/joint_states', self.on_joints, qos_profile_sensor_data)
        self.joints = None
        self.joints_at = 0.
        self.query = self.create_client(GetPlanningScene, '/get_planning_scene')
        self.validity = self.create_client(GetStateValidity, '/check_state_validity')
        self.confirmed = self.done = False
        self.future = None
        self.stage = 'query'
        self.last_scene_ids = None
        self.last_scene_joint_names = None
        self.started = time.monotonic()
        if self.manifest['expected'] == 'blocked':
            self.create_service(Trigger, '/pick_place/execute', self.reject)
        self.ready.publish(Bool(data=False))
        self.create_timer(.2, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def on_scene(self, message):
        self.confirmed = message.data

    def on_joints(self, message):
        self.joints = None
        if (len(message.name) != len(message.position) or len(set(message.name)) != len(message.name)
                or not all(map(math.isfinite, message.position))):
            return
        positions = dict(zip(message.name, message.position))
        required = {f'joint{i}' for i in range(1, 7)} | set(MIMICS) | {'ctm2f110_finger_joint'}
        if not required <= positions.keys():
            return
        finger = positions['ctm2f110_finger_joint']
        if any(abs(positions[name]-sign*finger) > .02 for name, sign in MIMICS.items()):
            return
        self.joints, self.joints_at = message, time.monotonic()

    def reject(self, _request, response):
        response.success = False
        response.message = 'Benchmark execution refused: ' + ', '.join(self.manifest['reason'])
        return response

    def finish(self, allowed, reason, **extra):
        self.done = True
        result = {'allowed': allowed, 'reason': reason, **extra}
        (self.archive / 'startup.json').write_text(json.dumps(result, indent=2)+'\n')
        self.ready.publish(Bool(data=allowed))
        self.status.publish(String(data=json.dumps(result)))
        (self.get_logger().info if allowed else self.get_logger().error)(json.dumps(result))

    def tick(self):
        if self.done:
            return
        if time.monotonic()-self.started > 90:
            self.finish(False, 'startup_timeout_or_missing_state_service', waiting={
                'scene_confirmed': self.confirmed,
                'joint_feedback_valid': self.joints is not None,
                'joint_feedback_age_sec': (time.monotonic()-self.joints_at
                                           if self.joints is not None else None),
                'planning_scene_service_ready': self.query.service_is_ready(),
                'state_validity_service_ready': self.validity.service_is_ready(),
                'request_stage': self.stage,
                'request_pending': self.future is not None and not self.future.done(),
                'last_scene_object_ids': self.last_scene_ids,
                'last_scene_joint_names': self.last_scene_joint_names,
            })
            return
        if not self.confirmed:
            return
        if self.joints is None or time.monotonic()-self.joints_at > 1.:
            return
        if self.future is None:
            if not self.query.service_is_ready() or not self.validity.service_is_ready():
                return
            request = GetPlanningScene.Request()
            request.components.components = (PlanningSceneComponents.ROBOT_STATE |
                                             PlanningSceneComponents.WORLD_OBJECT_GEOMETRY)
            self.future = self.query.call_async(request)
            self.stage = 'query'
            return
        if not self.future.done():
            return
        try:
            response = self.future.result()
            if self.stage == 'query':
                scene = response.scene
                self.last_scene_ids = sorted(obj.id for obj in scene.world.collision_objects)
                self.last_scene_joint_names = list(scene.robot_state.joint_state.name)
                if (not {f'joint{i}' for i in range(1, 7)} <= set(scene.robot_state.joint_state.name)
                        or 'pick_object' not in {obj.id for obj in scene.world.collision_objects}):
                    self.future = None
                    return
                # Do not collision-check a partially populated startup monitor
                # state (arm names may exist before the gripper/mimic feedback).
                scene.robot_state.joint_state = self.joints
                (self.archive / 'initial-joints.json').write_text(json.dumps({
                    'source': '/joint_states',
                    'positions': dict(zip(self.joints.name, self.joints.position))}, indent=2)+'\n')
                self.future = self.validity.call_async(GetStateValidity.Request(robot_state=scene.robot_state))
                self.stage = 'validity'
            elif not response.valid:
                self.finish(False, 'initial_robot_collision', contacts=[
                    [c.contact_body_1, c.contact_body_2] for c in response.contacts])
            elif self.manifest['expected'] == 'blocked':
                self.finish(False, 'expected_blocked_benchmark', initial_robot_valid=True,
                            violations=self.manifest['reason'])
            else:
                self.finish(True, 'initial_robot_collision_check_passed', initial_robot_valid=True)
        except Exception as error:
            self.finish(False, 'preflight_service_failed', detail=str(error))


def main():
    rclpy.init()
    node = Preflight()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
