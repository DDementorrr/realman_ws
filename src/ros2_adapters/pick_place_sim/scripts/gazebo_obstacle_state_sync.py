#!/usr/bin/env python3
"""Explicit static truth refresh -> standard observations -> public scene owner.

No direct ApplyPlanningScene client. No automatic dynamic updates. Refresh is
available only before any task execution status; restart after execution.
"""

import json
from pathlib import Path
import threading
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from moveit_msgs.msg import CollisionObject, PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from pick_place_scene.static_geometry import collision_object
from pick_place_scene.scene_sync import geometry_matches
from pick_place_sim.gazebo_geometry import read_world, extract_obstacles


class TruthSync(Node):
    def __init__(self, **kwargs):
        super().__init__('gazebo_obstacle_state_sync', **kwargs)
        self.declare_parameter('archive', '')
        self.archive = Path(self.get_parameter('archive').value)
        manifest = json.loads((self.archive/'manifest.json').read_text())
        self.ids = [b['id'] for b in manifest['geometry']['boxes'] if b['id'].startswith('obstacle_')]
        self.offset = manifest['world_to_base_translation']
        self.get_scene = self.create_client(GetPlanningScene, '/get_planning_scene')
        qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(CollisionObject, '/perception/collision_objects', qos)
        self.diagnostics = self.create_publisher(DiagnosticArray, '/perception/obstacle_status', qos)
        self.ready = self.create_publisher(Bool, '/obstacles/truth_ready', qos)
        self.ready.publish(Bool(data=False))
        self.create_subscription(String, '/pick_place/execution_status', self.on_execution, qos)
        self.create_service(Trigger, '/obstacles/refresh_truth', self.refresh)
        self.execution_seen = False
        self.worker = None
        self.pending = None
        self.future = None
        self.deadline = 0.
        self.serial = 0
        self.last_error = ''
        self.lock = threading.Lock()
        self.create_timer(.2, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def on_execution(self, message):
        # Even completion/reset is conservative: no refresh after motion in
        # this stage. Does not claim atomic ownership with external controllers.
        if message.data and not message.data.startswith('waiting for planned solution'):
            self.execution_seen = True

    def status(self, level, text):
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.status = [DiagnosticStatus(level=level, name='static_gazebo_obstacles',
            hardware_id='simulation-only', message=text, values=[
                KeyValue(key='source', value='gazebo_live_world_sdf'),
                KeyValue(key='mode', value='explicit_static_refresh')])]
        self.diagnostics.publish(msg)

    def start_read(self):
        self.ready.publish(Bool(data=False))
        self.deadline = time.monotonic()+30
        self.last_error = ''
        def read():
            try:
                sdf = read_world()
                result = extract_obstacles(sdf, self.ids, self.offset)
                with self.lock:
                    self.pending = (sdf, result, None)
            except Exception as error:
                with self.lock:
                    self.pending = ('', {}, str(error))
        self.worker = threading.Thread(target=read, daemon=True)
        self.worker.start()

    def refresh(self, _request, response):
        if self.execution_seen:
            response.message = 'Static refresh refused after execution status; restart the scene'
        elif self.last_error:
            response.message = 'Static truth fault requires restart: '+self.last_error
        elif self.worker is not None:
            response.message = 'Static truth read/confirmation already in progress'
        else:
            self.start_read()
            response.success = True
            response.message = 'Static Gazebo truth refresh started; inspect obstacle_status for confirmation'
        return response

    def finish(self, error=''):
        self.last_error = error
        self.status(DiagnosticStatus.ERROR if error else DiagnosticStatus.OK,
                    error or 'All configured static obstacle ADD/REMOVE observations confirmed')
        self.ready.publish(Bool(data=not bool(error)))
        self.worker = self.future = self.pending = None

    def tick(self):
        if self.execution_seen and self.worker is not None:
            self.finish('Execution began during static refresh; restart required')
            return
        if self.worker is None:
            if self.serial == 0 and self.get_clock().now().nanoseconds > 0:
                self.start_read()
                self.serial = 1
            return
        if time.monotonic() > self.deadline:
            self.finish('Static truth query/confirmation timeout; restart required')
            return
        if self.worker.is_alive():
            return
        with self.lock:
            payload = self.pending
        if payload is not None:
            sdf, expected, error = payload
            if error:
                self.finish('Gazebo truth unavailable: '+error)
                return
            self.expected = expected
            self.differences = {}
            self.sent = set()
            self.serial += 1
            (self.archive/f'truth-{self.serial}.sdf').write_text(sdf)
            self.pending = None
            self.next_send = 0.
        if self.future is not None:
            if not self.future.done():
                return
            try:
                scene = self.future.result().scene
                objects = {o.id: o for o in scene.world.collision_objects}
                self.differences = {name: box for name, box in self.expected.items() if not (
                    name not in objects if box is None else name in objects and
                    geometry_matches(objects[name], collision_object('base_link', box)))}
                if not self.differences:
                    self.finish()
                    return
            except Exception as error:
                self.finish('PlanningScene query failed: '+str(error))
                return
            self.future = None
        if (time.monotonic() >= self.next_send and self.get_scene.service_is_ready()
                and self.publisher.get_subscription_count() > 0):
            # Query first; only publish differences. No repeated writes or
            # revision churn when the actual scene already matches the export.
            stamp = self.get_clock().now().to_msg()
            for name, box in self.differences.items():
                if name in self.sent:
                    continue
                if box is None:
                    obj = CollisionObject(id=name, operation=CollisionObject.REMOVE)
                    obj.header.frame_id, obj.header.stamp = 'base_link', stamp
                else:
                    obj = collision_object('base_link', box, stamp)
                self.publisher.publish(obj)
                self.sent.add(name)
                # The compatible public observation subscription has depth 1.
                # One ID per query cycle prevents burst loss without changing
                # existing producers or inventing an atomic batch interface.
                break
            self.next_send = time.monotonic()+1.
            query = GetPlanningScene.Request()
            query.components.components = PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
            self.future = self.get_scene.call_async(query)


def main():
    rclpy.init()
    node = TruthSync()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
