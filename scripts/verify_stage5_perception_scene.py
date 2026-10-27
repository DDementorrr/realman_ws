#!/usr/bin/env python3
"""Verify a live RGB-D-only static scene, then move a Gazebo wall and refuse.

Requires the explicit stage5 single_wall fixture, no running execution. Mutation
is simulation-only and does not feed truth back into perception. Restart after.
"""
import argparse
import json
from pathlib import Path
import subprocess
import time

import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from moveit_msgs.msg import CollisionObject, PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from pick_place_scene.scene_sync import geometry_matches
from pick_place_scene.uncertainty import load_budget, inflate


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--archive', required=True)
    parser.add_argument('--uncertainty', required=True)
    args = parser.parse_args()
    root = Path(args.archive)
    manifest = json.loads((root/'manifest.json').read_text())
    if manifest['config']['benchmark'] != 'single_wall' or manifest['config']['obstacle_mode'] != 'fixed':
        raise RuntimeError('verifier mutates only a fixed single_wall fixture')
    rclpy.init()
    node = rclpy.create_node('verify_stage5_perception_scene')
    observations, statuses, scene_ready, input_ready = [], [], [], []
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(CollisionObject, '/perception/collision_objects', observations.append, qos)
    node.create_subscription(DiagnosticArray, '/perception/obstacle_status', statuses.append, qos)
    node.create_subscription(Bool, '/perception/scene_ready', lambda m: scene_ready.append(m.data), qos)
    node.create_subscription(Bool, '/perception/obstacles_ready', lambda m: input_ready.append(m.data), qos)
    scene = node.create_client(GetPlanningScene, '/get_planning_scene')
    execute = node.create_client(Trigger, '/pick_place/execute')

    def wait(condition, timeout=15):
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline and not condition():
            rclpy.spin_once(node, timeout_sec=.05)
        if not condition():
            raise RuntimeError('live stage5 bounded wait failed')

    def call(client, request):
        wait(client.service_is_ready)
        future = client.call_async(request)
        wait(future.done)
        return future.result()

    try:
        wait(lambda: observations and statuses and input_ready and scene_ready)
        if not input_ready[-1] or not scene_ready[-1]:
            raise RuntimeError('capture/monitor is not ready')
        quality = {v.key: json.loads(v.value) for v in statuses[-1].status[0].values}
        if quality['monitor_frames_confirmed'] < 3:
            raise RuntimeError('no sustained monitored RGB-D input evidence')
        request = GetPlanningScene.Request()
        request.components.components = PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
        world = call(scene, request).scene.world.collision_objects
        ids = {b.id for b in world}
        if any(name.startswith('obstacle_') for name in ids):
            raise RuntimeError('truth obstacle leaked into RGB-D scene')
        if not {'depth_obstacles', 'pick_table', 'left_table', 'right_table', 'pick_object'} <= ids:
            raise RuntimeError('missing perception/support/target geometry')
        expected = inflate(observations[-1], load_budget(args.uncertainty))
        actual = next(b for b in world if b.id == 'depth_obstacles')
        if not geometry_matches(actual, expected):
            raise RuntimeError('actual MoveIt compound does not equal observation plus explicit uncertainty')
        print('PASS RGB-D-only scene: no truth obstacle IDs, all inflated primitives match, sustained monitor', flush=True)
        # Move wall into a previously free visible region. Known fixture coords
        # are used only to mutate Gazebo, never to construct estimated geometry.
        box = next(b for b in manifest['geometry']['boxes'] if b['id'].startswith('obstacle_'))
        x, y, z = box['pose'][:3]
        offset = manifest['world_to_base_translation']
        command = ['gz', 'service', '-s', '/world/empty/set_pose', '--reqtype', 'gz.msgs.Pose',
                   '--reptype', 'gz.msgs.Boolean', '--timeout', '5000', '--req',
                   f'name: "{box["id"]}" position {{ x: {x+offset[0]-.12} y: {y+offset[1]} z: {z+offset[2]} }} orientation {{ w: 1 }}']
        result = subprocess.run(command, capture_output=True, text=True, timeout=10, check=True)
        if 'true' not in result.stdout:
            raise RuntimeError('Gazebo did not confirm wall mutation')
        wait(lambda: input_ready and not input_ready[-1] and scene_ready and not scene_ready[-1])
        wait(lambda: statuses[-1].status[0].message.startswith('monitor invalidated:'))
        reason = statuses[-1].status[0].message
        if 'coverage changed' not in reason:
            raise RuntimeError('source failed for an unrelated reason: '+reason)
        refused = call(execute, Trigger.Request())
        if refused.success:
            raise RuntimeError('unsafe scene accepted execution')
        result = {'scene_ids': sorted(ids), 'uncertainty_per_side_m': load_budget(args.uncertainty)[2],
                  'primitive_count': len(actual.primitives), 'monitor_frames_confirmed': quality['monitor_frames_confirmed'],
                  'geometry_matches': True, 'truth_obstacle_absent': True, 'mutation_monitor_reason': reason,
                  'execution_refused': refused.message}
        (root/'stage5-perception-scene.json').write_text(json.dumps(result, indent=2))
        print('PASS moved wall invalidates frozen coverage; scene not ready and execution refused: '+reason, flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
