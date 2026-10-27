#!/usr/bin/env python3
"""Simulation-only acceptance: mutate one idle fixture, refresh, reject old plan.

Requires the explicit gazebo_truth entry, no previous execution, and one wall.
Never commands motion. Leaves the obstacle restored, but its old plan invalid.
"""

import argparse
from copy import deepcopy
import json
from pathlib import Path
import subprocess
import time
import xml.etree.ElementTree as ET

import rclpy
from moveit_msgs.msg import PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, UInt64, String
from std_srvs.srv import Trigger

from pick_place_scene.static_geometry import collision_object
from pick_place_scene.scene_sync import geometry_matches
from pick_place_sim.gazebo_geometry import read_world, extract_obstacles


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--archive', type=Path, required=True)
    args = parser.parse_args()
    manifest = json.loads((args.archive/'manifest.json').read_text())
    ids = [b['id'] for b in manifest['geometry']['boxes'] if b['id'].startswith('obstacle_')]
    if len(ids) != 1:
        raise RuntimeError('acceptance requires the single_wall fixture')
    name = ids[0]
    offset = manifest['world_to_base_translation']
    original_sdf = read_world()
    original = ET.fromstring(original_sdf).find(f"world/model[@name='{name}']")
    if original is None:
        raise RuntimeError('initial obstacle absent')
    rclpy.init()
    node = rclpy.create_node('verify_stage3_truth')
    values, joints, results = {}, {}, []
    qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)
    for topic, kind, key in [('/perception/scene_ready', Bool, 'ready'),
                             ('/perception/scene_revision', UInt64, 'revision'),
                             ('/pick_place/candidate_scene_revision', UInt64, 'candidate'),
                             ('/pick_place/plan_ready', Bool, 'plan'),
                             ('/obstacles/truth_ready', Bool, 'truth'),
                             ('/pick_place/execution_status', String, 'execution')]:
        node.create_subscription(kind, topic, lambda m, key=key: values.update({key: m.data}), qos)
    node.create_subscription(JointState, '/joint_states',
                             lambda m: joints.update(zip(m.name, m.position)), qos_profile_sensor_data)

    def wait(predicate, timeout=30):
        end = time.monotonic()+timeout
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.05)
            if predicate():
                return
        raise RuntimeError(f'timeout: {values}')

    def call(kind, topic, request):
        client = node.create_client(kind, topic)
        if not client.wait_for_service(timeout_sec=10):
            raise RuntimeError(topic+' unavailable')
        future = client.call_async(request)
        wait(future.done, 12)
        result = future.result()
        node.destroy_client(client)
        if result is None:
            raise RuntimeError(topic+' empty response')
        return result

    def query():
        request = GetPlanningScene.Request()
        request.components.components = PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
        return {o.id: o for o in call(GetPlanningScene, '/get_planning_scene', request).scene.world.collision_objects}

    def gz(service, kind, request):
        result = subprocess.run(['gz', 'service', '-s', '/world/empty/'+service,
            '--reqtype', kind, '--reptype', 'gz.msgs.Boolean', '--timeout', '5000', '--req', request],
            text=True, capture_output=True, check=True, timeout=10)
        if 'data: true' not in result.stdout:
            raise RuntimeError('Gazebo rejected '+service+': '+result.stdout+result.stderr)

    def refresh():
        response = call(Trigger, '/obstacles/refresh_truth', Trigger.Request())
        if not response.success:
            raise RuntimeError(response.message)
        # Drain the false edge then await completion; service response itself
        # only accepts work and is not proof of scene confirmation.
        wait(lambda: values.get('truth') is False, 5)
        wait(lambda: values.get('truth') is True)

    def verify(label, present=True):
        sdf = read_world()
        (args.archive/f'acceptance-{label}.sdf').write_text(sdf)
        expected = extract_obstacles(sdf, ids, offset)[name]
        actual = query()
        if present:
            if expected is None or name not in actual or not geometry_matches(actual[name], collision_object('base_link', expected)):
                raise RuntimeError(label+': Gazebo and MoveIt differ')
        elif expected is not None or name in actual:
            raise RuntimeError(label+': removal not confirmed')
        results.append({'step': label, 'revision': values['revision'], 'present': present,
                        'scene_ready': values.get('ready')})

    def refuse():
        response = call(Trigger, '/pick_place/execute', Trigger.Request())
        if response.success or not any(k in response.message for k in ('stale_scene_revision', 'not_ready')):
            raise RuntimeError('old plan not explicitly refused: '+str(response))
        results.append({'step': 'old_plan_refused', 'reason': response.message})

    removed = False
    try:
        wait(lambda: values.get('plan') and values.get('truth') and values.get('ready') and
             'candidate' in values and 'revision' in values and 'execution' in values and len(joints) >= 12, 60)
        if not values.get('execution', '').startswith('waiting for planned solution'):
            raise RuntimeError('must run before any execution')
        initial_joints = dict(joints)
        revision = values['revision']
        candidate = values['candidate']
        verify('initial')
        refresh()
        wait(lambda: values.get('ready'))
        if values['revision'] != revision:
            raise RuntimeError('unchanged truth churned revision')
        results.append({'step': 'unchanged_refresh_no_write', 'revision': revision})
        pose = tuple(map(float, original.findtext('pose').split()))
        gz('set_pose', 'gz.msgs.Pose', f'name: "{name}" position {{ x: {pose[0]+.015} y: {pose[1]} z: {pose[2]} }} orientation {{ w: 1 }}')
        refresh()
        wait(lambda: values['revision'] > revision and values.get('ready'))
        verify('moved')
        if values['candidate'] != candidate:
            raise RuntimeError('candidate silently replaced')
        refuse()
        revision = values['revision']
        gz('remove', 'gz.msgs.Entity', f'name: "{name}" type: MODEL')
        removed = True
        refresh()
        wait(lambda: values['revision'] > revision and values.get('ready') is False)
        verify('removed', False)
        refuse()
        revision = values['revision']
        resized = deepcopy(original)
        for kind in ('visual', 'collision'):
            resized.find(f'link/{kind}/geometry/box/size').text = '.05 .28 .12'
        sdf = '<sdf version="1.9">'+ET.tostring(resized, encoding='unicode')+'</sdf>'
        gz('create', 'gz.msgs.EntityFactory', 'sdf: '+json.dumps(sdf)+' allow_renaming: false')
        removed = False
        refresh()
        wait(lambda: values['revision'] > revision and values.get('ready'))
        verify('recreated_resized_same_id')
        refuse()
        revision = values['revision']
        gz('remove', 'gz.msgs.Entity', f'name: "{name}" type: MODEL')
        removed = True
        sdf = '<sdf version="1.9">'+ET.tostring(original, encoding='unicode')+'</sdf>'
        gz('create', 'gz.msgs.EntityFactory', 'sdf: '+json.dumps(sdf)+' allow_renaming: false')
        removed = False
        refresh()
        wait(lambda: values['revision'] > revision and values.get('ready'))
        verify('restored')
        refuse()
        if any(abs(joints.get(k, float('inf'))-v) > .005 for k, v in initial_joints.items()):
            raise RuntimeError('robot moved during stale-plan acceptance')
        results.append({'step': 'robot_stationary', 'tolerance_rad': .005})
        (args.archive/'stage3-acceptance.json').write_text(json.dumps(results, indent=2)+'\n')
        print('PASS live static truth move/REMOVE/recreate/size/stable-ID confirmation; old plan refused; no motion')
        return 0
    finally:
        # If interrupted after deletion, leave an explicit diagnostic; do not
        # fabricate a success or silently run unconfirmed recovery operations.
        if removed:
            print('Obstacle remains deleted; scene requires explicit restore/restart')
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
