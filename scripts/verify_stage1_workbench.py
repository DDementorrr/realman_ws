#!/usr/bin/env python3
"""Compare actual Gazebo visual/collision and MoveIt boxes with stage-1 input.

Run before execution for the initial-state check, and after reset for regression.
This is a simulation check, not camera/hardware collision certification.
"""

import argparse
import ast
import json
from pathlib import Path
import subprocess
import time
import xml.etree.ElementTree as ET

import rclpy
from ament_index_python.packages import get_package_share_directory
from moveit_msgs.msg import PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene, GetStateValidity
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import Bool, UInt64

from pick_place_scene.static_geometry import collision_object, load_geometry
from pick_place_scene.scene_sync import geometry_matches


def near(actual, expected):
    return len(actual) == len(expected) and all(abs(a-b) <= 1e-6 for a, b in zip(actual, expected))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--geometry', default=str(Path(get_package_share_directory('pick_place_scene')) /
                                                'config/u_workbench.yaml'))
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    frame, boxes = load_geometry(args.geometry)
    gazebo = Path(get_package_share_directory('robot_gazebo'))
    mount = ET.parse(gazebo / 'urdf/rm65b_ctm2f110_gz.urdf.xacro').find(
        "joint[@name='world_to_rm65b']/origin")
    offset = tuple(map(float, mount.get('xyz').split()))
    result = subprocess.run([
        'gz', 'service', '-s', '/world/empty/generate_world_sdf',
        '--reqtype', 'gz.msgs.SdfGeneratorConfig', '--reptype', 'gz.msgs.StringMsg',
        '--timeout', '5000', '--req', 'global_entity_gen_config { expand_include_tags {data: true} }'],
        text=True, capture_output=True, check=True, timeout=12)
    data = result.stdout.strip()
    if not data.startswith('data: '):
        raise RuntimeError('Gazebo live world export unavailable')
    world = ET.fromstring(ast.literal_eval(data[6:])).find('world')
    rclpy.init()
    node = rclpy.create_node('verify_stage1_workbench')
    state = {}
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)
    node.create_subscription(Bool, '/perception/scene_ready', lambda m: state.update(ready=m.data), qos)
    node.create_subscription(UInt64, '/perception/scene_revision', lambda m: state.update(revision=m.data), qos)

    def call(kind, topic, request):
        client = node.create_client(kind, topic)
        if not client.wait_for_service(timeout_sec=10):
            raise RuntimeError(f'{topic} unavailable')
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=10)
        if not future.done() or future.result() is None:
            raise RuntimeError(f'{topic} timed out')
        node.destroy_client(client)
        return future.result()

    try:
        deadline = time.monotonic() + 10
        while (not state.get('ready') or 'revision' not in state) and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
        if not state.get('ready') or not state.get('revision'):
            raise RuntimeError('complete workbench not confirmed')
        query = GetPlanningScene.Request()
        query.components.components = (PlanningSceneComponents.WORLD_OBJECT_GEOMETRY |
                                       PlanningSceneComponents.ROBOT_STATE)
        scene = call(GetPlanningScene, '/get_planning_scene', query).scene
        records = []
        for box in boxes:
            models = world.findall(f"model[@name='{box.id}']")
            objects = [obj for obj in scene.world.collision_objects if obj.id == box.id]
            if len(models) != 1 or len(objects) != 1:
                raise RuntimeError(f'{box.id}: missing/duplicate model or collision object')
            model = models[0]
            pose = tuple(map(float, model.findtext('pose').split()))
            expected = tuple(x+t for x, t in zip(box.pose[:3], offset)) + box.pose[3:]
            if not near(pose, expected):
                raise RuntimeError(f'{box.id}: actual Gazebo pose differs: {pose}')
            for local_pose in ('link/pose', 'link/visual/pose', 'link/collision/pose'):
                if not near(tuple(map(float, model.findtext(local_pose, '0 0 0 0 0 0').split())), (0.,)*6):
                    raise RuntimeError(f'{box.id}: unexpected local {local_pose}')
            for kind in ('visual', 'collision'):
                dimensions = tuple(map(float, model.findtext(f'link/{kind}/geometry/box/size').split()))
                if not near(dimensions, box.size):
                    raise RuntimeError(f'{box.id}: Gazebo {kind} size differs')
            if not geometry_matches(objects[0], collision_object(frame, box)):
                raise RuntimeError(f'{box.id}: actual MoveIt geometry differs')
            records.append({'id':box.id, 'size_m':box.size, 'world_pose':expected})
        validity = call(GetStateValidity, '/check_state_validity',
                        GetStateValidity.Request(robot_state=scene.robot_state))
        if not validity.valid:
            raise RuntimeError('current robot state collides: ' + str([
                (c.contact_body_1, c.contact_body_2) for c in validity.contacts]))
        report = {'geometry':str(Path(args.geometry).resolve()), 'parts':records,
                  'revision':state['revision'], 'ready':True, 'current_state_valid':True}
        if args.output:
            args.output.write_text(json.dumps(report, indent=2)+'\n')
        print(f'PASS actual Gazebo visual/collision and MoveIt match all {len(boxes)} shared boxes; current robot state valid')
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
