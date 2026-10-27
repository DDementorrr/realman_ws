#!/usr/bin/env python3
"""Record real simulated RGB-D samples; replay them without a Gazebo process.

The bag contains unmodified sensor/state/target messages plus capture-time TF
samples queried from the actual TF buffer. Truth is exported separately ONLY
for the evaluator. No hardware recording is implied by this fixture.
"""
import argparse
from collections import deque
import hashlib
import json
from pathlib import Path
import time

import numpy as np
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from rclpy.serialization import serialize_message, deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py
from sensor_msgs.msg import CameraInfo, Image, JointState
from geometry_msgs.msg import PoseStamped
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from moveit_msgs.msg import CollisionObject, PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener
import yaml

from pick_place_perception.obstacle_geometry import stamp_ns
from pick_place_perception.self_filter import load_model


def settings(path):
    return yaml.safe_load(Path(path).read_text())['depth_obstacles']['ros__parameters']


def wait(node, condition, timeout=90):
    deadline = time.monotonic()+timeout
    while not condition() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=.05)
    if not condition():
        raise RuntimeError('bounded wait failed')


def record(node, args):
    c = settings(args.config)
    histories = {key: deque(maxlen=30) for key in ('rgb', 'depth', 'info', 'depth_info', 'joints', 'target')}
    specs = [('rgb', Image, c['rgb_image_topic']), ('depth', Image, c['depth_image_topic']),
             ('info', CameraInfo, c['camera_info_topic']), ('depth_info', CameraInfo, c['depth_info_topic']),
             ('joints', JointState, c['joint_states_topic']), ('target', PoseStamped, c['target_pose_topic'])]
    subs = [node.create_subscription(kind, topic, lambda msg, key=key: histories[key].append(msg), qos_profile_sensor_data)
            for key, kind, topic in specs]
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    models, _, model_hash = load_model(c['self_filter_model'])
    deadline = time.monotonic()+120
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=.05)
        if not all(histories.values()):
            continue
        depth = histories['depth'][-1]
        anchor = stamp_ns(depth)
        chosen = {key: min(items, key=lambda m: abs(stamp_ns(m)-anchor)) for key, items in histories.items()}
        chosen_stamps = [stamp_ns(msg) for msg in chosen.values()]
        if min(chosen_stamps) <= 0 or max(chosen_stamps)-min(chosen_stamps) > c['max_time_skew']*1e9:
            continue
        frames = {body.link for body in models} | {c['camera_frame'], chosen['target'].header.frame_id}
        try:
            transforms = [buffer.lookup_transform(c['output_frame'], frame, rclpy.time.Time.from_msg(depth.header.stamp))
                          for frame in frames if frame != c['output_frame']]
            target = chosen['target']
            if target.header.frame_id != c['output_frame']:
                transforms.append(buffer.lookup_transform(c['output_frame'], target.header.frame_id,
                                  rclpy.time.Time.from_msg(target.header.stamp)))
        except Exception:
            continue
        break
    else:
        raise RuntimeError('no synchronized recorded sensor/TF/state/target bundle')
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=args.bag, storage_id='sqlite3'), rosbag2_py.ConverterOptions('', ''))
    records = []
    for key, kind, topic in specs:
        kind_name = '/'.join(kind.__module__.split('.')[:2])+'/'+kind.__name__
        records.append((topic, kind_name, chosen[key]))
    records += [('/tf', 'tf2_msgs/msg/TFMessage', TFMessage(transforms=transforms)),
                ('/clock', 'rosgraph_msgs/msg/Clock', Clock(clock=depth.header.stamp))]
    for topic, kind_name, msg in records:
        writer.create_topic(rosbag2_py.TopicMetadata(name=topic, type=kind_name, serialization_format='cdr'))
        writer.write(topic, serialize_message(msg), anchor)
    del writer
    summary = {'source': 'Gazebo Harmonic native RGB-D; actual rendered sensor messages',
               'tf_source': 'actual TF buffer sampled at depth acquisition stamp; flattened to output frame',
               'target_source': 'separately supplied known simulation task pose, not image-estimated target',
               'stamp_ns': anchor, 'camera': c['camera_frame'], 'model_sha256': model_hash,
               'bundle_time_span_sec': (max(chosen_stamps)-min(chosen_stamps))*1e-9,
               'topics': [topic for topic, _, _ in records],
               'sha256': {path.name: hashlib.sha256(path.read_bytes()).hexdigest() for path in Path(args.bag).glob('*')}}
    Path(args.bag+'.source.json').write_text(json.dumps(summary, indent=2))
    print(json.dumps(summary, indent=2), flush=True)


def replay(node, args):
    messages, types = [], {}
    if args.mode == 'replay':
        reader = rosbag2_py.SequentialReader()
        reader.open(rosbag2_py.StorageOptions(uri=args.bag, storage_id='sqlite3'), rosbag2_py.ConverterOptions('', ''))
        types = {info.name: get_message(info.type) for info in reader.get_all_topics_and_types()}
        while reader.has_next():
            topic, data, _ = reader.read_next()
            messages.append((topic, deserialize_message(data, types[topic])))
        if args.fault:
            c = settings(args.config)
            for topic, msg in messages:
                if args.fault == 'future_tf' and topic == '/tf':
                    for tf in msg.transforms:
                        tf.header.stamp.sec += 2
                elif args.fault == 'bad_registration' and topic == c['depth_info_topic']:
                    msg.k[0] += 10.
                elif args.fault == 'missing_robot_state' and topic == c['joint_states_topic']:
                    msg.name, msg.position = [], []
    pubs = {topic: node.create_publisher(kind, topic, 10) for topic, kind in types.items()}
    client = node.create_client(Trigger, '/perception/capture_obstacles')
    received, observations, ready, scene_ready = [], [], [], []
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(DiagnosticArray, '/perception/obstacle_status', received.append, qos)
    node.create_subscription(CollisionObject, '/perception/collision_objects', observations.append, qos)
    node.create_subscription(Bool, '/perception/obstacles_ready', lambda m: ready.append(m.data), qos)
    if args.mode == 'live':
        node.create_subscription(Bool, '/depth_scene/ready', lambda m: scene_ready.append(m.data), qos)
    wait(node, client.service_is_ready, 30)
    # Repeated publication delivers recorded samples after discovery; timestamps
    # and payloads stay unchanged. This is a frozen static recording, not motion.
    request, deadline = None, time.monotonic()+25
    while time.monotonic() < deadline and not observations:
        for topic, msg in messages:
            pubs[topic].publish(msg)
        rclpy.spin_once(node, timeout_sec=.1)
        if request is None and (args.mode == 'live' or any(p.get_subscription_count() for p in pubs.values())):
            request = client.call_async(Trigger.Request())
        if args.fault and received and received[-1].status[0].message.startswith('capture failed:'):
            break
    if args.fault:
        if observations or any(ready) or not received[-1].status[0].message.startswith('capture failed:'):
            raise RuntimeError('fault did not fail closed')
        Path(args.output).write_text(json.dumps({'fault': args.fault, 'geometry_published': False,
            'ready': False, 'reason': received[-1].status[0].message}, indent=2))
        print(f'PASS {args.fault}: no geometry and not ready', flush=True)
        return
    wait(node, lambda: any(m.status and m.status[0].level == DiagnosticStatus.OK for m in received), 2)
    if not observations:
        raise RuntimeError('no geometry from recorded RGB-D')
    report = next(m for m in reversed(received) if m.status[0].level == DiagnosticStatus.OK)
    quality = {value.key: json.loads(value.value) for value in report.status[0].values}
    obj = observations[-1]
    quality['collision_boxes'] = [{'center': [p.position.x, p.position.y, p.position.z], 'size': list(s.dimensions)}
                                 for p, s in zip(obj.primitive_poses, obj.primitives)]
    quality['ros_nodes'] = node.get_node_names_and_namespaces()
    if args.mode == 'replay':
        forbidden = {'controller_manager', 'move_group', 'robot_state_publisher', 'ros_gz_bridge'}
        if any(name in forbidden for name, namespace in quality['ros_nodes']):
            raise RuntimeError('replay graph contains simulation/control nodes')
    if args.mode == 'live':
        from pick_place_scene.scene_sync import geometry_matches
        scene_client = node.create_client(GetPlanningScene, '/get_planning_scene')
        wait(node, scene_client.service_is_ready, 5)
        confirmed = False
        for _ in range(20):
            request = GetPlanningScene.Request()
            request.components.components = PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
            future = scene_client.call_async(request)
            wait(node, future.done, 3)
            actual = [b for b in future.result().scene.world.collision_objects if b.id == obj.id]
            if len(actual) == 1 and geometry_matches(actual[0], obj):
                confirmed = True
                break
            until = time.monotonic()+.1
            while time.monotonic() < until:
                rclpy.spin_once(node, timeout_sec=.02)
        if not confirmed:
            raise RuntimeError('live MoveIt compound geometry differs from perception observation')
        quality['moveit_geometry_confirmed'] = True
        other_ids = {b.id for b in future.result().scene.world.collision_objects if b.id != obj.id}
        if not {'pick_table', 'left_table', 'right_table', 'pick_object'}.issubset(other_ids):
            raise RuntimeError('support or separately managed target disappeared')
        quality['retained_scene_ids'] = sorted(other_ids)
        wait(node, lambda: any(scene_ready), 2)
    # A frozen /clock must still expire using the node's steady watchdog.
    wait(node, lambda: ready and not ready[-1], settings(args.config)['max_input_age']+3)
    quality['steady_expiry_confirmed'] = True
    if args.mode == 'live':
        wait(node, lambda: scene_ready and not scene_ready[-1], 2)
        quality['scene_gate_expiry_confirmed'] = True
    Path(args.output).write_text(json.dumps(quality, indent=2))
    print(f"PASS {args.mode} {quality['observed_boxes']} visible boxes, {quality['unknown_boxes']} unknown boxes; expiry confirmed", flush=True)


def evaluate(args):
    """Truth is accepted here only, never by the public estimator/replayer."""
    quality = json.loads(Path(args.output).read_text())
    from pick_place_sim.gazebo_geometry import extract_obstacles
    manifest = json.loads((Path(args.archive)/'manifest.json').read_text())
    ids = [b['id'] for b in manifest['geometry']['boxes'] if b['id'].startswith('obstacle_')]
    offset = manifest['world_to_base_translation']
    truth = extract_obstacles(Path(args.truth).read_text(), ids, offset)
    matches = []
    for name, box in truth.items():
        if box is None:
            raise RuntimeError('truth obstacle missing')
        if any(abs(a) > 1e-8 for a in box.pose[3:]):
            raise RuntimeError('first evaluator requires axis-aligned truth')
        lower, upper = np.array(box.pose[:3])-np.array(box.size)/2, np.array(box.pose[:3])+np.array(box.size)/2
        if not quality['bounds']:
            raise RuntimeError('no visible obstacle estimate')
        visible = min(quality['bounds'], key=lambda b: np.linalg.norm((np.array(b['lower'])+b['upper'])[:2]/2-np.array(box.pose[:2])))
        vlo, vhi = np.array(visible['lower']), np.array(visible['upper'])
        xy_error = float(max(np.max(np.abs(vlo[:2]-lower[:2])), np.max(np.abs(vhi[:2]-upper[:2]))))
        top_error = abs(float(vhi[2]-upper[2]))
        corners = np.array([[x, y, z] for x in (vlo[0], vhi[0]) for y in (vlo[1], vhi[1]) for z in (vlo[2], vhi[2])])
        outside = np.maximum(np.maximum(lower-corners, corners-upper), 0)
        distance = np.where(np.any(outside > 0, axis=1), np.linalg.norm(outside, axis=1),
                            np.minimum(np.abs(corners-lower), np.abs(corners-upper)).min(axis=1))
        surface_error = float(distance.max())
        grid = np.array(np.meshgrid(*[np.linspace(lower[i]+1e-5, upper[i]-1e-5, 7) for i in range(3)])).reshape(3, -1).T
        covered = np.zeros(len(grid), dtype=bool)
        for b in quality['collision_boxes']:
            center, half = np.array(b['center']), np.array(b['size'])/2
            covered |= ((grid >= center-half-1e-6) & (grid <= center+half+1e-6)).all(axis=1)
        matches.append({'id': name, 'xy_visible_boundary_error_m': xy_error, 'visible_top_error_m': top_error,
                        'visible_bounds_surface_error_m': surface_error, 'partial_view': args.partial_view,
                        'truth_volume_sample_coverage': float(covered.mean()), 'samples': len(grid)})
        if ((not args.partial_view and xy_error > .02) or surface_error > .01 or top_error > .01 or not covered.all()):
            raise RuntimeError(f'geometry error/coverage acceptance failed: {matches[-1]}')
    Path(args.output+'.truth-evaluation.json').write_text(json.dumps(matches, indent=2))
    print(json.dumps(matches, indent=2), flush=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('mode', choices=['record', 'replay', 'live', 'evaluate'])
    parser.add_argument('--config', required=True)
    parser.add_argument('--bag')
    parser.add_argument('--output')
    parser.add_argument('--truth')
    parser.add_argument('--archive')
    parser.add_argument('--fault', choices=['future_tf', 'bad_registration', 'missing_robot_state'])
    parser.add_argument('--partial-view', action='store_true', help='Assess seen surfaces and conservative coverage, without requiring hidden XY extents')
    args = parser.parse_args()
    if args.mode == 'evaluate':
        evaluate(args)
        return
    rclpy.init()
    node = rclpy.create_node('stage4_'+args.mode, parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])
    try:
        (record if args.mode == 'record' else replay)(node, args)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
