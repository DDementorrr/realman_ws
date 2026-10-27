#!/usr/bin/env python3
"""Replay-derived streaming fault injection; NO Gazebo or hardware required.

Recorded payloads are preserved except intentional faults; capture stamps are
shifted together into a synthetic progressing clock, explicitly not motion data.
"""
import argparse
from copy import deepcopy
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
import yaml


def case(args, fault, messages, types, anchor):
    env = {**os.environ, 'ROS_DOMAIN_ID': str(args.domain_id), 'ROS_LOCALHOST_ONLY': '1'}
    os.environ.update(ROS_DOMAIN_ID=env['ROS_DOMAIN_ID'], ROS_LOCALHOST_ONLY='1')
    root = Path(args.output_dir)
    with (root/(fault+'-node.log')).open('w') as log:
        process = subprocess.Popen(['ros2', 'launch', 'pick_place_perception', 'obstacles_external.launch.py',
            'perception_config:='+str(Path(args.config).resolve()), 'use_sim_time:=true'],
            env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        rclpy.init()
        node = rclpy.create_node('monitor_stream_fault_source')
        pubs = {topic: node.create_publisher(kind, topic, 10) for topic, kind in types.items()}
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        reports, readiness = [], []
        node.create_subscription(DiagnosticArray, '/perception/obstacle_status', reports.append, qos)
        node.create_subscription(Bool, '/perception/obstacles_ready', lambda m: readiness.append(m.data), qos)
        client = node.create_client(Trigger, '/perception/capture_obstacles')
        requested = False
        triggered = None
        ready_at = None
        started = time.monotonic()
        config = yaml.safe_load(Path(args.config).read_text())['depth_obstacles']['ros__parameters']
        try:
            while time.monotonic()-started < 35:
                elapsed = time.monotonic()-started
                target = int((1000+elapsed)*1e9)
                for topic, original in messages:
                    if triggered and fault == 'source_stale' and topic != '/clock':
                        continue
                    if triggered and fault == 'tf_lost' and topic == '/tf':
                        continue
                    msg = deepcopy(original)
                    stamps = [t.header.stamp for t in msg.transforms] if topic == '/tf' else ([msg.clock] if topic == '/clock' else [msg.header.stamp])
                    for stamp in stamps:
                        shifted = stamp.sec*10**9+stamp.nanosec+target-anchor
                        stamp.sec, stamp.nanosec = divmod(shifted, 10**9)
                    if triggered and fault == 'depth_quality' and topic == config['depth_image_topic']:
                        msg.data = bytes(len(msg.data))
                    pubs[topic].publish(msg)
                rclpy.spin_once(node, timeout_sec=.05)
                if not requested and client.service_is_ready() and elapsed > 2:
                    client.call_async(Trigger.Request())
                    requested = True
                if reports:
                    status = reports[-1].status[0]
                    quality = {v.key: json.loads(v.value) for v in status.values}
                    if status.level == DiagnosticStatus.OK and quality['monitor_frames_confirmed'] >= 2 and triggered is None:
                        triggered, ready_at = time.monotonic(), elapsed
                    if triggered and readiness and not readiness[-1]:
                        reason = status.message
                        expected = {'depth_quality': 'quality below', 'tf_lost': 'extrapolation', 'source_stale': 'expired'}[fault]
                        if expected.lower() not in reason.lower():
                            # Diagnostics and Bool can arrive in either order.
                            continue
                        if time.monotonic()-triggered > config['max_input_age']+4:
                            raise RuntimeError('invalidation exceeded bounded input lease')
                        result = {'source': 'recorded static RGB-D payload with synthetic shifted stamps', 'fault': fault,
                                  'ready_after_monitor_frames': quality['monitor_frames_confirmed'],
                                  'invalidated': True, 'reason': reason,
                                  'fault_to_invalid_sec': time.monotonic()-triggered, 'ready_at_sec': ready_at}
                        (root/(fault+'.json')).write_text(json.dumps(result, indent=2))
                        print(f'PASS streaming {fault}: {reason}', flush=True)
                        return
            raise RuntimeError(f'{fault}: did not observe monitored ready -> invalid transition; last reports={reports[-1:]}')
        finally:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGTERM)
                    process.wait(timeout=5)
            node.destroy_node()
            rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', required=True)
    parser.add_argument('--bag', required=True)
    parser.add_argument('--output-dir', required=True)
    parser.add_argument('--domain-id', type=int, default=205)
    args = parser.parse_args()
    Path(args.output_dir).mkdir(parents=True, exist_ok=False)
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=args.bag, storage_id='sqlite3'), rosbag2_py.ConverterOptions('', ''))
    types = {topic.name: get_message(topic.type) for topic in reader.get_all_topics_and_types()}
    messages, anchor = [], None
    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        messages.append((topic, deserialize_message(data, types[topic])))
        anchor = timestamp
    for fault in ('depth_quality', 'tf_lost', 'source_stale'):
        case(args, fault, messages, types, anchor)


if __name__ == '__main__':
    main()
