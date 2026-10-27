#!/usr/bin/env python3
"""Read-only stage-2 archive/live geometry + startup gate verification.

The negative case sends only the deliberately non-actuating rejection Trigger.
Full Gazebo execution remains a separate explicit stage-3 pipeline verifier.
"""

import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import time

import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import Bool
from std_srvs.srv import Trigger


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--archive', type=Path, required=True)
    parser.add_argument('--expect-no-plan', action='store_true',
                        help='After a separate bounded planning failure, verify execution refusal')
    args = parser.parse_args()
    manifest = json.loads((args.archive/'manifest.json').read_text())
    startup = json.loads((args.archive/'startup.json').read_text())
    geometry = args.archive/'geometry.yaml'
    if hashlib.sha256(geometry.read_bytes()).hexdigest() != manifest['geometry_sha256']:
        raise RuntimeError('archive geometry differs from manifest')
    subprocess.run([sys.executable, str(Path(__file__).with_name('verify_stage1_workbench.py')),
                    '--geometry', str(geometry), '--output', str(args.archive/'live-geometry.json')], check=True)
    blocked = manifest['expected'] == 'blocked'
    if startup['allowed'] != (not blocked) or not startup.get('initial_robot_valid'):
        raise RuntimeError(f'unexpected preflight result: {startup}')
    rclpy.init()
    node = rclpy.create_node('verify_stage2_obstacles')
    values = {}
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)
    node.create_subscription(Bool, '/obstacles/startup_ready', lambda m: values.update(ready=m.data), qos)
    node.create_subscription(Bool, '/pick_place/plan_ready', lambda m: values.update(plan=m.data), qos)
    try:
        deadline = time.monotonic()+3
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
        if values.get('ready') is not (not blocked):
            raise RuntimeError('startup topic does not match saved result')
        if blocked or args.expect_no_plan:
            client = node.create_client(Trigger, '/pick_place/execute')
            if not client.wait_for_service(timeout_sec=5):
                raise RuntimeError('benchmark rejection service missing')
            future = client.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(node, future, timeout_sec=5)
            if (not future.done() or future.result() is None or future.result().success or values.get('plan')):
                raise RuntimeError('unsolved scene was not refused')
            if blocked and 'Benchmark execution refused' not in future.result().message:
                raise RuntimeError('negative benchmark rejection reason missing')
            result = {'passed': True, 'execution_refused': True,
                      'reason': future.result().message, 'plan_published': False}
        else:
            result = {'passed': True, 'startup_ready': True,
                      'planning_feasibility': 'check_with_verify_mtc_stage2.py'}
        (args.archive/'verification.json').write_text(json.dumps(result, indent=2)+'\n')
        print('PASS stage-2 archive/shared geometry/preflight: ' + json.dumps(result))
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
