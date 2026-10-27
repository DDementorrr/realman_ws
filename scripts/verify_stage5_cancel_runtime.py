#!/usr/bin/env python3
"""Stop a real Gazebo/MTC action by invalidating its REAL perception lease.

This isolates stop confirmation using the established known-scene candidate;
it is not proof of an RGB-D-only successful grasp. No fake ready publisher.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import time

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--perception-launch-pid', type=int, required=True)
    parser.add_argument('--fault-file', required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    cmd = Path(f'/proc/{args.perception_launch_pid}/cmdline').read_bytes()
    if b'obstacles_external.launch.py' not in cmd:
        raise RuntimeError('PID is not the own perception launch')
    fault = Path(args.fault_file)
    if fault.exists():
        raise RuntimeError('use a new dedicated fault file')
    rclpy.init()
    n = rclpy.create_node('verify_stage5_real_cancel')
    statuses, ready, goals, joints = [], [], {}, []
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    n.create_subscription(String, '/pick_place/execution_status', lambda m: statuses.append(m.data), qos)
    n.create_subscription(Bool, '/perception/obstacles_ready', lambda m: ready.append(m.data), qos)
    n.create_subscription(JointState, '/joint_states', lambda m: joints.append((time.monotonic(), dict(zip(m.name, m.position)))), 10)

    def goal_status(msg):
        for item in msg.status_list:
            goals[bytes(item.goal_info.goal_id.uuid).hex()] = item.status

    n.create_subscription(GoalStatusArray, '/execute_task_solution/_action/status', goal_status, 10)
    execute = n.create_client(Trigger, '/pick_place/execute')
    capture = n.create_client(Trigger, '/perception/capture_obstacles')
    reset = n.create_client(Trigger, '/pick_place/reset')

    def wait(condition, timeout):
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline and not condition():
            rclpy.spin_once(n, timeout_sec=.05)
        if not condition():
            raise RuntimeError('runtime cancel wait failed: '+str(statuses[-4:]))

    def call(client):
        wait(client.service_is_ready, 30)
        f = client.call_async(Trigger.Request())
        wait(f.done, 5)
        return f.result()

    try:
        wait(capture.service_is_ready, 30)
        response = call(capture)
        if not response.success:
            raise RuntimeError(response.message)
        wait(lambda: ready and ready[-1], 20)
        accepted, deadline = None, time.monotonic()+90
        while time.monotonic() < deadline:
            accepted = call(execute)
            if accepted.success:
                break
            until = time.monotonic()+.3
            while time.monotonic() < until:
                rclpy.spin_once(n, timeout_sec=.05)
        if not accepted or not accepted.success:
            raise RuntimeError('known-scene candidate did not become ready: '+accepted.message)
        wait(lambda: any(status == GoalStatus.STATUS_EXECUTING for status in goals.values()), 20)
        stopped_source_at = time.monotonic()
        os.killpg(args.perception_launch_pid, signal.SIGINT)
        wait(lambda: any('requesting action cancellation' in s and 'perception_heartbeat_expired' in s for s in statuses), 8)
        wait(lambda: any('cancelled: stop acknowledged' in s for s in statuses), 15)
        wait(lambda: fault.is_file(), 5)
        wait(lambda: any(status == GoalStatus.STATUS_CANCELED for status in goals.values()), 5)
        until = time.monotonic()+1.5
        while time.monotonic() < until:
            rclpy.spin_once(n, timeout_sec=.05)
        samples = [sample for when, sample in joints if when > time.monotonic()-1.]
        if len(samples) < 5:
            raise RuntimeError('no post-cancel Gazebo joint feedback')
        names = {f'joint{i}' for i in range(1, 7)} | {'ctm2f110_finger_joint'}
        spread = {name: max(s[name] for s in samples)-min(s[name] for s in samples) for name in names}
        if any(value > .01 for value in spread.values()):
            raise RuntimeError('robot feedback not settled after cancel: '+str(spread))
        if len(goals) != 1:
            raise RuntimeError('later action dispatched after input lease lost')
        retry, reset_response = call(execute), call(reset)
        if retry.success or reset_response.success:
            raise RuntimeError('fault latch did not block repeat/reset')
        report = {'fixture': 'known-scene MTC candidate; actual RGB-D input lease; not vision-only task proof',
                  'source_stopped': True, 'terminal_canceled': True, 'goal_count': len(goals),
                  'settled_joint_spread_rad': spread, 'fault_file': str(fault), 'statuses': statuses,
                  'refused_retry': retry.message, 'refused_reset': reset_response.message,
                  'elapsed_after_source_stop_sec': time.monotonic()-stopped_source_at}
        Path(args.output).write_text(json.dumps(report, indent=2))
        print('PASS real MTC/Gazebo action: lost RGB-D producer -> cancel -> terminal CANCELED -> settled joint feedback; no next goal, repeat/reset refused', flush=True)
    finally:
        n.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
