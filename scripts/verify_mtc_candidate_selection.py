#!/usr/bin/env python3
"""Check reported candidate selection against the published six-axis trajectory.

Only the selected trajectory is published on /pick_place/solution. Other
candidate scores are checked as reported metrics, not recomputed trajectories.
"""

import argparse
import math
import re
import time

import rclpy
from moveit_task_constructor_msgs.msg import Solution
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--timeout', type=float, default=300)
    parser.add_argument('--min-candidates', type=int, default=3)
    args = parser.parse_args()
    if args.timeout <= 0 or args.min_candidates < 1:
        parser.error('timeout and min-candidates must be positive')
    rclpy.init()
    node = rclpy.create_node('verify_mtc_candidate_selection')
    received = {}
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)
    subscriptions = [node.create_subscription(kind, topic,
        lambda message, key=key: received.update({key: message}), qos)
        for key, kind, topic in (
            ('metrics', String, '/pick_place/planning_metrics'),
            ('solution', Solution, '/pick_place/solution'))]
    try:
        deadline = time.monotonic() + args.timeout
        while len(received) < 2 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
        if len(received) < 2:
            raise ValueError('metrics or selected solution unavailable')
        report = received['metrics'].data
        weights_match = re.search(r'weights=([^ ]+)', report)
        selected_match = re.search(r'selected=(\d+)', report)
        entries = re.findall(r'candidate\[(\d+)\]=([^ ]+)', report)
        if not weights_match or not selected_match or not entries:
            raise ValueError('malformed metrics')
        weights = [float(v) for v in weights_match[1].strip(',').split(',')]
        scores = {int(index): float(value) for index, value in entries}
        selected = int(selected_match[1])
        if (len(weights) != 6 or any(not math.isfinite(v) or v <= 0 for v in weights)
                or len(scores) != len(entries)
                or set(scores) != set(range(len(scores)))
                or len(scores) < args.min_candidates
                or selected not in scores
                or any(not math.isfinite(v) for v in scores.values())):
            raise ValueError('invalid weights, candidate count, indices, or scores')
        if scores[selected] != min(scores.values()):
            raise ValueError('selected candidate does not have minimum reported cost')
        previous, measured = {}, 0.0
        for part in received['solution'].sub_trajectory:
            trajectory = part.trajectory.joint_trajectory
            for point in trajectory.points:
                for axis, weight in enumerate(weights, start=1):
                    name = f'joint{axis}'
                    if name not in trajectory.joint_names:
                        continue
                    index = trajectory.joint_names.index(name)
                    if index >= len(point.positions) or not math.isfinite(point.positions[index]):
                        raise ValueError('missing or nonfinite arm waypoint')
                    value = point.positions[index]
                    if name in previous:
                        measured += weight * abs(value - previous[name])
                    previous[name] = value
        if len(previous) != 6 or abs(measured - scores[selected]) > 0.00001:
            raise ValueError('selected trajectory cost disagrees with reported score')
        print(f'PASS {len(scores)} reported candidates; selected={selected}; '
              f'published six-axis weighted travel={measured:.6f}; weights={weights}')
        return 0
    except (ValueError, OverflowError) as error:
        print(f'FAIL candidate selection: {error}')
        return 1
    finally:
        for subscription in subscriptions:
            node.destroy_subscription(subscription)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
