#!/usr/bin/env python3
"""Read-only CTM2F110 simulation checks; never sends an actuator goal."""

import argparse
import re
import subprocess
import time

import rclpy
from sensor_msgs.msg import JointState


def cli(*args):
    result = subprocess.run(args, capture_output=True, text=True, timeout=12, check=True)
    return re.sub(r"\x1b\[[0-9;]*[mK]", "", result.stdout)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout", type=float, default=10.0)
    args = parser.parse_args()
    failures = []

    controllers = cli("ros2", "control", "list_controllers")
    for name in ("rm65b_arm_controller", "ctm2f110_hand_controller", "joint_state_broadcaster"):
        active = any(
            line.split()[0] == name and line.split()[-1] == "active"
            for line in controllers.splitlines() if line.split()
        )
        print(f"{'PASS' if active else 'FAIL'} {name} active")
        if not active:
            failures.append(name)

    actions = cli("ros2", "action", "list")
    for name in ("/move_action", "/rm65b_arm_controller/follow_joint_trajectory",
                 "/ctm2f110_hand_controller/follow_joint_trajectory"):
        present = name in actions.splitlines()
        print(f"{'PASS' if present else 'FAIL'} action {name}")
        if not present:
            failures.append(name)

    rclpy.init()
    node = rclpy.create_node("verify_realman_gripper_gz_runtime")
    messages = []
    subscription = node.create_subscription(JointState, "/joint_states", messages.append, 10)
    deadline = time.monotonic() + args.timeout
    while not messages and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    if not messages:
        failures.append("/joint_states timeout")
        print("FAIL /joint_states timeout")
    else:
        msg = messages[-1]
        positions = dict(zip(msg.name, msg.position))
        finger = positions.get("ctm2f110_finger_joint")
        if finger is None:
            failures.append("finger joint missing")
            print("FAIL finger joint missing")
        else:
            print(f"INFO ctm2f110_finger_joint={finger:.5f} rad")
            expected = {
                "ctm2f110_rightout_joint": finger,
                "ctm2f110_left_kckle_joint": finger,
                "ctm2f110_right_kckle_joint": finger,
                "ctm2f110_leftinn_joint": -finger,
                "ctm2f110_rightinn_joint": -finger,
            }
            for name, target in expected.items():
                actual = positions.get(name)
                valid = actual is not None and abs(actual - target) <= 0.02
                print(f"{'PASS' if valid else 'FAIL'} mimic {name}: {actual}")
                if not valid:
                    failures.append(name)
            in_range = -0.002 <= finger <= 1.002
            print(f"{'PASS' if in_range else 'FAIL'} finger inside source joint limits")
            if not in_range:
                failures.append("finger limits")
    node.destroy_subscription(subscription)
    node.destroy_node()
    rclpy.shutdown()
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
