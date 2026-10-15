#!/usr/bin/env python3
"""Check that gripper TF uses Gazebo joint feedback, including mimic error.

Run while closing/carrying to distinguish measured linkage transforms from
the ideal mimic transforms used by MoveIt. No actuator commands are sent.
"""

import argparse
import math
import time
import xml.etree.ElementTree as ET

import rclpy
from rclpy.duration import Duration
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener


def multiply(a, b):
    x, y, z, w = a
    X, Y, Z, W = b
    return (w*X+x*W+y*Z-z*Y, w*Y-x*Z+y*W+z*X,
            w*Z+x*Y-y*X+z*W, w*W-x*X-y*Y-z*Z)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--duration', type=float, default=10)
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('verify_gripper_feedback_tf')
    node.set_parameters([Parameter('use_sim_time', value=True)])
    buffer = Buffer(cache_time=Duration(seconds=30))
    listener = TransformListener(buffer, node)
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)
    descriptions, pending = [], []
    subscriptions = [
        node.create_subscription(String, '/robot_description_feedback',
                                 lambda msg: descriptions.append(msg.data), qos),
        node.create_subscription(JointState, '/joint_states', pending.append, 10)]
    joints, counts = {}, {}
    max_angle, max_translation, max_mimic_error = 0., 0., 0.
    deadline = time.monotonic() + args.duration
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            if descriptions and not joints:
                model = ET.fromstring(descriptions[-1])
                for joint in model.findall('joint'):
                    name = joint.get('name', '')
                    if not name.startswith('ctm2f110_') or joint.get('type') != 'revolute':
                        continue
                    if joint.find('mimic') is not None:
                        print(f'FAIL feedback description still has mimic: {name}')
                        return 1
                    origin = joint.find('origin')
                    xyz = tuple(map(float, origin.get('xyz', '0 0 0').split()))
                    roll, pitch, yaw = map(float, origin.get('rpy', '0 0 0').split())
                    q = multiply(multiply((0., 0., math.sin(yaw/2), math.cos(yaw/2)),
                                          (0., math.sin(pitch/2), 0., math.cos(pitch/2))),
                                 (math.sin(roll/2), 0., 0., math.cos(roll/2)))
                    axis = tuple(map(float, joint.find('axis').get('xyz').split()))
                    joints[name] = (joint.find('parent').get('link'),
                                    joint.find('child').get('link'), xyz, q, axis)
                    counts[name] = 0
            retry = []
            for msg in pending[-200:]:
                positions = dict(zip(msg.name, msg.position))
                stamp = Time.from_msg(msg.header.stamp)
                transforms = {}
                for name, (parent, child, xyz, origin_q, axis) in joints.items():
                    if name not in positions:
                        continue
                    try:
                        transforms[name] = buffer.lookup_transform(parent, child, stamp).transform
                    except TransformException:
                        retry.append(msg)
                        break
                if len(transforms) != len(joints):
                    continue
                for name, tf in transforms.items():
                    parent, child, xyz, origin_q, axis = joints[name]
                    angle = positions[name]
                    length = math.sqrt(sum(v*v for v in axis))
                    motion = tuple(v/length*math.sin(angle/2) for v in axis) + (math.cos(angle/2),)
                    expected = multiply(origin_q, motion)
                    actual = (tf.rotation.x, tf.rotation.y, tf.rotation.z, tf.rotation.w)
                    dot = abs(sum(a*b for a, b in zip(expected, actual)))
                    max_angle = max(max_angle, 2*math.acos(min(1., dot)))
                    max_translation = max(max_translation, math.dist(
                        xyz, (tf.translation.x, tf.translation.y, tf.translation.z)))
                    counts[name] += 1
                finger = positions.get('ctm2f110_finger_joint')
                if finger is not None:
                    for name, sign in (('rightout', 1), ('leftinn', -1), ('rightinn', -1),
                                       ('left_kckle', 1), ('right_kckle', 1)):
                        value = positions.get(f'ctm2f110_{name}_joint')
                        if value is not None:
                            max_mimic_error = max(max_mimic_error, abs(value-sign*finger))
            pending[:] = retry[-200:]
        valid = (len(counts) == 6 and min(counts.values(), default=0) >= 5
                 and max_angle < 0.002 and max_translation < 0.00001)
        print(f'{"PASS" if valid else "FAIL"} measured gripper TF: samples={counts}; '
              f'max rotation error={max_angle:.6f} rad, translation={max_translation:.6f} m; '
              f'actual mimic error={max_mimic_error:.6f} rad')
        return 0 if valid else 1
    except KeyboardInterrupt:
        return 130
    finally:
        for subscription in subscriptions:
            node.destroy_subscription(subscription)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
