#!/usr/bin/env python3
"""Read-only FK check of every waypoint in the published OMPL carry stage."""

import argparse
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionFK
from moveit_task_constructor_msgs.msg import Solution, TaskDescription
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


def multiply(a, b):
    x, y, z, w = a
    X, Y, Z, W = b
    return (w*X+x*W+y*Z-z*Y, w*Y-x*Z+y*W+z*X,
            w*Z+x*Y-y*X+z*W, w*W-x*X-y*Y-z*Z)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--tolerance', type=float, default=0.2)
    parser.add_argument('--grasp-spin', type=float, default=math.pi/2)
    parser.add_argument('--check-yaw', action='store_true',
                        help='Also enforce the rotation-vector Z tolerance')
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('verify_mtc_transport_orientation')
    messages = {}
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)
    subscriptions = [node.create_subscription(kind, topic,
        lambda msg, key=key: messages.update({key: msg}), qos)
        for key, kind, topic in (
            ('solution', Solution, '/pick_place/solution'),
            ('description', TaskDescription, '/pick_place/task_description'))]
    subscriptions.append(node.create_subscription(PoseStamped, '/place_pose',
        lambda msg: messages.update(place=msg), 10))
    client = node.create_client(GetPositionFK, '/compute_fk')
    try:
        deadline = time.monotonic() + 120
        while len(messages) < 3 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
        if len(messages) < 3 or not client.wait_for_service(timeout_sec=10):
            raise RuntimeError('MTC solution, place pose, or FK service unavailable')
        names = {stage.id: stage.name for stage in messages['description'].stages}
        place = messages['place']
        q = place.pose.orientation
        goal = multiply((q.x, q.y, q.z, q.w), multiply(
            (0., math.sin(math.pi/4), 0., math.cos(math.pi/4)),
            (0., 0., math.sin(args.grasp_spin/2), math.cos(args.grasp_spin/2))))
        goal_norm = math.sqrt(sum(v*v for v in goal))
        if goal_norm < 1e-6:
            raise RuntimeError('Invalid place orientation')
        goal_inverse = tuple(-v/goal_norm for v in goal[:3]) + (goal[3]/goal_norm,)
        solution = messages['solution']
        start = solution.start_scene.robot_state.joint_state
        positions = dict(zip(start.name, start.position))
        maximum, count, stages = [0., 0., 0.], 0, 0
        endpoints = []
        for part in solution.sub_trajectory:
            transport = names.get(part.info.stage_id) == 'Transport (OMPL)'
            stages += int(transport)
            trajectory = part.trajectory.joint_trajectory
            for point in trajectory.points:
                positions.update(zip(trajectory.joint_names, point.positions))
                if not transport:
                    continue
                request = GetPositionFK.Request()
                request.header.frame_id = place.header.frame_id
                request.fk_link_names = ['ctm2f110_tool0']
                request.robot_state.joint_state.name = list(positions)
                request.robot_state.joint_state.position = list(positions.values())
                future = client.call_async(request)
                rclpy.spin_until_future_complete(node, future, timeout_sec=5)
                if not future.done() or future.result() is None:
                    raise RuntimeError('FK query timed out')
                result = future.result()
                if result.error_code.val != 1 or len(result.pose_stamped) != 1:
                    raise RuntimeError('FK query failed')
                actual = result.pose_stamped[0].pose.orientation
                # Rotation vector expressed in the fixed constraint frame,
                # as used by MoveIt's ROTATION_VECTOR parameterization.
                delta = multiply((actual.x, actual.y, actual.z, actual.w), goal_inverse)
                if delta[3] < 0:
                    delta = tuple(-v for v in delta)
                sine = math.sqrt(sum(v*v for v in delta[:3]))
                angle = 2*math.atan2(sine, delta[3])
                vector = [v*angle/sine if sine > 1e-9 else 0. for v in delta[:3]]
                if not endpoints:
                    endpoints.append(vector)
                if len(endpoints) == 1:
                    endpoints.append(vector)
                else:
                    endpoints[-1] = vector
                maximum = [max(old, abs(new)) for old, new in zip(maximum, vector)]
                count += 1
        if stages != 1 or count < 2:
            raise RuntimeError('No unique nonempty Transport (OMPL) trajectory')
        valid = max(maximum if args.check_yaw else maximum[:2]) <= args.tolerance + 0.001
        print(f'{"PASS" if valid else "FAIL"} transport FK: {count} waypoints; '
              f'max fixed-frame rotation-vector error={maximum} rad; tolerance={args.tolerance}; '
              f'start/end rotation vectors={endpoints}')
        return 0 if valid else 1
    except RuntimeError as error:
        print(f'FAIL {error}')
        return 1
    except KeyboardInterrupt:
        return 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
