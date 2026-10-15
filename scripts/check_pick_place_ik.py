#!/usr/bin/env python3
"""Probe RM65B TCP grasp and place reachability before running MTC."""

import argparse
import math
from pathlib import Path
import sys

import rclpy
import yaml
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene, GetPositionFK, GetPositionIK, GetStateValidity


CONFIG = Path(__file__).resolve().parents[1] / "src/ros2_adapters/pick_place_demo/config/known_scene.yaml"
PARAMS = yaml.safe_load(CONFIG.read_text())["known_pick_place_scene"]["ros__parameters"]
TARGETS = {"grasp": PARAMS["object_pose"][:3], "place": PARAMS["place_pose"][:3]}


def quaternion_product(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def call(node, client, request, timeout=10):
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    if not future.done() or future.result() is None:
        raise RuntimeError(f"{client.srv_name} timed out")
    return future.result()


def pose_error(expected, actual):
    ep, ap = expected.pose.position, actual.pose.position
    distance = math.sqrt((ep.x - ap.x) ** 2 + (ep.y - ap.y) ** 2 + (ep.z - ap.z) ** 2)
    eq, aq = expected.pose.orientation, actual.pose.orientation
    expected_norm = math.sqrt(eq.x ** 2 + eq.y ** 2 + eq.z ** 2 + eq.w ** 2)
    actual_norm = math.sqrt(aq.x ** 2 + aq.y ** 2 + aq.z ** 2 + aq.w ** 2)
    dot = abs(eq.x * aq.x + eq.y * aq.y + eq.z * aq.z + eq.w * aq.w) / (expected_norm * actual_norm)
    angle = 2 * math.acos(min(1.0, dot))
    return distance, angle


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--ignore-collisions", action="store_true")
    orientation = parser.add_mutually_exclusive_group()
    orientation.add_argument("--horizontal", action="store_const", dest="orientation", const="horizontal")
    orientation.add_argument("--tool-down", action="store_const", dest="orientation", const="down")
    orientation.add_argument("--tool-up", action="store_const", dest="orientation", const="up")
    parser.set_defaults(orientation="horizontal")
    parser.add_argument("--z-offset", type=float, default=0.0)
    parser.add_argument("--yaw-step", type=int, default=30)
    parser.add_argument("--grasp-xyz", type=float, nargs=3, default=TARGETS["grasp"])
    parser.add_argument("--place-xyz", type=float, nargs=3, default=TARGETS["place"])
    args = parser.parse_args()
    if args.yaw_step <= 0 or 360 % args.yaw_step:
        parser.error("--yaw-step must divide 360")
    rclpy.init()
    node = rclpy.create_node("check_pick_place_ik")
    scene_client = node.create_client(GetPlanningScene, "/get_planning_scene")
    fk_client = node.create_client(GetPositionFK, "/compute_fk")
    ik_client = node.create_client(GetPositionIK, "/compute_ik")
    validity_client = node.create_client(GetStateValidity, "/check_state_validity")
    try:
        if not (scene_client.wait_for_service(timeout_sec=10) and
                fk_client.wait_for_service(timeout_sec=10) and
                ik_client.wait_for_service(timeout_sec=10) and
                validity_client.wait_for_service(timeout_sec=10)):
            raise RuntimeError("MoveIt PlanningScene, IK, or state-validity service is unavailable")
        scene_req = GetPlanningScene.Request()
        scene_req.components.components = PlanningSceneComponents.ROBOT_STATE
        seed = call(node, scene_client, scene_req).scene.robot_state
        if not seed.joint_state.name:
            raise RuntimeError("MoveIt has no current joint state")
        all_ok = True
        for label, xyz in (("grasp", args.grasp_xyz), ("place", args.place_xyz)):
            success = []
            error_codes = set()
            orientations = ((degrees, spin) for degrees in range(0, 360, args.yaw_step)
                            for spin in (range(0, 360, 90) if args.orientation == "horizontal" else (0,)))
            for degrees, spin in orientations:
                angle = math.radians(degrees)
                request = GetPositionIK.Request()
                request.ik_request.group_name = "rm65_arm"
                request.ik_request.ik_link_name = "ctm2f110_tool0"
                request.ik_request.robot_state = seed
                request.ik_request.avoid_collisions = not (args.ignore_collisions or label == "grasp")
                request.ik_request.timeout = Duration(nanosec=250_000_000)
                pose = PoseStamped()
                pose.header.frame_id = "world"
                pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = xyz
                pose.pose.position.z += args.z_offset
                if args.orientation == "horizontal":
                    qz = (0.0, 0.0, math.sin(angle / 2), math.cos(angle / 2))
                    qy = (0.0, math.sin(math.pi / 4), 0.0, math.cos(math.pi / 4))
                    spin_angle = math.radians(spin)
                    qspin = (0.0, 0.0, math.sin(spin_angle / 2), math.cos(spin_angle / 2))
                    (pose.pose.orientation.x, pose.pose.orientation.y,
                     pose.pose.orientation.z, pose.pose.orientation.w) = quaternion_product(
                         quaternion_product(qz, qy), qspin)
                elif args.orientation == "up":
                    pose.pose.orientation.z = math.sin(angle / 2)
                    pose.pose.orientation.w = math.cos(angle / 2)
                else:
                    # Rz(yaw) * Rx(pi): tool Z points down into the cube.
                    pose.pose.orientation.x = math.cos(angle / 2)
                    pose.pose.orientation.y = math.sin(angle / 2)
                    pose.pose.orientation.w = 0.0
                request.ik_request.pose_stamped = pose
                result = call(node, ik_client, request)
                if result.error_code.val == result.error_code.SUCCESS:
                    fk_req = GetPositionFK.Request()
                    fk_req.header.frame_id = pose.header.frame_id
                    fk_req.fk_link_names = [request.ik_request.ik_link_name]
                    fk_req.robot_state = result.solution
                    fk = call(node, fk_client, fk_req)
                    if fk.error_code.val != fk.error_code.SUCCESS or len(fk.pose_stamped) != 1:
                        raise RuntimeError(f"FK failed for {label} yaw={degrees}")
                    distance, angle_error = pose_error(pose, fk.pose_stamped[0])
                    if distance > 0.005 or angle_error > 0.05:
                        print(f"INFO {label} yaw={degrees} IK/FK mismatch: "
                              f"translation={distance:.3f}m orientation={angle_error:.3f}rad")
                        continue
                    validity_req = GetStateValidity.Request()
                    validity_req.robot_state = result.solution
                    validity_req.group_name = "rm65_arm_hand"
                    validity = call(node, validity_client, validity_req)
                    contacts = sorted({tuple(sorted((c.contact_body_1, c.contact_body_2)))
                                       for c in validity.contacts})
                    expected_grasp_contact = bool(contacts) and all(
                        "pick_object" in pair and any(body.startswith("ctm2f110_") for body in pair)
                        for pair in contacts)
                    accepted = args.ignore_collisions or (expected_grasp_contact if label == "grasp"
                                                          else validity.valid)
                    if accepted:
                        success.append((degrees, spin) if args.orientation == "horizontal" else degrees)
                    if args.ignore_collisions or not accepted:
                        print(f"INFO {label} yaw={degrees} spin={spin} state_valid={validity.valid} contacts={contacts}")
                else:
                    error_codes.add(result.error_code.val)
            if success:
                print(f"PASS {label} contact-compatible IK at yaw degrees: {success}")
            else:
                print(f"FAIL {label} contact-compatible IK; MoveIt error codes: {sorted(error_codes)}")
                all_ok = False
        return 0 if all_ok else 1
    except RuntimeError as error:
        print(f"FAIL {error}", file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
