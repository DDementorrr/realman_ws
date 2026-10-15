#!/usr/bin/env python3
"""Runtime gate for visual localization and PlanningScene synchronization."""

import math
import sys
import time
import argparse

import rclpy
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--planning-only', action='store_true',
                        help='Skip Gazebo ground-truth requirement; does not validate localization accuracy')
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node("verify_pick_place_stage6")
    values = {}
    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
    subscriptions = [
        node.create_subscription(PoseStamped, "/object_pose",
                                 lambda message: values.update(object=message), latched),
        node.create_subscription(PoseStamped, "/pick_place/sim/object_pose",
                                 lambda message: values.update(gazebo=message), 10),
        node.create_subscription(PoseStamped, "/place_pose",
                                 lambda message: values.update(place=message), 10),
    ]
    client = node.create_client(GetPlanningScene, "/get_planning_scene")
    try:
        deadline = time.monotonic() + 45
        while time.monotonic() < deadline and (
                "object" not in values or (not args.planning_only and "gazebo" not in values)):
            rclpy.spin_once(node, timeout_sec=0.25)
        if "object" not in values:
            print("FAIL visual /object_pose unavailable", file=sys.stderr)
            return 1
        if values['object'].header.frame_id != 'base_link':
            print('FAIL visual pose is not expressed in base_link', file=sys.stderr)
            return 1
        if not args.planning_only and 'gazebo' not in values:
            print('FAIL Gazebo object ground truth unavailable', file=sys.stderr)
            return 1
        visual = values["object"].pose.position
        if visual.x == 0.0 and visual.y == 0.0:
            print("FAIL visual pose remained at zero", file=sys.stderr)
            return 1
        if not client.wait_for_service(timeout_sec=10):
            print("FAIL PlanningScene service unavailable", file=sys.stderr)
            return 1
        request = GetPlanningScene.Request()
        request.components.components = PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=10)
        if not future.done() or future.result() is None:
            print("FAIL PlanningScene query timed out", file=sys.stderr)
            return 1
        objects = [o for o in future.result().scene.world.collision_objects if o.id == "pick_object"]
        if len(objects) != 1 or len(objects[0].primitive_poses) != 1:
            print("FAIL visually tracked object missing from PlanningScene", file=sys.stderr)
            return 1
        object_msg = objects[0]
        # Geometry pose is relative to CollisionObject.pose; both transforms
        # can be nonidentity after detach or an external scene update.
        primitive = object_msg.primitive_poses[0]
        top = object_msg.pose
        q = top.orientation
        norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w
        if norm <= 0 or object_msg.header.frame_id not in ('world', 'base_link'):
            print('FAIL PlanningScene object transform is invalid or uses an unexpected frame', file=sys.stderr)
            return 1
        x, y, z, w, p = q.x, q.y, q.z, q.w, primitive.position
        rotated = (
            (1-2*(y*y+z*z)/norm)*p.x + 2*(x*y-z*w)/norm*p.y + 2*(x*z+y*w)/norm*p.z,
            2*(x*y+z*w)/norm*p.x + (1-2*(x*x+z*z)/norm)*p.y + 2*(y*z-x*w)/norm*p.z,
            2*(x*z-y*w)/norm*p.x + 2*(y*z+x*w)/norm*p.y + (1-2*(x*x+y*y)/norm)*p.z)
        # This simulation fixes world -> base_link at identity.
        scene = tuple(a+b for a, b in zip(
            (top.position.x, top.position.y, top.position.z), rotated))
        scene_error = math.dist((visual.x, visual.y, visual.z), scene)
        if scene_error > 0.015:
            print(f"FAIL visual pose differs from PlanningScene by {scene_error:.3f} m",
                  file=sys.stderr)
            return 1
        if "gazebo" in values:
            gazebo = values["gazebo"].pose.position
            error = math.dist((visual.x, visual.y, visual.z), (gazebo.x, gazebo.y, gazebo.z))
            if not args.planning_only and error > 0.02:
                print(f'FAIL visual localization differs from Gazebo by {error:.3f} m', file=sys.stderr)
                return 1
            gazebo_text = f"Gazebo error {error:.3f} m"
        else:
            gazebo_text = "Gazebo feedback not sampled in planning-only gate"
        print(f"PASS ArUco /object_pose in base_link; {gazebo_text}; "
              f"PlanningScene sync {scene_error:.3f} m")
        return 0
    finally:
        for subscription in subscriptions:
            node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
