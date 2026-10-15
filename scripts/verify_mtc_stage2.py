#!/usr/bin/env python3
"""Runtime gate for the complete MTC plan, with no Gazebo execution yet."""

import argparse
import sys
import time

import rclpy
from moveit_msgs.msg import PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--timeout', type=float, default=60.0,
                        help='Seconds to wait for plan_ready; use 300 for constrained multi-candidate planning')
    args = parser.parse_args()
    if args.timeout <= 0:
        parser.error('--timeout must be positive')
    rclpy.init()
    node = rclpy.create_node("verify_mtc_stage2")
    messages = []
    qos = QoSProfile(
        depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
        reliability=ReliabilityPolicy.RELIABLE,
    )
    subscription = node.create_subscription(
        Bool, "/pick_place/plan_ready", messages.append, qos,
    )
    client = node.create_client(GetPlanningScene, "/get_planning_scene")
    try:
        deadline = time.monotonic() + args.timeout
        while not messages and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.25)
        if not messages or not messages[-1].data:
            print("FAIL complete MTC solution was not published", file=sys.stderr)
            return 1
        print("PASS complete MTC solution published")
        if not client.wait_for_service(timeout_sec=5):
            print("FAIL MoveIt PlanningScene service unavailable", file=sys.stderr)
            return 1
        request = GetPlanningScene.Request()
        request.components.components = (
            PlanningSceneComponents.WORLD_OBJECT_GEOMETRY |
            PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
        )
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=10)
        if not future.done() or future.result() is None:
            print("FAIL PlanningScene query timed out", file=sys.stderr)
            return 1
        scene = future.result().scene
        world_ids = {obj.id for obj in scene.world.collision_objects}
        attached_ids = {obj.object.id for obj in scene.robot_state.attached_collision_objects}
        if "pick_object" not in world_ids or "pick_object" in attached_ids:
            print("FAIL plan-only stage changed the live object attachment state", file=sys.stderr)
            return 1
        print("PASS live object remains in world; task was planned without execution")
        return 0
    finally:
        node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
