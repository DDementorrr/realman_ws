#!/usr/bin/env python3
"""Runtime gate for one complete fixed-attachment Gazebo Pick and Place cycle."""

import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String


def main():
    rclpy.init()
    node = rclpy.create_node("verify_pick_place_stage3")
    values = {}
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)
    subscriptions = [
        node.create_subscription(Bool, "/pick_place/execution_success",
                                 lambda msg: values.update(success=msg.data), qos),
        node.create_subscription(String, "/pick_place/execution_status",
                                 lambda msg: values.update(status=msg.data), qos),
        node.create_subscription(String, "/pick_place/sim/attachment_state",
                                 lambda msg: values.update(attachment=msg.data), qos),
        node.create_subscription(PoseStamped, "/pick_place/sim/object_pose",
                                 lambda msg: values.update(object=msg), 10),
        node.create_subscription(PoseStamped, "/place_pose",
                                 lambda msg: values.update(place=msg), 10),
    ]
    client = node.create_client(GetPlanningScene, "/get_planning_scene")
    try:
        deadline = time.monotonic() + 180
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.25)
            status = values.get("status", "")
            if status.startswith("Pick and Place failed:"):
                print(f"FAIL execution: {status}", file=sys.stderr)
                return 1
            if (values.get("success") is True and status == "Pick and Place completed" and
                    "object" in values and "place" in values):
                actual = values["object"].pose.position
                target = values["place"].pose.position
                if math.dist((actual.x, actual.y, actual.z),
                             (target.x, target.y, target.z)) <= 0.04:
                    break
        else:
            print(f"FAIL execution: {values.get('status', 'timed out')}", file=sys.stderr)
            return 1
        state_deadline = time.monotonic() + 5
        while values.get("attachment") != "detached" and time.monotonic() < state_deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if values.get("attachment") != "detached":
            print("FAIL Gazebo joint remains attached", file=sys.stderr)
            return 1
        if "object" not in values or "place" not in values:
            print("FAIL missing Gazebo object or target pose", file=sys.stderr)
            return 1
        actual = values["object"].pose.position
        target = values["place"].pose.position
        error = math.dist((actual.x, actual.y, actual.z),
                          (target.x, target.y, target.z))
        if error > 0.04:
            print(f"FAIL Gazebo placement error {error:.3f} m", file=sys.stderr)
            return 1
        if not client.wait_for_service(timeout_sec=5):
            print("FAIL PlanningScene service unavailable", file=sys.stderr)
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
        world = [obj for obj in scene.world.collision_objects if obj.id == "pick_object"]
        attached = [obj for obj in scene.robot_state.attached_collision_objects
                    if obj.object.id == "pick_object"]
        if len(world) != 1 or attached:
            print("FAIL object attachment differs from Gazebo", file=sys.stderr)
            return 1
        print(f"PASS full MTC execution; Gazebo and PlanningScene detached; placement error {error:.3f} m")
        return 0
    finally:
        for subscription in subscriptions:
            node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
