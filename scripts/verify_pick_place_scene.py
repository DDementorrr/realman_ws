#!/usr/bin/env python3
"""Check the known-pose Gazebo and MoveIt scene while known_scene.launch.py runs."""

from pathlib import Path
import re
import subprocess
import sys
import time

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene


ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "src/ros2_adapters/pick_place_demo/config/known_scene.yaml"
PARAMS = yaml.safe_load(CONFIG.read_text())["known_pick_place_scene"]["ros__parameters"]


def require(condition, message):
    print(f"{'PASS' if condition else 'FAIL'} {message}")
    if not condition:
        raise RuntimeError(message)


def near(actual, expected, tolerance=0.005):
    return all(abs(a - b) <= tolerance for a, b in zip(actual, expected))


def gazebo_pose(model):
    result = subprocess.run(
        ["gz", "model", "-m", model, "-p"], text=True, capture_output=True,
        timeout=10, check=True,
    )
    match = re.search(r"\[([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)\]\s*\n\s*\[", result.stdout)
    require(match is not None, f"Gazebo model {model} has a pose")
    return tuple(float(value) for value in match.groups())


def main():
    rclpy.init()
    node = rclpy.create_node("verify_pick_place_scene")
    poses = {}
    subscriptions = [
        node.create_subscription(PoseStamped, topic, lambda msg, key=key: poses.__setitem__(key, msg), 10)
        for topic, key in (("/object_pose", "object_pose"), ("/place_pose", "place_pose"))
    ]
    client = node.create_client(GetPlanningScene, "/get_planning_scene")
    try:
        require(client.wait_for_service(timeout_sec=15), "MoveIt PlanningScene service available")
        deadline = time.monotonic() + 15
        while len(poses) < 2 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
        for key in ("object_pose", "place_pose"):
            pose = poses.get(key)
            require(pose is not None, f"{key} is published")
            xyz = (pose.pose.position.x, pose.pose.position.y, pose.pose.position.z)
            require(pose.header.frame_id == PARAMS["frame_id"] and near(xyz, PARAMS[key][:3]),
                    f"{key} matches configured world pose")

        request = GetPlanningScene.Request()
        request.components.components = PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=15)
        require(future.done() and future.result() is not None, "MoveIt returns world collision geometry")
        objects = {obj.id: obj for obj in future.result().scene.world.collision_objects}
        for key, size_key, pose_key in (("object_id", "object_size", "object_pose"),
                                        ("table_id", "table_size", "table_pose")):
            obj = objects.get(PARAMS[key])
            require(obj is not None, f"MoveIt contains {PARAMS[key]}")
            require(len(obj.primitives) == 1 and near(obj.primitives[0].dimensions, PARAMS[size_key]),
                    f"{PARAMS[key]} collision dimensions match Gazebo")
            xyz = (obj.pose.position.x + obj.primitive_poses[0].position.x,
                   obj.pose.position.y + obj.primitive_poses[0].position.y,
                   obj.pose.position.z + obj.primitive_poses[0].position.z)
            require(near(xyz, PARAMS[pose_key][:3]), f"{PARAMS[key]} MoveIt pose matches configuration")

        for model, key in ((PARAMS["table_id"], "table_pose"),
                           (PARAMS["object_id"], "object_pose")):
            require(near(gazebo_pose(model), PARAMS[key][:3]),
                    f"Gazebo {model} pose matches MoveIt configuration")
        zone = gazebo_pose("place_zone")
        require(near(zone[:2], PARAMS["place_pose"][:2]), "Gazebo placement marker matches target XY")
        top = PARAMS["table_pose"][2] + PARAMS["table_size"][2] / 2
        bottom = PARAMS["object_pose"][2] - PARAMS["object_size"][2] / 2
        require(abs(top - bottom) <= 0.005, "object rests on table top")
        require(abs(top - (PARAMS["place_pose"][2] - PARAMS["object_size"][2] / 2)) <= 0.005,
                "place target rests on table top")
        return 0
    except (RuntimeError, subprocess.SubprocessError) as exc:
        print(f"FAIL scene verification: {exc}", file=sys.stderr)
        return 1
    finally:
        subscriptions.clear()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
