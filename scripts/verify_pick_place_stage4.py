#!/usr/bin/env python3
"""Runtime gate for one contact-only Gazebo Pick and Place cycle.

Start this before triggering execution so it can observe the contact-only
milestones, not just the transient-local final result.
"""

import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from sensor_msgs.msg import JointState


def main():
    rclpy.init()
    node = rclpy.create_node("verify_pick_place_stage4")
    values = {"statuses": [], "attachment_history": [], "carrying": False,
              "contact_samples": 0, "bilateral_pad_samples": 0,
              "joint_samples": 0, "mimic_error": 0.0, "pad_displacement": {}}
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)

    def on_status(message):
        values["statuses"].append(message.data)
        print(f'INFO {message.data}', flush=True)
        if message.data.startswith("executing physical lift"):
            values["carrying"] = True
        elif message.data.startswith("executing physical lower"):
            values["carrying"] = False

    def on_contact(message):
        values["contact_samples"] += 1
        if (values["carrying"] and "left_pad" in message.data and
                "collision" in message.data and "right_pad" in message.data):
            values["bilateral_pad_samples"] += 1

    def on_joints(message):
        if not values['carrying']:
            return
        positions = dict(zip(message.name, message.position))
        finger = positions.get('ctm2f110_finger_joint')
        if finger is None:
            return
        values['joint_samples'] += 1
        for name, sign in (('rightout', 1), ('leftinn', -1), ('rightinn', -1),
                           ('left_kckle', 1), ('right_kckle', 1)):
            measured = positions.get(f'ctm2f110_{name}_joint')
            if measured is not None:
                values['mimic_error'] = max(values['mimic_error'], abs(measured - sign * finger))
        for side in ('left', 'right'):
            measured = positions.get(f'ctm2f110_{side}_pad_joint')
            if measured is not None:
                limits = values['pad_displacement'].setdefault(side, [measured, measured])
                limits[0], limits[1] = min(limits[0], measured), max(limits[1], measured)

    subscriptions = [
        node.create_subscription(Bool, "/pick_place/execution_success",
                                 lambda msg: values.update(success=msg.data), qos),
        node.create_subscription(String, "/pick_place/execution_status",
                                 on_status, qos),
        node.create_subscription(String, "/pick_place/sim/attachment_state",
                                 lambda msg: values["attachment_history"].append(msg.data), qos),
        node.create_subscription(String, "/pick_place/sim/contact_state", on_contact, 10),
        node.create_subscription(JointState, '/joint_states', on_joints, 10),
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
            statuses = values["statuses"]
            if statuses and statuses[-1].startswith("Pick and Place failed:"):
                print(f"FAIL execution: {statuses[-1]}; "
                      f"bilateral pad samples {values['bilateral_pad_samples']}/"
                      f"{values['contact_samples']}", file=sys.stderr)
                return 1
            if values.get("success") is True and "Pick and Place completed" in statuses:
                break
        else:
            print(f"FAIL execution: {values['statuses'][-1] if values['statuses'] else 'timed out'}",
                  file=sys.stderr)
            return 1

        required = ("after physical lift:", "after physical transport:",
                    "before physical release:", "after physical retreat:",
                    "executing return Home")
        missing = [marker for marker in required
                   if not any(marker in status for status in values["statuses"])]
        if missing:
            print(f"FAIL missing contact-only milestones: {missing}; start verifier before execution",
                  file=sys.stderr)
            return 1
        if not values["attachment_history"] or any(
                state != "detached" for state in values["attachment_history"]):
            print("FAIL Gazebo fixed attachment was used or state was not observed", file=sys.stderr)
            return 1
        if values["bilateral_pad_samples"] == 0:
            print(f"FAIL no simultaneous left/right pad contact during carry "
                  f"({values['contact_samples']} contact samples)", file=sys.stderr)
            return 1
        if "object" not in values or "place" not in values:
            print("FAIL missing Gazebo object or target pose", file=sys.stderr)
            return 1
        actual = values["object"].pose
        target_message = values["place"]
        target = target_message.pose
        if target_message.header.frame_id == "base_link":
            # Gazebo reports the physical object in world; the robot base is
            # mounted 0.15 m above world in the current simulation scene.
            target.position.z += 0.15
        elif target_message.header.frame_id not in ("", "world"):
            print(f"FAIL unsupported place target frame: {target_message.header.frame_id}",
                  file=sys.stderr)
            return 1
        ap, tp = actual.position, target.position
        error = math.dist((ap.x, ap.y, ap.z), (tp.x, tp.y, tp.z))
        aq, tq = actual.orientation, target.orientation
        def z_axis(quaternion):
            x, y, z, w = quaternion.x, quaternion.y, quaternion.z, quaternion.w
            norm = x * x + y * y + z * z + w * w
            if norm <= 0.0:
                raise ValueError("Invalid zero-norm object orientation")
            return (2 * (x * z + w * y) / norm,
                    2 * (y * z - w * x) / norm,
                    1 - 2 * (x * x + y * y) / norm)

        object_up, target_up = z_axis(aq), z_axis(tq)
        tilt = math.acos(max(-1.0, min(1.0, sum(
            a * b for a, b in zip(object_up, target_up)))))
        if error > 0.04 or tilt > 0.35:
            print(f"FAIL Gazebo placement error {error:.3f} m, tilt {tilt:.3f} rad",
                  file=sys.stderr)
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
        if len(world) != 1 or len(world[0].primitive_poses) != 1 or attached:
            print("FAIL PlanningScene object not detached in world", file=sys.stderr)
            return 1
        # MoveIt can normalize geometry into CollisionObject.pose. Compose
        # the top-level and primitive transforms instead of reading only the
        # primitive's local coordinates (which can be identity after detach).
        top, primitive = world[0].pose, world[0].primitive_poses[0]
        q = top.orientation
        norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w
        if norm <= 0:
            raise ValueError('Invalid PlanningScene object orientation')
        x, y, z, w = q.x, q.y, q.z, q.w
        p = primitive.position
        rotated = (
            (1-2*(y*y+z*z)/norm)*p.x + 2*(x*y-z*w)/norm*p.y + 2*(x*z+y*w)/norm*p.z,
            2*(x*y+z*w)/norm*p.x + (1-2*(x*x+z*z)/norm)*p.y + 2*(y*z-x*w)/norm*p.z,
            2*(x*z-y*w)/norm*p.x + 2*(y*z+x*w)/norm*p.y + (1-2*(x*x+y*y)/norm)*p.z)
        scene_position = (top.position.x + rotated[0], top.position.y + rotated[1],
                          top.position.z + rotated[2])
        scene_error = math.dist((ap.x, ap.y, ap.z), scene_position)
        if scene_error > 0.01:
            print(f"FAIL PlanningScene differs from Gazebo by {scene_error:.3f} m", file=sys.stderr)
            return 1
        print(f"PASS contact-only cycle; bilateral pad samples {values['bilateral_pad_samples']}; "
              f"placement {error:.3f} m, tilt {tilt:.3f} rad; "
              f"PlanningScene sync {scene_error:.3f} m")
        return 0
    finally:
        print(f"INFO carry joint samples={values['joint_samples']}, "
              f"max mimic error={values['mimic_error']:.6f} rad, "
              f"pad travel ranges(m)={values['pad_displacement']}", flush=True)
        for subscription in subscriptions:
            node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
