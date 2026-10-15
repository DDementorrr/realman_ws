#!/usr/bin/env python3
"""Execute an MTC solution in Gazebo-synchronized segments, once per trigger."""

import copy
import math
import threading
import time

import rclpy
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import CollisionObject, MoveItErrorCodes, PlanningScene, PlanningSceneComponents
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from moveit_task_constructor_msgs.action import ExecuteTaskSolution
from moveit_task_constructor_msgs.msg import Solution, TaskDescription
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger


def wait_future(future, timeout, label):
    deadline = time.monotonic() + timeout
    while rclpy.ok() and not future.done() and time.monotonic() < deadline:
        time.sleep(0.05)
    if not future.done():
        raise RuntimeError(f"{label} timed out after {timeout:.0f} s")
    result = future.result()
    if result is None:
        raise RuntimeError(f"{label} returned no result")
    return result


def quaternion_product(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def quaternion_tuple(value):
    return (value.x, value.y, value.z, value.w)


def upright_tilt(actual, target):
    """Angle between the two local Z axes, independent of yaw about Z."""
    def z_axis(quaternion):
        x, y, z, w = quaternion_tuple(quaternion)
        norm = x * x + y * y + z * z + w * w
        if norm <= 0.0:
            raise RuntimeError("Invalid zero-norm object orientation")
        return (2 * (x * z + w * y) / norm,
                2 * (y * z - w * x) / norm,
                1 - 2 * (x * x + y * y) / norm)

    a, b = z_axis(actual), z_axis(target)
    return math.acos(max(-1.0, min(1.0, sum(x * y for x, y in zip(a, b)))))


class PickPlaceExecution(Node):
    def __init__(self):
        super().__init__("pick_place_execution")
        self.declare_parameter("auto_execute", False)
        self.declare_parameter("physical_contact", False)
        self.solution = None
        self.description = None
        self.sim_state = None
        self.gazebo_pose = None
        self.tool_pose = None
        self.left_inner_pose = None
        self.right_inner_pose = None
        self.grasp_relative = None
        self.place_pose = None
        self.pick_pose = None
        self.joint_positions = {}
        self.contact_samples = 0
        self.bilateral_pad_samples = 0
        self.table_contact_samples = 0
        self.link6_contact_samples = 0
        self.busy = False
        self.started_automatically = False
        self.lock = threading.Lock()
        self.reset_group = ReentrantCallbackGroup()
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Solution, "/pick_place/solution", self.on_solution, qos)
        self.create_subscription(TaskDescription, "/pick_place/task_description", self.on_description, qos)
        self.create_subscription(String, "/pick_place/sim/attachment_state", self.on_sim_state, qos)
        self.create_subscription(PoseStamped, "/pick_place/sim/object_pose", self.on_gazebo_pose, 10)
        self.create_subscription(PoseStamped, "/pick_place/sim/tool_pose", self.on_tool_pose, 10)
        self.create_subscription(PoseStamped, "/pick_place/sim/left_inner_pose", self.on_left_inner_pose, 10)
        self.create_subscription(PoseStamped, "/pick_place/sim/right_inner_pose", self.on_right_inner_pose, 10)
        self.create_subscription(String, "/pick_place/sim/contact_state", self.on_contact_state, 10)
        self.create_subscription(PoseStamped, "/place_pose", self.on_place_pose, 10)
        self.create_subscription(PoseStamped, "/object_pose", self.on_pick_pose, 10)
        self.create_subscription(JointState, "/joint_states", self.on_joint_states, 10)
        self.status_pub = self.create_publisher(String, "/pick_place/execution_status", qos)
        self.success_pub = self.create_publisher(Bool, "/pick_place/execution_success", qos)
        self.action = ActionClient(self, ExecuteTaskSolution, "/execute_task_solution")
        self.sim_client = self.create_client(SetBool, "/pick_place/sim/set_attachment")
        self.sim_reset_client = self.create_client(
            Trigger, "/pick_place/sim/reset_object", callback_group=self.reset_group)
        self.scene_reset_client = self.create_client(
            Trigger, "/known_pick_place_scene/reset_scene", callback_group=self.reset_group)
        self.scene_client = self.create_client(GetPlanningScene, "/get_planning_scene")
        self.apply_client = self.create_client(ApplyPlanningScene, "/apply_planning_scene")
        self.create_service(Trigger, "/pick_place/execute", self.trigger)
        self.create_service(Trigger, "/pick_place/reset", self.reset,
                            callback_group=self.reset_group)
        self.create_timer(1.0, self.maybe_auto_execute)
        self.publish_status("waiting for planned solution and detached Gazebo object")

    def on_solution(self, message):
        self.solution = message

    def on_description(self, message):
        self.description = message

    def on_sim_state(self, message):
        self.sim_state = message.data

    def on_gazebo_pose(self, message):
        self.gazebo_pose = message

    def on_tool_pose(self, message):
        self.tool_pose = message

    def on_left_inner_pose(self, message):
        self.left_inner_pose = message

    def on_right_inner_pose(self, message):
        self.right_inner_pose = message

    def on_contact_state(self, message):
        self.contact_samples += 1
        if "left_pad_collision" in message.data and "right_pad_collision" in message.data:
            self.bilateral_pad_samples += 1
        if "pick_table::top::collision" in message.data:
            self.table_contact_samples += 1
        if "Link6::Link6_collision" in message.data:
            self.link6_contact_samples += 1

    def on_place_pose(self, message):
        self.place_pose = message

    def on_pick_pose(self, message):
        self.pick_pose = message

    def on_joint_states(self, message):
        self.joint_positions.update(zip(message.name, message.position))

    def publish_status(self, value):
        message = String()
        message.data = value
        self.status_pub.publish(message)
        self.get_logger().info(value)

    def ready(self):
        return all((self.solution, self.description, self.gazebo_pose,
                    self.place_pose, self.pick_pose)) and self.sim_state == "detached"

    def trigger(self, _request, response):
        if not self.ready():
            response.message = "No complete solution, Gazebo pose, place pose, or detached-state ack"
            return response
        with self.lock:
            if self.busy:
                response.message = "Pick and Place execution already running"
                return response
            self.busy = True
        threading.Thread(target=self.run_task, daemon=True).start()
        response.success = True
        response.message = "Pick and Place execution started"
        return response

    def reset(self, _request, response):
        with self.lock:
            if self.busy:
                response.message = "Cannot reset while Pick and Place execution is running"
                return response
            self.busy = True
        try:
            for label, client in (("Gazebo object", self.sim_reset_client),
                                  ("MoveIt known scene", self.scene_reset_client)):
                if not client.wait_for_service(timeout_sec=5):
                    raise RuntimeError(f"{label} reset service unavailable")
                result = wait_future(client.call_async(Trigger.Request()), 10, label)
                if not result.success:
                    raise RuntimeError(f"{label} reset failed: {result.message}")
            time.sleep(0.5)
            self.verify_pick_object("after reset")
            self.publish_status("Gazebo object and MoveIt scene reset for another cycle")
            response.success = True
            response.message = "Pick and Place scene reset"
        except Exception as error:  # noqa: BLE001
            response.message = str(error)
            self.publish_status(f"Pick and Place reset failed: {error}")
        finally:
            with self.lock:
                self.busy = False
        return response

    def maybe_auto_execute(self):
        if not self.get_parameter("auto_execute").value or self.started_automatically or not self.ready():
            return
        self.started_automatically = True
        self.trigger(None, Trigger.Response())

    def stage_segments(self):
        names = {stage.id: stage.name for stage in self.description.stages}
        trajectory = self.solution.sub_trajectory
        named = [names.get(part.info.stage_id, "") for part in trajectory]
        for stage in ("Close Gripper", "Attach in PlanningScene", "Detach in PlanningScene"):
            if named.count(stage) != 1:
                raise RuntimeError(f"Solution lacks unique {stage} boundary")
        close = named.index("Close Gripper")
        attach = named.index("Attach in PlanningScene")
        detach = named.index("Detach in PlanningScene")
        if not 0 < close < attach < detach < len(trajectory) - 1:
            raise RuntimeError("Invalid close/attach/detach ordering in MTC solution")
        return trajectory[:close], trajectory[close:attach], trajectory[attach:detach], trajectory[detach:]

    def verify_pick_object(self, label):
        actual = self.gazebo_pose.pose
        target = self.pick_pose.pose
        ap, tp = actual.position, target.position
        distance = math.sqrt((ap.x - tp.x) ** 2 + (ap.y - tp.y) ** 2 + (ap.z - tp.z) ** 2)
        aq, tq = actual.orientation, target.orientation
        dot = abs(aq.x * tq.x + aq.y * tq.y + aq.z * tq.z + aq.w * tq.w)
        angle = 2 * math.acos(min(1.0, dot))
        self.publish_status(f"{label}: Gazebo object offset {distance:.3f} m, tilt {angle:.3f} rad")
        angle_limit = 0.5 if self.get_parameter("physical_contact").value else 0.20
        if distance > 0.02 or angle > angle_limit:
            raise RuntimeError(f"Object displaced before attachment during {label}")

    def physical_carry_segments(self, parts):
        names = {stage.id: stage.name for stage in self.description.stages}
        named = [names.get(part.info.stage_id, "") for part in parts]
        transport = named.index("Transport (OMPL)")
        open_gripper = named.index("Open Gripper")
        return parts[:transport], parts[transport:open_gripper], parts[open_gripper:]

    def physical_travel_segments(self, parts):
        names = {stage.id: stage.name for stage in self.description.stages}
        named = [names.get(part.info.stage_id, "") for part in parts]
        lower = named.index("Place (Cartesian)")
        return parts[:lower], parts[lower:]

    def relative_grasp_pose(self):
        if self.tool_pose is None or self.gazebo_pose is None:
            raise RuntimeError("Missing Gazebo Link6 or object pose for contact check")
        tool, obj = self.tool_pose.pose, self.gazebo_pose.pose
        q = quaternion_tuple(tool.orientation)
        inverse = (-q[0], -q[1], -q[2], q[3])
        translation = (obj.position.x - tool.position.x,
                       obj.position.y - tool.position.y,
                       obj.position.z - tool.position.z, 0.0)
        relative_position = quaternion_product(quaternion_product(inverse, translation), q)[:3]
        relative_orientation = quaternion_product(inverse, quaternion_tuple(obj.orientation))
        return relative_position, relative_orientation

    def verify_grip(self, label):
        position, orientation = self.relative_grasp_pose()
        if self.grasp_relative is None:
            self.grasp_relative = position, orientation
            self.publish_status(f"{label}: Gazebo contact frame recorded")
            return
        initial_position, initial_orientation = self.grasp_relative
        offset = math.dist(position, initial_position)
        dot = abs(sum(a * b for a, b in zip(orientation, initial_orientation)))
        angle = 2 * math.acos(min(1.0, dot))
        tilt = upright_tilt(self.gazebo_pose.pose.orientation,
                            self.pick_pose.pose.orientation)
        self.publish_status(
            f"{label}: object-to-tool drift {offset:.3f} m, {angle:.3f} rad; "
            f"upright tilt {tilt:.3f} rad; object z {self.gazebo_pose.pose.position.z:.3f} m")
        # The circular place zone does not constrain yaw; bilateral pads may
        # allow the box to rotate about its upright axis without losing grip.
        if offset > 0.04 or tilt > 0.45:
            raise RuntimeError(f"Object slipped in the gripper during {label}")

    def report_pad_geometry(self, label):
        if not all((self.gazebo_pose, self.left_inner_pose, self.right_inner_pose)):
            self.publish_status(f"{label}: pad pose feedback unavailable")
            return
        object_position = self.gazebo_pose.pose.position
        readings = []
        for name, message in (("left", self.left_inner_pose), ("right", self.right_inner_pose)):
            pose = message.pose
            quaternion = quaternion_tuple(pose.orientation)
            conjugate = (-quaternion[0], -quaternion[1], -quaternion[2], quaternion[3])
            # URDF pad_joint origin in the inner finger link frame.
            pad_travel = self.joint_positions.get(f'ctm2f110_{name}_pad_joint', 0.0)
            pad_offset = quaternion_product(
                quaternion_product(quaternion, (-0.04597 + pad_travel, 0.041903, 0.0, 0.0)), conjugate)
            normal = quaternion_product(
                quaternion_product(quaternion, (1.0, 0.0, 0.0, 0.0)), conjugate)
            readings.append(
                f"{name} delta=({pose.position.x + pad_offset[0] - object_position.x:+.3f},"
                f"{pose.position.y + pad_offset[1] - object_position.y:+.3f},"
                f"{pose.position.z + pad_offset[2] - object_position.z:+.3f}) m, "
                f"normal=({normal[0]:+.2f},{normal[1]:+.2f},{normal[2]:+.2f})")
        finger = self.joint_positions.get("ctm2f110_finger_joint", float("nan"))
        self.publish_status(f"{label}: finger={finger:.3f} rad; " + "; ".join(readings))

    def physical_finish_segments(self, parts):
        names = {stage.id: stage.name for stage in self.description.stages}
        named = [names.get(part.info.stage_id, "") for part in parts]
        retreat = named.index("Retreat (Cartesian)")
        home = named.index("return Home")
        return parts[:retreat], parts[retreat:home], parts[home:]

    def report_start_error(self, label, part):
        trajectory = part.trajectory.joint_trajectory
        if not trajectory.points:
            return
        errors = [(abs(self.joint_positions[name] - planned), name)
                  for name, planned in zip(trajectory.joint_names, trajectory.points[0].positions)
                  if name in self.joint_positions]
        if errors:
            error, joint = max(errors)
            self.publish_status(f"{label}: measured trajectory start error {error:.4f} rad at {joint}")

    def verify_physical_object(self, label, target, tolerance):
        actual = self.gazebo_pose.pose
        goal = target.pose
        ap, gp = actual.position, goal.position
        distance = math.dist((ap.x, ap.y, ap.z), (gp.x, gp.y, gp.z))
        aq, gq = actual.orientation, goal.orientation
        dot = abs(aq.x * gq.x + aq.y * gq.y + aq.z * gq.z + aq.w * gq.w)
        orientation_error = 2 * math.acos(min(1.0, dot))
        tilt = upright_tilt(aq, gq)
        self.publish_status(
            f"{label}: Gazebo object offset {distance:.3f} m, "
            f"upright tilt {tilt:.3f} rad, orientation error {orientation_error:.3f} rad")
        if distance > tolerance or tilt > 0.5:
            raise RuntimeError(f"Physical object did not follow the gripper during {label}")

    def execute_segment(self, label, parts):
        if not self.action.wait_for_server(timeout_sec=10):
            raise RuntimeError("MoveIt ExecuteTaskSolution action is unavailable")
        goal = ExecuteTaskSolution.Goal()
        goal.solution = copy.deepcopy(self.solution)
        goal.solution.sub_trajectory = parts
        if label == "physical gripper release":
            # Contact can elastically spread the fingers after the planned
            # close. Start opening from measured feedback, never close again.
            joint = "ctm2f110_finger_joint"
            current = self.joint_positions.get(joint)
            if current is None or len(parts) != 1:
                raise RuntimeError("No current finger feedback for physical release")
            goal.solution.sub_trajectory = copy.deepcopy(parts)
            trajectory = goal.solution.sub_trajectory[0].trajectory.joint_trajectory
            if joint not in trajectory.joint_names or not trajectory.points:
                raise RuntimeError("Physical release trajectory lacks finger waypoints")
            index = trajectory.joint_names.index(joint)
            planned = trajectory.points[0].positions[index]
            if not 0.0 <= current - planned <= 0.25:
                raise RuntimeError(f"Finger deviated too far from release start: {current - planned:.3f} rad")
            for waypoint in trajectory.points:
                if waypoint.positions[index] < current:
                    waypoint.positions[index] = current
                    if waypoint.velocities:
                        waypoint.velocities[index] = 0.0
                    if waypoint.accelerations:
                        waypoint.accelerations[index] = 0.0
            self.publish_status(f"physical release starts at measured finger position {current:.3f} rad")
        self.publish_status(f"executing {label} ({len(parts)} MTC stages)")
        goal_handle = wait_future(self.action.send_goal_async(goal), 10, f"{label} goal")
        if not goal_handle.accepted:
            raise RuntimeError(f"{label} action goal rejected")
        result = wait_future(goal_handle.get_result_async(), 180, f"{label} execution")
        if result.result.error_code.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(f"{label} execution failed with MoveIt code {result.result.error_code.val}")

    def set_sim_attachment(self, attached):
        if not self.sim_client.wait_for_service(timeout_sec=5):
            raise RuntimeError("Gazebo attachment adapter service is unavailable")
        request = SetBool.Request()
        request.data = attached
        response = wait_future(self.sim_client.call_async(request), 8, "Gazebo attachment")
        if not response.success:
            raise RuntimeError(response.message)
        self.publish_status(response.message)

    def verify_and_sync_placement(self):
        time.sleep(1.0)  # let the detached object settle on the simulated table
        actual = self.gazebo_pose
        target = self.place_pose
        if actual is None or target is None:
            raise RuntimeError("Missing Gazebo or place pose after execution")
        ap, tp = actual.pose.position, target.pose.position
        error = math.sqrt((ap.x - tp.x) ** 2 + (ap.y - tp.y) ** 2 + (ap.z - tp.z) ** 2)
        if error > 0.04:
            raise RuntimeError(f"Gazebo object missed place target by {error:.3f} m")
        aq, tq = actual.pose.orientation, target.pose.orientation
        dot = abs(aq.x * tq.x + aq.y * tq.y + aq.z * tq.z + aq.w * tq.w)
        angle = 2 * math.acos(min(1.0, dot))
        physical = self.get_parameter("physical_contact").value
        tilt = upright_tilt(aq, tq) if physical else angle
        if physical and tilt > 0.35:
            raise RuntimeError(f"Physical object tilted {tilt:.3f} rad after placement")
        if not self.scene_client.wait_for_service(timeout_sec=5):
            raise RuntimeError("MoveIt PlanningScene query unavailable")
        request = GetPlanningScene.Request()
        request.components.components = (
            PlanningSceneComponents.WORLD_OBJECT_GEOMETRY |
            PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
        )
        response = wait_future(self.scene_client.call_async(request), 10, "PlanningScene query")
        if any(obj.object.id == "pick_object" for obj in response.scene.robot_state.attached_collision_objects):
            raise RuntimeError("Object remains attached in MoveIt PlanningScene after detach")
        objects = [obj for obj in response.scene.world.collision_objects if obj.id == "pick_object"]
        if len(objects) != 1 or len(objects[0].primitive_poses) != 1:
            raise RuntimeError("World object geometry missing after placement")
        obj = copy.deepcopy(objects[0])
        obj.header.frame_id = "world"
        # After detach MoveIt can store the world transform in obj.pose.
        # Reset it before assigning an absolute primitive pose, otherwise
        # ADD composes the old world transform twice.
        obj.pose = Pose()
        obj.pose.orientation.w = 1.0
        obj.primitive_poses[0] = actual.pose
        obj.operation = CollisionObject.ADD
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = [obj]
        update = ApplyPlanningScene.Request()
        update.scene = scene
        if not self.apply_client.wait_for_service(timeout_sec=5):
            raise RuntimeError("MoveIt PlanningScene update unavailable")
        applied = wait_future(self.apply_client.call_async(update), 10, "PlanningScene placement sync")
        if not applied.success:
            raise RuntimeError("MoveIt rejected Gazebo placement pose")
        self.publish_status(
            f"placed and synchronized with Gazebo ({error:.3f} m, {tilt:.3f} rad upright tilt)")

    def run_task(self):
        success = Bool()
        try:
            approach, close, carrying, after = self.stage_segments()
            self.verify_pick_object("before approach")
            self.execute_segment("Home, pre-grasp, and approach", approach)
            if self.get_parameter("physical_contact").value:
                self.report_pad_geometry("after physical approach")
            self.verify_pick_object("after approach")
            contact_start = self.bilateral_pad_samples
            self.execute_segment("close gripper", close)
            self.verify_pick_object("after gripper close")
            if self.get_parameter("physical_contact").value:
                self.report_pad_geometry("after physical close")
                self.publish_status(
                    f"after physical close: bilateral pad contact samples "
                    f"{self.bilateral_pad_samples - contact_start}")
            if not self.get_parameter("physical_contact").value:
                self.set_sim_attachment(True)
                self.execute_segment("lift, transport, and place", carrying)
                self.set_sim_attachment(False)
            else:
                self.publish_status("physical contact mode: no Gazebo attachment commanded")
                self.verify_grip("after physical close")
                lift, travel, release = self.physical_carry_segments(carrying)
                self.execute_segment("physical lift", lift)
                self.publish_status(
                    f"after physical lift: bilateral pad contact samples "
                    f"{self.bilateral_pad_samples - contact_start}")
                self.publish_status(
                    "after physical lift: finger "
                    f"{self.joint_positions.get('ctm2f110_finger_joint', float('nan')):.3f} rad, "
                    f"object z {self.gazebo_pose.pose.position.z:.3f} m")
                if self.gazebo_pose.pose.position.z < self.pick_pose.pose.position.z + 0.015:
                    raise RuntimeError("Physical object was not lifted from the table")
                self.verify_grip("after physical lift")
                transport, place = self.physical_travel_segments(travel)
                self.execute_segment("physical transport", transport)
                self.verify_grip("after physical transport")
                if len(place) != 2:
                    raise RuntimeError(f"Expected Cartesian lower and place alignment, got {len(place)} stages")
                table_contact_start = self.table_contact_samples
                link6_contact_start = self.link6_contact_samples
                self.report_start_error("physical lower", place[0])
                self.execute_segment("physical lower", place[:1])
                self.publish_status(
                    f"during physical lower: table contact samples "
                    f"{self.table_contact_samples - table_contact_start}, "
                    f"Link6 contact samples {self.link6_contact_samples - link6_contact_start}")
                self.verify_grip("after physical lower")
                self.execute_segment("physical place alignment", place[1:])
                self.verify_grip("before physical release")
                self.verify_physical_object("before physical release", self.place_pose, 0.06)
                self.execute_segment("physical gripper release", release)
                time.sleep(1.0)
                self.verify_physical_object("after physical release", self.place_pose, 0.06)
            if self.get_parameter("physical_contact").value:
                detach, retreat, home = self.physical_finish_segments(after)
                self.execute_segment("PlanningScene detach", detach)
                self.execute_segment("physical retreat", retreat)
                time.sleep(0.5)
                self.verify_physical_object("after physical retreat", self.place_pose, 0.06)
                self.execute_segment("return Home", home)
            else:
                self.execute_segment("detach, retreat, and Home", after)
            self.verify_and_sync_placement()
            success.data = True
            self.publish_status("Pick and Place completed")
        except Exception as error:  # noqa: BLE001
            self.publish_status(f"Pick and Place failed: {error}")
        finally:
            self.success_pub.publish(success)
            with self.lock:
                self.busy = False


def main():
    rclpy.init()
    node = PickPlaceExecution()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
