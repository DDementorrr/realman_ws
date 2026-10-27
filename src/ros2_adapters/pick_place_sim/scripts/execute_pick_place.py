#!/usr/bin/env python3
"""Execute an MTC solution in Gazebo-synchronized segments, once per trigger."""

import copy
import json
from pathlib import Path
import yaml
import os
import math
import threading
import time

import rclpy
from geometry_msgs.msg import Pose, PoseStamped
from diagnostic_msgs.msg import DiagnosticArray
from rclpy.parameter import Parameter
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
from std_msgs.msg import Bool, String, UInt64
from std_srvs.srv import SetBool, Trigger
from pick_place_runtime import (
    ExecutionCoordinator,
    ExecutionState,
    ProtocolError,
    scene_revision_status,
)

from ros_action_backend import RosActionBackend
from execution_fault_latch import ExecutionFaultLatch
from pick_place_runtime.input_gate import InputGate
from pick_place_mtc import candidate_contract
from rcl_interfaces.srv import GetParameters


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
        self.declare_parameter("world_base_z", 0.15)
        self.declare_parameter("perception_ready_topic", "")
        self.declare_parameter("perception_heartbeat_timeout", 1.0)
        self.declare_parameter("initial_state_ready_topic", "")
        initial_state_topic = self.get_parameter("initial_state_ready_topic").value
        self.initial_state_ready = not bool(initial_state_topic)
        perception_topic = self.get_parameter("perception_ready_topic").value
        self.input_gate = InputGate(bool(perception_topic), self.get_parameter("perception_heartbeat_timeout").value)
        self.declare_parameter(
            "execution_fault_latch_file",
            os.path.expanduser("~/.ros/pick_place_execution_fault.json"))
        self.perception_model_hash = None
        self.candidate_report = None
        self.active_candidate_id = None
        self.joint_feedback_wall = 0.
        self.joint_feedback_ros = 0.
        self.solution = None
        self.description = None
        self.active_solution = None
        self.active_description = None
        self.sim_state = None
        self.gazebo_pose = None
        self.tool_pose = None
        self.left_inner_pose = None
        self.right_inner_pose = None
        self.grasp_relative = None
        self.place_pose = None
        self.pick_pose = None
        self.active_place_pose = None
        self.active_pick_pose = None
        self.joint_positions = {}
        self.contact_samples = 0
        self.bilateral_pad_samples = 0
        self.table_contact_samples = 0
        self.link6_contact_samples = 0
        self.busy = False
        self.fault_latch = None
        try:
            self.fault_latch = ExecutionFaultLatch(
                self.get_parameter("execution_fault_latch_file").value)
            self.execution_fault = self.fault_latch.load()
        except Exception as error:  # noqa: BLE001 - storage errors forbid execution
            self.execution_fault = f"fault latch storage unavailable: {error}"
        self.task_motion_started = False
        self.task_cancel_requested = False
        self.started_automatically = False
        self.scene_revision = None
        self.scene_ready = False
        self.scene_revision_seen = False
        self.candidate_scene_revision = None
        self.lock = threading.RLock()
        self.execution_lock = threading.RLock()
        self.reset_group = ReentrantCallbackGroup()
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        if perception_topic:
            self.create_subscription(DiagnosticArray, "/perception/obstacle_status", self.on_perception_status, qos)
            # Volatile avoids accepting a retained true from a dead producer.
            self.create_subscription(Bool, perception_topic, self.on_perception_ready, 1)
        if initial_state_topic:
            self.create_subscription(Bool, initial_state_topic, self.on_initial_state_ready, qos)
        self.create_subscription(String, "/pick_place/planning_metrics", self.on_metrics, qos)
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
        self.create_subscription(UInt64, "/perception/scene_revision", self.on_scene_revision, qos)
        self.create_subscription(Bool, "/perception/scene_ready", self.on_scene_ready, qos)
        self.create_subscription(UInt64, "/pick_place/candidate_scene_revision",
                                 self.on_candidate_scene_revision, qos)
        self.status_pub = self.create_publisher(String, "/pick_place/execution_status", qos)
        self.success_pub = self.create_publisher(Bool, "/pick_place/execution_success", qos)
        self.action = ActionClient(self, ExecuteTaskSolution, "/execute_task_solution")
        self.execution_backend = RosActionBackend(self.action, self.build_goal)
        self.coordinator = ExecutionCoordinator(
            self.execution_backend, segment_timeout=180.0, cancel_timeout=10.0)
        self.active_candidate_revision = None
        self.sim_client = self.create_client(SetBool, "/pick_place/sim/set_attachment")
        self.sim_reset_client = self.create_client(
            Trigger, "/pick_place/sim/reset_object", callback_group=self.reset_group)
        self.scene_reset_client = self.create_client(
            Trigger, "/known_pick_place_scene/reset_scene", callback_group=self.reset_group)
        self.scene_client = self.create_client(GetPlanningScene, "/get_planning_scene", callback_group=self.reset_group)
        self.binding_clients = {
            node: self.create_client(GetParameters, f"/{node}/get_parameters", callback_group=self.reset_group)
            for node in ("plan_pick_place", "move_group", "depth_obstacles")}
        self.apply_client = self.create_client(ApplyPlanningScene, "/apply_planning_scene")
        self.create_service(Trigger, "/pick_place/execute", self.trigger,
                            callback_group=self.reset_group)
        self.create_service(Trigger, "/pick_place/cancel", self.cancel_execution)
        self.create_service(Trigger, "/pick_place/reset", self.reset,
                            callback_group=self.reset_group)
        self.create_timer(1.0, self.maybe_auto_execute, callback_group=self.reset_group)
        if self.execution_fault is not None:
            self.publish_status(f"execution is fault-latched: {self.execution_fault}")
        else:
            self.publish_status("waiting for planned solution and detached Gazebo object")

    def on_perception_status(self, message):
        try:
            details = {item.key: json.loads(item.value) for item in message.status[0].values}
            digest = details.get('model_sha256')
        except (IndexError, ValueError):
            digest = None
        with self.lock:
            self.perception_model_hash = digest

    def on_metrics(self, message):
        try:
            report = candidate_contract.decode_report(message.data)
        except Exception as error:  # invalid or legacy diagnostics do not authorize motion
            report = None
            self.get_logger().warning(f"Candidate binding rejected: {error}")
        with self.lock:
            self.candidate_report = report

    def on_solution(self, message):
        with self.lock:
            self.solution = message

    def on_candidate_scene_revision(self, message):
        with self.lock:
            self.candidate_scene_revision = int(message.data)

    def on_description(self, message):
        with self.lock:
            self.description = message

    def on_sim_state(self, message):
        with self.lock:
            self.sim_state = message.data

    def on_gazebo_pose(self, message):
        with self.lock:
            self.gazebo_pose = message

    def on_tool_pose(self, message):
        with self.lock:
            self.tool_pose = message

    def on_left_inner_pose(self, message):
        with self.lock:
            self.left_inner_pose = message

    def on_right_inner_pose(self, message):
        with self.lock:
            self.right_inner_pose = message

    def on_contact_state(self, message):
        with self.lock:
            self.contact_samples += 1
            if ("left_pad" in message.data and "collision" in message.data and
                    "right_pad" in message.data):
                self.bilateral_pad_samples += 1
            if "pick_table::top::collision" in message.data:
                self.table_contact_samples += 1
            if "Link6::Link6_collision" in message.data:
                self.link6_contact_samples += 1

    def on_place_pose(self, message):
        with self.lock:
            self.place_pose = message

    def on_pick_pose(self, message):
        with self.lock:
            self.pick_pose = message

    def on_joint_states(self, message):
        with self.lock:
            required = self.candidate_report['binding']['start_positions'] if self.candidate_report else {}
            if (len(message.name) != len(message.position) or len(set(message.name)) != len(message.name)
                    or not set(required).issubset(message.name) or not all(math.isfinite(q) for q in message.position)):
                return
            self.joint_positions.update(zip(message.name, message.position))
            self.joint_feedback_wall = time.monotonic()
            self.joint_feedback_ros = message.header.stamp.sec+message.header.stamp.nanosec*1e-9

    def on_scene_revision(self, message):
        with self.lock:
            self.scene_revision = int(message.data)
            self.scene_revision_seen = True

    def on_scene_ready(self, message):
        with self.lock:
            self.scene_ready = bool(message.data)

    def on_perception_ready(self, message):
        with self.lock:
            self.input_gate.update(message.data)

    def on_initial_state_ready(self, message):
        with self.lock:
            self.initial_state_ready = bool(message.data)

    def publish_status(self, value):
        message = String()
        message.data = value
        self.status_pub.publish(message)
        self.get_logger().info(value)

    def target_pose_in_world(self, target):
        """Convert the fixed base_link scene target into Gazebo's world frame."""
        if target.header.frame_id in ("", "world"):
            return target.pose
        if target.header.frame_id != "base_link":
            raise RuntimeError(
                f"Cannot compare Gazebo world pose with target frame {target.header.frame_id!r}")
        pose = copy.deepcopy(target.pose)
        pose.position.z += float(self.get_parameter("world_base_z").value)
        return pose

    def ready(self):
        with self.lock:
            scene_ok, _ = scene_revision_status(
                self.candidate_scene_revision, self.scene_revision, self.scene_ready,
                self.scene_revision_seen)
            return (all((self.solution, self.description, self.gazebo_pose,
                         self.place_pose, self.pick_pose))
                    and self.sim_state == "detached" and scene_ok and self.initial_state_ready
                    and self.input_gate.status()[0] and self.candidate_report is not None)

    def readiness_detail(self):
        with self.lock:
            if not self.initial_state_ready:
                return 'initial_robot_state_not_confirmed'
            allowed, reason = self.input_gate.status()
            if not allowed:
                return reason
            if self.candidate_report is None:
                return "candidate_binding_missing_or_unsupported"
            if not all((self.solution, self.description, self.gazebo_pose,
                        self.place_pose, self.pick_pose)):
                return "No complete solution, Gazebo pose, place pose, or detached-state ack"
            if self.sim_state != "detached":
                return "Gazebo object is not confirmed detached"
            _, detail = scene_revision_status(
                self.candidate_scene_revision, self.scene_revision, self.scene_ready,
                self.scene_revision_seen)
            return detail

    def trigger(self, _request, response):
        with self.lock:
            if self.execution_fault is not None:
                response.message = (
                    "Execution is fault-latched; supervised recovery is not implemented: "
                    f"{self.execution_fault}")
                return response
            if not self.ready():
                response.message = self.readiness_detail()
                return response
            if self.busy:
                response.message = "Pick and Place execution already running"
                return response
            self.busy = True
            self.task_motion_started = False
            self.task_cancel_requested = False
            report = copy.deepcopy(self.candidate_report)
            solution, description = copy.deepcopy(self.solution), copy.deepcopy(self.description)
            target, place = copy.deepcopy(self.pick_pose), copy.deepcopy(self.place_pose)
        try:
            candidate_contract.verify_payload(report, solution, description)
            candidate_contract.verify_files(report)
            candidate_contract.verify_targets(report, target, place)
            self.verify_candidate_parameters(report)
            request = GetPlanningScene.Request()
            request.components.components = 1023
            scene = wait_future(self.scene_client.call_async(request), 5., "candidate scene binding").scene
            candidate_contract.verify_scene(report, scene)
            with self.lock:
                if self.execution_fault is not None or not self.ready():
                    raise RuntimeError("candidate_inputs_changed_during_preflight")
                if self.joint_feedback_ros-self.get_clock().now().nanoseconds*1e-9 > .05:
                    raise RuntimeError("candidate_start_feedback_from_future")
                candidate_contract.verify_start(report, self.joint_positions,
                                                max(time.monotonic()-self.joint_feedback_wall,
                                                    self.get_clock().now().nanoseconds*1e-9-self.joint_feedback_ros))
                candidate_contract.verify_targets(report, self.pick_pose, self.place_pose)
                if self.candidate_report['selected_id'] != report['selected_id']:
                    raise RuntimeError("candidate_replaced_during_preflight")
                if self.task_cancel_requested:
                    raise RuntimeError("task_cancelled_during_candidate_preflight")
                allowed, reason = self.input_gate.begin()
                if not allowed:
                    raise RuntimeError(reason)
                self.active_solution = solution
                self.active_description = description
                self.active_place_pose = place
                self.active_pick_pose = target
                self.active_candidate_id = report['selected_id']
                self.active_candidate_revision = (
                    self.candidate_scene_revision if self.scene_revision_seen else None)
        except Exception as error:
            with self.lock:
                self.busy = False
            response.message = str(error)
            self.publish_status(f"Candidate execution refused: {error}")
            return response
        self.publish_status(f"Executing archived candidate {self.active_candidate_id}")
        threading.Thread(target=self.run_task, daemon=True).start()
        response.success = True
        response.message = "Pick and Place execution started"
        return response

    def verify_candidate_parameters(self, report):
        expected = candidate_contract.expected_parameters(report, 'planner_parameters')
        model = {name: value for name,value in expected.items()
                 if name in ('robot_description', 'robot_description_semantic')
                 or name.startswith(('robot_description_planning.', 'robot_description_kinematics.'))}
        sources = [('plan_pick_place', expected), ('move_group', model)]
        perception = None
        for path in report['binding'].get('files', {}):
            if Path(path).suffix not in ('.yaml', '.yml'):
                continue
            content = yaml.safe_load(Path(path).read_text())
            if isinstance(content, dict) and 'depth_obstacles' in content:
                candidate = content['depth_obstacles']['ros__parameters']
                if perception is None or candidate.get('additional_perception_configs'):
                    perception = candidate
        if self.input_gate.enabled:
            if perception is None or not perception.get('monitor_fixed_calibration', False):
                raise RuntimeError('candidate_fixed_calibration_binding_missing')
            sources.append(('depth_obstacles', {
                name: Parameter(name, value=value).to_parameter_msg().value for name,value in perception.items()}))
            from pick_place_perception.self_filter import load_model
            if load_model(perception['self_filter_model'])[2] != self.perception_model_hash:
                raise RuntimeError('candidate_perception_collision_model_changed')
        for node, parameters in sources:
            client = self.binding_clients[node]
            if not client.wait_for_service(timeout_sec=2.):
                raise RuntimeError('candidate_parameter_source_unavailable:'+node)
            request = GetParameters.Request(names=list(parameters))
            actual = wait_future(client.call_async(request), 5., 'candidate '+node+' parameters').values
            if len(actual) != len(parameters) or any(a != e for a,e in zip(actual, parameters.values())):
                raise RuntimeError('candidate_model_or_planning_parameters_changed:'+node)

    def reset(self, _request, response):
        with self.lock:
            if self.execution_fault is not None:
                response.message = (
                    "Cannot reset while execution outcome is fault-latched; "
                    "supervised recovery is not implemented")
                return response
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

    def cancel_execution(self, _request, response):
        with self.lock:
            if not self.busy:
                response.message = "No Pick and Place task is active"
                return response
            self.task_cancel_requested = True
        with self.execution_lock:
            event = self.coordinator.cancel()
        if event.state in (ExecutionState.STOPPING, ExecutionState.SUCCEEDED,
                           ExecutionState.FAILED, ExecutionState.IDLE):
            response.success = True
            if event.state is ExecutionState.STOPPING:
                response.message = "Cancellation requested; waiting for terminal CANCELED result"
            else:
                response.message = "Task cancellation queued; no new motion segment will start"
        elif event.state is ExecutionState.BLOCKED:
            response.message = "Execution is fault-latched BLOCKED; stop is unconfirmed"
        else:
            response.message = "No active motion segment can be canceled"
        return response

    def maybe_auto_execute(self):
        if not self.get_parameter("auto_execute").value or self.started_automatically or not self.ready():
            return
        self.started_automatically = True
        self.trigger(None, Trigger.Response())

    def stage_segments(self):
        names = {stage.id: stage.name for stage in self.active_description.stages}
        trajectory = self.active_solution.sub_trajectory
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
        pick_pose = self.active_pick_pose if self.busy and self.active_pick_pose else self.pick_pose
        target = self.target_pose_in_world(pick_pose)
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
        names = {stage.id: stage.name for stage in self.active_description.stages}
        named = [names.get(part.info.stage_id, "") for part in parts]
        transport = named.index("Transport (OMPL)")
        open_gripper = named.index("Open Gripper")
        return parts[:transport], parts[transport:open_gripper], parts[open_gripper:]

    def physical_travel_segments(self, parts):
        names = {stage.id: stage.name for stage in self.active_description.stages}
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
                            (self.active_pick_pose or self.pick_pose).pose.orientation)
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
        names = {stage.id: stage.name for stage in self.active_description.stages}
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
        if self.busy and self.active_place_pose is not None:
            target = self.active_place_pose
        actual = self.gazebo_pose.pose
        goal = self.target_pose_in_world(target)
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

    def build_goal(self, segment):
        label, parts = segment
        goal = ExecuteTaskSolution.Goal()
        solution = self.active_solution if self.active_solution is not None else self.solution
        goal.solution = copy.deepcopy(solution)
        goal.solution.sub_trajectory = parts
        if label == "physical gripper release":
            # Contact can elastically spread the fingers after the planned
            # close. Start opening from measured feedback, never close again.
            joint = "ctm2f110_finger_joint"
            with self.lock:
                current = self.joint_positions.get(joint)
            if current is None or len(parts) != 1:
                raise RuntimeError("No current finger feedback for physical release")
            goal.solution.sub_trajectory = copy.deepcopy(parts)
            trajectory = goal.solution.sub_trajectory[0].trajectory.joint_trajectory
            if joint not in trajectory.joint_names or not trajectory.points:
                raise RuntimeError("Physical release trajectory lacks finger waypoints")
            index = trajectory.joint_names.index(joint)
            planned = trajectory.points[0].positions[index]
            # Contact loading can spread the fingers by several tenths of a
            # radian before release; reject only a deviation inconsistent
            # with the measured grasp state.
            if not 0.0 <= current - planned <= 0.50:
                raise RuntimeError(f"Finger deviated too far from release start: {current - planned:.3f} rad")
            for waypoint in trajectory.points:
                if waypoint.positions[index] < current:
                    waypoint.positions[index] = current
                    if waypoint.velocities:
                        waypoint.velocities[index] = 0.0
                    if waypoint.accelerations:
                        waypoint.accelerations[index] = 0.0
            self.publish_status(f"physical release starts at measured finger position {current:.3f} rad")
        return goal

    def ensure_task_not_cancelled(self):
        with self.lock:
            allowed, reason = self.input_gate.status()
            if not allowed:
                raise RuntimeError(reason)
            if self.task_cancel_requested:
                raise RuntimeError("Pick and Place task cancellation requested")

    def latch_execution_fault(self, reason):
        with self.lock:
            self.execution_fault = reason
        if self.fault_latch is None:
            raise RuntimeError("execution fault latch storage is unavailable")
        self.fault_latch.latch(reason)

    def execute_segment(self, label, parts):
        self.ensure_task_not_cancelled()
        self.publish_status(f"executing {label} ({len(parts)} MTC stages)")
        with self.execution_lock:
            self.ensure_task_not_cancelled()
            if self.execution_fault is not None:
                raise ProtocolError(f"execution is fault-latched: {self.execution_fault}")
            with self.lock:
                revision_seen = self.scene_revision_seen
                current_revision = self.scene_revision if revision_seen else None
                scene_ready = self.scene_ready if revision_seen else True
                candidate_revision = self.active_candidate_revision

            def pre_dispatch_check():
                with self.lock:
                    if self.task_cancel_requested:
                        return "task cancellation requested before action dispatch"
                    allowed, reason = self.input_gate.status()
                    if not allowed:
                        return reason
                    current_seen = self.scene_revision_seen
                    current = self.scene_revision if current_seen else None
                    ready = self.scene_ready if current_seen else True
                    candidate = self.active_candidate_revision
                allowed, detail = scene_revision_status(
                    candidate, current, ready, current_seen)
                return None if allowed else detail

            event = self.coordinator.start(
                [(label, parts)],
                candidate_revision=candidate_revision,
                current_revision=current_revision,
                scene_ready=scene_ready,
                allow_unbound_scene=not revision_seen,
                pre_dispatch_check=pre_dispatch_check,
            )
            if event.state is ExecutionState.RUNNING or event.code == "start_failed":
                # Once an action goal may have left the process, later task
                # failure requires explicit external state inspection.
                self.task_motion_started = True
        if event.state is not ExecutionState.RUNNING:
            raise RuntimeError(f"{label} was not started: {event.code}: {event.detail}")

        last_event = event
        invalid_scene_reported = False
        while rclpy.ok():
            with self.execution_lock:
                if self.coordinator.state is ExecutionState.RUNNING:
                    with self.lock:
                        revision_seen = self.scene_revision_seen
                        scene_revision = self.scene_revision
                        scene_ready = self.scene_ready
                        candidate_revision = self.active_candidate_revision
                        input_ok, input_detail = self.input_gate.status()
                    scene_ok, scene_detail = scene_revision_status(
                        candidate_revision, scene_revision, scene_ready, revision_seen)
                    if (not scene_ok or not input_ok) and not invalid_scene_reported:
                        self.publish_status(
                            f"{label}: scene invalidated during execution ({scene_detail if not scene_ok else input_detail}); "
                            "requesting action cancellation")
                        event = self.coordinator.cancel()
                        invalid_scene_reported = True
                    else:
                        event = self.coordinator.tick()
                else:
                    event = self.coordinator.tick()
            if event.state is ExecutionState.SUCCEEDED:
                self.publish_status(f"{label} action completed and acknowledged")
                return
            if event.state in (ExecutionState.FAILED, ExecutionState.BLOCKED):
                raise RuntimeError(f"{label} ended {event.state.value}: {event.code}: {event.detail}")
            if event is not last_event and event.code != last_event.code:
                self.publish_status(f"{label} execution {event.state.value}: {event.code}: {event.detail}")
                last_event = event
            time.sleep(0.05)
        raise RuntimeError(f"ROS shutdown while {label} action outcome is unconfirmed")

    def set_sim_attachment(self, attached):
        self.ensure_task_not_cancelled()
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
        target = self.active_place_pose if self.active_place_pose is not None else self.place_pose
        if actual is None or target is None:
            raise RuntimeError("Missing Gazebo or place pose after execution")
        target_world = self.target_pose_in_world(target)
        ap, tp = actual.pose.position, target_world.position
        error = math.sqrt((ap.x - tp.x) ** 2 + (ap.y - tp.y) ** 2 + (ap.z - tp.z) ** 2)
        if error > 0.04:
            raise RuntimeError(f"Gazebo object missed place target by {error:.3f} m")
        aq, tq = actual.pose.orientation, target_world.orientation
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
            self.execute_segment("Pre-grasp and approach", approach)
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
                pick_world = self.target_pose_in_world(self.active_pick_pose)
                if self.gazebo_pose.pose.position.z < pick_world.position.z + 0.015:
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
            self.ensure_task_not_cancelled()
            success.data = True
            self.publish_status(f"Archived candidate completed: {self.active_candidate_id}")
            self.publish_status("Pick and Place completed")
        except Exception as error:  # noqa: BLE001
            with self.execution_lock:
                if self.coordinator.state is ExecutionState.RUNNING:
                    try:
                        self.coordinator.cancel()
                    except Exception:  # noqa: BLE001
                        pass
                if (self.task_motion_started or self.coordinator.state in (
                        ExecutionState.RUNNING, ExecutionState.STOPPING,
                        ExecutionState.BLOCKED)):
                    try:
                        self.latch_execution_fault(str(error))
                    except Exception as latch_error:  # noqa: BLE001
                        self.publish_status(
                            f"CRITICAL: cannot persist execution fault latch: {latch_error}")
            self.publish_status(f"Pick and Place failed: {error}")
            if self.execution_fault is not None:
                self.publish_status(
                    "Execution fault is latched; reset and further execution are disabled")
        finally:
            self.success_pub.publish(success)
            with self.lock:
                self.busy = False
                self.active_solution = None
                self.active_description = None
                self.active_place_pose = None
                self.active_pick_pose = None
                self.active_candidate_revision = None
                self.active_candidate_id = None
                self.input_gate.end()


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
