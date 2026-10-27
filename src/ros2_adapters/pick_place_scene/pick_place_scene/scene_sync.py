#!/usr/bin/env python3
"""Confirm observed BOX updates against external MoveIt services.

External attachment writers still require coordination to eliminate the
query/apply race. Local timeouts never imply remote service cancellation.
"""

from dataclasses import dataclass
import math
from rclpy.parameter import Parameter

import rclpy
from geometry_msgs.msg import TransformStamped
from moveit_msgs.msg import CollisionObject, PlanningScene, PlanningSceneComponents
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Bool, UInt64
from tf2_geometry_msgs import do_transform_pose
from pick_place_scene.static_geometry import collision_object, load_geometry
from pick_place_scene.uncertainty import inflate, load_budget


TOLERANCE = 1e-6


def pose_values(pose):
    return (pose.position.x, pose.position.y, pose.position.z,
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)


def near(actual, expected):
    return len(actual) == len(expected) and all(
        math.isfinite(a) and abs(a - b) <= TOLERANCE for a, b in zip(actual, expected))


def identity_pose(obj):
    if not hasattr(obj, "pose"):
        return True
    values = pose_values(obj.pose)
    # The existing Humble producer leaves the top-level pose at its default.
    return near(values, (0.0,) * 7) or (
        near(values[:6], (0.0,) * 6) and abs(abs(values[6]) - 1.0) <= TOLERANCE)


def supported_box(obj, require_identity=True, max_boxes=1):
    if (not 1 <= len(obj.primitives) <= max_boxes or len(obj.primitive_poses) != len(obj.primitives)
            or obj.meshes or obj.mesh_poses or obj.planes or obj.plane_poses
            or obj.subframe_names or obj.subframe_poses
            or (require_identity and not identity_pose(obj))):
        return False
    def valid_pair(shape, pose):
        values = pose_values(pose)
        return (shape.type == SolidPrimitive.BOX and len(shape.dimensions) == 3
            and all(math.isfinite(x) and x > 0 for x in shape.dimensions)
            and all(math.isfinite(x) for x in values)
            and abs(math.hypot(*values[3:]) - 1.0) <= TOLERANCE)
    return all(valid_pair(shape, pose) for shape, pose in zip(obj.primitives, obj.primitive_poses))


def effective_pose(obj, index=0):
    if identity_pose(obj):
        return obj.primitive_poses[index]
    values = pose_values(obj.pose)
    if (not all(math.isfinite(x) for x in values)
            or abs(math.hypot(*values[3:]) - 1.0) > TOLERANCE):
        raise ValueError("invalid object transform")
    transform = TransformStamped()
    transform.transform.translation.x = obj.pose.position.x
    transform.transform.translation.y = obj.pose.position.y
    transform.transform.translation.z = obj.pose.position.z
    transform.transform.rotation = obj.pose.orientation
    return do_transform_pose(obj.primitive_poses[index], transform)


def geometry_matches(actual, expected):
    if (actual.id != expected.id or actual.header.frame_id != expected.header.frame_id
            or not supported_box(actual, require_identity=False, max_boxes=len(expected.primitives))
            or len(actual.primitives) != len(expected.primitives)):
        return False
    for index, (shape, expected_shape) in enumerate(zip(actual.primitives, expected.primitives)):
        if not near(shape.dimensions, expected_shape.dimensions):
            return False
        a, b = pose_values(effective_pose(actual, index)), pose_values(effective_pose(expected, index))
        if not (near(a[:3], b[:3]) and (near(a[3:], b[3:]) or near(a[3:], tuple(-x for x in b[3:])))):
            return False
    return True


@dataclass
class Transaction:
    observation: CollisionObject
    stage: str = "query"
    deadline_ns: int = 0
    future: object = None
    client: object = None
    was_confirmed: bool = False


class SceneSync(Node):
    def __init__(self, **kwargs):
        super().__init__("pick_place_scene_sync", **kwargs)
        self.declare_parameter("observation_topic", "/perception/collision_objects")
        self.declare_parameter("apply_service", "/apply_planning_scene")
        self.declare_parameter("get_scene_service", "/get_planning_scene")
        self.declare_parameter("allowed_ids", ["pick_object"])
        self.declare_parameter("max_observation_age_sec", 1.0)
        self.declare_parameter("service_timeout_sec", 5.0)
        self.declare_parameter("revision_topic", "/perception/scene_revision")
        self.declare_parameter("ready_topic", "/perception/scene_ready")
        self.declare_parameter("static_scene_file", "")
        self.declare_parameter("required_ids", Parameter.Type.STRING_ARRAY)
        self.declare_parameter("input_ready_topic", "")
        self.declare_parameter("input_ready_timeout_sec", 0.0)
        input_timeout = self.get_parameter("input_ready_timeout_sec").value
        if (isinstance(input_timeout, bool) or not isinstance(input_timeout, (int, float))
                or not math.isfinite(input_timeout) or not 0 <= input_timeout <= 86400):
            raise ValueError('input_ready_timeout_sec must be finite and in [0, 86400]')
        if input_timeout and not self.get_parameter('input_ready_topic').value:
            raise ValueError('input heartbeat timeout requires input_ready_topic')
        self.input_timeout_ns = int(input_timeout*1e9)
        self.input_received_ns = None
        self.declare_parameter("uncertainty_config", "")
        budget_file = self.get_parameter("uncertainty_config").value
        self.uncertainty = load_budget(budget_file) if budget_file else None
        if self.uncertainty:
            _, terms, margin, digest = self.uncertainty
            self.get_logger().info(f"Collision uncertainty per-side={margin:.6f} m; terms={terms}; sha256={digest}")
        # Opt-in compound BOX snapshots; single-box default stays compatible.
        self.declare_parameter("max_primitives", 1)
        self.max_primitives = self.get_parameter("max_primitives").value
        if isinstance(self.max_primitives, bool) or not isinstance(self.max_primitives, int) or not 1 <= self.max_primitives <= 4096:
            raise ValueError("max_primitives must be an integer in [1, 4096]")
        self.allowed_ids = set(self.get_parameter("allowed_ids").value)
        static_file = self.get_parameter("static_scene_file").value
        self.static_frame, self.static_boxes = load_geometry(static_file) if static_file else ("", ())
        required = self.get_parameter_or("required_ids", Parameter("required_ids", value=[])).value
        if (not isinstance(required, list) or
                any(not isinstance(value, str) or not value for value in required)):
            raise ValueError("required_ids must be a list of non-empty strings")
        self.required_ids = {box.id for box in self.static_boxes} | set(required)
        self.allowed_ids.update(self.required_ids)
        self.max_age_ns = self.positive_seconds("max_observation_age_sec")
        self.timeout_ns = self.positive_seconds("service_timeout_sec")
        self.transactions = {}
        self.locked_ids = set()
        self.confirmed_ids = set()
        self.scene_revision = 0
        self.scene_ready = False
        self.input_ready = not bool(self.get_parameter("input_ready_topic").value)
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.apply_client = self.create_client(ApplyPlanningScene, self.get_parameter("apply_service").value)
        self.scene_client = self.create_client(GetPlanningScene, self.get_parameter("get_scene_service").value)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.revision_pub = self.create_publisher(
            UInt64, self.get_parameter("revision_topic").value, qos)
        self.ready_pub = self.create_publisher(
            Bool, self.get_parameter("ready_topic").value, qos)
        self.create_subscription(CollisionObject, self.get_parameter("observation_topic").value,
                                 self.on_observation, qos)
        if not self.input_ready:
            self.create_subscription(Bool, self.get_parameter("input_ready_topic").value,
                                     self.on_input_ready, 1 if self.input_timeout_ns else qos)
        self.create_timer(0.02, self.on_watchdog, clock=self.steady_clock)
        if self.static_boxes:
            self.create_timer(0.25, self.load_static_scene, clock=self.steady_clock)
        self.publish_ready()

    def load_static_scene(self):
        # Frozen startup geometry only. Reuse the existing query/apply/confirm
        # transaction; no second writer, periodic rewrites or dynamic protocol.
        if (not self.apply_client.service_is_ready() or not self.scene_client.service_is_ready()
                or self.get_clock().now().nanoseconds <= 0):
            return
        stamp = self.get_clock().now().to_msg()
        for box in self.static_boxes:
            if box.id not in self.confirmed_ids:
                self.on_observation(collision_object(self.static_frame, box, stamp))

    def complete(self):
        return (self.input_ready and bool(self.confirmed_ids) and not self.transactions
                and self.required_ids.issubset(self.confirmed_ids))

    def on_input_ready(self, message):
        self.input_received_ns = self.steady_clock.now().nanoseconds
        self.input_ready = message.data
        self.scene_ready = self.complete()
        self.publish_ready()

    def publish_ready(self):
        ready = Bool()
        ready.data = self.scene_ready
        self.ready_pub.publish(ready)
        revision = UInt64()
        revision.data = self.scene_revision
        self.revision_pub.publish(revision)

    def mark_not_ready(self):
        if self.scene_ready:
            self.scene_ready = False
            self.publish_ready()

    def positive_seconds(self, name):
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value) or value <= 0 or value > 86400:
            raise ValueError(f"{name} must be finite and in (0, 86400] seconds")
        return max(1, int(value * 1e9))

    def fresh(self, obj):
        stamp = obj.header.stamp.sec * 1_000_000_000 + obj.header.stamp.nanosec
        age = self.get_clock().now().nanoseconds - stamp
        return stamp > 0 and 0 <= age <= self.max_age_ns

    def valid(self, obj):
        identity = (obj.id in self.allowed_ids and bool(obj.id) and bool(obj.header.frame_id)
                    and self.fresh(obj))
        if not identity:
            return False
        if obj.operation == CollisionObject.REMOVE:
            return True
        return obj.operation == CollisionObject.ADD and supported_box(obj, max_boxes=self.max_primitives)

    def on_observation(self, obj):
        if obj.id in self.transactions or obj.id in self.locked_ids:
            return
        if not self.valid(obj):
            self.get_logger().warning("Rejected unsupported, invalid, stale or future collision observation")
            return
        obj = inflate(obj, self.uncertainty)
        transaction = Transaction(obj, was_confirmed=obj.id in self.confirmed_ids)
        self.confirmed_ids.discard(obj.id)
        self.mark_not_ready()
        self.transactions[obj.id] = transaction
        query = GetPlanningScene.Request()
        query.components.components = PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
        self.request(transaction, "query", self.scene_client, query)

    def request(self, transaction, stage, client, request):
        transaction.stage = stage
        transaction.client = client
        transaction.future = None
        transaction.deadline_ns = self.steady_clock.now().nanoseconds + self.timeout_ns
        if not client.service_is_ready():
            self.finish(transaction, f"{stage} service unavailable", uncertain=(stage == "confirm"))
            return
        try:
            transaction.future = client.call_async(request)
            transaction.future.add_done_callback(
                lambda future: self.on_response(transaction, stage, future))
        except Exception as error:
            self.finish(transaction, f"{stage} request failed: {error}", uncertain=(stage != "query"))

    def finish(self, transaction, reason, uncertain=False, confirmed=False, restore=False):
        object_id = transaction.observation.id
        if self.transactions.get(object_id) is not transaction:
            return
        self.transactions.pop(object_id)
        if transaction.future is not None:
            # Remove local bookkeeping only. This does not cancel remote work.
            transaction.client.remove_pending_request(transaction.future)
        if confirmed:
            if transaction.observation.operation == CollisionObject.ADD:
                self.confirmed_ids.add(object_id)
            else:
                self.confirmed_ids.discard(object_id)
            self.scene_revision += 1
            self.scene_ready = self.complete()
            self.publish_ready()
            action = "collision object" if transaction.observation.operation == CollisionObject.ADD else "removal"
            self.get_logger().info(f"Confirmed {action} {object_id} in PlanningScene")
        else:
            if restore and transaction.was_confirmed:
                self.confirmed_ids.add(object_id)
                self.scene_ready = self.complete()
                self.publish_ready()
            else:
                self.confirmed_ids.discard(object_id)
                self.mark_not_ready()
            if uncertain:
                self.locked_ids.add(object_id)
                self.get_logger().error(f"{object_id}: {reason}; outcome unknown, locked until restart")
            else:
                self.get_logger().warning(f"{object_id}: {reason}; observation not applied")

    def expire(self, transaction):
        if self.steady_clock.now().nanoseconds < transaction.deadline_ns:
            return False
        self.finish(transaction, f"{transaction.stage} timeout", uncertain=transaction.stage != "query")
        return True

    def on_watchdog(self):
        if (self.input_timeout_ns and self.input_ready and
                (self.input_received_ns is None or
                 self.steady_clock.now().nanoseconds-self.input_received_ns > self.input_timeout_ns)):
            self.input_ready = False
            self.mark_not_ready()
        for transaction in list(self.transactions.values()):
            self.expire(transaction)

    def on_response(self, transaction, stage, future):
        obj = transaction.observation
        if (self.transactions.get(obj.id) is not transaction or transaction.stage != stage
                or self.expire(transaction)):
            return
        try:
            response = future.result()
            if response is None:
                raise RuntimeError("empty response")
            if stage == "query":
                attached = response.scene.robot_state.attached_collision_objects
                if any(item.object.id == obj.id for item in attached):
                    self.finish(transaction, "attached object protected", restore=True)
                elif not self.fresh(obj):
                    self.finish(transaction, "observation expired during query")
                else:
                    request = ApplyPlanningScene.Request()
                    request.scene = PlanningScene(is_diff=True)
                    request.scene.robot_state.is_diff = True
                    request.scene.world.collision_objects = [obj]
                    self.request(transaction, "apply", self.apply_client, request)
            elif stage == "apply":
                if not response.success:
                    self.finish(transaction, "MoveIt rejected update", uncertain=True)
                else:
                    query = GetPlanningScene.Request()
                    query.components.components = PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
                    self.request(transaction, "confirm", self.scene_client, query)
            else:
                matches = [item for item in response.scene.world.collision_objects if item.id == obj.id]
                if obj.operation == CollisionObject.REMOVE and not matches:
                    self.finish(transaction, "confirmed", confirmed=True)
                elif (obj.operation == CollisionObject.ADD and len(matches) == 1
                      and geometry_matches(matches[0], obj)):
                    self.finish(transaction, "confirmed", confirmed=True)
                else:
                    self.finish(transaction, "confirmation missing or geometry differs", uncertain=True)
        except Exception as error:
            self.finish(transaction, f"{stage} response failed: {error}", uncertain=stage != "query")


def main():
    rclpy.init()
    node = SceneSync()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
