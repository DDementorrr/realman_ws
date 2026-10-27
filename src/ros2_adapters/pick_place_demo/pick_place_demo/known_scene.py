#!/usr/bin/env python3
import math

import rclpy
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger
from std_msgs.msg import Bool
from shape_msgs.msg import SolidPrimitive


def pose_from_xyz_rpy(values):
    x, y, z, roll, pitch, yaw = values
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = x, y, z
    pose.orientation.w = cr * cp * cy + sr * sp * sy
    pose.orientation.x = sr * cp * cy - cr * sp * sy
    pose.orientation.y = cr * sp * cy + sr * cp * sy
    pose.orientation.z = cr * cp * sy - sr * sp * cy
    return pose


class KnownScene(Node):
    def __init__(self, **kwargs):
        super().__init__("known_pick_place_scene", **kwargs)
        self.declare_parameter("frame_id", "world")
        self.declare_parameter("object_id", "pick_object")
        self.declare_parameter("object_size", [0.05, 0.05, 0.22])
        self.declare_parameter("object_pose", [0.43, -0.18, 0.52, 0.0, 0.0, 0.0])
        self.declare_parameter("place_pose", [0.43, 0.18, 0.52, 0.0, 0.0, 0.0])
        self.declare_parameter("table_id", "pick_table")
        self.declare_parameter("table_size", [0.42, 0.60, 0.06])
        self.declare_parameter("table_pose", [0.52, 0.0, 0.38, 0.0, 0.0, 0.0])
        self.declare_parameter("publish_known_object", True)
        self.declare_parameter("publish_table", True)
        self.declare_parameter("environment_ready_topic", "")
        self.frame = self.get_parameter("frame_id").value
        self.object_id = self.get_parameter("object_id").value
        self.object_size = list(self.get_parameter("object_size").value)
        self.object_pose = list(self.get_parameter("object_pose").value)
        self.place_pose = list(self.get_parameter("place_pose").value)
        self.table_id = self.get_parameter("table_id").value
        self.table_size = list(self.get_parameter("table_size").value)
        self.table_pose = list(self.get_parameter("table_pose").value)
        self.publish_known_object = self.get_parameter("publish_known_object").value
        self.publish_table = self.get_parameter("publish_table").value
        self.environment_topic = self.get_parameter("environment_ready_topic").value
        self.environment_ready = not bool(self.environment_topic)
        self.apply_client = self.create_client(ApplyPlanningScene, "/apply_planning_scene")
        pose_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.object_pub = self.create_publisher(PoseStamped, "/object_pose", pose_qos)
        self.place_pub = self.create_publisher(PoseStamped, "/place_pose", pose_qos)
        if self.environment_topic:
            self.create_subscription(Bool, self.environment_topic, self.on_environment_ready, pose_qos)
        self.reset_service = self.create_service(Trigger, "~/reset_scene", self.reset_scene)
        self.pose_timer = self.create_timer(0.5, self.publish_poses)
        self.create_timer(1.0, self.apply_scene_if_ready)
        self.scene_ready = False
        self.scene_request_pending = False

    def on_environment_ready(self, message):
        self.environment_ready = message.data

    def publish_poses(self):
        if self.environment_topic and (not self.environment_ready or not self.scene_ready):
            return
        poses = [(self.place_pub, self.place_pose)]
        if self.publish_known_object:
            poses.append((self.object_pub, self.object_pose))
        for publisher, values in poses:
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame
            msg.pose = pose_from_xyz_rpy(values)
            publisher.publish(msg)

    def make_object(self, object_id, size, values):
        obj = CollisionObject()
        obj.header.frame_id = self.frame
        obj.id = object_id
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = size
        obj.primitives = [primitive]
        obj.primitive_poses = [pose_from_xyz_rpy(values)]
        obj.operation = CollisionObject.ADD
        return obj

    def apply_scene_if_ready(self):
        if self.scene_ready or self.scene_request_pending or not self.apply_client.service_is_ready():
            return
        self._send_scene(add=True)

    def _send_scene(self, add):
        self.scene_request_pending = True
        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        if add:
            if self.publish_table:
                scene.world.collision_objects = [
                    self.make_object(self.table_id, self.table_size, self.table_pose)]
            if self.publish_known_object:
                scene.world.collision_objects.append(
                    self.make_object(self.object_id, self.object_size, self.object_pose))
        else:
            ids = ([self.table_id] if self.publish_table else [])
            if self.publish_known_object:
                ids.append(self.object_id)
            for object_id in ids:
                obj = CollisionObject()
                obj.header.frame_id = self.frame
                obj.id = object_id
                obj.operation = CollisionObject.REMOVE
                scene.world.collision_objects.append(obj)
        request = ApplyPlanningScene.Request()
        request.scene = scene
        future = self.apply_client.call_async(request)
        future.add_done_callback(lambda result: self._scene_result(result, add))

    def _scene_result(self, future, add):
        self.scene_request_pending = False
        try:
            response = future.result()
            if response.success:
                self.scene_ready = add
                self.get_logger().info("Known pick/place collision scene synchronized with MoveIt")
            else:
                self.scene_ready = False
                self.get_logger().error("MoveIt rejected the known pick/place PlanningScene")
        except Exception as exc:  # noqa: BLE001
            self.scene_ready = False
            self.get_logger().error(f"PlanningScene update failed: {exc}")

    def reset_scene(self, _request, response):
        if not self.apply_client.service_is_ready():
            response.success = False
            response.message = "/apply_planning_scene is not available"
            return response
        if self.scene_request_pending:
            response.success = False
            response.message = "PlanningScene update is already pending"
            return response
        self.scene_ready = False
        # ADD with existing object IDs replaces the known objects atomically.
        self._send_scene(add=True)
        response.success = True
        response.message = "Known scene reset requested"
        return response


def main():
    rclpy.init()
    node = KnownScene()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
