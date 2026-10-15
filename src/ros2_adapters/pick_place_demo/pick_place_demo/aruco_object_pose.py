#!/usr/bin/env python3
"""Estimate the marked box pose and keep its MoveIt world object synchronized."""

import math
import struct

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener


def rotate(q, v):
    x, y, z, w = q
    vx, vy, vz = v
    # q * [v, 0] * q^-1
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (vx + w * tx + y * tz - z * ty,
            vy + w * ty + z * tx - x * tz,
            vz + w * tz + x * ty - y * tx)


class ArucoObjectPose(Node):
    def __init__(self):
        super().__init__("aruco_object_pose")
        self.declare_parameter("marker_id", 0)
        self.declare_parameter("marker_size", 0.045)
        self.declare_parameter("object_size", [0.05, 0.07, 0.22])
        self.declare_parameter("object_id", "pick_object")
        self.declare_parameter("camera_frame", "camera_color_optical_frame")
        self.declare_parameter("output_frame", "base_link")
        self.declare_parameter("wait_for_observation", False)
        self.observation_ready = not self.get_parameter("wait_for_observation").value
        self.marker_id = int(self.get_parameter("marker_id").value)
        self.marker_size = float(self.get_parameter("marker_size").value)
        self.object_size = list(self.get_parameter("object_size").value)
        self.object_id = self.get_parameter("object_id").value
        self.camera_frame = self.get_parameter("camera_frame").value
        self.output_frame = self.get_parameter("output_frame").value
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.info = None
        self.depth = None
        pose_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pose_pub = self.create_publisher(PoseStamped, "/object_pose", pose_qos)
        self.apply_client = self.create_client(ApplyPlanningScene, "/apply_planning_scene")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(CameraInfo, "/camera/camera/color/camera_info",
                                 lambda message: setattr(self, "info", message), 10)
        self.create_subscription(Image, "/camera/camera/color/image_raw",
                                 self.on_image, qos_profile_sensor_data)
        self.create_subscription(Image, "/camera/camera/depth/image_rect_raw",
                                 lambda message: setattr(self, "depth", message), qos_profile_sensor_data)
        self.last_apply = self.get_clock().now()
        self.scene_frozen = False
        self.create_subscription(Bool, "/pick_place/plan_ready",
                                 self.on_plan_ready, pose_qos)
        self.create_subscription(Bool, "/pick_place/observation_ready",
                                 self.on_observation_ready, pose_qos)

    def on_observation_ready(self, message):
        self.observation_ready = message.data

    def on_plan_ready(self, message):
        if message.data:
            self.scene_frozen = True

    def depth_at(self, u, v):
        if self.depth is None or self.depth.encoding != "32FC1":
            return None
        x, y = int(round(u)), int(round(v))
        if x < 0 or y < 0 or x >= self.depth.width or y >= self.depth.height:
            return None
        offset = y * self.depth.step + x * 4
        value = struct.unpack_from("<f", self.depth.data, offset)[0]
        return value if math.isfinite(value) and 0.1 < value < 2.0 else None

    def on_image(self, message):
        if self.info is None or self.depth is None or message.encoding != "rgb8":
            return
        # Keep the pose used by the already planned task immutable during
        # execution. Gazebo feedback remains the execution truth; a new visual
        # estimate is accepted on the next launch/reset.
        if self.scene_frozen or not self.observation_ready:
            return
        image_time = rclpy.time.Time.from_msg(message.header.stamp)
        depth_time = rclpy.time.Time.from_msg(self.depth.header.stamp)
        if image_time.nanoseconds == 0 or abs((image_time - depth_time).nanoseconds) > 50_000_000:
            return
        image = np.frombuffer(message.data, np.uint8).reshape(message.height, message.step)
        image = image[:, :message.width * 3].reshape(message.height, message.width, 3)
        corners, ids, _ = cv2.aruco.detectMarkers(image, self.dictionary)
        if ids is None or self.marker_id not in ids.flatten():
            return
        index = int(np.where(ids.flatten() == self.marker_id)[0][0])
        center = corners[index][0].mean(axis=0)
        depth = self.depth_at(center[0], center[1])
        if depth is None or self.info.k[0] <= 0 or self.info.k[4] <= 0:
            return
        point = ((center[0] - self.info.k[2]) * depth / self.info.k[0],
                 (center[1] - self.info.k[5]) * depth / self.info.k[4], depth)
        try:
            transform = self.tf_buffer.lookup_transform(
                self.output_frame, self.camera_frame, image_time)
        except Exception as error:
            self.get_logger().warning(f'Image-time camera TF unavailable: {error}',
                                      throttle_duration_sec=5.0)
            return
        tq = (transform.transform.rotation.x, transform.transform.rotation.y,
              transform.transform.rotation.z, transform.transform.rotation.w)
        tt = transform.transform.translation
        world_point = rotate(tq, point)
        center_point = (world_point[0] + tt.x, world_point[1] + tt.y,
                        world_point[2] + tt.z - self.object_size[2] * 0.5)
        stamp = message.header.stamp
        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = self.output_frame
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = center_point
        pose.pose.orientation.w = 1.0
        self.pose_pub.publish(pose)
        if (not self.scene_frozen and
                (self.get_clock().now() - self.last_apply).nanoseconds > 200_000_000):
            self.apply_object(pose.pose, stamp)
            self.last_apply = self.get_clock().now()

    def apply_object(self, pose, stamp):
        if not self.apply_client.service_is_ready():
            return
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = self.object_size
        collision = CollisionObject()
        collision.header.frame_id = self.output_frame
        collision.header.stamp = stamp
        collision.id = self.object_id
        collision.primitives = [primitive]
        collision.primitive_poses = [pose]
        collision.operation = CollisionObject.ADD
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = [collision]
        request = ApplyPlanningScene.Request()
        request.scene = scene
        self.apply_client.call_async(request)


def main():
    rclpy.init()
    node = ArucoObjectPose()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
