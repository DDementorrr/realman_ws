#!/usr/bin/env python3
"""Estimate a marked object's pose and publish a scene observation."""

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import CollisionObject
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener
from pick_place_perception.rgbd_contract import depth_metres, validate_perception_config


def decode_rgb(message):
    if message.encoding != "rgb8" or message.width <= 0 or message.height <= 0:
        return None
    row_bytes = message.width * 3
    if message.step < row_bytes or len(message.data) < message.step * message.height:
        return None
    rows = np.frombuffer(message.data, dtype=np.uint8, count=message.step * message.height)
    return rows.reshape(message.height, message.step)[:, :row_bytes].reshape(
        message.height, message.width, 3)


def rotate(q, v):
    x, y, z, w = q
    vx, vy, vz = v
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (vx + w * tx + y * tz - z * ty,
            vy + w * ty + z * tx - x * tz,
            vz + w * tz + x * ty - y * tx)


class ArucoObjectPose(Node):
    def __init__(self):
        super().__init__("aruco_object_pose")
        required_defaults = {
            "marker_id": -1, "marker_size": float("nan"),
            "marker_center_offset": [float("nan")] * 3,
            "object_size": [float("nan")] * 3, "object_id": "", "camera_frame": "", "output_frame": "",
            "rgb_image_topic": "", "depth_image_topic": "", "camera_info_topic": "",
            "depth_encoding": "", "depth_scale": float("nan"), "max_time_skew": float("nan"),
        }
        self.declare_parameter("marker_id", required_defaults["marker_id"])
        self.declare_parameter("marker_size", required_defaults["marker_size"])
        self.declare_parameter("marker_center_offset", required_defaults["marker_center_offset"])
        self.declare_parameter("object_size", required_defaults["object_size"])
        self.declare_parameter("object_id", required_defaults["object_id"])
        self.declare_parameter("camera_frame", required_defaults["camera_frame"])
        self.declare_parameter("output_frame", required_defaults["output_frame"])
        self.declare_parameter("rgb_image_topic", required_defaults["rgb_image_topic"])
        self.declare_parameter("depth_image_topic", required_defaults["depth_image_topic"])
        self.declare_parameter("camera_info_topic", required_defaults["camera_info_topic"])
        self.declare_parameter("depth_encoding", required_defaults["depth_encoding"])
        self.declare_parameter("depth_scale", required_defaults["depth_scale"])
        self.declare_parameter("max_time_skew", required_defaults["max_time_skew"])
        config = {name: self.get_parameter(name).value for name in required_defaults}
        validate_perception_config(config)
        self.declare_parameter("wait_for_observation", False)
        self.observation_ready = not self.get_parameter("wait_for_observation").value
        self.marker_id = int(self.get_parameter("marker_id").value)
        self.marker_center_offset = tuple(float(v) for v in self.get_parameter("marker_center_offset").value)
        self.object_size = [float(v) for v in self.get_parameter("object_size").value]
        self.object_id = str(self.get_parameter("object_id").value)
        self.camera_frame = str(self.get_parameter("camera_frame").value)
        self.output_frame = str(self.get_parameter("output_frame").value)
        self.rgb_topic = self.get_parameter("rgb_image_topic").value
        self.depth_topic = self.get_parameter("depth_image_topic").value
        self.camera_info_topic = self.get_parameter("camera_info_topic").value
        self.depth_encoding = self.get_parameter("depth_encoding").value
        self.depth_scale = self.get_parameter("depth_scale").value
        self.max_time_skew = self.get_parameter("max_time_skew").value
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.info = None
        self.depth = None
        pose_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pose_pub = self.create_publisher(PoseStamped, "/object_pose", pose_qos)
        self.object_pub = self.create_publisher(CollisionObject, "/perception/collision_objects", pose_qos)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(CameraInfo, self.camera_info_topic,
                                 lambda message: setattr(self, "info", message), 10)
        self.create_subscription(Image, self.rgb_topic, self.on_image, qos_profile_sensor_data)
        self.create_subscription(Image, self.depth_topic,
                                 lambda message: setattr(self, "depth", message), qos_profile_sensor_data)
        self.create_subscription(Bool, "/pick_place/observation_ready", self.on_observation_ready, pose_qos)

    def on_observation_ready(self, message):
        self.observation_ready = message.data

    def depth_at(self, u, v):
        if self.depth is None:
            return None
        return depth_metres(self.depth, u, v, self.depth_encoding, self.depth_scale)

    def on_image(self, message):
        if self.info is None or self.depth is None or not self.observation_ready:
            return
        image_time = rclpy.time.Time.from_msg(message.header.stamp)
        depth_time = rclpy.time.Time.from_msg(self.depth.header.stamp)
        if image_time.nanoseconds == 0 or depth_time.nanoseconds == 0:
            return
        if abs((image_time - depth_time).nanoseconds) > self.max_time_skew * 1e9:
            return
        if self.depth.encoding != self.depth_encoding:
            return
        image = decode_rgb(message)
        if image is None:
            return
        gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.dictionary)
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
            transform = self.tf_buffer.lookup_transform(self.output_frame, self.camera_frame, image_time)
        except Exception as error:
            self.get_logger().warning(f"Image-time camera TF unavailable: {error}", throttle_duration_sec=5.0)
            return
        tq = (transform.transform.rotation.x, transform.transform.rotation.y,
              transform.transform.rotation.z, transform.transform.rotation.w)
        tt = transform.transform.translation
        rotated = rotate(tq, point)
        marker = (rotated[0] + tt.x, rotated[1] + tt.y, rotated[2] + tt.z)
        center_point = tuple(marker[i] + self.marker_center_offset[i] for i in range(3))
        pose = PoseStamped()
        pose.header.stamp = message.header.stamp
        pose.header.frame_id = self.output_frame
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = center_point
        pose.pose.orientation.w = 1.0
        self.pose_pub.publish(pose)

        observation = CollisionObject()
        observation.header = pose.header
        observation.id = self.object_id
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = self.object_size
        observation.primitives = [primitive]
        observation.primitive_poses = [pose.pose]
        observation.operation = CollisionObject.ADD
        self.object_pub.publish(observation)


def main():
    rclpy.init()
    node = ArucoObjectPose()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
