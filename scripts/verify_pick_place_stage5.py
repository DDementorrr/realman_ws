#!/usr/bin/env python3
"""Runtime gate for the isolated Gazebo Harmonic RGB-D camera."""

import argparse
import math
import struct
import sys
import time
from collections import deque

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from tf2_ros import Buffer, TransformListener


def stamp_seconds(header):
    return header.stamp.sec + header.stamp.nanosec * 1e-9


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--wrist-camera', action='store_true',
                        help='Verify flange Z +10 cm mounting instead of overhead placement')
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node("verify_pick_place_stage5")
    topics = (
        ("color", Image, "/camera/camera/color/image_raw"),
        ("depth", Image, "/camera/camera/depth/image_rect_raw"),
        ("info", CameraInfo, "/camera/camera/color/camera_info"),
        ("depth_info", CameraInfo, "/camera/camera/depth/camera_info"),
        ("points", PointCloud2, "/camera/camera/depth/color/points"),
    )
    received = {name: deque(maxlen=30) for name, _, _ in topics}
    subscriptions = [
        node.create_subscription(message_type, topic,
                                 lambda message, name=name: received[name].append(message),
                                 qos_profile_sensor_data)
        for name, message_type, topic in topics
    ]
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    try:
        deadline = time.monotonic() + 120
        bundle = None
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.25)
            if not all(received.values()):
                continue
            # Bridges deliver these large messages independently. Their first
            # arrivals can belong to different frames, especially with GUI
            # rendering. Find a timestamp-matched bundle in bounded histories.
            for color_sample in reversed(received['color']):
                anchor = stamp_seconds(color_sample.header)
                candidate = [min(received[name], key=lambda msg: abs(
                    stamp_seconds(msg.header) - anchor)) for name, _, _ in topics]
                stamps = [stamp_seconds(msg.header) for msg in candidate]
                if min(stamps) > 0 and max(stamps) - min(stamps) <= 0.5:
                    bundle = candidate
                    break
            if bundle is not None:
                break
        missing = [name for name, messages in received.items() if not messages]
        if missing:
            print(f"FAIL missing RGB-D messages: {sorted(missing)}",
                  file=sys.stderr)
            return 1
        if bundle is None:
            latest = {name: stamp_seconds(messages[-1].header)
                      for name, messages in received.items()}
            print(f"FAIL RGB-D timestamps are missing or not synchronized: {latest}",
                  file=sys.stderr)
            return 1
        color, depth, info, depth_info, points = bundle
        expected_frame = "camera_color_optical_frame"
        if any(message.header.frame_id != expected_frame for message in
               (color, depth, info, depth_info, points)):
            print("FAIL RGB-D headers do not share the optical frame", file=sys.stderr)
            return 1
        if (color.encoding != "rgb8" or depth.encoding != "32FC1" or
                color.width != 640 or color.height != 480 or
                depth.width != 640 or depth.height != 480 or
                info.width != 640 or info.height != 480 or
                depth_info.width != 640 or depth_info.height != 480 or
                points.width != 640 or points.height != 480 or
                info.k[0] <= 0 or info.k[4] <= 0 or
                tuple(info.k) != tuple(depth_info.k)):
            print("FAIL RGB-D encoding, dimensions, or intrinsics", file=sys.stderr)
            return 1
        if not any(color.data):
            print("FAIL RGB image contains no rendered pixels", file=sys.stderr)
            return 1
        valid_depth = False
        for index in range(0, depth.width * depth.height, 32):
            offset = index * 4
            if offset + 4 <= len(depth.data):
                value = struct.unpack_from("<f", depth.data, offset)[0]
                if math.isfinite(value) and 0.1 < value < 2.0:
                    valid_depth = True
                    break
        if not valid_depth or not points.data:
            print("FAIL depth image or point cloud has no valid geometry", file=sys.stderr)
            return 1
        stamps = [stamp_seconds(message.header) for message in
                  (color, depth, info, depth_info, points)]
        if min(stamps) <= 0 or max(stamps) - min(stamps) > 0.5:
            print("FAIL RGB-D timestamps are missing or not synchronized", file=sys.stderr)
            return 1
        transform_deadline = time.monotonic() + 10
        while not buffer.can_transform("base_link", expected_frame, rclpy.time.Time()):
            if time.monotonic() > transform_deadline:
                print("FAIL camera optical frame cannot transform to base_link", file=sys.stderr)
                return 1
            rclpy.spin_once(node, timeout_sec=0.2)
        pose = buffer.lookup_transform("base_link", expected_frame, rclpy.time.Time())
        if args.wrist_camera:
            if not buffer.can_transform('Link6', expected_frame, rclpy.time.Time()):
                print('FAIL wrist optical frame cannot transform to Link6', file=sys.stderr)
                return 1
            mount = buffer.lookup_transform('Link6', expected_frame, rclpy.time.Time())
            translation = mount.transform.translation
            if math.dist((translation.x, translation.y, translation.z), (0.0, 0.0, 0.10)) > 0.001:
                print('FAIL wrist camera is not mounted at flange Z +10 cm', file=sys.stderr)
                return 1
        elif pose.transform.translation.z <= 0.8:
            print("FAIL camera frame is not above the worktable", file=sys.stderr)
            return 1
        print("PASS native RGB-D: 640x480 rgb8/32FC1, color/depth CameraInfo, PointCloud2, "
              "valid depth, optical TF, synchronized timestamps")
        return 0
    finally:
        for subscription in subscriptions:
            node.destroy_subscription(subscription)
        del listener
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
