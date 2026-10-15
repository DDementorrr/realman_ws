#!/usr/bin/env python3
"""Read-only camera probe for the Stage 6 ArUco model texture."""

import time

import cv2
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


def main():
    rclpy.init()
    node = rclpy.create_node("probe_pick_place_marker")
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    found = []
    saved = False
    depth_stats = []

    def on_image(message):
        nonlocal saved
        if message.encoding != "rgb8":
            return
        frame = np.frombuffer(message.data, np.uint8).reshape(message.height, message.step)
        rgb = frame[:, :message.width * 3].reshape(message.height, message.width, 3)
        if not saved:
            cv2.imwrite("/tmp/pick_place_marker_probe.png", cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
            saved = True
        corners, ids, _ = cv2.aruco.detectMarkers(rgb, dictionary)
        if ids is not None and 0 in ids.flatten():
            found.append(corners[int(np.where(ids.flatten() == 0)[0][0])][0])

    subscription = node.create_subscription(
        Image, "/camera/camera/color/image_raw", on_image, qos_profile_sensor_data)
    def on_depth(message):
        if message.encoding == "32FC1" and not depth_stats:
            values = np.frombuffer(message.data, np.float32)
            finite = values[np.isfinite(values)]
            depth_stats.append((len(finite), float(np.min(finite)) if len(finite) else None,
                                float(np.median(finite)) if len(finite) else None))
    depth_subscription = node.create_subscription(
        Image, "/camera/camera/depth/image_rect_raw", on_depth, qos_profile_sensor_data)
    deadline = time.monotonic() + 10
    while time.monotonic() < deadline and not found:
        rclpy.spin_once(node, timeout_sec=0.25)
    print(f"{'PASS' if found else 'FAIL'} ArUco ID 0 visible; depth={depth_stats}; screenshot /tmp/pick_place_marker_probe.png"
          + (f"; corners={found[0].tolist()}" if found else ""))
    node.destroy_subscription(subscription)
    node.destroy_subscription(depth_subscription)
    node.destroy_node()
    rclpy.shutdown()
    return 0 if found else 1


if __name__ == "__main__":
    raise SystemExit(main())
