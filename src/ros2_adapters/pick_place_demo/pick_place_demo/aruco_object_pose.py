#!/usr/bin/env python3
"""Compatibility entrypoint for the perception package's ArUco node."""

from pick_place_perception.aruco_object_pose import ArucoObjectPose, main, rotate

__all__ = ["ArucoObjectPose", "main", "rotate"]


if __name__ == "__main__":
    main()
