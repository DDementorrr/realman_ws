#!/usr/bin/env python3
"""Regenerate the deterministic 4x4 ArUco ID 0 texture used by Stage 6.

Only used when updating the checked-in binary asset. Requires OpenCV contrib.
"""

from pathlib import Path

import cv2


def main():
    marker = cv2.aruco.drawMarker(
        cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50),
        0, 512, borderBits=1)
    output = (Path(__file__).resolve().parent.parent / "src" / "realman_rm65" /
              "robot_gazebo" / "worlds" / "pick_place_aruco_4x4_id0.png")
    if not cv2.imwrite(str(output), marker):
        raise RuntimeError(f"Failed to write {output}")
    print(output)


if __name__ == "__main__":
    main()
