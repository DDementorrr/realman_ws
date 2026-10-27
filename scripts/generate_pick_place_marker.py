#!/usr/bin/env python3
"""Regenerate the deterministic 4x4 ArUco textures used by Stage 6.

Only used when updating the checked-in binary asset. Requires OpenCV contrib.
"""

from pathlib import Path

import cv2


def main():
    root = (Path(__file__).resolve().parent.parent / "src" / "realman_rm65" /
            "robot_gazebo" / "worlds")
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    for marker_id in (0, 1):
        # Keep a 2.5 mm white margin on the 50 mm visual plane so the marker
        # detector sees a complete border instead of a texture-edge crop.
        marker = cv2.aruco.drawMarker(dictionary, marker_id, 461, borderBits=1)
        canvas = cv2.copyMakeBorder(marker, 25, 26, 25, 26,
                                    cv2.BORDER_CONSTANT, value=255)
        output = root / f"pick_place_aruco_4x4_id{marker_id}.png"
        if not cv2.imwrite(str(output), canvas):
            raise RuntimeError(f"Failed to write {output}")
        print(output)


if __name__ == "__main__":
    main()
