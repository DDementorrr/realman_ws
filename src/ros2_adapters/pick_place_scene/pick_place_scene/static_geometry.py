"""Small backend-neutral BOX definition and standard MoveIt conversion."""

from dataclasses import dataclass, replace
import math
from pathlib import Path
import re

import yaml


@dataclass(frozen=True)
class Box:
    id: str
    size: tuple
    pose: tuple  # xyz, roll/pitch/yaw in frame_id


def support_height_envelope(frame, boxes, support_id, support_frame, height,
                            xy_min, xy_max, tolerance):
    """Conservatively cover the declared +Z support contact-height error.

    This is known support geometry, retaining its ID and existing target
    contact rule. It never changes independent obstacle geometry or its budget.
    Reject a mismatched declaration before producing a usable scene.
    """
    if (frame != support_frame or not isinstance(tolerance, (int, float))
            or isinstance(tolerance, bool) or not math.isfinite(tolerance)
            or tolerance < 0 or not math.isfinite(height)):
        raise ValueError('support envelope requires matching frame and finite nonnegative tolerance')
    matches = [box for box in boxes if box.id == support_id]
    if len(matches) != 1:
        raise ValueError('support envelope requires exactly one known support ID')
    box = matches[0]
    if any(abs(value) > 1e-9 for value in box.pose[3:]):
        raise ValueError('support envelope requires an axis-aligned horizontal support')
    if abs(box.pose[2]+box.size[2]/2-height) > 1e-9:
        raise ValueError('declared support height differs from known support geometry')
    if (len(xy_min) != 2 or len(xy_max) != 2
            or any(not math.isfinite(v) for v in (*xy_min, *xy_max))
            or any(a >= b for a, b in zip(xy_min, xy_max))
            or any(xy_min[i] < box.pose[i]-box.size[i]/2-1e-9
                   or xy_max[i] > box.pose[i]+box.size[i]/2+1e-9 for i in range(2))):
        raise ValueError('declared support footprint exceeds known support geometry')
    size = (*box.size[:2], box.size[2]+tolerance)
    pose = (*box.pose[:2], box.pose[2]+tolerance/2, *box.pose[3:])
    return tuple(replace(item, size=size, pose=pose) if item.id == support_id else item for item in boxes)


def load_geometry(path):
    document = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    if not isinstance(document, dict) or set(document) != {"frame_id", "boxes"}:
        raise ValueError("geometry requires exactly frame_id and boxes")
    frame = document["frame_id"]
    if not isinstance(frame, str) or not re.fullmatch(r"[A-Za-z_][A-Za-z0-9_/]*", frame):
        raise ValueError("invalid geometry frame_id")
    entries = document["boxes"]
    if not isinstance(entries, list) or not entries:
        raise ValueError("geometry boxes must be a non-empty list")
    boxes, ids = [], set()
    for entry in entries:
        if not isinstance(entry, dict) or set(entry) != {"id", "size", "pose"}:
            raise ValueError("each box requires exactly id, size and pose")
        name = entry["id"]
        if (not isinstance(name, str) or not re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", name)
                or name in ids):
            raise ValueError("box IDs must be valid and unique")
        vectors = []
        for key, length in (("size", 3), ("pose", 6)):
            values = entry[key]
            if (not isinstance(values, list) or len(values) != length
                    or any(isinstance(x, bool) or not isinstance(x, (int, float))
                           or not math.isfinite(x) for x in values)):
                raise ValueError(f"{name}.{key} requires {length} finite numbers")
            vectors.append(tuple(float(x) for x in values))
        if any(x <= 0 for x in vectors[0]):
            raise ValueError(f"{name}.size must be positive")
        ids.add(name)
        boxes.append(Box(name, *vectors))
    return frame, tuple(boxes)


def collision_object(frame, box, stamp=None):
    from geometry_msgs.msg import Pose
    from moveit_msgs.msg import CollisionObject
    from shape_msgs.msg import SolidPrimitive

    obj = CollisionObject(id=box.id, operation=CollisionObject.ADD)
    obj.header.frame_id = frame
    if stamp is not None:
        obj.header.stamp = stamp
    obj.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=list(box.size))]
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = box.pose[:3]
    roll, pitch, yaw = box.pose[3:]
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    pose.orientation.x = sr * cp * cy - cr * sp * sy
    pose.orientation.y = cr * sp * cy + sr * cp * sy
    pose.orientation.z = cr * cp * sy - sr * sp * cy
    pose.orientation.w = cr * cp * cy + sr * sp * sy
    obj.primitive_poses = [pose]
    return obj
