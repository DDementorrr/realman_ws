"""Pure validation and depth decoding for the perception input contract."""

import math
import struct


DEPTH_ENCODINGS = {"16UC1": "H", "32FC1": "f"}
MIN_DEPTH_METRES = 0.1
MAX_DEPTH_METRES = 2.0


class InputContractError(ValueError):
    pass


def _positive_finite(value):
    return (not isinstance(value, bool) and isinstance(value, (int, float))
            and math.isfinite(value) and value > 0)


def validate_input_contract(rgb_topic, depth_topic, camera_info_topic,
                            depth_encoding, depth_scale, max_time_skew):
    for name, topic in (("rgb_image_topic", rgb_topic),
                        ("depth_image_topic", depth_topic),
                        ("camera_info_topic", camera_info_topic)):
        if not isinstance(topic, str) or not topic.startswith("/") or not topic.strip():
            raise InputContractError(f"{name} must be an absolute topic name")
    if not isinstance(depth_encoding, str) or depth_encoding not in DEPTH_ENCODINGS:
        raise InputContractError("depth_encoding must be 16UC1 or 32FC1")
    if isinstance(depth_scale, bool) or not isinstance(depth_scale, (int, float)):
        raise InputContractError("depth_scale must be a positive finite number")
    if not math.isfinite(depth_scale) or depth_scale <= 0:
        raise InputContractError("depth_scale must be a positive finite number")
    if isinstance(max_time_skew, bool) or not isinstance(max_time_skew, (int, float)):
        raise InputContractError("max_time_skew must be a positive finite number")
    if not math.isfinite(max_time_skew) or max_time_skew <= 0:
        raise InputContractError("max_time_skew must be a positive finite number")


def validate_perception_config(config):
    """Validate the complete external configuration before creating ROS I/O."""
    required = (
        "rgb_image_topic", "depth_image_topic", "camera_info_topic", "depth_encoding",
        "depth_scale", "max_time_skew", "marker_id", "marker_size",
        "marker_center_offset", "object_size", "object_id", "camera_frame", "output_frame",
    )
    missing = [name for name in required if name not in config or config[name] is None]
    if missing:
        raise InputContractError("missing required perception parameters: " + ", ".join(missing))
    validate_input_contract(
        config["rgb_image_topic"], config["depth_image_topic"], config["camera_info_topic"],
        config["depth_encoding"], config["depth_scale"], config["max_time_skew"],
    )
    for name in ("camera_frame", "output_frame"):
        value = config[name]
        if not isinstance(value, str) or not value.strip() or value.startswith("/"):
            raise InputContractError(f"{name} must be a non-empty TF frame id without a leading slash")
    if not isinstance(config["object_id"], str) or not config["object_id"].strip():
        raise InputContractError("object_id must be a non-empty string")
    marker_id = config["marker_id"]
    if isinstance(marker_id, bool) or not isinstance(marker_id, int) or marker_id < 0:
        raise InputContractError("marker_id must be a non-negative integer")
    if not _positive_finite(config["marker_size"]):
        raise InputContractError("marker_size must be a positive finite length in metres")
    for name in ("marker_center_offset", "object_size"):
        values = config[name]
        if not isinstance(values, (list, tuple)) or len(values) != 3:
            raise InputContractError(f"{name} must contain exactly three finite values in metres")
        if any(isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value)
               for value in values):
            raise InputContractError(f"{name} must contain exactly three finite values in metres")
        if name == "object_size" and any(value <= 0 for value in values):
            raise InputContractError("object_size dimensions must be positive lengths in metres")


def depth_metres(depth, u, v, encoding, scale):
    if encoding not in DEPTH_ENCODINGS or depth.encoding != encoding:
        return None
    item_size = struct.calcsize(DEPTH_ENCODINGS[encoding])
    if depth.width <= 0 or depth.height <= 0 or depth.step < depth.width * item_size:
        return None
    if len(depth.data) < depth.step * depth.height:
        return None
    x, y = int(round(u)), int(round(v))
    if x < 0 or y < 0 or x >= depth.width or y >= depth.height:
        return None
    endian = ">" if depth.is_bigendian else "<"
    raw = struct.unpack_from(endian + DEPTH_ENCODINGS[encoding], depth.data,
                             y * depth.step + x * item_size)[0]
    value = float(raw) * float(scale)
    if not math.isfinite(value) or not MIN_DEPTH_METRES < value < MAX_DEPTH_METRES:
        return None
    return value
