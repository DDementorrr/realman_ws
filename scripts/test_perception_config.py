#!/usr/bin/env python3
import copy
import unittest

from pick_place_perception.rgbd_contract import InputContractError, validate_perception_config


VALID = {
    "rgb_image_topic": "/rgb", "depth_image_topic": "/depth", "camera_info_topic": "/info",
    "depth_encoding": "32FC1", "depth_scale": 1.0, "max_time_skew": 0.1,
    "marker_id": 1, "marker_size": 0.045, "marker_center_offset": [0.026, 0.0, -0.055],
    "object_size": [0.05, 0.07, 0.22], "object_id": "pick_object",
    "camera_frame": "camera_color_optical_frame", "output_frame": "base_link",
}


class PerceptionConfigTest(unittest.TestCase):
    def test_complete_configuration(self):
        validate_perception_config(VALID)

    def test_required_fields_do_not_have_implicit_fallback(self):
        for name in VALID:
            with self.subTest(name=name):
                candidate = copy.deepcopy(VALID)
                del candidate[name]
                with self.assertRaises(InputContractError):
                    validate_perception_config(candidate)

    def test_invalid_values_are_rejected(self):
        invalid = (
            ("rgb_image_topic", "relative"), ("camera_frame", "/base_link"),
            ("output_frame", ""), ("depth_encoding", "16UC3"),
            ("depth_scale", 0.0), ("max_time_skew", -1.0), ("marker_id", -1),
            ("marker_size", 0.0), ("marker_center_offset", [0.0, 0.0]),
            ("object_size", [0.05, 0.0, 0.22]), ("object_id", ""),
        )
        for name, value in invalid:
            with self.subTest(name=name):
                candidate = copy.deepcopy(VALID)
                candidate[name] = value
                with self.assertRaises(InputContractError):
                    validate_perception_config(candidate)


if __name__ == "__main__":
    unittest.main()
