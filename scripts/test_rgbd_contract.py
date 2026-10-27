#!/usr/bin/env python3
"""Unit checks for the perception node's configurable RGB-D input contract."""

import math
import struct
import unittest
from types import SimpleNamespace

from pick_place_perception.rgbd_contract import (
    InputContractError,
    depth_metres,
    validate_input_contract,
)


class RgbdContractTest(unittest.TestCase):
    def image(self, encoding, value, bigendian=False):
        step = 2 if encoding == "16UC1" else 4
        if encoding == "16UC1":
            data = struct.pack((">" if bigendian else "<") + "H", value)
        else:
            data = struct.pack((">" if bigendian else "<") + "f", value)
        return SimpleNamespace(width=1, height=1, encoding=encoding, is_bigendian=bigendian,
                               step=step, data=data)

    def test_32fc1_metres_and_endianness(self):
        for bigendian in (False, True):
            with self.subTest(bigendian=bigendian):
                sample = self.image("32FC1", 0.75, bigendian)
                self.assertAlmostEqual(depth_metres(sample, 0, 0, "32FC1", 1.0), 0.75)

    def test_16uc1_scale_and_endianness(self):
        for bigendian in (False, True):
            with self.subTest(bigendian=bigendian):
                sample = self.image("16UC1", 750, bigendian)
                self.assertAlmostEqual(depth_metres(sample, 0, 0, "16UC1", 0.001), 0.75)

    def test_rejects_invalid_depth_and_truncated_rows(self):
        for value in (0.0, math.nan, math.inf, 2.1):
            self.assertIsNone(depth_metres(self.image("32FC1", value), 0, 0, "32FC1", 1.0))
        sample = self.image("16UC1", 750)
        sample.data = sample.data[:1]
        self.assertIsNone(depth_metres(sample, 0, 0, "16UC1", 0.001))

    def test_rejects_invalid_contract(self):
        args = ("/rgb", "/depth", "/info", "32FC1", 1.0, 0.1)
        validate_input_contract(*args)
        for changes in ({4: 0.0}, {5: math.inf}, {3: "mono16"}, {0: "relative"}):
            values = list(args)
            for index, value in changes.items():
                values[index] = value
            with self.subTest(changes=changes), self.assertRaises(InputContractError):
                validate_input_contract(*values)

if __name__ == "__main__":
    unittest.main()
