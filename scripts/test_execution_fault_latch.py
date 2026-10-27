#!/usr/bin/env python3
import json
import tempfile
import unittest
from pathlib import Path

from execution_fault_latch import ExecutionFaultLatch


class ExecutionFaultLatchTest(unittest.TestCase):
    def test_missing_marker_starts_clear_and_latched_reason_survives_reopen(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "fault.json"
            first = ExecutionFaultLatch(path)
            self.assertIsNone(first.load())
            first.latch("cancel outcome unconfirmed")

            restarted = ExecutionFaultLatch(path)
            self.assertEqual(restarted.load(), "cancel outcome unconfirmed")

    def test_malformed_marker_fails_closed(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "fault.json"
            path.write_text("not-json", encoding="utf-8")
            latch = ExecutionFaultLatch(path)
            with self.assertRaisesRegex(RuntimeError, "malformed"):
                latch.load()

    def test_unknown_marker_version_fails_closed(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "fault.json"
            path.write_text(json.dumps({"version": 2, "reason": "blocked"}), encoding="utf-8")
            latch = ExecutionFaultLatch(path)
            with self.assertRaisesRegex(RuntimeError, "malformed"):
                latch.load()

    def test_latch_write_failure_is_reported(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "fault.json"
            path.mkdir()
            latch = ExecutionFaultLatch(path)
            with self.assertRaises(IsADirectoryError):
                latch.latch("cancel outcome unconfirmed")

    def test_empty_fault_reason_is_rejected(self):
        with tempfile.TemporaryDirectory() as temporary:
            latch = ExecutionFaultLatch(Path(temporary) / "fault.json")
            with self.assertRaises(ValueError):
                latch.latch("  ")


if __name__ == "__main__":
    unittest.main(verbosity=2)
