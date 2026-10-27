#!/usr/bin/env python3
import unittest
from pick_place_runtime import scene_revision_status


class SceneRevisionGateTest(unittest.TestCase):
    def test_legacy_entry_remains_compatible_without_scene_stream(self):
        self.assertEqual(
            scene_revision_status(None, None, False, False),
            (True, "scene_revision_unavailable_legacy_mode"))

    def test_candidate_binds_to_revision_seen_before_solution(self):
        self.assertEqual(
            scene_revision_status(4, 4, True, True),
            (True, "scene_revision_confirmed"))

    def test_missing_candidate_revision_is_rejected_when_stream_exists(self):
        self.assertEqual(
            scene_revision_status(None, 4, True, True),
            (False, "candidate_has_no_scene_revision"))

    def test_unready_scene_rejects_candidate(self):
        self.assertEqual(
            scene_revision_status(4, 4, False, True),
            (False, "scene_not_ready"))

    def test_scene_change_rejects_old_candidate(self):
        self.assertEqual(
            scene_revision_status(4, 5, True, True),
            (False, "stale_scene_revision"))


if __name__ == "__main__":
    unittest.main()
