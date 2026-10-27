#!/usr/bin/env python3
"""Behavioral mutation tests for a real archived MTC candidate.

Requires an archive from an actual planner run, not a fabricated task schema.
No controllers, Gazebo services or hardware are contacted by this test.
"""
import argparse
import copy
import math
import unittest
from pathlib import Path
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import PlanningScene
from moveit_task_constructor_msgs.msg import Solution, TaskDescription
from pick_place_mtc import candidate_contract as contract

REPORT = None


class CandidateContractTests(unittest.TestCase):
    def setUp(self):
        self.report = copy.deepcopy(REPORT)
        binding = self.report['binding']
        self.solution = contract.decoded(binding['solution_cdr_hex'], Solution)
        self.description = contract.decoded(binding['description_cdr_hex'], TaskDescription)
        self.scene = contract.decoded(binding['scene_cdr_hex'], PlanningScene)
        self.target = contract.decoded(binding['target_cdr_hex'], PoseStamped)
        self.place = contract.decoded(binding['place_cdr_hex'], PoseStamped)

    def test_real_payload_and_scene(self):
        contract.verify_payload(self.report, self.solution, self.description)
        contract.verify_scene(self.report, self.scene)
        contract.verify_targets(self.report, self.target, self.place)
        contract.verify_files(self.report)
        contract.verify_start(self.report, self.report['binding']['start_positions'], .1)

    def test_changed_trajectory_refused(self):
        for part in self.solution.sub_trajectory:
            if part.trajectory.joint_trajectory.points:
                part.trajectory.joint_trajectory.points[-1].positions[0] += .01
                break
        with self.assertRaisesRegex(ValueError, 'payload_mismatch'):
            contract.verify_payload(self.report, self.solution, self.description)

    def test_changed_scene_diff_in_solution_refused(self):
        self.solution.start_scene.is_diff = not self.solution.start_scene.is_diff
        with self.assertRaisesRegex(ValueError, 'payload_mismatch'):
            contract.verify_payload(self.report, self.solution, self.description)

    def test_changed_description_refused(self):
        self.description.stages[0].name += ' changed'
        with self.assertRaisesRegex(ValueError, 'description_payload_mismatch'):
            contract.verify_payload(self.report, self.solution, self.description)

    def test_model_parameter_wire_roundtrip(self):
        expected = contract.expected_parameters(self.report, 'planner_parameters')
        self.assertTrue(expected['robot_description'].string_value)
        self.assertTrue(expected['robot_description_semantic'].string_value)

    def test_changed_actual_geometry_refused(self):
        self.scene.world.collision_objects[0].primitive_poses[0].position.x += .02
        with self.assertRaisesRegex(ValueError, 'planning_scene_changed'):
            contract.verify_scene(self.report, self.scene)

    def test_changed_acm_refused(self):
        matrix = self.scene.allowed_collision_matrix
        matrix.entry_values[0].enabled[0] = not matrix.entry_values[0].enabled[0]
        with self.assertRaisesRegex(ValueError, 'planning_scene_changed'):
            contract.verify_scene(self.report, self.scene)

    def test_changed_transform_refused(self):
        self.scene.fixed_frame_transforms[0].transform.translation.z += .001
        with self.assertRaisesRegex(ValueError, 'planning_scene_changed'):
            contract.verify_scene(self.report, self.scene)

    def test_stamps_and_object_order_do_not_change_geometry(self):
        self.scene.world.collision_objects.reverse()
        for obj in self.scene.world.collision_objects:
            obj.header.stamp.sec += 1
        contract.verify_scene(self.report, self.scene)

    def test_changed_start_and_missing_feedback_refused(self):
        positions = dict(self.report['binding']['start_positions'])
        name = next(iter(positions))
        positions[name] += .02
        with self.assertRaisesRegex(ValueError, 'start_state_mismatch'):
            contract.verify_start(self.report, positions, .1)
        positions[name] = math.nan
        with self.assertRaisesRegex(ValueError, 'start_state_mismatch'):
            contract.verify_start(self.report, positions, .1)
        with self.assertRaisesRegex(ValueError, 'feedback_stale'):
            contract.verify_start(self.report, self.report['binding']['start_positions'], 1.01)

    def test_changed_target_refused(self):
        self.target.pose.position.x += .001
        with self.assertRaisesRegex(ValueError, 'target_or_place_changed'):
            contract.verify_targets(self.report, self.target, self.place)

    def test_selected_candidate_must_have_passed_hard_gates(self):
        import yaml
        self.report['candidates'][self.report['selected_index']]['accepted'] = False
        with self.assertRaisesRegex(ValueError, 'not_accepted'):
            contract.decode_report(yaml.safe_dump(self.report))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('archive', type=Path)
    args = parser.parse_args()
    REPORT = contract.decode_report(args.archive.read_text())
    unittest.main(argv=[__file__, '-v'])
