#!/usr/bin/env python3
"""Verify actual planner config/model failures without starting a backend."""

from copy import deepcopy
from pathlib import Path
import subprocess
import sys
import tempfile
import os
import argparse
import yaml
import xacro
from ament_index_python.packages import get_package_share_directory


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('executable')
    parser.add_argument('--fixture-out', type=Path,
                        help='Export explicit model/task input for the public launch probe')
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    executable = args.executable
    demo = yaml.safe_load((root / 'src/ros2_adapters/pick_place_demo/config/mtc_demo.yaml').read_text())
    base = demo['plan_pick_place']['ros__parameters']
    share = Path(get_package_share_directory('robot_realman_moveit_config'))
    model = xacro.process_file(str(share / 'urdf/rm65b_ctm2f110.urdf.xacro'),
                               mappings={'link6_type': 'Link6', 'wrist_camera': 'false',
                                         'pad_compliance': 'false'}).toxml()
    base.update(robot_description=model,
                robot_description_semantic=(share / 'config/rm65b_ctm2f110.srdf').read_text(),
                input_wait_timeout_sec=0.3)
    if args.fixture_out:
        args.fixture_out.mkdir(parents=True, exist_ok=True)
        params = deepcopy(base)
        params['input_wait_timeout_sec'] = 5.0
        # Public launch must override this name-scoped value with its explicit
        # use_sim_time argument, including on Humble's parameter-file parser.
        params['use_sim_time'] = True
        model_params = {key: params.pop(key) for key in
                        ('robot_description', 'robot_description_semantic')}
        (args.fixture_out / 'planning.yaml').write_text(yaml.safe_dump(
            {'plan_pick_place': {'ros__parameters': model_params}}))
        (args.fixture_out / 'task.yaml').write_text(yaml.safe_dump(
            {'plan_pick_place': {'ros__parameters': params}}))
    cases = [
        ('missing', {}, 'arm_group'),
        ('wrong_group', {'arm_group': 'nonexistent'}, 'arm_group'),
        ('wrong_tool', {'tool_link': 'nonexistent'}, 'tool_link'),
        ('wrong_hand_joint', {'hand_joint': 'joint1'}, 'hand_joint'),
        ('wrong_score_joint', {'scoring_joints': ['not_a_joint'], 'joint_motion_weights': [1.0]}, 'scoring_joints'),
        ('partial_score', {'scoring_joints': ['joint1'], 'joint_motion_weights': [1.0]}, 'scoring_joints'),
        ('duplicate_links', {'contact_links': ['ctm2f110_left_pad', 'ctm2f110_left_pad']}, 'contact_links'),
        ('non_gripper_contact', {'contact_links': ['Link3']}, 'contact_links'),
        ('bad_distance', {'approach_min': 0.2, 'approach_max': 0.1}, 'approach'),
        ('bad_direction', {'approach_direction': [0.0, 0.0, 0.0]}, 'approach_direction'),
        ('same_ids', {'support_id': base['object_id']}, 'support_id'),
        ('bad_weights', {'joint_motion_weights': [0.0] * 6}, 'joint_motion_weights'),
        ('bad_hand_target', {'open_target': 9.0}, 'open_target'),
        ('valid_wait', {}, 'Timed out waiting'),
    ]
    with tempfile.TemporaryDirectory(prefix='realman-mtc-params-') as directory:
        for name, changes, expected in cases:
            params = {} if name == 'missing' else deepcopy(base)
            params.update(changes)
            path = Path(directory) / (name + '.yaml')
            path.write_text(yaml.safe_dump({'plan_pick_place': {'ros__parameters': params}}))
            env = os.environ.copy()
            env.setdefault('ROS_LOG_DIR', directory)
            result = subprocess.run([executable, '--ros-args', '--params-file', str(path)],
                                    text=True, capture_output=True, timeout=12, env=env)
            output = result.stdout + result.stderr
            if result.returncode != 1 or expected not in output or 'Complete MTC solution published' in output:
                raise RuntimeError(f'{name}: expected orderly failure mentioning {expected!r}, '
                                   f'got exit {result.returncode}: {output[-5000:]}')
            print(f'PASS {name}: {expected}')
    print('PASS MTC config/model validation and bounded no-input wait (no backend)')


if __name__ == '__main__':
    main()
