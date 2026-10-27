#!/usr/bin/env python3
"""Read effective MTC launch overrides without starting Gazebo or controllers."""
import argparse
import json
import os
from pathlib import Path
import runpy
import signal
import subprocess
import sys
import time


def planner_only():
    from launch import LaunchDescription, LaunchService
    from launch.actions import DeclareLaunchArgument, SetLaunchConfiguration
    from launch_ros.actions import Node
    path = Path(__file__).resolve().parents[1] / 'src/ros2_adapters/pick_place_sim/launch/plan_pick_place_sim.launch.py'
    entities = runpy.run_path(str(path))['generate_launch_description']().entities
    # Execute the actual planner Node and its declarations. The simulator's
    # IncludeLaunchDescription is deliberately absent from this test process.
    declarations = [a for a in entities if isinstance(a, DeclareLaunchArgument)]
    planners = [a for a in entities if isinstance(a, Node)]
    assert len(planners) == 1
    overrides = {'grasp_spin': '1.5707963267948966', 'planning_candidates': '2',
                 'open_target': '0.7', 'motion_velocity_scaling': '0.2',
                 'transport_keep_orientation': 'true'}
    service = LaunchService()
    service.include_launch_description(LaunchDescription([
        *declarations, *[SetLaunchConfiguration(k, v) for k, v in overrides.items()], *planners]))
    return service.run()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--planner-only', action='store_true')
    parser.add_argument('--output-dir', type=Path)
    parser.add_argument('--domain-id', type=int, default=194)
    args = parser.parse_args()
    if args.planner_only:
        return planner_only()
    if args.output_dir is None:
        parser.error('--output-dir is required')
    args.output_dir.mkdir(parents=True, exist_ok=False)
    os.environ.update(ROS_DOMAIN_ID=str(args.domain_id), ROS_LOCALHOST_ONLY='1')
    import rclpy
    from rcl_interfaces.srv import GetParameters
    rclpy.init()
    observer = rclpy.create_node('verify_effective_mtc_launch_overrides')
    expected = {'grasp_spin': ('double_value', 1.5707963267948966),
                'planning_candidates': ('integer_value', 2), 'open_target': ('double_value', .7),
                'motion_velocity_scaling': ('double_value', .2),
                'transport_keep_orientation': ('bool_value', True), 'use_sim_time': ('bool_value', True)}
    with (args.output_dir/'planner.log').open('w') as log:
        process = subprocess.Popen([sys.executable, __file__, '--planner-only'],
                                   stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            client = observer.create_client(GetParameters, '/plan_pick_place/get_parameters')
            if not client.wait_for_service(timeout_sec=25):
                raise RuntimeError('planner parameter service unavailable; see planner.log')
            future = client.call_async(GetParameters.Request(names=list(expected)))
            rclpy.spin_until_future_complete(observer, future, timeout_sec=10)
            if not future.done() or future.result() is None:
                raise RuntimeError('parameter query timed out')
            actual = {}
            for (name, (field, value)), parameter in zip(expected.items(), future.result().values):
                actual[name] = getattr(parameter, field)
            (args.output_dir/'parameter-response.txt').write_text(str(future.result()))
            (args.output_dir/'effective-parameters.json').write_text(json.dumps(actual, indent=2))
            for name, (_, value) in expected.items():
                if actual[name] != value:
                    raise RuntimeError(f'{name}: expected {value!r}, got {actual[name]!r}')
            deadline = time.monotonic()+1
            while time.monotonic() < deadline:
                rclpy.spin_once(observer, timeout_sec=.1)
            names = set(observer.get_node_names())
            unexpected = {name for name in names if name not in {'plan_pick_place', observer.get_name()}
                          and not name.startswith('introspection_')}
            if unexpected:
                raise RuntimeError(f'unexpected backend node: {names}')
            print('PASS actual simulation planner launch: all six typed overrides effective; no backend nodes')
        finally:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGTERM)
                    process.wait(timeout=5)
            observer.destroy_node()
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
