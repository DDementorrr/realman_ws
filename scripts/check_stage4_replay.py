#!/usr/bin/env python3
"""Independent recorded RGB-D acceptance; starts public perception only.

Source the desired public ROS installation first. Each case gets a new node and
an isolated domain; no simulator, hardware driver or controller is launched.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', required=True)
    parser.add_argument('--bag', required=True)
    parser.add_argument('--output-dir', required=True)
    parser.add_argument('--domain-id', type=int, default=176)
    args = parser.parse_args()
    if not 0 <= args.domain_id <= 228:
        parser.error('domain-id must be in [0, 228]')
    root = Path(args.output_dir)
    root.mkdir(parents=True, exist_ok=False)
    script = Path(__file__).with_name('stage4_rgbd_evidence.py')
    results = []
    for index, fault in enumerate((None, 'future_tf', 'bad_registration', 'missing_robot_state')):
        case = fault or 'valid'
        env = {**os.environ, 'ROS_LOCALHOST_ONLY': '1', 'ROS_DOMAIN_ID': str(args.domain_id+index)}
        with (root/(case+'-node.log')).open('w') as log:
            process = subprocess.Popen(['ros2', 'launch', 'pick_place_perception', 'obstacles_external.launch.py',
                'perception_config:='+str(Path(args.config).resolve()), 'use_sim_time:=true'],
                env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            try:
                command = [sys.executable, str(script), 'replay', '--config', args.config,
                           '--bag', args.bag, '--output', str(root/(case+'.json'))]
                if fault:
                    command += ['--fault', fault]
                with (root/(case+'-probe.log')).open('w') as output:
                    subprocess.run(command, env=env, stdout=output, stderr=subprocess.STDOUT, timeout=45, check=True)
                if process.poll() is not None:
                    raise RuntimeError(f'{case}: perception exited unexpectedly')
                results.append(case)
                print(f'PASS {case}', flush=True)
            finally:
                if process.poll() is None:
                    os.killpg(process.pid, signal.SIGINT)
                    try:
                        process.wait(timeout=8)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid, signal.SIGTERM)
                        process.wait(timeout=5)
    (root/'acceptance.json').write_text(json.dumps({'passed': results}, indent=2))


if __name__ == '__main__':
    main()
