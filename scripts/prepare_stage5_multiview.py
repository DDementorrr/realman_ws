#!/usr/bin/env python3
"""Prepare the explicit ideal-Gazebo five-view fixture; starts no ROS nodes."""
import argparse
from copy import deepcopy
import json
import math
from pathlib import Path
import yaml

from pick_place_perception.multi_view import view_configs
from pick_place_perception.self_filter import load_model
from pick_place_scene.uncertainty import load_budget


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--self-filter-model', required=True, type=Path)
    parser.add_argument('--output-dir', required=True, type=Path)
    parser.add_argument('--camera-config', type=Path, help='Explicit external simulation camera fixture')
    parser.add_argument('--target-pose-topic', help='Separately supplied acquisition-time target pose')
    parser.add_argument('--monitor-known-occlusion', action='store_true',
                        help='Opt in to finite robot and separately known target surface explanations')
    parser.add_argument('--image-width', type=int, default=640)
    parser.add_argument('--image-height', type=int, default=480)
    parser.add_argument('--sample-stride', type=int, default=2)
    parser.add_argument('--sampling-bound', type=float, default=.009)
    args = parser.parse_args()
    if not (16 <= args.image_width <= 4096 and 16 <= args.image_height <= 4096):
        parser.error('--image-width/--image-height must be in [16, 4096]')
    if args.sample_stride < 1 or args.sampling_bound <= 0:
        parser.error('--sample-stride must be positive and --sampling-bound must be positive')
    sample_spacing = (2.0 * 2.0 * math.tan(1.20 / 2.0)
                      * args.sample_stride / args.image_width)
    if sample_spacing > args.sampling_bound:
        parser.error(f'configured sampling is too coarse: {sample_spacing:.6f} m '
                     f'exceeds --sampling-bound {args.sampling_bound:.6f} m')
    root = Path(__file__).resolve().parents[1]
    model = args.self_filter_model.resolve(strict=True)
    load_model(str(model))
    output = args.output_dir.resolve()
    base = root/'src/ros2_adapters'
    template = yaml.safe_load((base/'pick_place_perception/config/obstacles_template.yaml').read_text())
    cameras_file = (args.camera_config.resolve(strict=True) if args.camera_config else
                    base/'pick_place_sim/config/sim_multiview_cameras.yaml')
    names = [camera['name'] for camera in yaml.safe_load(cameras_file.read_text())['cameras']]
    configs = []
    for name in names:
        config = deepcopy(template['depth_obstacles']['ros__parameters'])
        config.update(camera_frame=name+'_camera_optical_frame',
                      rgb_image_topic=f'/{name}/camera/color/image_raw',
                      depth_image_topic=f'/{name}/camera/depth/image_rect_raw',
                      camera_info_topic=f'/{name}/camera/color/camera_info',
                      depth_info_topic=f'/{name}/camera/depth/camera_info',
                      self_filter_model=str(model), sensor_reliability='reliable',
                      workspace_min=[.305, -.305, .26], workspace_max=[.735, .305, .65],
                      voxel_size=.01, max_input_age=3., monitor_inputs=True, monitor_fixed_calibration=True,
                      far_no_return=True, visibility_margin=0., visibility_roundoff_m=1e-6,
                      sample_stride=args.sample_stride, sampling_bound_m=args.sampling_bound,
                      support_surface_enabled=True,
                      support_surface_height=.26, support_surface_normal=[0., 0., 1.],
                      support_surface_xy_min=[.31, -.30], support_surface_xy_max=[.73, .30],
                      support_contact_height_tolerance=.003,
                      support_uncertainty_budget_m=.035)
        if args.target_pose_topic:
            if not args.target_pose_topic.startswith('/') or args.target_pose_topic.strip() != args.target_pose_topic:
                parser.error('--target-pose-topic must be an absolute ROS topic')
            config['target_pose_topic'] = args.target_pose_topic
        if args.monitor_known_occlusion:
            config.update(self_surface_tolerance=.003, monitor_robot_occlusion=True,
                          monitor_target_occlusion=True)
        configs.append(config)
    configs[0]['additional_perception_configs'] = [str(output/(name+'.yaml')) for name in names[1:]]
    budget = yaml.safe_load((base/'pick_place_scene/config/sim_uncertainty_example.yaml').read_text())
    budget.update(sampling_bound_m=args.sampling_bound, basis=(
        'Ideal Gazebo fixture only: exact simulated extrinsics; 3mm depth allowance '
        'includes <=1um numeric roundoff; explicit REP117 +Inf far/no-return at 2m. '
        f'{args.image_width}x{args.image_height} at stride {args.sample_stride} gives '
        f'{sample_spacing*1000:.2f}mm worst-case angular sample spacing at 2m. '
        'Assumed 3mm tracking, 0.1m/s relative speed, 150ms skew and 5mm empirical safety; '
        '35mm per-side total. These are not measured hardware or stopping bounds.'))
    output.mkdir(parents=True, exist_ok=False)
    for name, config in zip(names, configs):
        (output/(name+'.yaml')).write_text(yaml.safe_dump({'depth_obstacles': {'ros__parameters': config}}))
    (output/'uncertainty.yaml').write_text(yaml.safe_dump(budget))
    (output/'fixture.json').write_text(json.dumps({
        'image_width': args.image_width, 'image_height': args.image_height,
        'sample_stride': args.sample_stride, 'sampling_bound_m': args.sampling_bound,
        'sample_spacing_upper_bound_m': sample_spacing, 'horizontal_fov_rad': 1.20,
        'depth_max_m': 2.0,
        'policy': 'simulation fixture dimensions only; live CameraInfo and runtime spacing gate remain authoritative',
    }, indent=2)+'\n')
    view_configs(configs[0])
    inflation = load_budget(str(output/'uncertainty.yaml'))[2]
    print(f'Prepared {len(configs)} mandatory views at {args.image_width}x{args.image_height}; '
          f'stride {args.sample_stride}; per-side inflation {inflation:.3f} m')
    print(f'perception_config:={output / (names[0]+".yaml")}')
    print(f'uncertainty_config:={output / "uncertainty.yaml"}')
    print(f'rgbd_sensor_config:={cameras_file} rgbd_update_rate:=5 '
          f'rgbd_width:={args.image_width} rgbd_height:={args.image_height} '
          'grasp_spin:=1.5707963267948966')


if __name__ == '__main__':
    main()
