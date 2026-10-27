"""Stage 2: archived fixed/random/benchmark geometry and startup preflight."""

import os
import json
import math
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET
import yaml

from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from pick_place_sim.obstacle_scene import config_from, generate_scene
from pick_place_sim.workbench_world import generate_world, prepare_rgbd_fixture


def setup(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    sim, gazebo = Path(share('pick_place_sim')), Path(share('robot_gazebo'))
    overrides = {key: arg(key) for key in ('obstacle_mode', 'benchmark') if arg(key)}
    for key in ('obstacle_seed', 'obstacle_count'):
        if arg(key):
            overrides[key] = int(arg(key))
    config = config_from(arg('obstacle_config'), **overrides)
    root = arg('scene_archive')
    # Retain full evidence after shutdown. Explicit user paths must be new.
    archive = Path(root) if root else Path(tempfile.mkdtemp(prefix='rm65b-obstacles-')) / 'scene'
    geometry, manifest = generate_scene(
        arg('workbench_file'), arg('scene_file'), gazebo / 'urdf/rm65b_ctm2f110_gz.urdf.xacro',
        config, archive)
    # This stage retains the existing fixed object/attachment backend; reject
    # custom task/world disagreement rather than planning from a false pose.
    template = ET.parse(gazebo / 'worlds/rm65b_pick_place_fixed.sdf').find('world')
    target = template.find("model[@name='pick_object']")
    place_marker = list(map(float, template.findtext("model[@name='place_zone']/pose").split()))
    task = manifest['task']
    if (list(map(float, target.findtext('pose').split())) != task['object_pose']
            or list(map(float, target.findtext('link/visual/geometry/box/size').split())) != task['object_size']
            or task['place_pose'][:2] != place_marker[:2]
            or task['place_pose'][2] != task['object_pose'][2]):
        raise ValueError('stage 2 fixed backend requires the unchanged known task poses and object size')
    world = generate_world(gazebo / 'worlds/rm65b_pick_place_fixed.sdf', geometry,
                           gazebo / 'urdf/rm65b_ctm2f110_gz.urdf.xacro', archive / 'world.sdf')
    blocked = manifest['expected'] == 'blocked'
    source = arg('obstacle_source')
    if source not in ('snapshot', 'gazebo_truth', 'rgbd'):
        raise ValueError('obstacle_source must be snapshot, gazebo_truth or rgbd')
    if source in ('gazebo_truth', 'rgbd') and arg('auto_execute').lower() != 'false':
        raise ValueError('observed scenes require explicit execution; auto_execute must be false')
    if source == 'rgbd':
        # Known task poses do not need fiducials or the raised visual-only
        # place disk. Keep rendered depth consistent with physical geometry.
        (archive/'rgbd_fixture.json').write_text(json.dumps(prepare_rgbd_fixture(world), indent=2))
    static_file = geometry
    extra = []
    required_ids = []
    if source in ('gazebo_truth', 'rgbd'):
        # Bench stays immutable; obstacle poses/sizes come only from live export.
        support = {**manifest['geometry'], 'boxes': [b for b in manifest['geometry']['boxes']
                   if not b['id'].startswith('obstacle_')]}
        static_file = archive / 'support.yaml'
        static_file.write_text(yaml.safe_dump(support, sort_keys=False))
        if source == 'gazebo_truth':
            required_ids = [b['id'] for b in manifest['geometry']['boxes'] if b['id'].startswith('obstacle_')]
            extra = [Node(package='pick_place_sim', executable='gazebo_obstacle_state_sync.py',
                          output='screen', parameters=[{'use_sim_time': True, 'archive': str(archive)}])]
        else:
            if not arg('perception_config') or not arg('uncertainty_config'):
                raise ValueError('rgbd requires explicit perception_config and uncertainty_config')
            perception = yaml.safe_load(Path(arg('perception_config')).read_text())['depth_obstacles']['ros__parameters']
            if perception.get('monitor_inputs') is not True:
                raise ValueError('rgbd planning requires continuous monitor_inputs=true')
            from pick_place_scene.uncertainty import load_budget
            from pick_place_perception.multi_view import view_configs
            views = view_configs(perception)
            budget = load_budget(arg('uncertainty_config'))[0]
            for view in views:
                if (view['depth_max'] > budget['observation_range_m']
                        or view['max_time_skew'] > budget['time_skew_bound_sec']
                        or view.get('sampling_bound_m', float('inf')) > budget['sampling_bound_m']
                        or view.get('visibility_roundoff_m', 0.) > budget['depth_bound_m']
                        or view.get('self_surface_tolerance', 0.) > budget['depth_bound_m']
                        or 'depth_obstacles' not in budget['object_ids']):
                    raise ValueError('perception depth/time/sampling/roundoff limits exceed uncertainty bounds')
            if perception.get('support_surface_enabled', False):
                from pick_place_scene.static_geometry import load_geometry, support_height_envelope
                frame, nominal_boxes = load_geometry(static_file)
                tolerance = perception['support_contact_height_tolerance']
                if (tolerance > budget['depth_bound_m']
                        or abs(perception['support_uncertainty_budget_m']
                               -load_budget(arg('uncertainty_config'))[2]) > 1e-9):
                    raise ValueError('support contact tolerance/budget differs from uncertainty contract')
                envelope = support_height_envelope(
                    frame, nominal_boxes, task['table_id'], perception['output_frame'],
                    perception['support_surface_height'], perception['support_surface_xy_min'],
                    perception['support_surface_xy_max'], tolerance)
                (archive/'support.nominal.yaml').write_bytes(static_file.read_bytes())
                static_file.write_text(yaml.safe_dump({'frame_id': frame, 'boxes': [
                    {'id': box.id, 'size': list(box.size), 'pose': list(box.pose)} for box in envelope]}, sort_keys=False))
                (archive/'support_contact.json').write_text(json.dumps({
                    'support_id': task['table_id'], 'frame_id': frame,
                    'normal': perception['support_surface_normal'],
                    'nominal_height_m': perception['support_surface_height'],
                    'height_tolerance_m': tolerance,
                    'planning_upper_height_m': perception['support_surface_height']+tolerance,
                    'independent_obstacle_per_side_margin_m': load_budget(arg('uncertainty_config'))[2],
                    'policy': 'finite depth support witness required; missing and occluded cells remain unknown',
                }, indent=2))
            cameras = [{'name': 'stage4', 'x': arg('camera_x'), 'y': arg('camera_y'), 'z': arg('camera_z'),
                        'roll': 0., 'pitch': math.pi/2, 'yaw': 0.}]
            if arg('rgbd_sensor_config'):
                cameras = yaml.safe_load(Path(arg('rgbd_sensor_config')).read_text())['cameras']
                if not isinstance(cameras, list) or not 1 <= len(cameras) <= 5:
                    raise ValueError('rgbd sensor fixture requires one to five cameras')
                for camera in cameras:
                    if set(camera) != {'name', 'x', 'y', 'z', 'roll', 'pitch', 'yaw'} or not isinstance(camera['name'], str):
                        raise ValueError('each sensor requires name, x/y/z and roll/pitch/yaw')
                    if any(isinstance(camera[k], bool) or not isinstance(camera[k], (int, float))
                           or not math.isfinite(camera[k]) for k in ('x', 'y', 'z', 'roll', 'pitch', 'yaw')):
                        raise ValueError('sensor poses must be finite numbers')
            if ({c['name']+'_camera_optical_frame' for c in cameras} != {v['camera_frame'] for v in views}
                    or len(cameras) != len(views)):
                raise ValueError('sensor fixture must match every configured perception camera')
            required_ids = ['depth_obstacles']
            extra = [
                IncludeLaunchDescription(PythonLaunchDescriptionSource(str(
                    Path(share('pick_place_perception'))/'launch/obstacles_external.launch.py')),
                    launch_arguments={'perception_config': arg('perception_config'), 'use_sim_time': 'true'}.items()),
                *[IncludeLaunchDescription(PythonLaunchDescriptionSource(str(sim/'launch/obstacle_rgbd_sensor.launch.py')),
                    launch_arguments={**{'camera_'+key: str(value) for key, value in camera.items()},
                                      'camera_update_rate': arg('rgbd_update_rate'),
                                      'camera_width': arg('rgbd_width'),
                                      'camera_height': arg('rgbd_height')}.items())
                  for camera in cameras],
            ]
    binding_files = [str(Path(arg('candidate_config')).resolve()), str(archive/'task.yaml'), str(static_file.resolve())]
    if source == 'rgbd':
        binding_files += [str(Path(arg(key)).resolve()) for key in ('perception_config', 'uncertainty_config', 'rgbd_sensor_config') if arg(key)]
        binding_files += [str(Path(path).resolve()) for path in perception.get('additional_perception_configs', [])]
        binding_files += [str(Path(perception['self_filter_model']).resolve())]
        if not perception.get('monitor_fixed_calibration', False):
            raise ValueError('stage6 fixed-camera candidates require monitor_fixed_calibration=true')
    entry = 'plan_pick_place_sim.launch.py' if blocked else 'execute_pick_place.launch.py'
    arguments = {
        'headless': arg('headless'), 'use_rviz': arg('use_rviz'),
        'wrist_camera_sensor': arg('wrist_camera_sensor'),
        'world_file': str(world), 'scene_file': str(archive / 'task.yaml'),
        # RGB-D capture needs the separately managed target before the depth
        # scene is ready. The planner still waits for SceneSync ready/revision.
        'publish_table': 'false', 'environment_ready_topic': '' if source == 'rgbd' else '/obstacles/startup_ready',
        'grasp_spin': arg('grasp_spin'),
        'candidate_config': arg('candidate_config'),
        'candidate_archive': str(archive/'candidates.yaml'),
        'planning_candidates': arg('planning_candidates'),
        'candidate_binding_files': json.dumps(binding_files),
    }
    if not blocked:
        arguments.update(auto_execute=arg('auto_execute'),
                         execution_fault_latch_file=arg('execution_fault_latch_file'))
        if source == 'rgbd':
            arguments.update(perception_ready_topic='/perception/obstacles_ready',
                             perception_heartbeat_timeout='3.0',
                             initial_state_ready_topic='/obstacles/startup_ready')
    return [
        LogInfo(msg=f'Stage 2 archive: {archive}; expected={manifest["expected"]}; '
                f'execution={"REFUSED: " + str(manifest["reason"]) if blocked else "explicit trigger"}'),
        Node(package='pick_place_scene', executable='scene_sync', output='screen', parameters=[{
            'use_sim_time': True, 'static_scene_file': str(static_file),
            **({'required_ids': required_ids, 'allowed_ids': required_ids,
                'input_ready_topic': '/perception/obstacles_ready' if source == 'rgbd' else '/obstacles/truth_ready',
                **({'max_primitives': 1024, 'uncertainty_config': arg('uncertainty_config'),
                    'max_observation_age_sec': 5.0, 'input_ready_timeout_sec': 3.0} if source == 'rgbd' else {}),
                'observation_topic': '/perception/collision_objects'} if source in ('gazebo_truth', 'rgbd') else
               {'observation_topic': '/workbench/collision_objects'}),
        }]),
        *extra,
        Node(package='pick_place_sim', executable='obstacle_preflight.py', output='screen',
             parameters=[{'use_sim_time': True, 'archive': str(archive)}]),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(sim / 'launch' / entry)),
                                 launch_arguments=arguments.items()),
    ]


def generate_launch_description():
    defaults = {
        'headless': 'false', 'use_rviz': 'true', 'auto_execute': 'false',
        'execution_fault_latch_file': os.path.expanduser('~/.ros/pick_place_execution_fault.json'),
        'workbench_file': str(Path(share('pick_place_scene')) / 'config/u_workbench.yaml'),
        'scene_file': str(Path(share('pick_place_demo')) / 'config/known_scene.yaml'),
        'obstacle_config': str(Path(share('pick_place_sim')) / 'config/obstacles.yaml'),
        'obstacle_mode': '', 'obstacle_seed': '', 'obstacle_count': '', 'benchmark': '',
        'obstacle_source': 'snapshot',
        'perception_config': '', 'uncertainty_config': '',
        'camera_x': '0.55', 'camera_y': '0', 'camera_z': '1.20',
        'rgbd_sensor_config': '',
        'rgbd_update_rate': '15',
        # External views keep the wrist collision model, without rendering an
        # unused sixth RGB-D stream. Legacy wrist-vision launches stay enabled.
        'wrist_camera_sensor': 'false',
        'rgbd_width': '640', 'rgbd_height': '480',
        'grasp_spin': '0.0', 'planning_candidates': '5',
        'candidate_config': str(Path(share('pick_place_mtc'))/'config/baseline_candidates.yaml'),
        'scene_archive': '',
    }
    return LaunchDescription([*(DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()),
                              OpaqueFunction(function=setup)])
