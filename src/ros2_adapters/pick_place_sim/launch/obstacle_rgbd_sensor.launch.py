"""Optional external RGB-D sensor for independent obstacle-error evaluation.

Uses separate topics/optical frame, so the existing wrist sensor is unchanged.
No truth-to-perception connection, planner or execution trigger is provided.
"""
from pathlib import Path
import math
import re
import tempfile
import xml.etree.ElementTree as ET
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    def arg(key):
        return LaunchConfiguration(key).perform(context)
    name = arg('camera_name')
    if not re.fullmatch(r'[A-Za-z][A-Za-z0-9_]*', name):
        raise ValueError('camera_name must be a simple ROS frame/topic identifier')
    gazebo = Path(get_package_share_directory('robot_gazebo'))
    source = (gazebo/'worlds/pick_place_rgbd_model.sdf').read_text()
    source = source.replace('/pick_place/rgbd', f'/{name}/rgbd').replace(
        'camera_color_optical_frame', f'{name}_camera_optical_frame')
    rate = float(arg('camera_update_rate'))
    if not math.isfinite(rate) or not 1 <= rate <= 30:
        raise ValueError('camera_update_rate must be in [1, 30] Hz')
    try:
        width, height = int(arg('camera_width')), int(arg('camera_height'))
    except ValueError as error:
        raise ValueError('camera_width/camera_height must be integers') from error
    if not 16 <= width <= 4096 or not 16 <= height <= 4096:
        raise ValueError('camera_width/camera_height must be in [16, 4096]')
    tree = ET.fromstring(source)
    tree.find('model/link/sensor/update_rate').text = str(rate)
    image = tree.find('model/link/sensor/camera/image')
    image.find('width').text = str(width)
    image.find('height').text = str(height)
    with tempfile.NamedTemporaryFile(prefix='stage4-rgbd-', suffix='.sdf', mode='w', delete=False) as file:
        file.write(ET.tostring(tree, encoding='unicode'))
        model = file.name
    topics = [('image', 'Image', 'Image', 'color/image_raw'),
              ('depth_image', 'Image', 'Image', 'depth/image_rect_raw'),
              ('camera_info', 'CameraInfo', 'CameraInfo', 'color/camera_info'),
              ('camera_info', 'CameraInfo', 'CameraInfo', 'depth/camera_info')]
    image_bridges = topics[:2]
    info_bridges = topics[2:]
    return [
        Node(package='ros_gz_sim', executable='create', output='screen',
             arguments=['-world', 'empty', '-name', f'{name}_overhead', '-file', model,
                        '-x', arg('camera_x'), '-y', arg('camera_y'), '-z', arg('camera_z'),
                        '-R', arg('camera_roll'), '-P', arg('camera_pitch'), '-Y', arg('camera_yaw')]),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name=f'{name}_image_bridge',
             output='screen',
             arguments=[f'/{name}/rgbd/{source_name}@sensor_msgs/msg/{ros}[gz.msgs.{gz}'
                        for source_name, ros, gz, _ in image_bridges],
             remappings=[(f'/{name}/rgbd/{source_name}', f'/{name}/camera/{target}')
                         for source_name, _, _, target in image_bridges]),
        *[Node(package='ros_gz_bridge', executable='parameter_bridge',
               name=f'{name}_{target.split("/")[0]}_info_bridge', output='screen',
               arguments=[f'/{name}/rgbd/{source_name}@sensor_msgs/msg/{ros}[gz.msgs.{gz}'],
               remappings=[(f'/{name}/rgbd/{source_name}', f'/{name}/camera/{target}')])
          for source_name, ros, gz, target in info_bridges],
        Node(package='tf2_ros', executable='static_transform_publisher', output='screen',
             arguments=['--x', arg('camera_x'), '--y', arg('camera_y'), '--z', arg('camera_z'),
                        '--roll', arg('camera_roll'), '--pitch', arg('camera_pitch'),
                        '--yaw', arg('camera_yaw'), '--frame-id', 'world',
                        '--child-frame-id', f'{name}_camera_sensor_frame']),
        # Gazebo camera axes: +X forward, +Y left, +Z up. ROS optical:
        # +Z forward, +X right, +Y down. Compose the same fixed convention
        # after every configured sensor pose, including the original overhead.
        Node(package='tf2_ros', executable='static_transform_publisher', output='screen',
             arguments=['--roll', '-1.57079632679', '--pitch', '0', '--yaw', '-1.57079632679',
                        '--frame-id', f'{name}_camera_sensor_frame',
                        '--child-frame-id', f'{name}_camera_optical_frame']),
    ]


def generate_launch_description():
    defaults = {'camera_name': 'stage4', 'camera_x': '0.55', 'camera_y': '0', 'camera_z': '1.20',
                'camera_roll': '0', 'camera_pitch': '1.57079632679', 'camera_yaw': '0',
                'camera_update_rate': '15', 'camera_width': '640', 'camera_height': '480'}
    return LaunchDescription([*(DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()),
                              OpaqueFunction(function=setup)])
