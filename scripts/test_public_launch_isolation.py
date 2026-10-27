#!/usr/bin/env python3
"""Run public perception and scene launches without sensor or execution inputs."""

import os
import json
from contextlib import nullcontext
from pathlib import Path
import signal
import subprocess
import tempfile
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters
from rcl_interfaces.msg import ParameterType
from sensor_msgs.msg import CameraInfo, Image
from geometry_msgs.msg import PoseStamped, TransformStamped
from moveit_msgs.msg import CollisionObject
from tf2_ros import StaticTransformBroadcaster


ROOT = Path(__file__).resolve().parents[1]
STARTUP_TIMEOUT = 20.0
NO_INPUT_OBSERVATION_WINDOW = 2.0
FORBIDDEN_EXECUTABLES = {
    "gz", "gzserver", "gzclient", "realsense2_camera_node",
    "camera_node", "ros2_control_node", "controller_manager",
    "fake_joint_driver", "joint_state_publisher",
}
FORBIDDEN_NODES = {
    "/controller_manager", "/robot_state_publisher", "/joint_state_publisher",
    "/joint_state_publisher_gui", "/ros2_control_node",
}


def report(condition, message):
    print(f"{'PASS' if condition else 'FAIL'} {message}", flush=True)
    if not condition:
        raise RuntimeError(message)


def process_executable(pid):
    try:
        return Path(f"/proc/{pid}/exe").resolve().name
    except (FileNotFoundError, PermissionError, ProcessLookupError):
        return ""


def forbidden_processes(process_group):
    found = []
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit():
            continue
        try:
            stat = (entry / "stat").read_text(encoding="utf-8")
            fields = stat[stat.rfind(")") + 2:].split()
            session_id = int(fields[3])
        except (OSError, IndexError, ValueError):
            continue
        if session_id != process_group:
            continue
        executable = process_executable(entry.name)
        if executable in FORBIDDEN_EXECUTABLES:
            found.append(f"{entry.name}:{executable}")
    return found


def terminate(process):
    if process.poll() is not None:
        return
    try:
        os.killpg(process.pid, signal.SIGINT)
        process.wait(timeout=8)
    except (ProcessLookupError, subprocess.TimeoutExpired):
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=5)


def start_launch(package, launch_file, config, log_path):
    command = [
        "ros2", "launch", package, launch_file,
        f"{config}_config:={log_path.parent / (config + '.yaml')}",
        "use_sim_time:=false",
    ]
    log = log_path.open("w", encoding="utf-8")
    process = subprocess.Popen(
        command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT,
        start_new_session=True, env=os.environ.copy(),
    )
    return process, log


def wait_for_node(observer, expected, process, log_path):
    deadline = time.monotonic() + STARTUP_TIMEOUT
    while time.monotonic() < deadline:
        if process.poll() is not None:
            raise RuntimeError(f"launch exited with {process.returncode}: {log_path.read_text(errors='replace')}")
        names = {name if name.startswith("/") else f"/{name}" for name in observer.get_node_names()}
        if expected in names:
            return names
        rclpy.spin_once(observer, timeout_sec=0.1)
    raise RuntimeError(f"timed out waiting for {expected}: {log_path.read_text(errors='replace')}")


def save_graph(observer, path, process):
    path.write_text(json.dumps({
        'nodes': observer.get_node_names_and_namespaces(),
        'services': observer.get_service_names_and_types(),
        'topics': observer.get_topic_names_and_types(),
        'forbidden_processes': forbidden_processes(process.pid),
    }, indent=2))


def probe_perception(observer):
    """Synthetic standard ROS messages prove wiring, not real-camera accuracy."""
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    canvas = np.full((480, 640), 255, dtype=np.uint8)
    canvas[190:290, 270:370] = cv2.aruco.drawMarker(dictionary, 0, 100)
    rgb_data = np.repeat(canvas[:, :, None], 3, axis=2).tobytes()
    publishers = [observer.create_publisher(kind, topic, 10) for kind, topic in (
        (Image, '/camera/camera/color/image_raw'),
        (Image, '/camera/camera/depth/image_rect_raw'),
        (CameraInfo, '/camera/camera/color/camera_info'))]
    received = []
    subscription = observer.create_subscription(PoseStamped, '/object_pose', received.append, 10)
    broadcaster = StaticTransformBroadcaster(observer)
    transform = TransformStamped()
    transform.header.frame_id = 'base_link'
    transform.child_frame_id = 'camera_color_optical_frame'
    transform.transform.rotation.w = 1.0
    broadcaster.sendTransform(transform)
    try:
        deadline = time.monotonic() + 8.0
        while not received and time.monotonic() < deadline:
            rgb = Image(height=480, width=640, encoding='rgb8', step=640*3, data=rgb_data)
            rgb.header.frame_id = 'camera_color_optical_frame'
            rgb.header.stamp = observer.get_clock().now().to_msg()
            depth = Image(height=480, width=640, encoding='32FC1', step=640*4,
                          data=np.full((480, 640), 0.5, dtype='<f4').tobytes())
            depth.header = rgb.header
            info = CameraInfo(height=480, width=640)
            info.header = rgb.header
            info.k = [500., 0., 320., 0., 500., 240., 0., 0., 1.]
            # CameraInfo and depth must arrive before the RGB trigger.
            publishers[2].publish(info)
            publishers[1].publish(depth)
            publishers[0].publish(rgb)
            rclpy.spin_once(observer, timeout_sec=0.1)
        report(bool(received), 'perception: external RGB/Depth/CameraInfo/TF produce a pose')
        pose = received[-1]
        report(pose.header.frame_id == 'base_link' and
               abs(pose.pose.position.x) < 0.005 and abs(pose.pose.position.y) < 0.005 and
               abs(pose.pose.position.z - 0.39) < 0.005,
               'perception: synthetic pose matches the configured offset within 5 mm')
    finally:
        observer.destroy_subscription(subscription)
        for publisher in publishers:
            observer.destroy_publisher(publisher)


def launch_case(package, launch_file, config, expected_node, config_text, observer,
                forbidden_topics, log_dir):
    before = {topic: count[0] for topic, count in forbidden_topics.items()}
    config_path = log_dir / f"{config}.yaml"
    config_path.write_text(config_text, encoding="utf-8")
    log_path = log_dir / f"{config}.log"
    process, log = start_launch(package, launch_file, config, log_path)
    try:
        names = wait_for_node(observer, expected_node, process, log_path)
        report(True, f"{config}: independent launch exposes {expected_node}")
        unexpected_nodes = sorted(names & FORBIDDEN_NODES)
        report(not unexpected_nodes, f"{config}: no controller or fake-state ROS nodes" if not unexpected_nodes
               else f"{config}: forbidden ROS nodes {unexpected_nodes}")
        unexpected_processes = forbidden_processes(process.pid)
        report(not unexpected_processes, f"{config}: no Gazebo, camera or controller processes"
               if not unexpected_processes else f"{config}: forbidden processes {unexpected_processes}")
        observe_until = time.monotonic() + NO_INPUT_OBSERVATION_WINDOW
        while time.monotonic() < observe_until:
            rclpy.spin_once(observer, timeout_sec=0.1)
        for topic, count in forbidden_topics.items():
            delta = count[0] - before[topic]
            report(delta == 0, f"{config}: no fabricated data on {topic}"
                   if delta == 0 else f"{config}: received {delta} unexpected message(s) on {topic}")
        report(process.poll() is None, f"{config}: remains bounded-waiting for external inputs/services")
        save_graph(observer, log_dir / f'{config}-graph.json', process)
        if config == 'perception':
            probe_perception(observer)
    finally:
        terminate(process)
        log.close()
    # Drain replies queued by the synthetic input probe before the next case.
    drain_until = time.monotonic() + 0.5
    while time.monotonic() < drain_until:
        rclpy.spin_once(observer, timeout_sec=0.05)
    report(process.poll() is not None, f"{config}: launch process cleaned up")


def planner_case(observer, log_dir):
    """Valid external model, no poses/scene: wait then fail without any backend."""
    log_path = log_dir / 'planning.log'
    with log_path.open('w') as log:
        process = subprocess.Popen([
            'ros2', 'launch', 'pick_place_mtc', 'plan_pick_place.launch.py',
            f'planning_config:={log_dir / "planning.yaml"}',
            f'task_config:={log_dir / "task.yaml"}', 'use_sim_time:=false',
        ], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            names = wait_for_node(observer, '/plan_pick_place', process, log_path)
            parameters = observer.create_client(GetParameters, '/plan_pick_place/get_parameters')
            try:
                if not parameters.wait_for_service(timeout_sec=3):
                    raise RuntimeError('planner parameter service unavailable')
                request = GetParameters.Request(names=['use_sim_time'])
                result = parameters.call_async(request)
                rclpy.spin_until_future_complete(observer, result, timeout_sec=3)
                report(result.done() and result.result() is not None and
                       len(result.result().values) == 1 and
                       result.result().values[0].type == ParameterType.PARAMETER_BOOL and
                       not result.result().values[0].bool_value,
                       'planning: explicit clock argument overrides name-scoped task YAML')
            finally:
                observer.destroy_client(parameters)
            report(not names.intersection(FORBIDDEN_NODES), 'planning: no state/control nodes')
            report(not forbidden_processes(process.pid), 'planning: no backend processes')
            save_graph(observer, log_dir / 'planning-graph.json', process)
            process.wait(timeout=15)
            output = log_path.read_text()
            report('Timed out waiting for known poses' in output and
                   'Complete MTC solution published' not in output,
                   'planning: missing external inputs cause bounded failure without a solution')
        finally:
            terminate(process)


def main():
    rclpy.init()
    observer = Node("public_launch_isolation_observer")
    messages = {"/object_pose": [0], "/perception/collision_objects": [0]}
    observer.create_subscription(PoseStamped, "/object_pose",
                                 lambda _: messages["/object_pose"].__setitem__(0, messages["/object_pose"][0] + 1), 10)
    observer.create_subscription(CollisionObject, "/perception/collision_objects",
                                 lambda _: messages["/perception/collision_objects"].__setitem__(
                                     0, messages["/perception/collision_objects"][0] + 1), 10)
    # Explicitly subscribe to sensor topics so this process cannot be mistaken for an input source.
    observer.create_subscription(Image, "/camera/camera/color/image_raw", lambda _: None, 10)
    observer.create_subscription(Image, "/camera/camera/depth/image_rect_raw", lambda _: None, 10)
    observer.create_subscription(CameraInfo, "/camera/camera/color/camera_info", lambda _: None, 10)

    try:
        evidence = os.environ.get('STAGE0_EVIDENCE_DIR')
        context = nullcontext(evidence) if evidence else tempfile.TemporaryDirectory(prefix="pick-place-g0-2-")
        with context as temporary:
            log_dir = Path(temporary)
            perception_config = """/**:\n  ros__parameters:\n    marker_id: 0\n    marker_size: 0.045\n    marker_center_offset: [0.0, 0.0, -0.11]\n    object_size: [0.05, 0.07, 0.22]\n    object_id: pick_object\n    camera_frame: camera_color_optical_frame\n    output_frame: base_link\n    rgb_image_topic: /camera/camera/color/image_raw\n    depth_image_topic: /camera/camera/depth/image_rect_raw\n    camera_info_topic: /camera/camera/color/camera_info\n    depth_encoding: 32FC1\n    depth_scale: 1.0\n    max_time_skew: 0.1\n"""
            scene_config = """/**:\n  ros__parameters:\n    observation_topic: /perception/collision_objects\n    apply_service: /apply_planning_scene\n    get_scene_service: /get_planning_scene\n    allowed_ids: [pick_object]\n    max_observation_age_sec: 1.0\n    service_timeout_sec: 1.0\n"""
            launch_case("pick_place_perception", "perception_external.launch.py", "perception",
                        "/aruco_object_pose", perception_config, observer,
                        {"/object_pose": messages["/object_pose"],
                         "/perception/collision_objects": messages["/perception/collision_objects"]}, log_dir)
            launch_case("pick_place_scene", "scene_sync_external.launch.py", "scene",
                        "/pick_place_scene_sync", scene_config, observer,
                        {"/object_pose": messages["/object_pose"],
                         "/perception/collision_objects": messages["/perception/collision_objects"]}, log_dir)
            if (log_dir / 'planning.yaml').is_file():
                planner_case(observer, log_dir)
    finally:
        observer.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    try:
        main()
    except Exception as error:
        print(f"FAIL public launch isolation: {error}", flush=True)
        raise SystemExit(1)
