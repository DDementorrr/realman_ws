"""Explicit one-shot RGB-D capture, standard observations, no scene writes.

Restart for a new static scene. Dynamic updates/ownership are outside this node's
contract. A snapshot expires using both ROS time and a steady clock.
"""

from collections import deque
from concurrent.futures import ProcessPoolExecutor
import multiprocessing
import os
import json
import time

import numpy as np
from scipy.spatial.transform import Rotation, Slerp
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import CollisionObject
from rclpy.clock import Clock, ClockType
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, ReliabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener, ExtrapolationException

from .obstacle_geometry import matrix, stamp_ns, validate_bundle
from .self_filter import load_model
from .geometry_monitor import occupancy, check_frozen_coverage, joint_motion
from .multi_view import (view_configs, calculate_packets, initialize_geometry_worker,
                         geometry_payload, calculate_pose_packets)


def pose_matrix(pose):
    return matrix((pose.position.x, pose.position.y, pose.position.z),
                  (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w))


def collision_object(boxes, frame, stamp, object_id):
    obj = CollisionObject(id=object_id, operation=CollisionObject.ADD)
    obj.header.frame_id, obj.header.stamp = frame, stamp
    for box in boxes:
        obj.primitives.append(SolidPrimitive(type=SolidPrimitive.BOX, dimensions=box.size.tolist()))
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = map(float, box.center)
        pose.orientation.w = 1.
        obj.primitive_poses.append(pose)
    return obj


class GeometryPending(Exception):
    """A bounded geometry job is still running; it does not renew any lease."""


class DepthObstacles(Node):
    def __init__(self, **kwargs):
        super().__init__('depth_obstacles', automatically_declare_parameters_from_overrides=True, **kwargs)
        self.config = {name: self.get_parameter(name).value for name in self._parameters}
        self.views = view_configs(self.config)
        self.monitor_inputs = self.config.get('monitor_inputs', False)
        self.fixed_calibrations = {}
        self.frozen_coverage = None
        self.last_valid_stamp = None
        self.last_valid_wall = 0.
        self.lease_expiry = None
        self.monitor_count = 0
        self.last_robot_occlusion = []
        self.geometry_elapsed_sec = None
        self.geometry_compute_sec = None
        self.robot_occlusion_compute_sec = None
        self.last_joint_positions = None
        self.last_motion_wall = 0.
        self.models, self.joint_names, self.model_hash = load_model(self.config['self_filter_model'])
        # The independently tracked target can arrive at the same high rate
        # as JointState. A 30-message queue discarded acquisition-time poses
        # while RGB-D was still in transport, leaving only newer moving poses.
        self.histories = {'joints': deque(maxlen=1000), 'target': deque(maxlen=1000)}
        self.histories.update({name: deque(maxlen=20) for name in ('rgb', 'depth', 'info', 'depth_info')})
        self.view_histories = [self.histories]
        for _ in self.views[1:]:
            self.view_histories.append({**{name: self.histories[name] for name in ('joints', 'target')},
                                        **{name: deque(maxlen=20) for name in ('rgb', 'depth', 'info', 'depth_info')}})
        self.last_view_stamps = None
        self.geometry_worker = ProcessPoolExecutor(
            max_workers=1, mp_context=multiprocessing.get_context('spawn'),
            initializer=initialize_geometry_worker,
            initargs=(self.config['self_filter_model'], self.model_hash))
        # Spawn/import before a capture acquisition starts consuming its lease.
        self.geometry_worker.submit(os.getpid)
        self.geometry_job = None
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.pending = False
        self.finished = False
        self.last_error = 'awaiting explicit capture and external inputs'
        self.received = self.dropped = 0
        self.snapshot = None
        self.capture_wall = self.deadline = 0
        self.ready = False
        self.steady = Clock(clock_type=ClockType.STEADY_TIME)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.output = self.create_publisher(CollisionObject, '/perception/collision_objects', qos)
        self.visible = self.create_publisher(CollisionObject, '/perception/visible_obstacles', qos)
        self.quality = self.create_publisher(DiagnosticArray, '/perception/obstacle_status', qos)
        self.ready_pub = self.create_publisher(Bool, '/perception/obstacles_ready', qos)
        def sensor_qos(config):
            return (QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
                    if config.get('sensor_reliability', 'best_effort') == 'reliable'
                    else qos_profile_sensor_data)
        for key, kind, topic in (
            ('rgb', Image, 'rgb_image_topic'), ('depth', Image, 'depth_image_topic'),
            ('info', CameraInfo, 'camera_info_topic'), ('depth_info', CameraInfo, 'depth_info_topic'),
            ('joints', JointState, 'joint_states_topic'), ('target', PoseStamped, 'target_pose_topic')):
            self.create_subscription(kind, self.config[topic], lambda msg, key=key: self.receive(key, msg),
                                     sensor_qos(self.config) if key in ('rgb', 'depth', 'info', 'depth_info')
                                     else qos_profile_sensor_data)
        for config, history in zip(self.views[1:], self.view_histories[1:]):
            for key, kind, topic in (('rgb', Image, 'rgb_image_topic'), ('depth', Image, 'depth_image_topic'),
                                     ('info', CameraInfo, 'camera_info_topic'), ('depth_info', CameraInfo, 'depth_info_topic')):
                self.create_subscription(kind, config[topic],
                    lambda msg, key=key, history=history: self.receive(key, msg, history),
                    sensor_qos(config))
        self.create_service(Trigger, '/perception/capture_obstacles', self.capture)
        self.create_timer(.05, self.tick, clock=self.steady)
        self.publish_status()

    def receive(self, key, message, history=None):
        if key == 'depth':
            self.received += 1
        (self.histories if history is None else history)[key].append(message)

    def capture(self, request, response):
        if self.finished or self.pending:
            response.success, response.message = False, 'static capture already requested; restart for a new scene'
        else:
            self.pending = True
            self.deadline = time.monotonic()+10
            response.success, response.message = True, 'capture requested; check obstacle_status and obstacles_ready'
        return response

    def lookup(self, frame, stamp):
        tf = self.tf.lookup_transform(self.config['output_frame'], frame, rclpy.time.Time.from_msg(stamp)).transform
        return matrix((tf.translation.x, tf.translation.y, tf.translation.z),
                      (tf.rotation.x, tf.rotation.y, tf.rotation.z, tf.rotation.w))

    def paired_inputs(self, config, history, anchor_ns=None, span_ns=()):
        histories = {key: tuple(value) for key, value in history.items()}
        if not all(histories.values()):
            raise ValueError('missing RGB/depth/CameraInfo/joints/target')
        # Prefer the newest complete acquisition. bundles() tries older stamps
        # if acquisition-time TF is not available; no fixed frame delay or
        # latest-transform substitution is needed.
        candidates = histories['depth']
        if anchor_ns is not None:
            candidates = sorted(candidates, key=lambda m: abs(stamp_ns(m)-anchor_ns), reverse=True)
        nearest = None
        for depth in reversed(candidates):
            anchor = stamp_ns(depth)
            age = (self.get_clock().now().nanoseconds-anchor)*1e-9
            if anchor <= 0 or not 0 <= age <= config['max_input_age']:
                continue
            paired = {key: min(history, key=lambda m: abs(stamp_ns(m)-anchor)) for key, history in histories.items()}
            bracket = None
            if config.get('monitor_target_occlusion', False):
                before = [m for m in histories['target'] if stamp_ns(m) <= anchor]
                after = [m for m in histories['target'] if stamp_ns(m) >= anchor]
                if not before or not after:
                    continue  # A moving target must never be extrapolated.
                first = max(before, key=stamp_ns)
                second = min(after, key=stamp_ns)
                if first.header.frame_id != second.header.frame_id:
                    continue
                bracket = (first, second)
                paired['target'] = first
            stamps = [stamp_ns(m) for m in paired.values()]
            if bracket is not None:
                stamps.append(stamp_ns(bracket[1]))
            comparison = stamps+list(span_ns)+([] if anchor_ns is None else [anchor_ns])
            if min(stamps) > 0 and max(comparison)-min(comparison) <= config['max_time_skew']*1e9:
                nearest = paired
                if bracket is not None:
                    nearest['target_bracket'] = bracket
                break
        if nearest is None:
            raise ValueError('no fresh capture-time RGB/depth/info/joints/target bundle')
        return nearest, age, stamps

    def packet(self, config, paired):
        nearest, age, stamps = paired
        depth = nearest['depth']
        joints = nearest['joints']
        if (len(joints.name) != len(joints.position) or len(set(joints.name)) != len(joints.name)
                or not self.joint_names.issubset(joints.name) or not np.isfinite(joints.position).all()):
            raise ValueError('incomplete or invalid capture-time robot state')
        k = validate_bundle(nearest['rgb'], depth, nearest['info'], nearest['depth_info'], config)
        poses = {body.link: self.lookup(body.link, depth.header.stamp) for body in self.models}
        bodies = [body.at(poses[body.link]) for body in self.models]
        camera_pose = self.lookup(config['camera_frame'], depth.header.stamp)
        if config.get('monitor_fixed_calibration', False):
            # Explicit fixed-camera opt-in. Capture-time transforms still apply;
            # a changed calibration invalidates the frozen scene even when its
            # occupancy happens to rasterize to the same grid cells.
            from .geometry_monitor import calibration_signature
            witness = (k.copy(), camera_pose.copy(), calibration_signature(nearest["info"]),
                       calibration_signature(nearest["depth_info"]))
            key = config['camera_frame']
            previous = self.fixed_calibrations.get(key)
            if previous is not None and (
                    not np.array_equal(previous[0], witness[0])
                    or not np.allclose(previous[1], witness[1], atol=1e-9, rtol=0.)
                    or previous[2:] != witness[2:]):
                raise ValueError('fixed camera calibration changed: '+key)
            self.fixed_calibrations[key] = witness
        target_pose = self.target_pose_at(nearest, stamp_ns(depth))
        joint_positions = {name: float(position) for name, position in zip(joints.name, joints.position)
                           if name in self.joint_names}
        return depth, k, bodies, camera_pose, target_pose, age, stamps, joint_positions

    def target_pose_at(self, paired, acquisition_ns):
        target = paired['target']
        first_pose = self.lookup(target.header.frame_id, target.header.stamp) @ pose_matrix(target.pose)
        bracket = paired.get('target_bracket')
        if bracket is None or stamp_ns(bracket[0]) == stamp_ns(bracket[1]):
            return first_pose
        first, second = bracket
        fraction = (acquisition_ns-stamp_ns(first))/(stamp_ns(second)-stamp_ns(first))
        if not 0 <= fraction <= 1:
            raise ValueError('moving target pose would require extrapolation')
        second_pose = self.lookup(second.header.frame_id, second.header.stamp) @ pose_matrix(second.pose)
        # The same linear translation / shortest rotation interpolation used
        # for acquisition-time TF. Both real samples participated in the
        # global time-span gate; no later pose or static target is substituted.
        result = np.eye(4)
        result[:3, 3] = first_pose[:3, 3]+fraction*(second_pose[:3, 3]-first_pose[:3, 3])
        result[:3, :3] = Slerp([0., 1.], Rotation.from_matrix(
            np.stack((first_pose[:3, :3], second_pose[:3, :3]))))(fraction).as_matrix()
        return result

    def bundle(self, config, history, anchor_ns=None, span_ns=()):
        return self.packet(config, self.paired_inputs(config, history, anchor_ns, span_ns))

    def bundles(self):
        candidates = tuple(self.view_histories[0]['depth'])
        # The newest independently paired primary frame need not be available
        # in every other history. Search older primary anchors for one complete
        # global join; do not repeatedly renew or select an incomplete bundle.
        last_transform_error = None
        for candidate in reversed(candidates):
            try:
                first = self.paired_inputs(self.views[0], self.view_histories[0], stamp_ns(candidate))
                paired, stamps = [first], first[2]
                for config, history in zip(self.views[1:], self.view_histories[1:]):
                    inputs = self.paired_inputs(config, history, stamp_ns(first[0]['depth']), (min(stamps), max(stamps)))
                    paired.append(inputs)
                    stamps = stamps+inputs[2]
                if self.last_view_stamps is not None and any(
                        stamp_ns(inputs[0]['depth']) < previous
                        for inputs, previous in zip(paired, self.last_view_stamps)):
                    # A delayed view can make an older common join selectable
                    # again. It cannot refresh or regress a confirmed lease.
                    continue
                # No geometry/TF work until all timestamps form one join.
                return [self.packet(config, inputs) for config, inputs in zip(self.views, paired)]
            except ExtrapolationException as error:
                # Acquisition TF can lag the images by several frames. Try
                # older acquisition times, never a latest-TF substitute. A
                # lost stream cannot renew the original bounded input lease.
                last_transform_error = error
            except ValueError as error:
                if str(error) != 'no fresh capture-time RGB/depth/info/joints/target bundle':
                    raise
        if last_transform_error is not None:
            raise last_transform_error
        raise ValueError('no fresh capture-time RGB/depth/info/joints/target bundle')

    def calculate(self, packets, *, allow_motion_occlusion=False):
        return calculate_packets(packets, self.views, allow_motion_occlusion=allow_motion_occlusion)

    def motion_evidence(self, packets):
        current = packets[0][7]
        moved = joint_motion(self.last_joint_positions, current,
                             self.config.get('motion_joint_delta', .005))
        if moved:
            self.last_motion_wall = time.monotonic()
        self.last_joint_positions = current
        return moved or (self.last_motion_wall > 0 and
                         time.monotonic()-self.last_motion_wall <= self.config.get('motion_hold_sec', 2.0))

    def next_geometry(self):
        # A single process avoids Python GIL contention with high-rate ROS I/O.
        # Only geometry runs in the worker. ROS callbacks, timestamp joins, TF,
        # publications and lease decisions stay on the single executor thread.
        # At most one immutable acquisition bundle is in flight; no job queue.
        if self.geometry_job is None:
            packets = self.bundles()
            if self.ready and not all(stamp_ns(p[0]) > previous
                                      for p, previous in zip(packets, self.last_view_stamps)):
                raise GeometryPending()
            started = time.monotonic()
            baseline = self.frozen_coverage if self.ready else None
            self.geometry_job = (self.geometry_worker.submit(calculate_pose_packets, geometry_payload(packets),
                                                             self.views, baseline), packets, started)
        future, packets, started = self.geometry_job
        if not future.done():
            raise GeometryPending()
        self.geometry_job = None
        result = future.result()
        self.geometry_elapsed_sec = time.monotonic()-started
        self.geometry_compute_sec = result[3].get('geometry_compute_sec')
        self.robot_occlusion_compute_sec = result[3].get('robot_occlusion_compute_sec')
        return packets, result, started

    def expire_lease(self):
        if self.ready:
            age = (self.get_clock().now().nanoseconds-self.last_valid_stamp)*1e-9
            wall_age = time.monotonic()-self.last_valid_wall
            if not 0 <= age <= self.config['max_input_age'] or wall_age > self.config['max_input_age']:
                job = getattr(self, 'geometry_job', None)
                self.lease_expiry = {
                    'ros_age_sec': age, 'steady_age_sec': wall_age,
                    'geometry_job_age_sec': time.monotonic()-job[2] if job else None,
                    'geometry_job_done': job[0].done() if job else None,
                    'latest_input_stamps_ns': [
                        {key: stamp_ns(messages[-1]) if messages else None
                         for key, messages in history.items()}
                        for history in getattr(self, 'view_histories', [])],
                }
                self.ready = False
                self.last_error = 'static snapshot expired; restart and recapture before use'

    def tick(self):
        # Check before collecting a worker result: a late result cannot revive
        # an already expired scene. Continue publishing false once invalidated.
        self.expire_lease()
        captured_now = False
        if self.pending and time.monotonic() >= self.deadline:
            self.pending, self.finished = False, True
            self.last_error = 'capture failed: capture deadline exceeded'
        if self.pending:
            try:
                packets, result, started = self.next_geometry()
                depth = min((p[0] for p in packets), key=stamp_ns)
                age = max(p[5] for p in packets)
                observed, unknown, quality, details = result
                completed_age = max((self.get_clock().now().nanoseconds-stamp_ns(depth))*1e-9,
                                    age+time.monotonic()-started)
                if completed_age > self.config['max_input_age']:
                    raise ValueError(f'geometry expired during processing: input_age={age:.3f}s, '
                                     f'processing={time.monotonic()-started:.3f}s, '
                                     f'completed_age={completed_age:.3f}s')
                if not observed and not unknown:
                    raise ValueError('empty snapshot needs an explicit confirmed-absence scene protocol')
                # A single compound CollisionObject replaces one static snapshot
                # atomically. Support and target IDs are never removed or edited.
                self.output.publish(collision_object(observed+unknown, self.config['output_frame'],
                                                     depth.header.stamp, 'depth_obstacles'))
                if observed:
                    self.visible.publish(collision_object(observed, self.config['output_frame'],
                                                          depth.header.stamp, 'visible_depth_samples'))
                self.snapshot = {**quality, **details, 'stamp_ns': stamp_ns(depth), 'source': 'registered_rgbd',
                                 'model_sha256': self.model_hash, 'input_age_sec': age,
                                 'tf_policy': 'capture_time_only',
                                 'target_mask_padding_m': self.config['target_mask_padding'],
                                 'mesh_unknown_cells_carved': False,
                                 'processing_sec': time.monotonic()-started,
                                 'bounds': [{'lower': b.lower.tolist(), 'upper': b.upper.tolist()} for b in observed]}
                self.capture_wall = time.monotonic()
                self.last_valid_stamp = stamp_ns(depth)
                self.last_view_stamps = details['view_stamps_ns']
                self.last_valid_wall = started-age
                self.last_joint_positions = packets[0][7]
                self.last_motion_wall = 0.
                # Unknown-space changes remain fail-closed. Joint motion alone
                # cannot prove that new missing data is robot self-occlusion.
                # A future exemption needs geometric evidence for each cell.
                self.frozen_coverage = (occupancy(observed, self.config, outward=True)
                                        | occupancy(unknown, self.config))
                self.ready, self.finished, self.pending = True, True, False
                self.last_error = 'static geometry captured; unknown regions excluded'
                captured_now = True
            except GeometryPending:
                if time.monotonic() >= self.deadline:
                    self.pending, self.finished = False, True
                    self.last_error = "capture failed: geometry job exceeded capture deadline"
            except Exception as error:
                self.last_error = str(error)
                self.dropped += 1
                if time.monotonic() >= self.deadline:
                    self.pending, self.finished = False, True
                    self.last_error = 'capture failed: '+self.last_error
                    self.get_logger().error(self.last_error)
        if self.ready and self.monitor_inputs and not captured_now:
            try:
                packets, result, started = self.next_geometry()
                stamps = [stamp_ns(p[0]) for p in packets]
                depth = min((p[0] for p in packets), key=stamp_ns)
                if any(a < b for a, b in zip(stamps, self.last_view_stamps)):
                    raise ValueError('sensor acquisition clock moved backwards')
                if all(a > b for a, b in zip(stamps, self.last_view_stamps)):
                    observed, unknown, quality, details = result
                    current = (occupancy(observed, self.config, outward=True)
                               | occupancy(unknown, self.config))
                    explained = details.get('robot_occlusion_indices', [])
                    if explained:
                        # Monitoring explanation only: the frozen collision
                        # object and raw unknown geometry are never carved.
                        positive = occupancy(observed, self.config, outward=True)
                        proven = np.array(explained, dtype=int)
                        current[tuple(proven.T)] = positive[tuple(proven.T)]
                    check_frozen_coverage(self.frozen_coverage, current)
                    if max((self.get_clock().now().nanoseconds-stamp_ns(depth))*1e-9,
                           max(p[5] for p in packets)+time.monotonic()-started) > self.config['max_input_age']:
                        raise ValueError('monitored frame expired during processing')
                    self.last_valid_stamp = stamp_ns(depth)
                    self.last_valid_wall = started-max(p[5] for p in packets)
                    self.last_view_stamps = stamps
                    self.last_robot_occlusion = explained
                    self.monitor_count += 1
            except GeometryPending:
                pass  # In-flight work never advances the existing input lease.
            except Exception as error:
                if str(error) == 'no fresh capture-time RGB/depth/info/joints/target bundle':
                    # Asynchronous delivery can temporarily leave no paired
                    # frame. Keep only the existing lease; the expiry check
                    # below still invalidates a stalled producer in steady time.
                    self.last_error = 'waiting for paired monitor frame within existing input lease'
                    error = None
                # Once invalidated, never silently restore a task's old scene.
                if error is not None:
                    self.ready = False
                    self.last_error = 'monitor invalidated: '+str(error)
                    self.get_logger().warning(self.last_error)
        self.expire_lease()
        self.publish_status()

    def publish_status(self):
        self.ready_pub.publish(Bool(data=self.ready))
        status = DiagnosticStatus(name='rgbd_obstacle_geometry', hardware_id='external_rgbd',
                                  level=DiagnosticStatus.OK if self.ready else DiagnosticStatus.WARN,
                                  message=self.last_error)
        values = {'received_depth': self.received, 'rejected_attempts': self.dropped,
                  'static_only': True, 'monitor_inputs': self.monitor_inputs,
                  'monitor_frames_confirmed': self.monitor_count, 'last_valid_stamp_ns': self.last_valid_stamp,
                  'robot_occlusion_cells': len(self.last_robot_occlusion),
                  'known_target_occlusion_enabled': self.config.get('monitor_target_occlusion', False),
                  'robot_occlusion_sample_indices': self.last_robot_occlusion[:8],
                  'latest_geometry_elapsed_sec': self.geometry_elapsed_sec,
                  'latest_geometry_compute_sec': self.geometry_compute_sec,
                  'latest_robot_occlusion_compute_sec': self.robot_occlusion_compute_sec,
                  'lease_expiry': self.lease_expiry,
                  **(self.snapshot or {})}
        status.values = [KeyValue(key=key, value=json.dumps(value, allow_nan=False)) for key, value in values.items()]
        message = DiagnosticArray(status=[status])
        message.header.stamp = self.get_clock().now().to_msg()
        self.quality.publish(message)


def main():
    rclpy.init()
    node = DepthObstacles()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.geometry_worker.shutdown(wait=True, cancel_futures=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
