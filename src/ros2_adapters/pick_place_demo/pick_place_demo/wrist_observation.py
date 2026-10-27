"""Move to a configured workspace observation pose before accepting vision."""

import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Bool


def main():
    rclpy.init()
    node = Node('wrist_observation')
    node.declare_parameter('observation_xyz', [0.18, -0.075, 0.40])
    # Ry(pi/2) * Rz(pi/2): optical Z faces base +X while the open
    # fingers rotate out of the top marker's viewing direction.
    node.declare_parameter('observation_quaternion', [0.5, 0.5, 0.5, 0.5])
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)
    publisher = node.create_publisher(Bool, '/pick_place/observation_ready', qos)
    client = ActionClient(node, MoveGroup, '/move_action')
    hand = ActionClient(node, FollowJointTrajectory,
                        '/ctm2f110_hand_controller/follow_joint_trajectory')
    try:
        if not client.wait_for_server(timeout_sec=90):
            raise RuntimeError('MoveGroup unavailable for wrist observation')
        # The camera sits behind the fingertips: open before moving to the
        # observation pose to avoid imaging the rubber pads at close range.
        if not hand.wait_for_server(timeout_sec=30):
            raise RuntimeError('Gripper controller unavailable for observation')
        opening = FollowJointTrajectory.Goal()
        opening.trajectory.joint_names = ['ctm2f110_finger_joint']
        opening.trajectory.points = [JointTrajectoryPoint(
            positions=[0.8], time_from_start=Duration(sec=2))]
        opening_future = hand.send_goal_async(opening)
        rclpy.spin_until_future_complete(node, opening_future, timeout_sec=10)
        if (not opening_future.done() or opening_future.result() is None or
                not opening_future.result().accepted):
            raise RuntimeError('Observation gripper opening rejected')
        opening_result = opening_future.result().get_result_async()
        rclpy.spin_until_future_complete(node, opening_result, timeout_sec=15)
        if (not opening_result.done() or opening_result.result() is None or
                opening_result.result().result.error_code != 0):
            raise RuntimeError('Observation gripper opening failed')
        goal = MoveGroup.Goal()
        request = goal.request
        request.group_name = 'rm65_arm'
        request.pipeline_id = 'ompl'
        request.num_planning_attempts = 5
        request.allowed_planning_time = 15.0
        request.max_velocity_scaling_factor = 0.25
        request.max_acceleration_scaling_factor = 0.25
        request.start_state.is_diff = True
        position = PositionConstraint()
        position.header.frame_id = 'base_link'
        position.link_name = 'ctm2f110_tool0'
        position.weight = 1.0
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.005]
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = node.get_parameter('observation_xyz').value
        pose.orientation.w = 1.0
        position.constraint_region.primitives = [sphere]
        position.constraint_region.primitive_poses = [pose]
        orientation = OrientationConstraint()
        orientation.header.frame_id = 'base_link'
        orientation.link_name = position.link_name
        (orientation.orientation.x, orientation.orientation.y,
         orientation.orientation.z, orientation.orientation.w) = node.get_parameter('observation_quaternion').value
        # A few degrees of upward error can push the top marker to the
        # image edge. Keep the observation direction tighter than grasp IK.
        orientation.absolute_x_axis_tolerance = 0.01
        orientation.absolute_y_axis_tolerance = 0.01
        orientation.absolute_z_axis_tolerance = 0.01
        orientation.weight = 1.0
        constraints = Constraints()
        constraints.position_constraints = [position]
        constraints.orientation_constraints = [orientation]
        request.goal_constraints = [constraints]
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True
        # MoveGroup starts with controller startup; allow current-state monitor
        # to receive Gazebo feedback before requesting execution.
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, future, timeout_sec=30)
        if not future.done() or future.result() is None or not future.result().accepted:
            raise RuntimeError('Observation goal rejected or timed out')
        result = future.result().get_result_async()
        rclpy.spin_until_future_complete(node, result, timeout_sec=90)
        if not result.done() or result.result().result.error_code.val != 1:
            raise RuntimeError('Observation planning or execution failed')
        publisher.publish(Bool(data=True))
        node.get_logger().info('Wrist observation reached; visual localization enabled')
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError as error:
        node.get_logger().error(str(error))
        raise SystemExit(1) from error
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
