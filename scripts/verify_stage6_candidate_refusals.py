#!/usr/bin/env python3
"""Exercise live execution-consumer bindings in an idle, isolated simulation.

Every mutation is restored. No full task is authorized by this verifier. A
separate explicit controller motion can test the measured start-state gate.
"""
import argparse
import copy
import json
import math
from pathlib import Path
import time

import rclpy
from rcl_interfaces.srv import SetParameters
from rclpy.parameter import Parameter
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from pick_place_mtc import candidate_contract as contract
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, Trigger
from trajectory_msgs.msg import JointTrajectoryPoint


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--archive',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--mutate-file',type=Path,help='Own generated bound YAML only; contents are restored')
    parser.add_argument('--move-start',action='store_true',help='Move actual simulated joint1 by .02 rad and restore it')
    args=parser.parse_args()
    if args.output.exists():raise RuntimeError('use a new result path')
    report=contract.decode_report(args.archive.read_text())
    rclpy.init();n=rclpy.create_node('verify_stage6_live_refusals');values={};results=[]
    qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    n.create_subscription(JointState,'/joint_states',lambda m:values.update(joints=dict(zip(m.name,m.position))),10)
    n.create_subscription(Bool,'/perception/obstacles_ready',lambda m:values.update(ready=m.data),qos)
    execute=n.create_client(Trigger,'/pick_place/execute')
    parameters=n.create_client(SetParameters,'/plan_pick_place/set_parameters')
    scene=n.create_client(ApplyPlanningScene,'/apply_planning_scene')
    simulator=n.create_client(SetBool,'/pick_place/sim/set_attachment')
    def spin(duration):
        end=time.monotonic()+duration
        while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.03)
    def call(client,request,timeout=15.):
        if not client.wait_for_service(timeout_sec=5.):raise RuntimeError('required service unavailable')
        f=client.call_async(request);end=time.monotonic()+timeout
        while not f.done() and time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.03)
        if not f.done() or f.result() is None:raise RuntimeError('service result missing')
        return f.result()
    def refuse(label,expected):
        response=call(execute,Trigger.Request())
        result={'case':label,'success':response.success,'message':response.message}
        results.append(result)
        if response.success or expected not in response.message:
            if response.success:
                call(n.create_client(Trigger,'/pick_place/cancel'),Trigger.Request())
            raise RuntimeError('unexpected live refusal: '+str(result))
        print('PASS '+label+': '+response.message,flush=True)
    def set_parameter(value):
        request=SetParameters.Request(parameters=[Parameter('grasp_depth',value=value).to_parameter_msg()])
        response=call(parameters,request)
        if not response.results or not all(x.successful for x in response.results):raise RuntimeError('parameter mutation refused')
    def update_object(obj):
        message=PlanningScene(is_diff=True,world=type(PlanningScene().world)(collision_objects=[obj]))
        if not call(scene,ApplyPlanningScene.Request(scene=message)).success:raise RuntimeError('scene mutation refused')
    def move(joints):
        action=ActionClient(n,FollowJointTrajectory,'/rm65b_arm_controller/follow_joint_trajectory')
        if not action.wait_for_server(timeout_sec=5.):raise RuntimeError('actual simulated arm controller unavailable')
        goal=FollowJointTrajectory.Goal();goal.trajectory.joint_names=list(joints)
        point=JointTrajectoryPoint(positions=list(joints.values()))
        point.time_from_start.sec=2;goal.trajectory.points=[point]
        f=action.send_goal_async(goal);end=time.monotonic()+5.
        while not f.done() and time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.03)
        handle=f.result()
        if not handle or not handle.accepted:raise RuntimeError('simulator did not accept start-state test motion')
        done=handle.get_result_async();end=time.monotonic()+8.
        while not done.done() and time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.03)
        if not done.done() or done.result().status!=4:raise RuntimeError('simulated start-state test motion failed')
        spin(.5);action.destroy()
    ok=False
    try:
        if not simulator.wait_for_service(timeout_sec=5.):raise RuntimeError('simulation-only fixture unavailable')
        spin(1.)
        expected=contract.expected_parameters(report,'planner_parameters')['grasp_depth'].double_value
        try:
            set_parameter(expected+.001)
            refuse('live_planner_parameter_changed','planning_parameters_changed')
        finally:set_parameter(expected)
        object=CollisionObject(id='stage6_direct_scene_change',header=type(CollisionObject().header)(frame_id='base_link'))
        pose=Pose();pose.orientation.w=1.;pose.position.x=-2.;pose.position.z=.5
        object.primitives=[SolidPrimitive(type=SolidPrimitive.BOX,dimensions=[.01,.01,.01])]
        object.primitive_poses=[pose];object.operation=CollisionObject.ADD
        try:
            update_object(object)
            refuse('direct_scene_edit_without_revision_change','planning_scene_changed')
        finally:
            object.operation=CollisionObject.REMOVE;update_object(object)
        if args.mutate_file:
            path=args.mutate_file.resolve()
            if (str(path) not in report['binding'].get('files',{})
                    or path != args.archive.resolve().parent / 'task.yaml'):
                raise RuntimeError('file mutation requires own generated bound evidence file')
            raw=path.read_bytes()
            try:
                path.write_bytes(raw+b'\n# stage6 temporary version-binding test\n')
                refuse('bound_configuration_file_changed','configuration_or_calibration_file_changed')
            finally:path.write_bytes(raw)
        if args.move_start:
            before={k:v for k,v in values['joints'].items() if k.startswith('joint')}
            moved=dict(before);moved['joint1']+=.02
            try:
                move(moved)
                if abs(values['joints']['joint1']-before['joint1'])<.015:raise RuntimeError('joint feedback did not prove actual start motion')
                refuse('actual_simulator_start_changed','start_state_mismatch')
            finally:move(before)
        ok=True
    except Exception as error:
        results.append({'error':str(error)});print('FAIL '+str(error),flush=True)
    finally:
        args.output.write_text(json.dumps({'accepted':ok,'results':results},indent=2))
        n.destroy_node();rclpy.shutdown()
    return 0 if ok else 1

if __name__=='__main__':raise SystemExit(main())
