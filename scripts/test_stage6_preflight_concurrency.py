#!/usr/bin/env python3
"""Production ROS service callback scheduling using a real archived solution.

External scene/validation are controlled test doubles; no motion is sent.
Checks that long preflight leaves /clock, joint feedback and heartbeat live,
then that a cancel during that preflight forbids task dispatch.
"""
import argparse
import copy
from pathlib import Path
import sys
import tempfile
import threading
import time

import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import PlanningScene
from moveit_msgs.srv import GetPlanningScene
from moveit_task_constructor_msgs.msg import Solution, TaskDescription
from pick_place_mtc import candidate_contract as contract
from rclpy.clock import Clock,ClockType
from rclpy.executors import MultiThreadedExecutor
from rosgraph_msgs.msg import Clock as ClockMessage
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

sys.path.insert(0, str(Path(__file__).resolve().parents[1] /
                       'src/ros2_adapters/pick_place_sim/scripts'))
from execute_pick_place import PickPlaceExecution


def run_case(report,cancel):
    with tempfile.TemporaryDirectory() as root:
        rclpy.init(args=['--ros-args','-p','use_sim_time:=true','-p','perception_ready_topic:=/preflight_test_ready',
                        '-p','perception_heartbeat_timeout:=0.5','-p',f'execution_fault_latch_file:={root}/fault.json'])
        execution=PickPlaceExecution();source=rclpy.create_node('stage6_preflight_test_source')
        b=report['binding'];execution.candidate_report=copy.deepcopy(report)
        execution.solution=contract.decoded(b['solution_cdr_hex'],Solution)
        execution.description=contract.decoded(b['description_cdr_hex'],TaskDescription)
        execution.pick_pose=contract.decoded(b['target_cdr_hex'],PoseStamped)
        execution.place_pose=contract.decoded(b['place_cdr_hex'],PoseStamped)
        execution.gazebo_pose=copy.deepcopy(execution.pick_pose);execution.sim_state='detached'
        initial=contract.decoded(b['scene_cdr_hex'],PlanningScene)
        source.create_service(GetPlanningScene,'/get_planning_scene',lambda request,response:GetPlanningScene.Response(scene=copy.deepcopy(initial)))
        ready=source.create_publisher(Bool,'/preflight_test_ready',10)
        joints=source.create_publisher(JointState,'/joint_states',10)
        clock=source.create_publisher(ClockMessage,'/clock',10)
        started=time.monotonic()
        def publish():
            elapsed=100.+time.monotonic()-started
            stamp=Time(sec=int(elapsed),nanosec=int((elapsed%1.)*1e9))
            clock.publish(ClockMessage(clock=stamp))
            msg=JointState(name=list(b['start_positions']),position=list(b['start_positions'].values()))
            msg.header.stamp=stamp;joints.publish(msg);ready.publish(Bool(data=True))
        source.create_timer(.03,publish,clock=Clock(clock_type=ClockType.STEADY_TIME))
        entered=threading.Event();dispatched=threading.Event()
        def costly_validation(report):
            entered.set();time.sleep(1.4)
        execution.verify_candidate_parameters=costly_validation
        execution.run_task=lambda:dispatched.set()
        execute=source.create_client(Trigger,'/pick_place/execute')
        stop=source.create_client(Trigger,'/pick_place/cancel')
        executor=MultiThreadedExecutor(num_threads=6)
        executor.add_node(execution);executor.add_node(source)
        spin=threading.Thread(target=executor.spin,daemon=True);spin.start()
        def wait(predicate,timeout,label):
            deadline=time.monotonic()+timeout
            while not predicate() and time.monotonic()<deadline:time.sleep(.01)
            if not predicate():raise RuntimeError('timeout: '+label)
        try:
            wait(lambda:execution.joint_feedback_ros>100. and execution.input_gate.status()[0],5.,'real callbacks warmup')
            future=execute.call_async(Trigger.Request());wait(entered.is_set,5.,'costly preflight entered')
            if cancel:
                cancellation=stop.call_async(Trigger.Request());wait(cancellation.done,1.,'cancel while service is busy')
                assert cancellation.result().success,cancellation.result()
            wait(future.done,5.,'production execute service result')
            result=future.result()
            if cancel:
                assert not result.success and 'cancelled_during_candidate_preflight' in result.message,result
                assert not dispatched.is_set()
                assert not execution.input_gate.active
                print('PASS real execute/cancel services: cancel during long preflight forbids task dispatch',flush=True)
            else:
                assert result.success,result
                wait(dispatched.is_set,1.,'selected task dispatch')
                assert execution.input_gate.status()[0]
                assert time.monotonic()-execution.joint_feedback_wall<.2
                print('PASS real execute service: 1.4s validation leaves /clock, joint feedback and .5s heartbeat lease current',flush=True)
        finally:
            executor.shutdown();spin.join(timeout=2.)
            execution.destroy_node();source.destroy_node();rclpy.shutdown()

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('archive',type=Path);args=parser.parse_args()
    report=contract.decode_report(args.archive.read_text())
    for cancel in (False,True):run_case(report,cancel)
