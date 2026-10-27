#!/usr/bin/env python3
"""Observe actual multi-candidate production and optionally execute in simulation.

Truth is used only for the final simulation evaluation. This verifier never
adds truth obstacles to scoring, perception, or the selected trajectory.
"""
import argparse
import copy
import json
import math
from pathlib import Path
import time

import rclpy
from action_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PoseStamped
from diagnostic_msgs.msg import DiagnosticArray
from moveit_msgs.msg import PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene, GetStateValidity
from moveit_task_constructor_msgs.msg import Solution, TaskDescription
from pick_place_mtc import candidate_contract as contract
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--timeout',type=float,default=240.)
    parser.add_argument('--execute',action='store_true',help='Explicitly dispatch the selected fixed-attachment simulation task')
    parser.add_argument('--payload-refusal',action='store_true',help='Exercise actual consumer using one altered solution, then restore it')
    args=parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=False)
    rclpy.init(); node=rclpy.create_node('verify_stage6_candidates')
    values={}; terminals={}; statuses=[]; samples=[]; diagnostics=[]
    qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
    def status(message):
        values['status']=message.data; statuses.append(message.data)
    def diagnostic(message):
        if not message.status:return
        status=message.status[0]
        details={item.key:json.loads(item.value) for item in status.values}
        diagnostics.append({'at':time.monotonic(),'level':(int.from_bytes(status.level,'little') if isinstance(status.level,bytes) else int(status.level)),'message':status.message,
                            'confirmed_frames':details.get('monitor_frames_confirmed'),
                            'geometry_elapsed_sec':details.get('latest_geometry_elapsed_sec'),
                            'geometry_compute_sec':details.get('latest_geometry_compute_sec'),
                            'robot_occlusion_compute_sec':details.get('latest_robot_occlusion_compute_sec'),
                            'lease_expiry':details.get('lease_expiry'),
                            'occlusion_cells':details.get('robot_occlusion_cells'),
                            'model_sha256':details.get('model_sha256')})
    def actions(message):
        for item in message.status_list:
            if item.status in (4,5,6): terminals[bytes(item.goal_info.goal_id.uuid).hex()]=item.status
    node.create_subscription(String,'/pick_place/planning_metrics',lambda m:values.update(metrics=m.data),qos)
    node.create_subscription(Solution,'/pick_place/solution',lambda m:values.update(solution=m),qos)
    node.create_subscription(TaskDescription,'/pick_place/task_description',lambda m:values.update(description=m),qos)
    # Completion emits the archived identity and legacy terminal text back to
    # back. A depth-one reader can overwrite the identity before spin_once().
    status_qos=QoSProfile(depth=100,durability=DurabilityPolicy.TRANSIENT_LOCAL,
                          reliability=ReliabilityPolicy.RELIABLE)
    node.create_subscription(String,'/pick_place/execution_status',status,status_qos)
    node.create_subscription(Bool,'/pick_place/execution_success',lambda m:values.update(success=m.data),qos)
    node.create_subscription(PoseStamped,'/pick_place/sim/object_pose',lambda m:values.update(actual=m),10)
    node.create_subscription(PoseStamped,'/place_pose',lambda m:values.update(place=m),10)
    node.create_subscription(String,'/pick_place/sim/attachment_state',lambda m:values.update(attachment=m.data),qos)
    node.create_subscription(JointState,'/joint_states',lambda m:values.update(joints=dict(zip(m.name,m.position))),10)
    node.create_subscription(DiagnosticArray,"/perception/obstacle_status",diagnostic,10)
    node.create_subscription(GoalStatusArray,'/execute_task_solution/_action/status',actions,10)
    execute=node.create_client(Trigger,'/pick_place/execute')
    scene_client=node.create_client(GetPlanningScene,'/get_planning_scene')
    def spin_until(predicate,timeout,label):
        deadline=time.monotonic()+timeout
        while not predicate() and time.monotonic()<deadline: rclpy.spin_once(node,timeout_sec=.05)
        if not predicate(): raise RuntimeError('timeout: '+label+'; latest='+str(values.get('status')))
    def call(client,request):
        if not client.wait_for_service(timeout_sec=5.):raise RuntimeError('service unavailable')
        future=client.call_async(request);spin_until(future.done,10.,'service response');return future.result()
    result={'accepted':False}
    try:
        spin_until(lambda:all(k in values for k in ('metrics','solution','description','actual','place','joints')),args.timeout,'actual candidate publication')
        report=contract.decode_report(values['metrics']); contract.verify_payload(report,values['solution'],values['description'])
        (args.output/'planning-metrics.yaml').write_text(values['metrics'])
        valid=[c for c in report['candidates'] if c['accepted']]
        if len(valid)<2:raise RuntimeError('fewer than two hard-valid full-task candidates')
        chosen=report['candidates'][report['selected_index']]
        if chosen['score']!=min(c['score'] for c in valid):raise RuntimeError('selection does not minimize normalized cost')
        def summary(c):return {k:c[k] for k in ('candidate_id','profile','accepted','rejection','score','minimum_world_clearance_m','collision_checked_states') if k in c}
        result.update(selected_id=report['selected_id'],candidates=[summary(c) for c in report['candidates']],diversity=report.get('diversity'))
        request=GetPlanningScene.Request();request.components.components=1023
        contract.verify_scene(report,call(scene_client,request).scene)
        if args.payload_refusal:
            publisher=node.create_publisher(Solution,'/pick_place/solution',qos)
            spin_until(lambda:publisher.get_subscription_count()>=1,5.,'execution solution subscription')
            wrong=copy.deepcopy(values['solution'])
            for part in wrong.sub_trajectory:
                if part.trajectory.joint_trajectory.points:
                    part.trajectory.joint_trajectory.points[-1].positions[0]+=.02;break
            original=copy.deepcopy(values['solution']);publisher.publish(wrong)
            # Allow both the verifier and the production executor to receive the actual mutation.
            end=time.monotonic()+.7
            while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.05)
            refusal=call(execute,Trigger.Request());result['payload_refusal']={'success':refusal.success,'message':refusal.message}
            publisher.publish(original)
            end=time.monotonic()+.7
            while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.05)
            if refusal.success or 'payload_mismatch' not in refusal.message:raise RuntimeError('actual consumer did not refuse altered candidate payload: '+refusal.message)
            node.destroy_publisher(publisher)
        if args.execute:
            response=call(execute,Trigger.Request());result['trigger']={'success':response.success,'message':response.message}
            if not response.success:raise RuntimeError('selected candidate refused: '+response.message)
            started=time.monotonic()
            spin_until(lambda:(values.get('success') is True and values.get('status')=='Pick and Place completed')
                       or values.get('status','').startswith(('Pick and Place failed:', 'Execution fault is latched')),
                       180.,'complete fixed-attachment task')
            finished=time.monotonic()
            result['task_duration_sec']=finished-started
            task_diagnostics=[d for d in diagnostics if started<=d["at"]<=finished]
            if diagnostics:
                if not task_diagnostics or any(d["level"]!=0 for d in task_diagnostics):
                    failures=[d for d in task_diagnostics if d['level']!=0]
                    reason=failures[0]['message'] if failures else 'no diagnostic samples'
                    raise RuntimeError("perception diagnostic invalid during full task: "+reason+
                                       "; task="+str(values.get('status')))
                frames=[d["confirmed_frames"] for d in task_diagnostics]
                if frames[-1]<=frames[0]:raise RuntimeError("no fresh geometry confirmations during task")
                elapsed=[d["geometry_elapsed_sec"] for d in task_diagnostics if d["geometry_elapsed_sec"] is not None]
                result["task_perception"]={"samples":len(task_diagnostics),"confirmed_frames_start":frames[0],"confirmed_frames_end":frames[-1],
                    "peak_geometry_elapsed_sec":max(elapsed) if elapsed else None,"all_ok":True}
            if values.get('success') is not True:raise RuntimeError(values.get('status'))
            expected='Archived candidate completed: '+report['selected_id']
            spin_until(lambda:expected in statuses,5.,'original archived candidate completion identity')
            if len(terminals)!=4 or set(terminals.values())!={4}:raise RuntimeError('expected exactly four successful MTC actions: '+str(terminals))
            scene=call(scene_client,request).scene
            if values['attachment']!='detached' or scene.robot_state.attached_collision_objects:raise RuntimeError('attachment remains after full task')
            target=copy.deepcopy(values['place'].pose.position)
            if values['place'].header.frame_id=='base_link':target.z+=.15
            actual=values['actual'].pose.position
            error=math.dist((target.x,target.y,target.z),(actual.x,actual.y,actual.z))
            if error>.04:raise RuntimeError('placement error exceeds simulation gate')
            start=report['binding']['start_positions']
            home=max(abs(values['joints'][j]-start[j]) for j in start if j.startswith('joint'))
            if home>.01:raise RuntimeError('robot did not return to initial ready arm state')
            checker=node.create_client(GetStateValidity,'/check_state_validity')
            state_request=GetStateValidity.Request();state_request.robot_state=scene.robot_state
            state=call(checker,state_request)
            if not state.valid:raise RuntimeError('final robot state invalid')
            result.update(placement_error_m=error,home_joint_max_error_rad=home,terminal_actions=terminals,final_state_valid=state.valid)
        result['accepted']=True
        print('PASS actual candidate archive/selection'+('/full fixed-attachment task' if args.execute else ''),flush=True)
    except Exception as error:
        result['error']=str(error);print('FAIL '+str(error),flush=True)
    finally:
        result['statuses']=statuses
        (args.output/'perception-monitor.jsonl').write_text(''.join(json.dumps(d)+'\n' for d in diagnostics))
        (args.output/'result.json').write_text(json.dumps(result,indent=2,ensure_ascii=False))
        node.destroy_node();rclpy.shutdown()
    return 0 if result['accepted'] else 1

if __name__=='__main__':raise SystemExit(main())
