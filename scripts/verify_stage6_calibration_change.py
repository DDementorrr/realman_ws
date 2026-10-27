#!/usr/bin/env python3
"""Inject a changed fixed-camera calibration in the own idle Gazebo fixture.

Stops only the two own CameraInfo bridges. Actual RGB/depth acquisition keeps
running. Both registered CameraInfo messages change together, so the fixed
calibration gate must reject before geometry, rather than only pair mismatch.
Restart the isolated simulation after this destructive fault test.
"""
import argparse
import copy
import json
import os
from pathlib import Path
import signal
import time
import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool
from std_srvs.srv import Trigger


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--camera',default='stage4')
    args=parser.parse_args()
    domain=os.environ.get('ROS_DOMAIN_ID')
    if not domain:raise RuntimeError('explicit isolated ROS domain required')
    rclpy.init();n=rclpy.create_node('verify_stage6_fixed_calibration_change')
    values={};reports=[];injected=[0]
    base=f'/{args.camera}/camera'
    color=n.create_publisher(CameraInfo,base+'/color/camera_info',10)
    depth=n.create_publisher(CameraInfo,base+'/depth/camera_info',10)
    n.create_subscription(CameraInfo,base+'/color/camera_info',lambda m:values.update(color=m),10)
    n.create_subscription(CameraInfo,base+'/depth/camera_info',lambda m:values.update(depth=m),10)
    n.create_subscription(DiagnosticArray,'/perception/obstacle_status',lambda m:reports.append(m.status[0]),10)
    n.create_subscription(Bool,'/perception/obstacles_ready',lambda m:values.update(ready=m.data),10)
    execute=n.create_client(Trigger,'/pick_place/execute')
    def wait(predicate,timeout,label):
        end=time.monotonic()+timeout
        while not predicate() and time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.03)
        if not predicate():raise RuntimeError('timeout: '+label)
    result={'accepted':False}
    try:
        wait(lambda:all(k in values for k in ('color','depth','ready')) and values['ready'],10.,'valid real sensor fixture')
        templates=[copy.deepcopy(values[k]) for k in ('color','depth')]
        own=[]
        for proc in Path('/proc').iterdir():
            if not proc.name.isdigit():continue
            try:
                env=(proc/'environ').read_bytes().split(b'\0');cmd=(proc/'cmdline').read_bytes()
            except OSError:continue
            if (f'ROS_DOMAIN_ID={domain}'.encode() in env and b'parameter_bridge' in cmd
                    and f'/{args.camera}/rgbd/camera_info@'.encode() in cmd):
                own.append(int(proc.name))
        if len(own)!=2:raise RuntimeError('expected exactly the two own CameraInfo bridges: '+str(own))
        for pid in own:os.kill(pid,signal.SIGINT)
        def acquisition(image):
            for template,publisher in zip(templates,(color,depth)):
                msg=copy.deepcopy(template);msg.header.stamp=image.header.stamp
                msg.k[0]*=1.0001;msg.p[0]*=1.0001;publisher.publish(msg)
            injected[0]+=1
        n.create_subscription(Image,base+'/color/image_raw',acquisition,10)
        started=time.monotonic()
        wait(lambda:reports and 'fixed camera calibration changed' in reports[-1].message,8.,'fixed-calibration invalidation')
        wait(lambda:values.get('ready') is False,2.,'ready false')
        if not execute.wait_for_service(timeout_sec=3.):raise RuntimeError('execute service unavailable')
        f=execute.call_async(Trigger.Request());wait(f.done,6.,'refusal')
        response=f.result()
        if response.success:raise RuntimeError('execution accepted changed calibration')
        result.update(accepted=True,latency_sec=time.monotonic()-started,bridge_pids=own,injected_acquisitions=injected[0],
                      diagnostic=reports[-1].message,execute_refusal=response.message)
        print('PASS real fixed-camera calibration change invalidates readiness and refuses execution',flush=True)
    except Exception as error:
        result['error']=str(error);print('FAIL '+str(error),flush=True)
    finally:
        args.output.write_text(json.dumps(result,indent=2));n.destroy_node();rclpy.shutdown()
    return 0 if result['accepted'] else 1

if __name__=='__main__':raise SystemExit(main())
