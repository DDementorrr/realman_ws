#!/usr/bin/env python3
"""Real rclpy action/adapter integration with a MOCK execution server.

Exercises the production task loop, heartbeat callback, coordinator, cancellation
result and persistent latch. No Gazebo or hardware execution is claimed.
"""
from pathlib import Path
import sys
import tempfile
import threading
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from moveit_task_constructor_msgs.action import ExecuteTaskSolution
from pick_place_runtime import ExecutionState

from test_ros_action_backend_rclpy import MockExecuteTaskServer, wait_until
from execute_pick_place import PickPlaceExecution


def run_case(mode):
    with tempfile.TemporaryDirectory() as root:
        latch = Path(root)/'fault.json'
        rclpy.init(args=['--ros-args', '-p', 'perception_ready_topic:=/stage5/perception_ready',
                        '-p', 'perception_heartbeat_timeout:=0.5', '-p', f'execution_fault_latch_file:={latch}'])
        execution, source, server = PickPlaceExecution(), Node('stage5_gate_source'), MockExecuteTaskServer()
        server.mode = 'reject_cancel' if mode == 'reject_cancel' else 'accept_cancel'
        execution.coordinator.cancel_timeout = .4
        execution.execution_backend._goal_builder = lambda segment: ExecuteTaskSolution.Goal()
        execution.stage_segments = lambda: ([object()], [object()], [object()], [object()])
        execution.verify_pick_object = lambda label: None
        execution.scene_revision_seen = True
        execution.scene_revision = execution.active_candidate_revision = 1
        execution.scene_ready = True
        execution.busy = True
        publisher = source.create_publisher(Bool, '/stage5/perception_ready', 10)
        statuses = []
        source.create_subscription(String, '/pick_place/execution_status', lambda m: statuses.append(m.data), 10)
        executor = MultiThreadedExecutor(num_threads=6)
        for node in (execution, source, server):
            executor.add_node(node)
        spin = threading.Thread(target=executor.spin, daemon=True)
        spin.start()
        worker = None
        try:
            wait_until(lambda: publisher.get_subscription_count() == 1, 5, 'production heartbeat subscription')
            publisher.publish(Bool(data=False))
            wait_until(lambda: execution.input_gate.received_at is not None, 2, 'false heartbeat')
            assert not execution.ready()
            assert execution.readiness_detail() == 'perception_input_not_ready'
            publisher.publish(Bool(data=True))
            wait_until(lambda: execution.input_gate.status()[0], 2, 'valid input')
            assert execution.input_gate.begin()[0]
            worker = threading.Thread(target=execution.run_task)
            worker.start()
            wait_until(lambda: server.goal_count == 1, 3, 'active mock action')
            if mode != 'producer_lost':
                publisher.publish(Bool(data=False))
            worker.join(timeout=4)
            assert not worker.is_alive(), 'task did not finish after stop/fault'
            assert server.cancel_count == 1, server.cancel_count
            assert server.goal_count == 1, server.goal_count
            assert latch.is_file(), statuses
            if mode == 'reject_cancel':
                assert execution.coordinator.state is ExecutionState.BLOCKED
                wait_until(lambda: any('cancel_unconfirmed' in s for s in statuses), 2, 'blocked diagnostic delivery')
                assert any('cancel_unconfirmed' in s for s in statuses), statuses
            else:
                assert execution.coordinator.state is ExecutionState.FAILED
                wait_until(lambda: any('cancelled: stop acknowledged' in s for s in statuses), 2, 'terminal diagnostic delivery')
                assert any('cancelled: stop acknowledged' in s for s in statuses), statuses
            assert not execution.trigger(None, Trigger.Response()).success
            assert not execution.reset(None, Trigger.Response()).success
            print(f'PASS {mode}: production task loop cancels once, waits for result, starts no next goal and latches execute/reset', flush=True)
        finally:
            server.release.set()
            if server.goal_count:
                wait_until(lambda: server.finished_count == server.goal_count, 3, 'mock server terminal cleanup')
            if worker:
                worker.join(timeout=4)
            executor.shutdown()
            spin.join(timeout=2)
            for node in (execution, source, server):
                node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    for mode in ('input_invalid', 'producer_lost', 'reject_cancel'):
        run_case(mode)
