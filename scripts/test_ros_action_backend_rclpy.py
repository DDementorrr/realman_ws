#!/usr/bin/env python3
"""Exercise the ROS action adapter against an in-process mock action server."""

import threading
import time
import sys
import tempfile
from pathlib import Path

import rclpy
from action_msgs.msg import GoalStatus
from moveit_msgs.msg import MoveItErrorCodes
from moveit_task_constructor_msgs.action import ExecuteTaskSolution
from pick_place_runtime import ExecutionCoordinator, ExecutionState
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/ros2_adapters/pick_place_sim/scripts"))
from execution_fault_latch import ExecutionFaultLatch
from execute_pick_place import PickPlaceExecution
from ros_action_backend import RosActionBackend


class MockExecuteTaskServer(Node):
    def __init__(self):
        super().__init__("mock_execute_task_server")
        self.mode = "success"
        self.release = threading.Event()
        self.goal_count = 0
        self.cancel_count = 0
        self.finished_count = 0
        self.action = ActionServer(
            self,
            ExecuteTaskSolution,
            "/execute_task_solution",
            execute_callback=self.execute,
            goal_callback=self.goal,
            cancel_callback=self.cancel,
            callback_group=ReentrantCallbackGroup(),
        )

    def goal(self, _goal):
        return GoalResponse.ACCEPT

    def cancel(self, _goal_handle):
        self.cancel_count += 1
        if self.mode == "reject_cancel":
            return CancelResponse.REJECT
        return CancelResponse.ACCEPT

    def execute(self, goal_handle):
        self.goal_count += 1
        result = ExecuteTaskSolution.Result()
        if self.mode == "success":
            result.error_code.val = MoveItErrorCodes.SUCCESS
            goal_handle.succeed()
            self.finished_count += 1
            return result

        deadline = time.monotonic() + 3.0
        while (rclpy.ok() and not goal_handle.is_cancel_requested
               and not self.release.is_set() and time.monotonic() < deadline):
            time.sleep(0.01)
        if goal_handle.is_cancel_requested:
            result.error_code.val = MoveItErrorCodes.FAILURE
            goal_handle.canceled()
        elif self.release.is_set():
            result.error_code.val = MoveItErrorCodes.SUCCESS
            goal_handle.succeed()
        else:
            result.error_code.val = MoveItErrorCodes.FAILURE
            goal_handle.abort()
        self.finished_count += 1
        return result


class MockExecuteTaskClient(Node):
    def __init__(self):
        super().__init__("mock_execute_task_client")
        self.action = ActionClient(self, ExecuteTaskSolution, "/execute_task_solution")


def wait_until(predicate, timeout, label):
    deadline = time.monotonic() + timeout
    while rclpy.ok() and time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.01)
    raise AssertionError(f"timed out waiting for {label}")


def tick_until(coordinator, states, timeout):
    deadline = time.monotonic() + timeout
    event = None
    while rclpy.ok() and time.monotonic() < deadline:
        event = coordinator.tick()
        if event.state in states:
            return event
        time.sleep(0.01)
    raise AssertionError(f"coordinator did not reach {states}; last event={event!r}")


def run():
    temp_dir = tempfile.TemporaryDirectory()
    latch_path = Path(temp_dir.name) / "fault.json"
    ExecutionFaultLatch(latch_path).latch("persisted action outcome unknown")
    rclpy.init(args=["--ros-args", "-p", f"execution_fault_latch_file:={latch_path}"])
    server = MockExecuteTaskServer()
    client = MockExecuteTaskClient()
    execution_node = PickPlaceExecution()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(server)
    executor.add_node(client)
    executor.add_node(execution_node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    try:
        backend = RosActionBackend(client.action, lambda segment: ExecuteTaskSolution.Goal())

        server.mode = "success"
        success = ExecutionCoordinator(backend, segment_timeout=2.0, cancel_timeout=0.5)
        event = success.start(["one", "two"], candidate_revision=1, current_revision=1)
        assert event.state is ExecutionState.RUNNING
        event = tick_until(success, {ExecutionState.SUCCEEDED, ExecutionState.FAILED,
                                     ExecutionState.BLOCKED}, 3.0)
        assert event.state is ExecutionState.SUCCEEDED, event
        assert server.goal_count == 2, server.goal_count
        print("PASS rclpy action success advances only after terminal SUCCEEDED results")

        server.mode = "accept_cancel"
        server.release.clear()
        accepted_cancel = ExecutionCoordinator(
            backend, segment_timeout=2.0, cancel_timeout=0.75)
        event = accepted_cancel.start(
            ["one", "two"], candidate_revision=2, current_revision=2)
        assert event.state is ExecutionState.RUNNING
        accepted_cancel.cancel()
        event = tick_until(accepted_cancel,
                           {ExecutionState.SUCCEEDED, ExecutionState.FAILED,
                            ExecutionState.BLOCKED}, 3.0)
        assert event.state is ExecutionState.FAILED and event.code == "cancelled", event
        assert server.goal_count == 3, server.goal_count
        assert server.cancel_count >= 1
        print("PASS accepted cancellation waits for terminal CANCELED and starts no next segment")

        server.mode = "reject_cancel"
        server.release.clear()
        rejected_cancel = ExecutionCoordinator(
            backend, segment_timeout=2.0, cancel_timeout=0.3)
        event = rejected_cancel.start(
            ["one", "two"], candidate_revision=3, current_revision=3)
        assert event.state is ExecutionState.RUNNING
        rejected_cancel.cancel()
        event = tick_until(rejected_cancel, {ExecutionState.BLOCKED}, 2.0)
        assert event.code == "cancel_unconfirmed", event
        assert server.goal_count == 4, server.goal_count
        print("PASS rejected cancellation remains blocked and starts no next segment")
        server.release.set()
        wait_until(lambda: server.finished_count >= 4, 2.0, "mock goal cleanup")

        response = execution_node.trigger(None, Trigger.Response())
        assert not response.success and "fault-latched" in response.message, response
        response = execution_node.reset(None, Trigger.Response())
        assert not response.success and "fault-latched" in response.message, response
        print("PASS persisted fault marker blocks execute/reset services")
    finally:
        server.release.set()
        executor.shutdown()
        spin_thread.join(timeout=2.0)
        execution_node.destroy_node()
        server.destroy_node()
        client.destroy_node()
        rclpy.shutdown()
        temp_dir.cleanup()


if __name__ == "__main__":
    run()
