#!/usr/bin/env python3
"""Cancel one live Gazebo/MTC segment and verify the persistent task latch."""

import sys
import time
from pathlib import Path

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from std_msgs.msg import String
from std_srvs.srv import Trigger

ROOT = Path(__file__).resolve().parents[1]


def spin_until(node, predicate, timeout, label):
    deadline = time.monotonic() + timeout
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if predicate():
            return
    raise AssertionError(f"timed out waiting for {label}")


def call_service(node, client, request, timeout=5.0):
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    if not future.done() or future.result() is None:
        raise AssertionError(f"service {client.srv_name} timed out")
    return future.result()


def run():
    latch_path = Path(sys.argv[1]).expanduser()
    if latch_path.exists():
        raise AssertionError(f"fault latch already exists: {latch_path}")

    rclpy.init()
    node = rclpy.create_node("verify_pick_place_cancel_runtime")
    statuses = []
    observed_goal_ids = set()
    active_goal_seen = {"value": False}

    def on_goal_status(message):
        for item in message.status_list:
            goal_id = bytes(item.goal_info.goal_id.uuid)
            observed_goal_ids.add(goal_id)
            if item.status in (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING,
                               GoalStatus.STATUS_CANCELING):
                active_goal_seen["value"] = True

    node.create_subscription(
        String, "/pick_place/execution_status",
        lambda message: statuses.append(message.data), 10)
    node.create_subscription(
        GoalStatusArray, "/execute_task_solution/_action/status", on_goal_status, 10)
    execute = node.create_client(Trigger, "/pick_place/execute")
    cancel = node.create_client(Trigger, "/pick_place/cancel")
    reset = node.create_client(Trigger, "/pick_place/reset")
    try:
        for client in (execute, cancel, reset):
            if not client.wait_for_service(timeout_sec=20.0):
                raise AssertionError(f"required service unavailable: {client.srv_name}")

        deadline = time.monotonic() + 40.0
        started = False
        last_message = ""
        while rclpy.ok() and time.monotonic() < deadline and not started:
            response = call_service(node, execute, Trigger.Request())
            last_message = response.message
            started = response.success
            if not started:
                rclpy.spin_once(node, timeout_sec=0.25)
        if not started:
            raise AssertionError(f"execution did not become ready: {last_message}")
        print("PASS execution service accepted the ready simulation candidate")

        spin_until(node, lambda: active_goal_seen["value"], 30.0,
                   "an active ExecuteTaskSolution goal")
        response = call_service(node, cancel, Trigger.Request())
        if not response.success:
            raise AssertionError(f"cancel service rejected the request: {response.message}")
        print("PASS cancel service requested stop while the MTC action was active")

        def failed_with_confirmation():
            return any("Pick and Place failed:" in status and
                       "cancelled: stop acknowledged" in status
                       for status in statuses)

        spin_until(node, failed_with_confirmation, 25.0,
                   "task failure after terminal CANCELED acknowledgement")
        spin_until(node, lambda: latch_path.is_file(), 5.0,
                   "persistent fault marker")
        if len(observed_goal_ids) != 1:
            raise AssertionError(
                f"expected exactly one action goal before/after cancellation, "
                f"observed {len(observed_goal_ids)}")
        print("PASS final CANCELED acknowledgement stopped the task without a later goal")

        reset_response = call_service(node, reset, Trigger.Request())
        if reset_response.success or "fault-latched" not in reset_response.message:
            raise AssertionError(f"reset was not blocked by the task fault: {reset_response}")
        print("PASS persistent execution fault blocks Gazebo/PlanningScene reset")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    run()
