#!/usr/bin/env python3
import sys
import unittest
import ast
import uuid
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/ros2_adapters/pick_place_runtime"))
sys.path.insert(0, str(ROOT / "src/ros2_adapters/pick_place_sim/scripts"))

from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal
from moveit_msgs.msg import MoveItErrorCodes
from pick_place_runtime import BackendState, ExecutionCoordinator, ExecutionState
from ros_action_backend import RosActionBackend


class FakeFuture:
    def __init__(self):
        self._done = False
        self._result = None
        self._error = None
        self._callbacks = []

    def done(self):
        return self._done

    def result(self):
        if not self._done:
            raise RuntimeError("future is not done")
        if self._error is not None:
            raise self._error
        return self._result

    def add_done_callback(self, callback):
        self._callbacks.append(callback)
        if self._done:
            callback(self)

    def set_result(self, result):
        self._done = True
        self._result = result
        for callback in tuple(self._callbacks):
            callback(self)

    def set_exception(self, error):
        self._done = True
        self._error = error
        for callback in tuple(self._callbacks):
            callback(self)


class FakeGoalHandle:
    def __init__(self, accepted=True):
        self.accepted = accepted
        self.goal_id = SimpleNamespace(uuid=list(uuid.uuid4().bytes))
        self.result_future = FakeFuture()
        self.cancel_future = FakeFuture()
        self.cancel_calls = 0

    def get_result_async(self):
        return self.result_future

    def cancel_goal_async(self):
        self.cancel_calls += 1
        return self.cancel_future


class FakeActionClient:
    def __init__(self):
        self.goal_futures = []
        self.goal_messages = []
        self.ready = True

    def wait_for_server(self, timeout_sec):
        return self.ready

    def send_goal_async(self, goal):
        future = FakeFuture()
        self.goal_futures.append(future)
        self.goal_messages.append(goal)
        return future


def result_response(status, error_code=MoveItErrorCodes.SUCCESS):
    return SimpleNamespace(
        status=status,
        result=SimpleNamespace(error_code=SimpleNamespace(val=error_code)),
    )


def cancel_response(goal_ids=()):
    return SimpleNamespace(
        return_code=CancelGoal.Response.ERROR_NONE,
        goals_canceling=[
            SimpleNamespace(goal_id=SimpleNamespace(uuid=goal_id.goal_id.uuid))
            for goal_id in goal_ids
        ]
    )


class RosActionBackendTest(unittest.TestCase):
    def setUp(self):
        self.action = FakeActionClient()
        self.backend = RosActionBackend(self.action, lambda segment: {"segment": segment})

    def accept(self, index=0, handle=None):
        handle = handle or FakeGoalHandle()
        self.action.goal_futures[index].set_result(handle)
        return handle

    def test_action_success_is_reported_only_from_successful_terminal_result(self):
        self.backend.start("one")
        self.assertIs(self.backend.poll(), BackendState.RUNNING)
        handle = self.accept()
        self.assertIs(self.backend.poll(), BackendState.RUNNING)
        handle.result_future.set_result(result_response(GoalStatus.STATUS_SUCCEEDED))
        self.assertIs(self.backend.poll(), BackendState.SUCCEEDED)

    def test_rejected_goal_and_aborted_or_failed_results_are_failures(self):
        self.backend.start("rejected")
        self.accept(handle=FakeGoalHandle(accepted=False))
        self.assertIs(self.backend.poll(), BackendState.FAILED)

        self.backend.start("aborted")
        aborted = self.accept(index=1)
        self.backend.poll()
        aborted.result_future.set_result(result_response(GoalStatus.STATUS_ABORTED))
        self.assertIs(self.backend.poll(), BackendState.FAILED)

        self.backend.start("bad-code")
        bad_code = self.accept(index=2)
        self.backend.poll()
        bad_code.result_future.set_result(
            result_response(GoalStatus.STATUS_SUCCEEDED, error_code=MoveItErrorCodes.FAILURE))
        self.assertIs(self.backend.poll(), BackendState.FAILED)

    def test_cancel_service_response_does_not_confirm_stop(self):
        self.backend.start("one")
        handle = self.accept()
        self.backend.poll()
        self.backend.cancel()
        handle.cancel_future.set_result(cancel_response((handle,)))
        self.assertIs(self.backend.poll(), BackendState.RUNNING)
        handle.result_future.set_result(result_response(GoalStatus.STATUS_CANCELED))
        self.assertIs(self.backend.poll(), BackendState.CANCELLED)

    def test_cancel_before_goal_acceptance_is_sent_when_goal_handle_arrives(self):
        self.backend.start("one")
        self.backend.cancel()
        handle = self.accept()
        self.assertEqual(handle.cancel_calls, 1)
        handle.cancel_future.set_result(cancel_response((handle,)))
        self.backend.poll()
        handle.result_future.set_result(result_response(GoalStatus.STATUS_CANCELED))
        self.assertIs(self.backend.poll(), BackendState.CANCELLED)

    def test_cancel_rejection_is_not_treated_as_a_stop_acknowledgement(self):
        self.backend.start("one")
        handle = self.accept()
        self.backend.poll()
        self.backend.cancel()
        handle.cancel_future.set_result(cancel_response())
        with self.assertRaisesRegex(RuntimeError, "did not accept cancellation"):
            self.backend.poll()

    def test_coordinator_latches_if_action_outcome_cannot_be_confirmed(self):
        now = [0.0]
        coordinator = ExecutionCoordinator(
            self.backend, segment_timeout=1.0, cancel_timeout=1.0,
            clock=lambda: now[0])
        started = coordinator.start(
            ["one", "two"], candidate_revision=3, current_revision=3)
        self.assertIs(started.state, ExecutionState.RUNNING)
        self.assertEqual(len(self.action.goal_futures), 1)
        self.accept()
        coordinator.tick()
        coordinator.cancel()
        handle = self.backend._goal_handle
        handle.cancel_future.set_result(cancel_response((handle,)))
        self.assertIs(coordinator.tick().state, ExecutionState.STOPPING)
        now[0] = 1.1
        event = coordinator.tick()
        self.assertIs(event.state, ExecutionState.BLOCKED)
        self.assertEqual(len(self.action.goal_futures), 1)

    def test_confirmed_cancel_fails_task_without_starting_next_segment(self):
        coordinator = ExecutionCoordinator(self.backend)
        coordinator.start(["one", "two"], candidate_revision=3, current_revision=3)
        handle = self.accept()
        coordinator.tick()
        coordinator.cancel()
        handle.cancel_future.set_result(cancel_response((handle,)))
        self.assertIs(coordinator.tick().state, ExecutionState.STOPPING)
        handle.result_future.set_result(result_response(GoalStatus.STATUS_CANCELED))
        event = coordinator.tick()
        self.assertIs(event.state, ExecutionState.FAILED)
        self.assertEqual(len(self.action.goal_futures), 1)

    def test_ros_execution_node_routes_segments_cancel_and_reset_through_latch(self):
        source_path = ROOT / "src/ros2_adapters/pick_place_sim/scripts/execute_pick_place.py"
        tree = ast.parse(source_path.read_text(encoding="utf-8"))
        node_class = next(
            item for item in tree.body
            if isinstance(item, ast.ClassDef) and item.name == "PickPlaceExecution")
        methods = {
            item.name: item for item in node_class.body
            if isinstance(item, (ast.FunctionDef, ast.AsyncFunctionDef))
        }
        execute = methods["execute_segment"]
        calls = [
            node for node in ast.walk(execute)
            if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
        ]
        self.assertTrue(any(call.func.attr == "start" and
                            isinstance(call.func.value, ast.Attribute) and
                            call.func.value.attr == "coordinator" for call in calls))
        self.assertTrue(any(call.func.attr == "tick" and
                            isinstance(call.func.value, ast.Attribute) and
                            call.func.value.attr == "coordinator" for call in calls))
        self.assertIn("BLOCKED", ast.unparse(execute))
        self.assertIn("FAILED", ast.unparse(execute))
        self.assertIn("/pick_place/cancel", ast.unparse(methods["__init__"]))
        self.assertIn("coordinator.cancel", ast.unparse(methods["cancel_execution"]))
        self.assertIn("pre_dispatch_check", ast.unparse(execute))
        self.assertIn("execution_fault", ast.unparse(methods["trigger"]))
        self.assertIn("execution_fault", ast.unparse(methods["reset"]))
        self.assertIn("ExecutionFaultLatch", source_path.read_text(encoding="utf-8"))
        self.assertIn("fault_latch.latch", ast.unparse(methods["latch_execution_fault"]))
        self.assertNotIn("fault_latch.clear", source_path.read_text(encoding="utf-8"))
        self.assertNotIn("send_goal_async", source_path.read_text(encoding="utf-8"))


if __name__ == "__main__":
    unittest.main(verbosity=2)
