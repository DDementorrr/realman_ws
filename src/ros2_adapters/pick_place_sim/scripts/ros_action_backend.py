"""Asynchronous ROS action adapter for the backend-neutral execution protocol."""

import threading

from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal
from moveit_msgs.msg import MoveItErrorCodes

from pick_place_runtime import BackendState


class RosActionBackend:
    """Adapt a rclpy action client to the runtime execution backend contract.

    A cancel service response only acknowledges receipt of the cancel request.
    This adapter reports ``CANCELLED`` only after the action result carries the
    terminal ``STATUS_CANCELED`` status.
    """

    def __init__(self, action_client, goal_builder, *, server_timeout=10.0):
        self._action = action_client
        self._goal_builder = goal_builder
        self._server_timeout = float(server_timeout)
        self._lock = threading.RLock()
        self._goal_future = None
        self._goal_handle = None
        self._result_future = None
        self._cancel_future = None
        self._cancel_requested = False
        self._cancel_attempted = False
        self._error = None
        self._terminal = None

    def ready(self):
        return self._action.wait_for_server(timeout_sec=self._server_timeout)

    def start(self, segment):
        with self._lock:
            if self._goal_future is not None:
                raise RuntimeError("previous action goal has not reached a terminal result")
            goal = self._goal_builder(segment)
            self._cancel_requested = False
            self._cancel_attempted = False
            self._error = None
            self._terminal = None
            self._goal_handle = None
            self._result_future = None
            self._cancel_future = None
            self._goal_future = self._action.send_goal_async(goal)
            self._goal_future.add_done_callback(self._on_goal_response)

    def _process_goal_response(self, future):
        if self._error is not None or self._goal_handle is not None or self._terminal is not None:
            return
        try:
            goal_handle = future.result()
        except Exception as error:  # noqa: BLE001 - preserve ambiguous transport failures
            self._error = error
            return
        if not goal_handle.accepted:
            self._terminal = BackendState.FAILED
            return
        self._goal_handle = goal_handle
        try:
            self._result_future = goal_handle.get_result_async()
            if self._cancel_requested:
                self._request_cancel()
        except Exception as error:  # noqa: BLE001 - goal may still be executing
            self._error = error

    def _on_goal_response(self, future):
        with self._lock:
            self._process_goal_response(future)

    def _request_cancel(self):
        self._cancel_requested = True
        if self._goal_handle is None or self._cancel_attempted:
            return
        self._cancel_attempted = True
        try:
            self._cancel_future = self._goal_handle.cancel_goal_async()
        except Exception as error:  # noqa: BLE001 - outcome is now uncertain
            self._error = error

    def cancel(self):
        with self._lock:
            if self._goal_future is None:
                raise RuntimeError("there is no active action goal to cancel")
            self._request_cancel()
            if self._error is not None:
                raise RuntimeError(f"action cancellation request failed: {self._error}")

    def _finish(self, state):
        self._goal_future = None
        self._goal_handle = None
        self._result_future = None
        self._cancel_future = None
        self._cancel_requested = False
        self._cancel_attempted = False
        self._error = None
        self._terminal = None
        return state

    def poll(self):
        with self._lock:
            if self._goal_future is None:
                raise RuntimeError("there is no active action goal to poll")
            if not self._goal_future.done():
                return BackendState.RUNNING
            self._process_goal_response(self._goal_future)
            if self._error is not None:
                raise RuntimeError(f"action goal state is unavailable: {self._error}")
            if self._terminal is not None:
                return self._finish(self._terminal)
            if self._cancel_future is not None and self._cancel_future.done():
                try:
                    response = self._cancel_future.result()
                    goal_uuid = bytes(self._goal_handle.goal_id.uuid)
                    accepted_ids = {
                        bytes(goal_info.goal_id.uuid)
                        for goal_info in response.goals_canceling
                    }
                    if (response.return_code != CancelGoal.Response.ERROR_NONE
                            or goal_uuid not in accepted_ids):
                        raise RuntimeError(
                            "action server did not accept cancellation for the active goal")
                    # Service acceptance begins cancellation; it does not
                    # prove motion stopped. The final result must be CANCELED.
                except Exception as error:  # noqa: BLE001 - remain fail-closed
                    self._error = error
                    raise RuntimeError(f"action cancel response is unavailable: {error}") from error
            if self._result_future is None or not self._result_future.done():
                return BackendState.RUNNING
            try:
                response = self._result_future.result()
            except Exception as error:  # noqa: BLE001 - terminal status is unknown
                self._error = error
                raise RuntimeError(f"action result is unavailable: {error}") from error
            if response.status == GoalStatus.STATUS_CANCELED:
                return self._finish(BackendState.CANCELLED)
            if response.status == GoalStatus.STATUS_ABORTED:
                return self._finish(BackendState.FAILED)
            if response.status == GoalStatus.STATUS_SUCCEEDED:
                if response.result.error_code.val == MoveItErrorCodes.SUCCESS:
                    return self._finish(BackendState.SUCCEEDED)
                return self._finish(BackendState.FAILED)
            raise RuntimeError(f"action returned non-terminal result status {response.status}")
