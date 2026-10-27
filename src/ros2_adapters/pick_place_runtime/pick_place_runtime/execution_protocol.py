"""Backend-neutral execution state machine for staged task execution.

The backend is deliberately injected. This module has no ROS, simulation, or
vendor dependency and cannot send a command when no backend is supplied.

Lost or invalid backend status is not evidence that an action stopped. The
coordinator requests cancellation, waits for an explicit ``BackendState``
acknowledgement, and latches ``BLOCKED`` if cancellation remains unconfirmed.
"""

from dataclasses import dataclass
from enum import Enum
from typing import Any, Optional, Sequence


class ProtocolError(RuntimeError):
    """A task violated the execution protocol."""


class BackendState(str, Enum):
    READY = "ready"
    RUNNING = "running"
    SUCCEEDED = "succeeded"
    FAILED = "failed"
    CANCELLED = "cancelled"


class ExecutionState(str, Enum):
    IDLE = "idle"
    RUNNING = "running"
    STOPPING = "stopping"
    SUCCEEDED = "succeeded"
    FAILED = "failed"
    BLOCKED = "blocked"


@dataclass(frozen=True)
class ExecutionEvent:
    state: ExecutionState
    segment_index: Optional[int]
    code: str
    detail: str


class ExecutionCoordinator:
    """Advance segments only after explicit backend acknowledgements.

    A backend implements ``ready()``, ``start(segment)``, ``poll()`` and
    ``cancel()``. ``allow_unbound_scene`` is reserved for callers that have
    explicitly confirmed their documented legacy no-revision mode. ``clock``
    is injected so timeout and cancellation behavior is deterministic in
    offline tests and independent of ROS time.
    """

    def __init__(self, backend: Any = None, *, segment_timeout: float = 30.0,
                 cancel_timeout: float = 5.0, clock=None):
        if segment_timeout <= 0 or cancel_timeout <= 0:
            raise ValueError("timeouts must be positive")
        self.backend = backend
        self.segment_timeout = float(segment_timeout)
        self.cancel_timeout = float(cancel_timeout)
        self.clock = clock or __import__("time").monotonic
        self.state = ExecutionState.IDLE
        self.segments: Sequence[Any] = ()
        self.index: Optional[int] = None
        self.started_at = None
        self.stop_requested_at = None
        self.events = []

    def _event(self, state, code, detail):
        self.state = state
        event = ExecutionEvent(state, self.index, code, detail)
        self.events.append(event)
        return event

    def _last_event(self):
        return self.events[-1] if self.events else self._event(
            self.state, "idle", "no active execution")

    def _request_stop(self, code, detail, cause):
        """Request cancellation without claiming the backend has stopped."""
        try:
            self.backend.cancel()
        except Exception as error:
            return self._event(
                ExecutionState.BLOCKED,
                "cancel_request_failed",
                f"{cause}: cancellation request failed: {error}; "
                "backend state is not confirmed",
            )
        self.stop_requested_at = self.clock()
        return self._event(ExecutionState.STOPPING, code, detail)

    def start(self, segments, *, candidate_revision=None, current_revision=None,
              scene_ready=True, allow_unbound_scene=False,
              pre_dispatch_check=None):
        if self.state == ExecutionState.BLOCKED:
            raise ProtocolError(
                "execution is latched BLOCKED; no explicit recovery/reset protocol exists")
        if self.state in (ExecutionState.RUNNING, ExecutionState.STOPPING):
            raise ProtocolError("execution already active")
        if not segments:
            raise ProtocolError("cannot execute an empty task")
        if not scene_ready:
            return self._event(ExecutionState.FAILED, "scene_not_ready", "planning scene is not confirmed ready")
        if candidate_revision is None and current_revision is None:
            if not allow_unbound_scene:
                return self._event(ExecutionState.FAILED, "scene_revision_unavailable",
                                   "candidate has no scene revision and legacy mode is disabled")
        elif (candidate_revision is None or current_revision is None
              or candidate_revision != current_revision):
            return self._event(ExecutionState.FAILED, "stale_scene_revision",
                               "candidate scene revision does not match the confirmed scene")
        if self.backend is None:
            return self._event(ExecutionState.FAILED, "no_backend", "execution backend is not configured")
        if not self.backend.ready():
            return self._event(ExecutionState.FAILED, "backend_not_ready", "execution backend is not ready")
        if pre_dispatch_check is not None:
            try:
                rejection = pre_dispatch_check()
            except Exception as error:
                return self._event(
                    ExecutionState.FAILED, "pre_dispatch_check_failed",
                    f"pre-dispatch safety check failed: {error}")
            if rejection:
                return self._event(
                    ExecutionState.FAILED, "pre_dispatch_rejected", str(rejection))
        self.segments = tuple(segments)
        self.index = 0
        self.started_at = self.clock()
        try:
            self.backend.start(self.segments[self.index])
        except Exception as error:
            return self._event(ExecutionState.FAILED, "start_failed", str(error))
        return self._event(ExecutionState.RUNNING, "segment_started", "segment 0 started")

    def tick(self):
        if self.state == ExecutionState.BLOCKED:
            # A later status cannot silently recover a fault whose physical
            # outcome is unknown. Recovery requires a future explicit protocol.
            return self.events[-1]
        if self.state not in (ExecutionState.RUNNING, ExecutionState.STOPPING):
            return self._last_event()
        try:
            status = self.backend.poll()
        except Exception as error:
            if self.state == ExecutionState.STOPPING:
                return self._stopping_tick(None, error)
            return self._request_stop(
                "status_lost_stop_requested",
                f"backend status query failed: {error}; cancellation requested, "
                "waiting for stop acknowledgement",
                "execution status lost",
            )
        now = self.clock()
        if self.state == ExecutionState.STOPPING:
            return self._stopping_tick(status, None)
        if not isinstance(status, BackendState):
            return self._request_stop(
                "status_invalid_stop_requested",
                f"backend returned invalid status {status!r}; cancellation requested, "
                "waiting for stop acknowledgement",
                "execution status invalid",
            )
        if status is BackendState.CANCELLED:
            return self._event(ExecutionState.FAILED, "cancelled", "stop acknowledged")
        if status is BackendState.FAILED:
            return self._event(ExecutionState.FAILED, "segment_failed", f"segment {self.index} failed")
        if status is not BackendState.SUCCEEDED:
            if now - self.started_at > self.segment_timeout:
                return self._request_stop(
                    "timeout_stop_requested",
                    "segment timeout; cancellation requested, waiting for stop acknowledgement",
                    "segment timeout",
                )
            return self.events[-1]
        if self.index + 1 == len(self.segments):
            return self._event(ExecutionState.SUCCEEDED, "task_succeeded", "all segments acknowledged")
        self.index += 1
        self.started_at = now
        try:
            self.backend.start(self.segments[self.index])
        except Exception as error:
            return self._event(ExecutionState.FAILED, "start_failed", str(error))
        return self._event(ExecutionState.RUNNING, "segment_started", f"segment {self.index} started")

    def _stopping_tick(self, status, error):
        """Only an explicit cancellation acknowledgement confirms a stop."""
        now = self.clock()
        if status is BackendState.CANCELLED:
            self.stop_requested_at = None
            return self._event(ExecutionState.FAILED, "cancelled", "stop acknowledged")
        if now - self.stop_requested_at > self.cancel_timeout:
            if error is not None:
                reason = f"stop was not acknowledged and status query kept failing: {error}"
            elif isinstance(status, BackendState):
                reason = f"stop was not acknowledged (last backend status: {status.value})"
            else:
                reason = f"stop was not acknowledged (invalid backend status: {status!r})"
            return self._event(
                ExecutionState.BLOCKED,
                "cancel_unconfirmed",
                f"{reason}; backend state and cancellation are unconfirmed",
            )
        # A status-query error or any non-CANCELLED value is not stop evidence.
        return self.events[-1]

    def cancel(self):
        if self.state == ExecutionState.BLOCKED:
            return self.events[-1]
        if self.state != ExecutionState.RUNNING:
            return self._last_event()
        return self._request_stop(
            "stop_requested",
            "waiting for stop acknowledgement",
            "cancellation requested",
        )
