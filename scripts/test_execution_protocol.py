#!/usr/bin/env python3
import unittest

from pick_place_runtime.execution_protocol import (
    BackendState, ExecutionCoordinator, ExecutionState, ProtocolError,
)


class Clock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now


class MockBackend:
    def __init__(self, ready=True, statuses=(), poll_errors=0, cancel_errors=0):
        self.is_ready = ready
        self.statuses = list(statuses)
        self.started = []
        self.cancel_requests = 0
        self.poll_errors = poll_errors
        self.poll_count = 0
        self.cancel_errors = cancel_errors

    def ready(self):
        return self.is_ready

    def start(self, segment):
        self.started.append(segment)

    def poll(self):
        self.poll_count += 1
        if self.poll_errors > 0:
            self.poll_errors -= 1
            raise RuntimeError("backend status unavailable")
        return self.statuses.pop(0) if self.statuses else BackendState.RUNNING

    def cancel(self):
        self.cancel_requests += 1
        if self.cancel_errors > 0:
            self.cancel_errors -= 1
            raise RuntimeError("cancel transport unavailable")


class ExecutionProtocolTest(unittest.TestCase):
    def test_missing_backend_forbids_command(self):
        coordinator = ExecutionCoordinator()
        event = coordinator.start(["one"], candidate_revision=1, current_revision=1)
        self.assertEqual(event.state, ExecutionState.FAILED)
        self.assertEqual(event.code, "no_backend")

    def test_success_requires_each_segment_ack(self):
        clock = Clock()
        backend = MockBackend(statuses=[BackendState.SUCCEEDED, BackendState.SUCCEEDED])
        coordinator = ExecutionCoordinator(backend, clock=clock)
        self.assertEqual(coordinator.start(["one", "two"], candidate_revision=4,
                                           current_revision=4).state, ExecutionState.RUNNING)
        self.assertEqual(coordinator.tick().code, "segment_started")
        self.assertEqual(backend.started, ["one", "two"])
        self.assertEqual(coordinator.tick().state, ExecutionState.SUCCEEDED)

    def test_backend_failure_stops_before_next_segment(self):
        backend = MockBackend(statuses=[BackendState.FAILED])
        coordinator = ExecutionCoordinator(backend)
        coordinator.start(["one", "two"], candidate_revision=4, current_revision=4)
        event = coordinator.tick()
        self.assertEqual(event.state, ExecutionState.FAILED)
        self.assertEqual(backend.started, ["one"])

    def test_cancel_waits_for_acknowledgement(self):
        clock = Clock()
        backend = MockBackend(statuses=[BackendState.CANCELLED])
        coordinator = ExecutionCoordinator(backend, clock=clock, cancel_timeout=2.0)
        coordinator.start(["one"], candidate_revision=4, current_revision=4)
        self.assertEqual(coordinator.cancel().state, ExecutionState.STOPPING)
        self.assertEqual(coordinator.tick().state, ExecutionState.FAILED)
        self.assertEqual(backend.cancel_requests, 1)

    def test_unconfirmed_cancel_blocks_and_never_advances(self):
        clock = Clock()
        backend = MockBackend(statuses=[BackendState.RUNNING, BackendState.RUNNING])
        coordinator = ExecutionCoordinator(backend, segment_timeout=1.0, cancel_timeout=2.0, clock=clock)
        coordinator.start(["one", "two"], candidate_revision=4, current_revision=4)
        clock.now = 2.0
        self.assertEqual(coordinator.tick().state, ExecutionState.STOPPING)
        clock.now = 5.0
        self.assertEqual(coordinator.tick().state, ExecutionState.BLOCKED)
        self.assertEqual(backend.started, ["one"])

    def test_stale_or_unready_scene_forbids_execution(self):
        backend = MockBackend()
        stale = ExecutionCoordinator(backend).start(["one"], candidate_revision=3, current_revision=4)
        self.assertEqual(stale.code, "stale_scene_revision")
        unready = ExecutionCoordinator(backend).start(["one"], candidate_revision=4,
                                                       current_revision=4, scene_ready=False)
        self.assertEqual(unready.code, "scene_not_ready")

    def test_unbound_scene_revision_requires_explicit_legacy_opt_in(self):
        backend = MockBackend()
        rejected = ExecutionCoordinator(backend).start(["one"])
        self.assertEqual(rejected.code, "scene_revision_unavailable")
        self.assertEqual(backend.started, [])

        accepted = ExecutionCoordinator(backend).start(
            ["one"], allow_unbound_scene=True, scene_ready=True)
        self.assertEqual(accepted.state, ExecutionState.RUNNING)
        self.assertEqual(backend.started, ["one"])

    def test_pre_dispatch_recheck_rejects_without_sending_a_goal(self):
        backend = MockBackend()
        event = ExecutionCoordinator(backend).start(
            ["one"], candidate_revision=4, current_revision=4,
            pre_dispatch_check=lambda: "scene changed while action server became ready")
        self.assertEqual(event.state, ExecutionState.FAILED)
        self.assertEqual(event.code, "pre_dispatch_rejected")
        self.assertIn("scene changed", event.detail)
        self.assertEqual(backend.started, [])

    def test_pre_dispatch_check_error_fails_closed_without_goal(self):
        backend = MockBackend()

        def unavailable():
            raise RuntimeError("scene monitor unavailable")

        event = ExecutionCoordinator(backend).start(
            ["one"], candidate_revision=4, current_revision=4,
            pre_dispatch_check=unavailable)
        self.assertEqual(event.state, ExecutionState.FAILED)
        self.assertEqual(event.code, "pre_dispatch_check_failed")
        self.assertEqual(backend.started, [])

    def test_status_query_loss_requests_stop_and_waits_for_ack(self):
        clock = Clock()
        backend = MockBackend(statuses=[BackendState.CANCELLED], poll_errors=1)
        coordinator = ExecutionCoordinator(backend, clock=clock)
        coordinator.start(["one", "two"], candidate_revision=4, current_revision=4)
        event = coordinator.tick()
        self.assertEqual(event.state, ExecutionState.STOPPING)
        self.assertEqual(event.code, "status_lost_stop_requested")
        self.assertEqual(backend.cancel_requests, 1)
        self.assertEqual(backend.started, ["one"])
        self.assertEqual(coordinator.tick().code, "cancelled")

    def test_invalid_status_in_running_requests_stop(self):
        clock = Clock()
        backend = MockBackend(statuses=[None, BackendState.CANCELLED])
        coordinator = ExecutionCoordinator(backend, clock=clock)
        coordinator.start(["one", "two"], candidate_revision=4, current_revision=4)
        event = coordinator.tick()
        self.assertEqual(event.state, ExecutionState.STOPPING)
        self.assertEqual(event.code, "status_invalid_stop_requested")
        self.assertIn("invalid status None", event.detail)
        self.assertEqual(backend.cancel_requests, 1)
        self.assertEqual(backend.started, ["one"])
        self.assertEqual(coordinator.tick().code, "cancelled")

    def test_invalid_stopping_status_blocks_after_timeout_without_advancing(self):
        clock = Clock()
        backend = MockBackend(statuses=[None, "cancelled", "cancelled"])
        coordinator = ExecutionCoordinator(backend, clock=clock, cancel_timeout=2.0)
        coordinator.start(["one", "two"], candidate_revision=4, current_revision=4)
        self.assertEqual(coordinator.cancel().state, ExecutionState.STOPPING)
        self.assertEqual(coordinator.tick().state, ExecutionState.STOPPING)
        clock.now = 2.0
        self.assertEqual(coordinator.tick().state, ExecutionState.STOPPING)
        clock.now = 2.0001
        event = coordinator.tick()
        self.assertEqual(event.state, ExecutionState.BLOCKED)
        self.assertEqual(event.code, "cancel_unconfirmed")
        self.assertIn("invalid backend status", event.detail)
        self.assertIn("'cancelled'", event.detail)
        self.assertEqual(backend.started, ["one"])

    def test_blocked_failure_is_latched_and_refuses_restart(self):
        clock = Clock()
        backend = MockBackend(statuses=[None])
        coordinator = ExecutionCoordinator(backend, clock=clock, cancel_timeout=1.0)
        coordinator.start(["one", "two"], candidate_revision=4, current_revision=4)
        self.assertEqual(coordinator.tick().state, ExecutionState.STOPPING)
        clock.now = 1.0001
        self.assertEqual(coordinator.tick().state, ExecutionState.BLOCKED)
        polls = backend.poll_count
        self.assertEqual(coordinator.tick().state, ExecutionState.BLOCKED)
        self.assertEqual(backend.poll_count, polls)
        with self.assertRaises(ProtocolError):
            coordinator.start(["one"], candidate_revision=4, current_revision=4)
        self.assertEqual(backend.started, ["one"])

    def test_cancel_request_failure_after_status_loss_blocks(self):
        backend = MockBackend(poll_errors=1, cancel_errors=1)
        coordinator = ExecutionCoordinator(backend)
        coordinator.start(["one", "two"], candidate_revision=4, current_revision=4)
        event = coordinator.tick()
        self.assertEqual(event.state, ExecutionState.BLOCKED)
        self.assertEqual(event.code, "cancel_request_failed")
        self.assertIn("backend state is not confirmed", event.detail)
        self.assertEqual(backend.started, ["one"])

    def test_persistent_status_errors_block_without_advancing(self):
        clock = Clock()
        backend = MockBackend(poll_errors=1000)
        coordinator = ExecutionCoordinator(
            backend, clock=clock, segment_timeout=100.0, cancel_timeout=2.0)
        coordinator.start(["one", "two"], candidate_revision=4, current_revision=4)
        self.assertEqual(coordinator.tick().state, ExecutionState.STOPPING)
        clock.now = 1.0
        event = coordinator.tick()
        self.assertEqual(event.state, ExecutionState.STOPPING)
        self.assertEqual(backend.cancel_requests, 1)
        self.assertEqual(backend.started, ["one"])
        clock.now = 2.0001
        event = coordinator.tick()
        self.assertEqual(event.state, ExecutionState.BLOCKED)
        self.assertEqual(event.code, "cancel_unconfirmed")
        self.assertIn("status query kept failing", event.detail)
        self.assertIn("unconfirmed", event.detail)
        self.assertEqual(backend.started, ["one"])

    def test_cancel_timeout_boundary_does_not_confirm_stop(self):
        clock = Clock()
        backend = MockBackend(statuses=[BackendState.RUNNING, BackendState.RUNNING])
        coordinator = ExecutionCoordinator(backend, clock=clock, cancel_timeout=2.0)
        coordinator.start(["one", "two"], candidate_revision=4, current_revision=4)
        self.assertEqual(coordinator.cancel().state, ExecutionState.STOPPING)
        clock.now = 2.0
        self.assertEqual(coordinator.tick().state, ExecutionState.STOPPING)
        clock.now = 2.0001
        event = coordinator.tick()
        self.assertEqual(event.state, ExecutionState.BLOCKED)
        self.assertEqual(event.code, "cancel_unconfirmed")
        self.assertEqual(backend.started, ["one"])

    def test_direct_cancel_request_failure_latches_blocked(self):
        backend = MockBackend(cancel_errors=1)
        coordinator = ExecutionCoordinator(backend)
        coordinator.start(["one", "two"], candidate_revision=4, current_revision=4)
        event = coordinator.cancel()
        self.assertEqual(event.state, ExecutionState.BLOCKED)
        self.assertEqual(event.code, "cancel_request_failed")
        poll_count = backend.poll_count
        self.assertEqual(coordinator.tick().state, ExecutionState.BLOCKED)
        self.assertEqual(backend.poll_count, poll_count)
        with self.assertRaises(ProtocolError):
            coordinator.start(["one"], candidate_revision=4, current_revision=4)
        self.assertEqual(backend.started, ["one"])


if __name__ == "__main__":
    unittest.main()
