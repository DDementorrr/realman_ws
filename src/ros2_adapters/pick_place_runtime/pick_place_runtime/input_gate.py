"""Steady-time input lease shared by execution consumers; no ROS dependency.

A lost lease stays invalid for the current task even if the producer recovers.
The caller must request cancellation and confirm the backend result.
"""
import math
import time


class InputGate:
    def __init__(self, enabled=False, timeout=1., clock=time.monotonic):
        if isinstance(timeout, bool) or not math.isfinite(timeout) or timeout <= 0:
            raise ValueError('input gate timeout must be finite and positive')
        self.enabled, self.timeout, self.clock = enabled, timeout, clock
        self.received_at = None
        self.value = False
        self.active = False
        self.failure = None

    def update(self, value):
        # Check the old lease before replacing its receipt time.
        self.status()
        self.value, self.received_at = bool(value), self.clock()
        if self.enabled and self.active and not value:
            self.failure = 'perception_invalid_during_task'

    def status(self):
        if not self.enabled:
            return True, 'input_gate_disabled_legacy_mode'
        if self.failure:
            return False, self.failure
        if self.received_at is None:
            reason = 'perception_input_missing'
        elif self.clock()-self.received_at > self.timeout:
            reason = 'perception_heartbeat_expired'
        elif not self.value:
            reason = 'perception_input_not_ready'
        else:
            return True, 'perception_input_current'
        if self.active:
            self.failure = reason
        return False, reason

    def begin(self):
        allowed, reason = self.status()
        if allowed:
            self.active = True
        return allowed, reason

    def end(self):
        self.active = False
        # No automatic recovery after an active task has lost its lease.
