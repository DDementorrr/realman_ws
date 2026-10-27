"""Persistent fail-closed fault marker for the simulation execution adapter."""

import json
import os
import tempfile
from pathlib import Path


class ExecutionFaultLatch:
    """Persist ambiguous task outcomes so node restart cannot clear the fault."""

    def __init__(self, path):
        self.path = Path(path).expanduser()
        self.path.parent.mkdir(parents=True, exist_ok=True)
        probe_path = None
        try:
            with tempfile.NamedTemporaryFile(
                    mode="w", encoding="utf-8", prefix=".pick-place-latch-probe-",
                    dir=self.path.parent, delete=False) as probe:
                probe_path = Path(probe.name)
                probe.write("ready\n")
                probe.flush()
                os.fsync(probe.fileno())
        finally:
            if probe_path is not None:
                probe_path.unlink(missing_ok=True)

    def load(self):
        try:
            payload = json.loads(self.path.read_text(encoding="utf-8"))
        except FileNotFoundError:
            return None
        except (json.JSONDecodeError, UnicodeDecodeError) as error:
            raise RuntimeError("execution fault latch file is malformed") from error
        if (not isinstance(payload, dict) or payload.get("version") != 1
                or not isinstance(payload.get("reason"), str)
                or not payload["reason"].strip()):
            raise RuntimeError("execution fault latch file is malformed")
        return payload["reason"]

    def latch(self, reason):
        if not isinstance(reason, str) or not reason.strip():
            raise ValueError("fault reason must be a non-empty string")
        payload = json.dumps({"version": 1, "reason": reason}, ensure_ascii=False) + "\n"
        temp_path = None
        try:
            with tempfile.NamedTemporaryFile(
                    mode="w", encoding="utf-8", prefix=".pick-place-latch-",
                    dir=self.path.parent, delete=False) as temp:
                temp_path = Path(temp.name)
                temp.write(payload)
                temp.flush()
                os.fsync(temp.fileno())
            os.replace(temp_path, self.path)
            directory_fd = os.open(self.path.parent, os.O_RDONLY)
            try:
                os.fsync(directory_fd)
            finally:
                os.close(directory_fd)
        finally:
            if temp_path is not None:
                temp_path.unlink(missing_ok=True)
