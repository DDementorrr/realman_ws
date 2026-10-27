#!/usr/bin/env bash
set -euo pipefail
workspace_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export PYTHONPATH="$workspace_root/src/ros2_adapters/pick_place_runtime:${PYTHONPATH:-}"
exec /usr/bin/python3 "$workspace_root/scripts/test_execution_protocol.py" -v
