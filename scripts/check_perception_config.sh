#!/usr/bin/env bash
set -eo pipefail
workspace_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
export PYTHONPATH="$workspace_root/src/ros2_adapters/pick_place_perception:${PYTHONPATH:-}"
exec /usr/bin/python3 "$workspace_root/scripts/test_perception_config.py" -v
