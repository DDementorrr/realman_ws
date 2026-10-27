#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
workspace_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export PYTHONPATH="$workspace_root/src/ros2_adapters/pick_place_scene:$workspace_root/src/ros2_adapters/pick_place_perception:${PYTHONPATH:-}"
export ROS_DOMAIN_ID=173
export ROS_LOCALHOST_ONLY=1
exec /usr/bin/python3 "$workspace_root/scripts/test_scene_sync.py" -v
