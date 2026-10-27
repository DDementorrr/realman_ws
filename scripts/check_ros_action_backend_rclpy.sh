#!/usr/bin/env bash
set -eo pipefail
workspace_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$workspace_root/scripts/source_pick_place_env.sh" >/dev/null
set -u
export PYTHONPATH="$workspace_root/src/ros2_adapters/pick_place_sim/scripts:$workspace_root/src/ros2_adapters/pick_place_runtime:${PYTHONPATH:-}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$((20 + $$ % 200))}"
export ROS_LOCALHOST_ONLY=1
exec /usr/bin/python3 "$workspace_root/scripts/test_ros_action_backend_rclpy.py"
