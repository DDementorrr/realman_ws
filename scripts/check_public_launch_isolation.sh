#!/usr/bin/env bash
set -eo pipefail

workspace_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
if [[ ! -f "$workspace_root/install/setup.bash" ]]; then
  echo "FAIL: workspace is not built; missing install/setup.bash" >&2
  exit 2
fi
source "$workspace_root/install/setup.bash"

export ROS_DOMAIN_ID="$((10 + $$ % 200))"
export ROS_LOCALHOST_ONLY=1
exec /usr/bin/python3 "$workspace_root/scripts/test_public_launch_isolation.py"
