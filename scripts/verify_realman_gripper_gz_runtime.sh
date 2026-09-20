#!/usr/bin/env bash
set -eo pipefail
workspace_root="${REALMAN_WS_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
source /opt/ros/humble/setup.bash
source "$workspace_root/install/setup.bash"
set -u
exec python3 "$workspace_root/scripts/verify_realman_gripper_gz_runtime.py" "$@"
