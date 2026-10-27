#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
task_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# The worker copy uses installed dependencies from the existing local workspace.
# All build products and probes remain in a temporary directory.
fixture_root="${REALMAN_MTC_FIXTURE_ROOT:-/home/ww/realman_ws}"
source "$fixture_root/install/setup.bash"
mtc_check_prefix="$fixture_root/.third_party/mtc/debroot/opt/ros/humble"
export AMENT_PREFIX_PATH="$mtc_check_prefix:${AMENT_PREFIX_PATH:-}"
export CMAKE_PREFIX_PATH="$mtc_check_prefix:${CMAKE_PREFIX_PATH:-}"
export LD_LIBRARY_PATH="$mtc_check_prefix/lib:${LD_LIBRARY_PATH:-}"
export ROS_DOMAIN_ID=176
export ROS_LOCALHOST_ONLY=1
export CMAKE_BUILD_PARALLEL_LEVEL=2
check_root="$(mktemp -d /tmp/realman-mtc-config.XXXXXX)"
colcon --log-base "$check_root/log" build \
  --base-paths "$task_root/src/ros2_adapters/pick_place_mtc" \
  --build-base "$check_root/build" --install-base "$check_root/install" \
  --cmake-args -DPICK_PLACE_REQUIRE_MTC=ON
/usr/bin/python3 "$task_root/scripts/test_mtc_config.py" \
  "$check_root/install/pick_place_mtc/lib/pick_place_mtc/plan_pick_place"
echo "MTC configuration check artifacts: $check_root"
