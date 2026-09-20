#!/usr/bin/env bash
set -o pipefail
workspace_root="${REALMAN_WS_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
source /opt/ros/humble/setup.bash 2>/dev/null || { echo "FAIL ROS 2 Humble setup missing"; exit 1; }
source "$workspace_root/install/setup.bash" 2>/dev/null || { echo "FAIL workspace setup missing"; exit 1; }

failed=0
pass_or_fail() {
  local label="$1"
  local needle="$2"
  local value="$3"
  if grep -Fq "$needle" <<<"$value"; then
    echo "PASS $label"
  else
    echo "FAIL $label: expected '$needle'"
    failed=1
  fi
}

controllers=$(timeout 10 ros2 control list_controllers 2>&1 || true)
pass_or_fail "RM65B arm controller listed" "rm65b_arm_controller" "$controllers"
pass_or_fail "joint-state broadcaster listed" "joint_state_broadcaster" "$controllers"
active_count=$(grep -c ' active$' <<<"$controllers" || true)
if (( active_count >= 2 )); then echo "PASS controllers active"; else echo "FAIL controllers not active"; failed=1; fi

actions=$(timeout 10 ros2 action list 2>&1 || true)
pass_or_fail "MoveIt move action" "/move_action" "$actions"
pass_or_fail "trajectory action" "/rm65b_arm_controller/follow_joint_trajectory" "$actions"

joints=$(timeout 10 ros2 topic echo /joint_states --once --field name 2>&1 || true)
for joint in joint1 joint2 joint3 joint4 joint5 joint6; do
  pass_or_fail "joint state $joint" "$joint" "$joints"
done

description_info=$(timeout 10 ros2 topic info /robot_description_gazebo 2>&1 || true)
pass_or_fail "isolated Gazebo description" "Publisher count: 1" "$description_info"

gz_services=$(timeout 10 gz service -l 2>&1 || true)
pass_or_fail "Gazebo world control" "/world/empty/control" "$gz_services"

exit "$failed"
