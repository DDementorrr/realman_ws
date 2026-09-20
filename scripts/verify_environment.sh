#!/usr/bin/env bash
set -o pipefail
workspace_root="${REALMAN_WS_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
source /opt/ros/humble/setup.bash 2>/dev/null || { echo "FAIL: /opt/ros/humble/setup.bash missing"; exit 1; }
workspace_setup="$workspace_root/install/setup.bash"
if [[ -f "$workspace_setup" ]]; then
  source "$workspace_setup"
else
  echo "WARN workspace is not built: $workspace_setup"
fi
checks=(ros2 colcon cmake gz rviz2)
failed=0
for c in "${checks[@]}"; do
  if command -v "$c" >/dev/null 2>&1; then printf 'PASS %-8s %s\n' "$c" "$(command -v "$c")"; else printf 'FAIL %-8s missing\n' "$c"; failed=1; fi
done
if command -v gz >/dev/null 2>&1 && gz sim --versions 2>/dev/null | grep -Eq '(^|[[:space:]])8(\.|$)'; then
  echo "PASS Gazebo Harmonic (gz-sim 8)"
else
  echo "FAIL Gazebo Harmonic (gz-sim 8) not detected"
  failed=1
fi
for p in moveit_ros_planning ros_gz_sim ros_gz_bridge gz_ros2_control tf2_tools controller_manager joint_trajectory_controller; do
  if ros2 pkg prefix "$p" >/dev/null 2>&1; then echo "PASS package $p"; else echo "FAIL package $p"; failed=1; fi
done
for p in manipulation_moveit_adapter manipulation_bringup robot_gazebo robot_realman_moveit_config robot_changingtek_description rm_description; do
  if ros2 pkg prefix "$p" >/dev/null 2>&1; then echo "PASS workspace package $p"; else echo "FAIL workspace package $p"; failed=1; fi
done
timeout 10s ros2 doctor --report >/tmp/manipulation_ros2_doctor.txt 2>&1 || true
echo "doctor report: /tmp/manipulation_ros2_doctor.txt"
exit "$failed"
