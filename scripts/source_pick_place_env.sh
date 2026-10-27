#!/usr/bin/env bash
# Source the complete local Pick & Place runtime environment.
# Usage: source scripts/source_pick_place_env.sh

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  echo "请使用 source scripts/source_pick_place_env.sh，而不是直接执行该脚本。" >&2
  exit 2
fi

pick_place_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash

gz_control_setup="$pick_place_root/.third_party/gz_ros2_control/install/setup.bash"
if [[ ! -f "$gz_control_setup" ]]; then
  echo "缺少 gz_ros2_control 环境：$gz_control_setup" >&2
  return 1
fi
source "$gz_control_setup"

workspace_setup="$pick_place_root/install/setup.bash"
if [[ ! -f "$workspace_setup" ]]; then
  echo "缺少工作空间安装环境：$workspace_setup；请先 colcon build。" >&2
  return 1
fi
source "$workspace_setup"

mtc_prefix="$pick_place_root/.third_party/mtc/debroot/opt/ros/humble"
if [[ -d "$mtc_prefix/lib" && -d "$mtc_prefix/share/moveit_task_constructor_msgs" ]]; then
  export AMENT_PREFIX_PATH="$mtc_prefix:${AMENT_PREFIX_PATH:-}"
  export LD_LIBRARY_PATH="$mtc_prefix/lib:${LD_LIBRARY_PATH:-}"
  export PYTHONPATH="$mtc_prefix/local/lib/python3.10/dist-packages:$mtc_prefix/lib/python3.10/site-packages:${PYTHONPATH:-}"
  pick_place_mtc_origin="本地 MTC"
elif ros2 pkg prefix moveit_task_constructor_core >/dev/null 2>&1 && \
     ros2 pkg prefix moveit_task_constructor_msgs >/dev/null 2>&1; then
  pick_place_mtc_origin="系统 MTC"
else
  echo "缺少 MTC。安装 Humble 的 moveit-task-constructor core/capabilities/visualization，或提供本地二进制前缀：$mtc_prefix" >&2
  return 1
fi

echo "Pick & Place 环境已加载：ROS 2 Humble + gz_ros2_control + $pick_place_mtc_origin"
