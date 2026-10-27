#!/usr/bin/env bash
# Incremental add-on: build this project's plugin; never patch /opt/ros or RViz sources.
set -eo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
source scripts/source_pick_place_env.sh
colcon build --symlink-install --packages-select rm65b_operator_panel robot_realman_moveit_config
printf '%s\n' '构建完成。新终端 source scripts/source_pick_place_env.sh 后按 README 启动。' \
  '已有 RViz 可用 Panels -> Add New Panel -> rm65b_operator_panel/OperatorPanel 加载。'
