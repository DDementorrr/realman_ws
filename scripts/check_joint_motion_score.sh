#!/usr/bin/env bash
set -eo pipefail
workspace_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
score_binary="$(mktemp /tmp/realman-joint-score.XXXXXX)"
trap 'rm -f "$score_binary"' EXIT
g++ -std=c++17 -Wall -Wextra -Werror -pedantic \
  -I "$workspace_root/src/ros2_adapters/pick_place_mtc/include" \
  "$workspace_root/scripts/test_joint_motion_score.cpp" -o "$score_binary"
"$score_binary"
