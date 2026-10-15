#!/usr/bin/env bash
set -euo pipefail

workspace_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
mkdir -p "$workspace_root/upstream"
touch "$workspace_root/upstream/COLCON_IGNORE"

expected_commit="c941b565e4f9174afa36561f143ef5fbbb744750"
if [[ -d "$workspace_root/upstream/realman_ros2/.git" ]]; then
  actual_commit="$(git -C "$workspace_root/upstream/realman_ros2" rev-parse HEAD)"
  [[ "$actual_commit" == "$expected_commit" ]] || {
    echo "FAIL upstream/realman_ros2 is at $actual_commit, expected $expected_commit" >&2
    exit 1
  }
  echo "PASS upstream/realman_ros2 matches pinned commit $expected_commit"
  exit 0
fi

if command -v vcs >/dev/null 2>&1; then
  vcs import "$workspace_root/upstream" < "$workspace_root/upstream/realman_humble.repos"
else
  echo "WARN vcstool is missing; using the single-repository git fallback" >&2
  git clone https://github.com/RealManRobot/ros2_rm_robot.git "$workspace_root/upstream/realman_ros2"
  git -C "$workspace_root/upstream/realman_ros2" fetch --depth 1 origin "$expected_commit"
  git -C "$workspace_root/upstream/realman_ros2" checkout --detach "$expected_commit"
fi
actual_commit="$(git -C "$workspace_root/upstream/realman_ros2" rev-parse HEAD)"
[[ "$actual_commit" == "$expected_commit" ]] || {
  echo "FAIL imported commit $actual_commit, expected $expected_commit" >&2
  exit 1
}
echo "PASS imported pinned RealMan Humble source at $actual_commit"
