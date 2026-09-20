#!/usr/bin/env bash
set -euo pipefail

workspace_root="${REALMAN_WS_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
default_commit="522ca0e0d3560dc8c52628e9f52b86db78673920"
commit="${GZ_ROS2_CONTROL_COMMIT:-$default_commit}"
if [[ -z "$commit" || ! "$commit" =~ ^[0-9a-fA-F]{40}$ ]]; then
  echo "FAIL GZ_ROS2_CONTROL_COMMIT must be a 40-character commit hash" >&2
  exit 2
fi

source /opt/ros/humble/setup.bash
third_party="$workspace_root/.third_party/gz_ros2_control"
mkdir -p "$workspace_root/.third_party/src"
if [[ ! -d "$third_party/src/gz_ros2_control/.git" ]]; then
  git clone --branch humble https://github.com/ros-controls/gz_ros2_control.git \
    "$third_party/src/gz_ros2_control"
fi
git -C "$third_party/src/gz_ros2_control" fetch --depth 1 origin "$commit"
git -C "$third_party/src/gz_ros2_control" checkout --detach "$commit"
actual="$(git -C "$third_party/src/gz_ros2_control" rev-parse HEAD)"
[[ "$actual" == "$commit" ]] || { echo "FAIL checkout mismatch" >&2; exit 1; }
printf '%s\n' "$actual" > "$workspace_root/.third_party/gz_ros2_control.commit"

export GZ_VERSION=harmonic
rosdep install -r --from-paths "$third_party/src" --ignore-src --rosdistro humble -y \
  --skip-keys="ros_gz_bridge ros_gz_sim"
colcon build --base-paths "$third_party/src" --build-base "$third_party/build" \
  --install-base "$third_party/install" --log-base "$third_party/log" --symlink-install
echo "PASS built gz_ros2_control for Gazebo Harmonic at $actual"
echo "Source it before building or launching this workspace:"
echo "  source $third_party/install/setup.bash"
