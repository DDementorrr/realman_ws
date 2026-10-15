#!/usr/bin/env bash
set -eo pipefail

# ROS Humble's setup script reads this variable while nounset is enabled.
: "${AMENT_TRACE_SETUP_FILES:=}"
set -u

workspace_root="${REALMAN_WS_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
default_commit="522ca0e0d3560dc8c52628e9f52b86db78673920"
commit="${GZ_ROS2_CONTROL_COMMIT:-$default_commit}"
if [[ -z "$commit" || ! "$commit" =~ ^[0-9a-fA-F]{40}$ ]]; then
  echo "FAIL GZ_ROS2_CONTROL_COMMIT must be a 40-character commit hash" >&2
  exit 2
fi

set +u
source /opt/ros/humble/setup.bash
set -u
third_party="$workspace_root/.third_party/gz_ros2_control"
mkdir -p "$workspace_root/.third_party/src"
if [[ ! -d "$third_party/src/gz_ros2_control/.git" && \
      ! -f "$third_party/src/gz_ros2_control/.source_commit" ]]; then
  mkdir -p "$third_party/src"
  if [[ -e "$third_party/src/gz_ros2_control" ]]; then
    mv "$third_party/src/gz_ros2_control" \
      "$third_party/src/gz_ros2_control.failed-$(date +%Y%m%d%H%M%S)"
  fi
  if ! git -c http.version=HTTP/1.1 clone --depth 1 --branch humble \
      https://github.com/ros-controls/gz_ros2_control.git \
      "$third_party/src/gz_ros2_control"; then
    echo "WARN Git clone failed; downloading the pinned GitHub API archive" >&2
    archive_dir="$(mktemp -d)"
    trap 'rm -rf "$archive_dir"' EXIT
    curl -fL --retry 3 --connect-timeout 15 --max-time 180 \
      -o "$archive_dir/gz_ros2_control.tar.gz" \
      "https://api.github.com/repos/ros-controls/gz_ros2_control/tarball/$commit"
    tar -xzf "$archive_dir/gz_ros2_control.tar.gz" -C "$archive_dir"
    extracted="$(find "$archive_dir" -mindepth 1 -maxdepth 1 -type d -name '*gz_ros2_control-*' -print -quit)"
    [[ -n "$extracted" ]] || { echo "FAIL archive did not contain gz_ros2_control" >&2; exit 1; }
    mv "$extracted" "$third_party/src/gz_ros2_control"
  fi
fi
if [[ -d "$third_party/src/gz_ros2_control/.git" ]]; then
  git -C "$third_party/src/gz_ros2_control" fetch --depth 1 origin "$commit"
  git -C "$third_party/src/gz_ros2_control" checkout --detach "$commit"
  actual="$(git -C "$third_party/src/gz_ros2_control" rev-parse HEAD)"
else
  actual="$commit"
  printf '%s\n' "$actual" > "$third_party/src/gz_ros2_control/.source_commit"
fi
[[ "$actual" == "$commit" ]] || { echo "FAIL checkout/archive mismatch" >&2; exit 1; }
printf '%s\n' "$actual" > "$workspace_root/.third_party/gz_ros2_control.commit"

export GZ_VERSION=harmonic
rosdep install -r --from-paths "$third_party/src" --ignore-src --rosdistro humble -y \
  --skip-keys="ros_gz_bridge ros_gz_sim"
colcon --log-base "$third_party/log" build --base-paths "$third_party/src" \
  --build-base "$third_party/build" --install-base "$third_party/install" \
  --symlink-install
echo "PASS built gz_ros2_control for Gazebo Harmonic at $actual"
echo "Source it before building or launching this workspace:"
echo "  source $third_party/install/setup.bash"
