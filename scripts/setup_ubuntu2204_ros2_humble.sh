#!/usr/bin/env bash
set -euo pipefail

if [[ "${EUID}" -eq 0 ]]; then SUDO=(); else SUDO=(sudo); fi
source /etc/os-release
[[ "${ID}" == "ubuntu" && "${VERSION_ID}" == "22.04" ]] || {
  echo "Expected Ubuntu 22.04, found ${PRETTY_NAME}" >&2
  exit 2
}

"${SUDO[@]}" apt-get update
for conflicting in ros-humble-ros-gz-sim ros-humble-ros-gz-bridge ros-humble-gz-ros2-control; do
  if dpkg-query -W -f='${Status}' "$conflicting" 2>/dev/null | grep -q 'install ok installed'; then
    echo "FAIL $conflicting is installed; remove Fortress ROS-Gazebo packages before selecting Harmonic" >&2
    exit 3
  fi
done
"${SUDO[@]}" apt-get install -y software-properties-common curl wget gnupg lsb-release ca-certificates
"${SUDO[@]}" add-apt-repository -y universe
if [[ ! -f /usr/share/keyrings/ros-archive-keyring.gpg ]]; then
  curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    | "${SUDO[@]}" gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg
fi
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu ${UBUNTU_CODENAME} main" \
  | "${SUDO[@]}" tee /etc/apt/sources.list.d/ros2.list >/dev/null
if [[ ! -f /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg ]]; then
  curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
    | "${SUDO[@]}" gpg --dearmor -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
fi
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable ${UBUNTU_CODENAME} main" \
  | "${SUDO[@]}" tee /etc/apt/sources.list.d/gazebo-stable.list >/dev/null
"${SUDO[@]}" apt-get update
for required in gz-harmonic ros-humble-ros-gzharmonic ros-humble-ros-gzharmonic-bridge ros-humble-ros-gzharmonic-sim; do
  apt-cache show "$required" >/dev/null 2>&1 || {
    echo "FAIL $required is not available from the configured OSRF repositories" >&2
    echo "Do not substitute ros-humble-ros-gz-*; those packages are the Fortress ABI on Humble." >&2
    exit 4
  }
done
"${SUDO[@]}" apt-get install -y \
  build-essential cmake git unzip zip pkg-config python3-pip python3-venv python3-dev \
  python3-colcon-common-extensions python3-vcstool python3-rosdep ros-dev-tools \
  ros-humble-desktop \
  ros-humble-moveit ros-humble-moveit-setup-assistant \
  gz-harmonic ros-humble-ros-gzharmonic ros-humble-ros-gzharmonic-bridge \
  ros-humble-ros-gzharmonic-sim \
  ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-controller-manager \
  ros-humble-joint-state-broadcaster ros-humble-joint-trajectory-controller \
  ros-humble-robot-state-publisher ros-humble-rviz2 ros-humble-xacro \
  ros-humble-tf2-tools python3-yaml
if [[ $EUID -eq 0 ]]; then
  rosdep init 2>/dev/null || true
elif [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  echo "WARN rosdep is not initialized. Run once with sudo: sudo rosdep init" >&2
fi
wget -q https://raw.githubusercontent.com/osrf/osrf-rosdep/master/gz/00-gazebo.list \
  -O /tmp/realman_ws_00-gazebo.list
"${SUDO[@]}" install -m 0644 /tmp/realman_ws_00-gazebo.list \
  /etc/ros/rosdep/sources.list.d/00-gazebo.list
rosdep update
echo "ROS 2 Humble / Gazebo Harmonic base installation complete. Run build_gz_ros2_control_harmonic.sh next."
