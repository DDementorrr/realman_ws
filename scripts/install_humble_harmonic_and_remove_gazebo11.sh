#!/usr/bin/env bash
set -euo pipefail

# Preview is the default. Use --execute for system changes.
execute=0
[[ "${1:-}" == "--execute" ]] && execute=1
[[ -z "${1:-}" || "${1:-}" == "--execute" ]] || { echo "usage: $0 [--execute]" >&2; exit 2; }
source /etc/os-release
[[ "$ID" == ubuntu && "$VERSION_ID" == 22.04 ]] || { echo "Expected Ubuntu 22.04, found ${PRETTY_NAME}" >&2; exit 2; }

classic=(gazebo gazebo-common gazebo-plugin-base libgazebo-dev libgazebo11
  ros-humble-gazebo-dev ros-humble-gazebo-msgs ros-humble-gazebo-plugins
  ros-humble-gazebo-ros ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control)
harmonic=(gz-harmonic ros-humble-ros-gzharmonic ros-humble-ros-gzharmonic-bridge
  ros-humble-ros-gzharmonic-sim ros-humble-ros2-control ros-humble-ros2-controllers
  ros-humble-controller-manager ros-humble-joint-state-broadcaster
  ros-humble-joint-trajectory-controller ros-humble-robot-state-publisher
  ros-humble-rviz2 ros-humble-xacro ros-humble-tf2-tools python3-yaml
  ros-humble-moveit ros-humble-moveit-setup-assistant)

echo "Gazebo Classic packages currently installed and targeted for removal:"
for p in "${classic[@]}"; do dpkg-query -W -f='${Status}' "$p" 2>/dev/null | grep -q 'install ok installed' && echo "  $p" || true; done
echo "Harmonic/ROS packages targeted for installation:"
printf '  %s\n' "${harmonic[@]}"
echo "This script does not delete source workspaces, Git data, or model assets."
(( execute )) || { echo "Preview only. Re-run with --execute to apply changes."; exit 0; }

if [[ $EUID -eq 0 ]]; then SUDO=(); else SUDO=(sudo); fi
apt_opts=(-o Acquire::Retries=3 -o Acquire::http::Timeout=20 -o Acquire::https::Timeout=20)
"${SUDO[@]}" apt-get "${apt_opts[@]}" update
bootstrap=()
for p in software-properties-common curl wget gnupg lsb-release ca-certificates; do
  dpkg-query -W -f='${Status}' "$p" 2>/dev/null | grep -q 'install ok installed' || bootstrap+=("$p")
done
if ((${#bootstrap[@]})); then
  "${SUDO[@]}" apt-get "${apt_opts[@]}" install -y --no-install-recommends "${bootstrap[@]}"
fi
if command -v add-apt-repository >/dev/null 2>&1; then "${SUDO[@]}" add-apt-repository -y universe; fi
if [[ ! -f /usr/share/keyrings/ros-archive-keyring.gpg ]]; then
  curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | "${SUDO[@]}" gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg
fi
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu ${UBUNTU_CODENAME} main" | "${SUDO[@]}" tee /etc/apt/sources.list.d/ros2.list >/dev/null
if [[ ! -f /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg ]]; then
  curl -fsSL https://packages.osrfoundation.org/gazebo.gpg | "${SUDO[@]}" gpg --dearmor -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
fi
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable ${UBUNTU_CODENAME} main" | "${SUDO[@]}" tee /etc/apt/sources.list.d/gazebo-stable.list >/dev/null
"${SUDO[@]}" apt-get "${apt_opts[@]}" update
for p in "${harmonic[@]}"; do apt-cache show "$p" >/dev/null 2>&1 || { echo "Missing package $p; aborting before purge." >&2; exit 4; }; done
"${SUDO[@]}" apt-get "${apt_opts[@]}" install -y "${harmonic[@]}"
"${SUDO[@]}" apt-get "${apt_opts[@]}" purge -y "${classic[@]}" || true
"${SUDO[@]}" apt-get "${apt_opts[@]}" autoremove -y

rc="$HOME/.bashrc"
if [[ -f "$rc" ]]; then
  cp -a "$rc" "$rc.bak.gazebo-classic-$(date +%Y%m%d%H%M%S)"
  sed -i '/# >>> Gazebo classic <<</d; /source \/usr\/share\/gazebo-11\/setup.sh/d; /^export GAZEBO_/d; /source ~\/tracked_robot_ws\/install\/setup.bash/d' "$rc"
  if ! grep -Fq 'source ~/realman_ws/install/setup.bash' "$rc"; then
    printf '\n# ROS 2 Humble + realman_ws\nsource /opt/ros/humble/setup.bash\n[ -f ~/realman_ws/install/setup.bash ] && source ~/realman_ws/install/setup.bash\n' >> "$rc"
  fi
fi
echo "Gazebo 11 removed and Harmonic installed. Open a new shell and source /opt/ros/humble/setup.bash."
