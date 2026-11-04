#!/usr/bin/env bash

set -Eeuo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export DEBIAN_FRONTEND=noninteractive
trap 'echo "Setup failed at line $LINENO. Resolve the error above and run ./run_simulation.sh --setup again." >&2' ERR

source /etc/os-release
if [[ "$ID" != ubuntu || "$VERSION_ID" != 22.04 ]]; then
  echo 'Automatic setup requires Ubuntu 22.04 (ROS 2 Humble).' >&2
  exit 1
fi

SUDO=()
if [[ "$EUID" -ne 0 ]]; then
  SUDO=(sudo)
  sudo -v
fi

"${SUDO[@]}" apt-get update
"${SUDO[@]}" apt-get install -y git curl ca-certificates gnupg lsb-release software-properties-common locales

echo 'Checking GitHub for project updates...'
if git -C "$ROOT_DIR" remote get-url origin >/dev/null 2>&1; then
  git -C "$ROOT_DIR" fetch origin
  upstream="$(git -C "$ROOT_DIR" rev-parse --abbrev-ref --symbolic-full-name '@{upstream}' 2>/dev/null || true)"
  if [[ -z "$upstream" ]]; then
    echo 'No tracking branch configured; fetched updates without changing the checkout.'
  elif [[ -n "$(git -C "$ROOT_DIR" status --porcelain)" ]]; then
    echo 'Local changes found; fetched updates without overwriting your work.'
  else
    git -C "$ROOT_DIR" merge --ff-only "$upstream"
  fi
else
  echo 'No origin remote configured; skipping project update.'
fi

"${SUDO[@]}" locale-gen en_US.UTF-8
export LANG=en_US.UTF-8
"${SUDO[@]}" add-apt-repository -y universe
"${SUDO[@]}" install -d -m 0755 /usr/share/keyrings
curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key |
  "${SUDO[@]}" gpg --dearmor --yes -o /usr/share/keyrings/trace-ros-archive-keyring.gpg
curl -fsSL https://packages.osrfoundation.org/gazebo.key |
  "${SUDO[@]}" gpg --dearmor --yes -o /usr/share/keyrings/trace-gazebo-archive-keyring.gpg
architecture="$(dpkg --print-architecture)"
# Reuse an existing repository definition to avoid conflicting Signed-By settings.
apt_sources="$(cat /etc/apt/sources.list 2>/dev/null || true)
$(find /etc/apt/sources.list.d -type f -exec cat {} +)"
if [[ "$apt_sources" != *packages.ros.org/ros2/ubuntu* ]]; then
  echo "deb [arch=$architecture signed-by=/usr/share/keyrings/trace-ros-archive-keyring.gpg] https://packages.ros.org/ros2/ubuntu jammy main" |
    "${SUDO[@]}" tee /etc/apt/sources.list.d/trace-ros2.list >/dev/null
fi
if [[ "$apt_sources" != *packages.osrfoundation.org/gazebo/ubuntu* ]]; then
  echo "deb [arch=$architecture signed-by=/usr/share/keyrings/trace-gazebo-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable jammy main" |
    "${SUDO[@]}" tee /etc/apt/sources.list.d/trace-gazebo.list >/dev/null
fi

"${SUDO[@]}" apt-get update
packages=(
  ros-dev-tools ros-humble-desktop ignition-fortress
  ros-humble-ros-ign ros-humble-ros-ign-interfaces
  ros-humble-navigation2 ros-humble-nav2-bringup
  ros-humble-slam-toolbox ros-humble-robot-localization
  ros-humble-xacro ros-humble-cv-bridge ros-humble-teleop-twist-keyboard
  ros-humble-rqt-gui ros-humble-rqt-gui-py ros-humble-python-qt-binding
  ros-humble-rqt-image-view ros-humble-rqt-robot-steering
  ros-humble-rqt-publisher ros-humble-rqt-topic ros-humble-rqt-action
  ros-humble-sensor-msgs-py ros-humble-robot-state-publisher ros-humble-rviz2
  ros-humble-ament-cmake-python ros-humble-pluginlib
  libopencv-dev python3-numpy python3-scipy python3-opencv
)
echo 'Installing/checking required packages...'
"${SUDO[@]}" apt-get install -y "${packages[@]}"

set +u
source /opt/ros/humble/setup.bash
set -u
if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  "${SUDO[@]}" rosdep init
fi
rosdep update --rosdistro humble
cd "$ROOT_DIR/ros2_simulation"
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y
colcon build --symlink-install
echo 'Setup complete. Run ./run_simulation.sh and select 1 to configure and start.'
