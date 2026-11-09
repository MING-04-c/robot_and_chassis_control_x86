#!/usr/bin/env bash
# Install dependencies for the same package set used by build_robot.sh.
set -eo pipefail
robot_ws_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ -n "${ROS_DISTRO:-}" && "$ROS_DISTRO" != jazzy ]]; then
  echo 'Use a fresh terminal with ROS 2 Jazzy.' >&2
  exit 1
fi
source /opt/ros/jazzy/setup.bash
robot_package_paths="$(colcon list --base-paths "$robot_ws_dir/src" \
  --packages-up-to robot_task_control eli_cs_robot_calibration orbbec_description --paths-only)"
mapfile -t robot_package_paths <<< "$robot_package_paths"
sudo apt-get update
# The Elite SDK is built separately. Pillow ImageTk is an Ubuntu package without
# a matching key in the currently installed rosdep database.
sudo apt-get install -y build-essential cmake python3-colcon-common-extensions \
  python3-rosdep python3-setuptools libboost-dev libssh-dev libeigen3-dev \
  libyaml-cpp-dev libopencv-dev python3-pil.imagetk
if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  sudo rosdep init
fi
rosdep update --rosdistro jazzy
rosdep install --from-paths "${robot_package_paths[@]}" --ignore-src --rosdistro jazzy \
  --skip-keys 'ament_python python3-pil.imagetk' -y
