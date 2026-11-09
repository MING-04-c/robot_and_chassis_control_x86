#!/usr/bin/env bash
# Build the local control stack on Ubuntu 24.04 / ROS 2 Jazzy (amd64 or arm64).
set -eo pipefail
robot_ws_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ -z "${ROS_DISTRO:-}" ]]; then
  source /opt/ros/jazzy/setup.bash
fi
if [[ "${ROS_VERSION:-}" != 2 || "${ROS_DISTRO:-}" != jazzy ]]; then
  echo 'Use a fresh terminal with ROS 2 Jazzy; do not mix ROS distributions.' >&2
  exit 1
fi
robot_jobs="${ROBOT_BUILD_JOBS:-2}"
if [[ ! "$robot_jobs" =~ ^[1-9][0-9]*$ ]]; then
  echo 'ROBOT_BUILD_JOBS must be a positive integer.' >&2
  exit 1
fi
robot_sdk_prefix="$robot_ws_dir/local/elite_sdk"
# Keep ordinary `colcon build` from discovering the installed SDK's package.xml.
# Recreate the markers when rebuilding after a local/build directory cleanup.
mkdir -p "$robot_ws_dir/local" "$robot_ws_dir/build_elite_sdk"
touch "$robot_ws_dir/local/COLCON_IGNORE" "$robot_ws_dir/build_elite_sdk/COLCON_IGNORE"
cmake -S "$robot_ws_dir/../Elite_Robots_CS_SDK" \
  -B "$robot_ws_dir/build_elite_sdk" \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$robot_sdk_prefix" \
  -DCMAKE_INSTALL_LIBDIR=lib -DELITE_ROS2_BUILD=OFF \
  -DELITE_COMPILE_TESTS=OFF -DELITE_COMPILE_EXAMPLES=OFF -DELITE_COMPILE_DOC=OFF
cmake --build "$robot_ws_dir/build_elite_sdk" --parallel "$robot_jobs"
cmake --install "$robot_ws_dir/build_elite_sdk"
export CMAKE_PREFIX_PATH="$robot_sdk_prefix${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
export LD_LIBRARY_PATH="$robot_sdk_prefix/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export CMAKE_BUILD_PARALLEL_LEVEL="$robot_jobs"
cd "$robot_ws_dir"
# Resolve the dependency closure, including the newer camera/detection packages.
# The legacy Gazebo/ign_ros2_control package is not needed by this control stack.
colcon build --base-paths src --executor sequential --cmake-force-configure \
  --packages-up-to robot_task_control eli_cs_robot_calibration orbbec_description \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_TESTING="${ROBOT_BUILD_TESTING:-OFF}" \
    -DMASC_CHASSIS_CAN_SDK_BUILD_TESTS="${ROBOT_BUILD_TESTING:-OFF}" \
    -DMASC_CHASSIS_CAN_SDK_BUILD_EXAMPLES=OFF \
    -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3
printf '%s\n' 'Build finished. Source scripts/robot_env.bash before launching.'
