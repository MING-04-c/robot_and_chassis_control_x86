# Source this file to load the Jazzy control stack built in this workspace.
_robot_load_workspace() {
  local robot_ws_dir
  robot_ws_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
  if [[ -n "${ROS_DISTRO:-}" && "$ROS_DISTRO" != jazzy ]]; then
    echo 'Use a fresh terminal; this workspace requires ROS 2 Jazzy.' >&2
    return 1
  fi
  if [[ ! -f "$robot_ws_dir/install/setup.bash" ||
        ! -f "$robot_ws_dir/install/robot_task_control/share/robot_task_control/package.xml" ]]; then
    echo 'Workspace not built. Run: bash scripts/build_robot.sh' >&2
    return 1
  fi
  source /opt/ros/jazzy/setup.bash || return
  export CMAKE_PREFIX_PATH="$robot_ws_dir/local/elite_sdk${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
  export LD_LIBRARY_PATH="$robot_ws_dir/local/elite_sdk/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
  source "$robot_ws_dir/install/setup.bash"
}
if _robot_load_workspace; then
  unset -f _robot_load_workspace
else
  unset -f _robot_load_workspace
  return 1
fi
