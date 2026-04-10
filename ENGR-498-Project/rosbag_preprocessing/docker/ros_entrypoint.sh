#!/usr/bin/env bash
set -e

ENTRYPOINT_ARGS=("$@")
set --

source /opt/ros/${ROS_DISTRO:-noetic}/setup.bash

export PORTABLE_ROS_HOME="/home/portable"
export HOME="${PORTABLE_ROS_HOME}"
export LD_LIBRARY_PATH="/usr/local/lib:${LD_LIBRARY_PATH:-}"

if [[ -f "${PORTABLE_ROS_HOME}/iridescence/build/libiridescence.so" ]]; then
  export LD_LIBRARY_PATH="${PORTABLE_ROS_HOME}/iridescence/build:${LD_LIBRARY_PATH}"
fi

if [[ -d /usr/lib/wsl/lib ]]; then
  export LD_LIBRARY_PATH="/usr/lib/wsl/lib:${LD_LIBRARY_PATH}"
fi

if [[ -f "${PORTABLE_ROS_HOME}/ws_calib/devel/local_setup.bash" ]]; then
  source "${PORTABLE_ROS_HOME}/ws_calib/devel/local_setup.bash"
fi

if [[ -f "${PORTABLE_ROS_HOME}/ws_livox/devel/local_setup.bash" ]]; then
  source "${PORTABLE_ROS_HOME}/ws_livox/devel/local_setup.bash"
fi

workspace_src_paths=()
if [[ -d "${PORTABLE_ROS_HOME}/ws_calib/src" ]]; then
  workspace_src_paths+=("${PORTABLE_ROS_HOME}/ws_calib/src")
fi
if [[ -d "${PORTABLE_ROS_HOME}/ws_livox/src" ]]; then
  workspace_src_paths+=("${PORTABLE_ROS_HOME}/ws_livox/src")
fi

if (( ${#workspace_src_paths[@]} > 0 )); then
  existing_ros_package_path="${ROS_PACKAGE_PATH:-}"
  if [[ -n "${existing_ros_package_path}" ]]; then
    workspace_src_paths+=("${existing_ros_package_path}")
  fi
  export ROS_PACKAGE_PATH="$(IFS=:; echo "${workspace_src_paths[*]}")"
fi

export PATH="${PORTABLE_ROS_HOME}/ws_calib/scripts:${PORTABLE_ROS_HOME}/ws_livox/scripts:${PATH}"

set -- "${ENTRYPOINT_ARGS[@]}"

exec "$@"
