#!/usr/bin/env bash
set -e

ENTRYPOINT_ARGS=("$@")
set --

source /opt/ros/${ROS_DISTRO:-noetic}/setup.bash

export HOME="${HOME:-/home/portable}"
export PORTABLE_ROS_HOME="${PORTABLE_ROS_HOME:-$HOME}"
export LD_LIBRARY_PATH="/usr/local/lib:${LD_LIBRARY_PATH:-}"

if [[ -f "${PORTABLE_ROS_HOME}/iridescence/build/libiridescence.so" ]]; then
  export LD_LIBRARY_PATH="${PORTABLE_ROS_HOME}/iridescence/build:${LD_LIBRARY_PATH}"
fi

if [[ -d /usr/lib/wsl/lib ]]; then
  export LD_LIBRARY_PATH="/usr/lib/wsl/lib:${LD_LIBRARY_PATH}"
fi

if [[ -f "${PORTABLE_ROS_HOME}/ws_calib/devel/setup.bash" ]]; then
  source "${PORTABLE_ROS_HOME}/ws_calib/devel/setup.bash"
fi

if [[ -f "${PORTABLE_ROS_HOME}/ws_livox/devel/setup.bash" ]]; then
  source "${PORTABLE_ROS_HOME}/ws_livox/devel/setup.bash"
fi

export PATH="${PORTABLE_ROS_HOME}/ws_calib/scripts:${PORTABLE_ROS_HOME}/ws_livox/scripts:${PATH}"

set -- "${ENTRYPOINT_ARGS[@]}"

exec "$@"
