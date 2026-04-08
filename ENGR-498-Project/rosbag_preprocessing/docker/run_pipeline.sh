#!/usr/bin/env bash
set -euo pipefail

MODE="${1:-}"
WORKSPACE_ROOT="${PORTABLE_ROS_WORKSPACE_ROOT:-/workspace}"
RUNTIME_ROOT="${PORTABLE_ROS_HOME:-$HOME}"
DEFAULT_OUTPUT_ROOT="${PORTABLE_ROS_OUTPUT_ROOT:-${WORKSPACE_ROOT}/outputs}"
CALIBRATION_OUTPUT_ROOT="${PORTABLE_ROS_CALIBRATION_OUTPUT_ROOT:-${DEFAULT_OUTPUT_ROOT}/calibration}"
POSE_OUTPUT_ROOT="${PORTABLE_ROS_POSE_OUTPUT_ROOT:-${DEFAULT_OUTPUT_ROOT}/pose_recovery}"
CALIBRATION_WORKFLOW="${RUNTIME_ROOT}/ws_calib/scripts/run_direct_visual_lidar_calibration_workflow.sh"
POSE_RECOVERY_WORKFLOW="${RUNTIME_ROOT}/ws_livox/scripts/run_pose_recovery_camera_gps.sh"

mkdir -p "$CALIBRATION_OUTPUT_ROOT" "$POSE_OUTPUT_ROOT"

has_option() {
  local option="$1"
  shift
  local arg
  for arg in "$@"; do
    if [[ "$arg" == "$option" || "$arg" == "${option}="* ]]; then
      return 0
    fi
  done
  return 1
}

case "$MODE" in
  calibration)
    shift
    if ! has_option --run-root "$@"; then
      set -- "$@" --run-root "$CALIBRATION_OUTPUT_ROOT"
    fi
    exec "$CALIBRATION_WORKFLOW" "$@"
    ;;
  pose-recovery)
    shift
    if ! has_option --output-root "$@"; then
      set -- "$@" --output-root "$POSE_OUTPUT_ROOT"
    fi
    exec "$POSE_RECOVERY_WORKFLOW" "$@"
    ;;
  bash|"")
    if [[ -n "$MODE" ]]; then
      shift
    fi
    if [[ $# -gt 0 ]]; then
      exec "$@"
    fi
    exec bash
    ;;
  *)
    exec "$@"
    ;;
esac
