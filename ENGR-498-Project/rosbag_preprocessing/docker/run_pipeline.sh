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

get_option_value() {
  local option="$1"
  shift
  local arg
  while [[ $# -gt 0 ]]; do
    arg="$1"
    if [[ "$arg" == "$option" ]]; then
      if [[ $# -lt 2 ]]; then
        return 1
      fi
      printf '%s\n' "$2"
      return 0
    fi
    if [[ "$arg" == "${option}="* ]]; then
      printf '%s\n' "${arg#*=}"
      return 0
    fi
    shift
  done
  return 1
}

strip_option_and_value() {
  local option="$1"
  shift
  local -a result=()
  local arg
  while [[ $# -gt 0 ]]; do
    arg="$1"
    if [[ "$arg" == "$option" ]]; then
      shift
      if [[ $# -gt 0 ]]; then
        shift
      fi
      continue
    fi
    if [[ "$arg" == "${option}="* ]]; then
      shift
      continue
    fi
    result+=("$arg")
    shift
  done
  printf '%s\0' "${result[@]}"
}

strip_flag() {
  local option="$1"
  shift
  local -a result=()
  local arg
  for arg in "$@"; do
    if [[ "$arg" == "$option" ]]; then
      continue
    fi
    result+=("$arg")
  done
  printf '%s\0' "${result[@]}"
}

workflow_supports_blur_controls() {
  local workflow="$1"
  [[ -f "$workflow" ]] || return 1
  grep -q -- "--disable-blur-filter" "$workflow"
}

apply_workspace_blur_filter() {
  local output_root="$1"
  local blur_threshold="$2"
  local latest_run=""

  latest_run="$(find "$output_root" -mindepth 1 -maxdepth 1 -type d -printf '%T@ %p\n' | sort -nr | head -n 1 | cut -d' ' -f2-)"
  if [[ -z "$latest_run" ]]; then
    echo "ERROR: Could not locate pose-recovery output run under $output_root for workspace blur filtering." >&2
    return 2
  fi

  python3 "/workspace/overrides/ws_livox/scripts/filter_blurry_pose_recovery_images.py" \
    --images-dir "$latest_run/images" \
    --image-timestamps-csv "$latest_run/image_timestamps.csv" \
    --camera-csv "$latest_run/tf_camera_out.csv" \
    --blur-threshold "$blur_threshold"
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
    BLUR_FILTER_ENABLED="${BLUR_FILTER_ENABLED:-1}"
    BLUR_THRESHOLD="${BLUR_THRESHOLD:-100.0}"

    if has_option --disable-blur-filter "$@"; then
      BLUR_FILTER_ENABLED=0
    fi
    if blur_threshold_value="$(get_option_value --blur-threshold "$@" 2>/dev/null)"; then
      BLUR_THRESHOLD="$blur_threshold_value"
    fi

    mapfile -d '' -t filtered_args < <(strip_flag --disable-blur-filter "$@")
    mapfile -d '' -t filtered_args < <(strip_option_and_value --blur-threshold "${filtered_args[@]}")
    set -- "${filtered_args[@]}"

    if ! has_option --output-root "$@"; then
      set -- "$@" --output-root "$POSE_OUTPUT_ROOT"
    fi
    pose_output_root="$(get_option_value --output-root "$@" || true)"
    if [[ -z "$pose_output_root" ]]; then
      pose_output_root="$POSE_OUTPUT_ROOT"
    fi

    export BLUR_FILTER_ENABLED
    export BLUR_THRESHOLD

    if workflow_supports_blur_controls "$POSE_RECOVERY_WORKFLOW"; then
      exec "$POSE_RECOVERY_WORKFLOW" "$@"
    fi

    if "$POSE_RECOVERY_WORKFLOW" "$@"; then
      if [[ "$BLUR_FILTER_ENABLED" == "1" ]]; then
        echo "[blur-filter] applying workspace blur filter fallback"
        apply_workspace_blur_filter "$pose_output_root" "$BLUR_THRESHOLD"
      else
        echo "[blur-filter] disabled"
      fi
      exit 0
    fi
    rc=$?
    exit "$rc"
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
