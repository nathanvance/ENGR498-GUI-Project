#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  run_direct_visual_lidar_calibration_workflow.sh DATASET_DIR [options]

Description:
  End-user wrapper for the manual direct_visual_lidar_calibration workflow in WSL.
  It can:
  - start roscore if needed
  - normalize bags so image data is available on /camera/image_raw
  - preprocess all bags in a dataset directory
  - launch manual initial guess
  - run calibrate
  - launch viewer

Arguments:
  DATASET_DIR
      Directory containing one or more ROS1 .bag files for calibration.

Options:
  --run-name NAME
      Name of the calibration run folder under ~/calibration_runs.
      Default: <dataset_basename>_manual_<timestamp>

  --run-root DIR
      Root directory for generated run folders.
      Default: $PORTABLE_ROS_CALIBRATION_OUTPUT_ROOT, otherwise
      $PORTABLE_ROS_OUTPUT_ROOT, otherwise ~/calibration_runs

  --stop-after STAGE
      Stop after one stage.
      Stages: convert, preprocess, manual, calibrate, viewer
      Default: viewer

  --preprocess-viz
      Enable the -v flag during preprocessing.
      Default: disabled because visual preprocess is currently unstable on this WSL setup.

  --points-topic TOPIC
      Point cloud topic to preserve during normalization.
      Default: /livox/lidar

  --camera-info-topic TOPIC
      CameraInfo topic to preserve during normalization.
      Default: /camera/camera_info

  --compressed-base-topic TOPIC
      Base topic used by image_transport compressed images.
      The republisher will subscribe to TOPIC/compressed.
      Default: /camera/image

  --raw-image-topic TOPIC
      Raw image topic required by calibration.
      Default: /camera/image_raw

Examples:
  run_direct_visual_lidar_calibration_workflow.sh \
    /mnt/c/Users/leifh/Documents/UofA/Senior_Design/Software/Calibration/Test_4_2_26/Test1

  run_direct_visual_lidar_calibration_workflow.sh \
    /mnt/c/path/to/dataset \
    --run-name test1_manual \
    --stop-after preprocess
EOF
}

die() {
  echo "ERROR: $*" >&2
  exit 1
}

wait_for_master() {
  local timeout_s="${1:-10}"
  local start
  start="$(date +%s)"
  while true; do
    if rostopic list >/dev/null 2>&1; then
      return 0
    fi
    if (( $(date +%s) - start >= timeout_s )); then
      return 1
    fi
    sleep 0.1
  done
}

wait_for_pid_exit() {
  local pid="$1"
  local timeout_s="${2:-20}"
  local start
  start="$(date +%s)"
  while kill -0 "$pid" 2>/dev/null; do
    if (( $(date +%s) - start >= timeout_s )); then
      return 1
    fi
    sleep 0.2
  done
  return 0
}

bag_has_topic_type() {
  local bag="$1"
  local topic="$2"
  local type="$3"
  rosbag info "$bag" | grep -F " ${topic} " | grep -F "${type}" >/dev/null 2>&1
}

detect_bag_mode() {
  local bag="$1"

  if bag_has_topic_type "$bag" "$RAW_IMAGE_TOPIC" "sensor_msgs/Image"; then
    echo "raw_image_raw"
    return 0
  fi

  if bag_has_topic_type "$bag" "${COMPRESSED_BASE_TOPIC}/compressed" "sensor_msgs/CompressedImage"; then
    echo "compressed"
    return 0
  fi

  if bag_has_topic_type "$bag" "$COMPRESSED_BASE_TOPIC" "sensor_msgs/Image"; then
    echo "raw_image"
    return 0
  fi

  echo "unsupported"
}

ensure_required_topics() {
  local bag="$1"

  bag_has_topic_type "$bag" "$CAMERA_INFO_TOPIC" "sensor_msgs/CameraInfo" || \
    die "Bag is missing required camera info topic ${CAMERA_INFO_TOPIC}: $bag"

  bag_has_topic_type "$bag" "$POINTS_TOPIC" "sensor_msgs/PointCloud2" || \
    die "Bag is missing required point cloud topic ${POINTS_TOPIC}: $bag"
}

start_roscore_if_needed() {
  if rostopic list >/dev/null 2>&1; then
    echo "[info] roscore already running"
    return 0
  fi

  echo "[info] starting roscore"
  : > "$ROSCORE_LOG"
  roscore >"$ROSCORE_LOG" 2>&1 &
  ROSCORE_PID=$!
  STARTED_ROSCORE=1

  wait_for_master 10 || die "roscore did not start. See $ROSCORE_LOG"
}

run_with_shim() {
  local gui_lib_dir="$HOME/iridescence/build"
  local gui_ld_path="${LD_LIBRARY_PATH:-}"

  if [[ -f "$gui_lib_dir/libiridescence.so" ]]; then
    if [[ -n "$gui_ld_path" ]]; then
      gui_ld_path="$gui_lib_dir:$gui_ld_path"
    else
      gui_ld_path="$gui_lib_dir"
    fi
  fi

  LD_LIBRARY_PATH="$gui_ld_path" \
  LD_PRELOAD="$SHIM_PATH" \
  "$@"
}

warn() {
  echo "WARNING: $*" >&2
}

pretty_print_calib_json() {
  local json_path="$1"
  local out_path="$2"
  python3 -m json.tool "$json_path" | tee "$out_path"
}

assert_manual_result_present() {
  local json_path="$1"
  python3 - "$json_path" <<'PY'
import json
import sys

path = sys.argv[1]
with open(path, "r", encoding="utf-8") as f:
    data = json.load(f)

results = data.get("results", {})
value = results.get("init_T_lidar_camera")
if not isinstance(value, list) or len(value) != 7:
    raise SystemExit(1)
PY
}

assert_final_result_present() {
  local json_path="$1"
  python3 - "$json_path" <<'PY'
import json
import sys

path = sys.argv[1]
with open(path, "r", encoding="utf-8") as f:
    data = json.load(f)

results = data.get("results", {})
value = results.get("T_lidar_camera")
if not isinstance(value, list) or len(value) != 7:
    raise SystemExit(1)
PY
}

normalize_bag_to_raw_image_topic() {
  local src_bag="$1"
  local mode="$2"
  local bag_base
  local bag_stem
  local dst_bag
  local republish_transport
  local republish_in_topic

  bag_base="$(basename "$src_bag")"
  bag_stem="${bag_base%.bag}"

  if [[ "$mode" == "raw_image_raw" ]]; then
    ln -sfn "$src_bag" "$NORMALIZED_DATASET_DIR/$bag_base"
    echo "[normalize] reuse existing raw-image bag: $bag_base"
    return 0
  fi

  if [[ "$mode" == "compressed" ]]; then
    republish_transport="compressed"
    republish_in_topic="$COMPRESSED_BASE_TOPIC"
  elif [[ "$mode" == "raw_image" ]]; then
    republish_transport="raw"
    republish_in_topic="$COMPRESSED_BASE_TOPIC"
  else
    die "Unsupported image layout in $src_bag"
  fi

  dst_bag="$NORMALIZED_DATASET_DIR/${bag_stem}_raw.bag"
  local republish_log="$LOG_DIR/normalize_${bag_stem}_republish.log"
  local record_log="$LOG_DIR/normalize_${bag_stem}_record.log"
  local play_log="$LOG_DIR/normalize_${bag_stem}_play.log"

  echo "[normalize] converting $bag_base -> $(basename "$dst_bag")"
  echo "[normalize] transport=${republish_transport} in=${republish_in_topic} out=${RAW_IMAGE_TOPIC}"

  CURRENT_REPUBLISH_PID=""
  CURRENT_RECORD_PID=""

  : > "$republish_log"
  rosrun image_transport republish "$republish_transport" \
    in:="$republish_in_topic" \
    raw out:="$RAW_IMAGE_TOPIC" >"$republish_log" 2>&1 &
  CURRENT_REPUBLISH_PID=$!

  : > "$record_log"
  rosbag record -O "$dst_bag" "$RAW_IMAGE_TOPIC" "$CAMERA_INFO_TOPIC" "$POINTS_TOPIC" >"$record_log" 2>&1 &
  CURRENT_RECORD_PID=$!

  sleep 2

  : > "$play_log"
  rosbag play --delay=2.0 "$src_bag" >"$play_log" 2>&1

  sleep 2

  if [[ -n "${CURRENT_RECORD_PID:-}" ]]; then
    kill -INT "$CURRENT_RECORD_PID" 2>/dev/null || true
    wait_for_pid_exit "$CURRENT_RECORD_PID" 20 || die "rosbag record did not exit cleanly for $src_bag"
    CURRENT_RECORD_PID=""
  fi

  if [[ -n "${CURRENT_REPUBLISH_PID:-}" ]]; then
    kill "$CURRENT_REPUBLISH_PID" 2>/dev/null || true
    wait_for_pid_exit "$CURRENT_REPUBLISH_PID" 10 || true
    CURRENT_REPUBLISH_PID=""
  fi

  [[ -f "$dst_bag" ]] || die "Expected normalized bag was not created: $dst_bag"
  bag_has_topic_type "$dst_bag" "$RAW_IMAGE_TOPIC" "sensor_msgs/Image" || \
    die "Normalized bag does not contain ${RAW_IMAGE_TOPIC}: $dst_bag"
}

cleanup() {
  if [[ -n "${CURRENT_RECORD_PID:-}" ]]; then
    kill -INT "$CURRENT_RECORD_PID" 2>/dev/null || true
  fi
  if [[ -n "${CURRENT_REPUBLISH_PID:-}" ]]; then
    kill "$CURRENT_REPUBLISH_PID" 2>/dev/null || true
  fi
  if [[ "${STARTED_ROSCORE:-0}" == "1" && -n "${ROSCORE_PID:-}" ]]; then
    kill "$ROSCORE_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT

DATASET_DIR=""
RUN_ROOT="${PORTABLE_ROS_CALIBRATION_OUTPUT_ROOT:-${PORTABLE_ROS_OUTPUT_ROOT:-$HOME/calibration_runs}}"
RUN_NAME=""
STOP_AFTER="viewer"
PREPROCESS_VISUALIZE=0
POINTS_TOPIC="/livox/lidar"
CAMERA_INFO_TOPIC="/camera/camera_info"
COMPRESSED_BASE_TOPIC="/camera/image"
RAW_IMAGE_TOPIC="/camera/image_raw"

while [[ $# -gt 0 ]]; do
  case "$1" in
    -h|--help)
      usage
      exit 0
      ;;
    --run-name)
      RUN_NAME="$2"
      shift 2
      ;;
    --run-root)
      RUN_ROOT="$2"
      shift 2
      ;;
    --stop-after)
      STOP_AFTER="$2"
      shift 2
      ;;
    --preprocess-viz)
      PREPROCESS_VISUALIZE=1
      shift
      ;;
    --points-topic)
      POINTS_TOPIC="$2"
      shift 2
      ;;
    --camera-info-topic)
      CAMERA_INFO_TOPIC="$2"
      shift 2
      ;;
    --compressed-base-topic)
      COMPRESSED_BASE_TOPIC="$2"
      shift 2
      ;;
    --raw-image-topic)
      RAW_IMAGE_TOPIC="$2"
      shift 2
      ;;
    --*)
      die "Unknown option: $1"
      ;;
    *)
      if [[ -z "$DATASET_DIR" ]]; then
        DATASET_DIR="$1"
      else
        die "Unexpected positional argument: $1"
      fi
      shift
      ;;
  esac
done

[[ -n "$DATASET_DIR" ]] || { usage; exit 2; }
case "$STOP_AFTER" in
  convert|preprocess|manual|calibrate|viewer) ;;
  *)
    die "Invalid --stop-after stage: $STOP_AFTER"
    ;;
esac

export ROS_DISTRO="${ROS_DISTRO:-noetic}"
set +u
source /opt/ros/${ROS_DISTRO}/setup.bash
source "$HOME/ws_calib/devel/setup.bash"
export ROS_PACKAGE_PATH="$HOME/ws_calib/src${ROS_PACKAGE_PATH:+:$ROS_PACKAGE_PATH}"
set -u

SHIM_PATH="$HOME/lib/libglfw_hint_shim.so"
[[ -f "$SHIM_PATH" ]] || die "OpenGL shim not found: $SHIM_PATH"

if [[ "$DATASET_DIR" != /* ]]; then
  DATASET_DIR="$PWD/$DATASET_DIR"
fi
[[ -d "$DATASET_DIR" ]] || die "Dataset directory not found: $DATASET_DIR"

mapfile -t BAGS < <(find "$DATASET_DIR" -maxdepth 1 -type f -name '*.bag' | sort)
[[ ${#BAGS[@]} -gt 0 ]] || die "No .bag files found in dataset directory: $DATASET_DIR"

if [[ -z "$RUN_NAME" ]]; then
  RUN_NAME="$(basename "$DATASET_DIR")_manual_$(date +%Y%m%d_%H%M%S)"
fi

RUN_DIR="$RUN_ROOT/$RUN_NAME"
NORMALIZED_DATASET_DIR="$RUN_ROOT/${RUN_NAME}_raw_input"
LOG_DIR="$RUN_DIR/logs"
ROSCORE_LOG="$LOG_DIR/roscore.log"

mkdir -p "$RUN_ROOT"
if [[ -d "$RUN_DIR" ]] && find "$RUN_DIR" -mindepth 1 -print -quit 2>/dev/null | grep -q .; then
  die "Run directory already exists and is not empty: $RUN_DIR"
fi
mkdir -p "$RUN_DIR" "$LOG_DIR"

if [[ -d "$NORMALIZED_DATASET_DIR" ]] && find "$NORMALIZED_DATASET_DIR" -mindepth 1 -print -quit 2>/dev/null | grep -q .; then
  die "Normalized dataset directory already exists and is not empty: $NORMALIZED_DATASET_DIR"
fi

STARTED_ROSCORE=0
ROSCORE_PID=""
CURRENT_REPUBLISH_PID=""
CURRENT_RECORD_PID=""

echo "[info] dataset dir: $DATASET_DIR"
echo "[info] run dir:     $RUN_DIR"
echo "[info] stop after:  $STOP_AFTER"

declare -A BAG_MODES=()
NEEDS_NORMALIZATION=0
for bag in "${BAGS[@]}"; do
  ensure_required_topics "$bag"
  BAG_MODES["$bag"]="$(detect_bag_mode "$bag")"
  if [[ "${BAG_MODES[$bag]}" == "unsupported" ]]; then
    die "Unsupported or missing image topic in bag: $bag"
  fi
  if [[ "${BAG_MODES[$bag]}" != "raw_image_raw" ]]; then
    NEEDS_NORMALIZATION=1
  fi
  echo "[info] $(basename "$bag"): ${BAG_MODES[$bag]}"
done

PREPROCESS_INPUT_DIR="$DATASET_DIR"
if (( NEEDS_NORMALIZATION == 1 )); then
  mkdir -p "$NORMALIZED_DATASET_DIR"
  start_roscore_if_needed
  rosparam set /use_sim_time false >/dev/null 2>&1 || true

  for bag in "${BAGS[@]}"; do
    normalize_bag_to_raw_image_topic "$bag" "${BAG_MODES[$bag]}"
  done

  PREPROCESS_INPUT_DIR="$NORMALIZED_DATASET_DIR"
else
  echo "[info] all bags already contain ${RAW_IMAGE_TOPIC}; no normalization needed"
fi

if [[ "$STOP_AFTER" == "convert" ]]; then
  echo "[done] normalized dataset directory: $PREPROCESS_INPUT_DIR"
  exit 0
fi

start_roscore_if_needed

echo "[step] preprocess"
PREPROCESS_CMD=(
  rosrun direct_visual_lidar_calibration preprocess
  -a
)
if (( PREPROCESS_VISUALIZE == 1 )); then
  PREPROCESS_CMD+=(-v)
  run_with_shim "${PREPROCESS_CMD[@]}" "$PREPROCESS_INPUT_DIR" "$RUN_DIR"
else
  "${PREPROCESS_CMD[@]}" "$PREPROCESS_INPUT_DIR" "$RUN_DIR"
fi

[[ -f "$RUN_DIR/calib.json" ]] || die "Preprocess did not create calib.json in $RUN_DIR"

if [[ "$STOP_AFTER" == "preprocess" ]]; then
  echo "[done] preprocess output: $RUN_DIR"
  exit 0
fi

echo "[step] manual initial guess"
echo "[note] In the manual viewer, 'Estimate' does not save the result."
echo "[note] After estimating, click the 'Save' button, confirm it says 'saved to .../calib.json', then close the viewer."
MANUAL_RC=0
run_with_shim rosrun direct_visual_lidar_calibration initial_guess_manual "$RUN_DIR" || MANUAL_RC=$?

[[ -f "$RUN_DIR/calib.json" ]] || die "calib.json is missing after manual initial guess"
pretty_print_calib_json "$RUN_DIR/calib.json" "$LOG_DIR/calib_after_manual.pretty.json"
if assert_manual_result_present "$RUN_DIR/calib.json"; then
  if (( MANUAL_RC != 0 )); then
    warn "Manual initial guess exited with code ${MANUAL_RC}, but init_T_lidar_camera was saved. Continuing."
  fi
else
  die "Manual initial guess was not saved. Click 'Save' in the manual viewer before closing it. Expected results.init_T_lidar_camera in $RUN_DIR/calib.json"
fi

if [[ "$STOP_AFTER" == "manual" ]]; then
  echo "[done] manual initial guess saved in $RUN_DIR/calib.json"
  exit 0
fi

echo "[step] calibrate"
CALIBRATE_RC=0
run_with_shim rosrun direct_visual_lidar_calibration calibrate "$RUN_DIR" || CALIBRATE_RC=$?

[[ -f "$RUN_DIR/calib.json" ]] || die "calib.json is missing after calibration"
pretty_print_calib_json "$RUN_DIR/calib.json" "$LOG_DIR/calib_after_calibrate.pretty.json"
if assert_final_result_present "$RUN_DIR/calib.json"; then
  if (( CALIBRATE_RC != 0 )); then
    warn "Calibration exited with code ${CALIBRATE_RC}, but T_lidar_camera was saved. Continuing."
  fi
else
  die "Calibration result was not saved. Expected results.T_lidar_camera in $RUN_DIR/calib.json"
fi

if [[ "$STOP_AFTER" == "calibrate" ]]; then
  echo "[done] calibration completed: $RUN_DIR/calib.json"
  exit 0
fi

echo "[step] viewer"
echo "[note] The viewer may segfault on close under WSL/GLFW teardown even after it has worked correctly."
VIEWER_RC=0
run_with_shim rosrun direct_visual_lidar_calibration viewer "$RUN_DIR" || VIEWER_RC=$?
if (( VIEWER_RC != 0 )); then
  warn "Viewer exited with code ${VIEWER_RC}."
fi

echo "[done] calibration workflow completed"
echo "       preprocess input: $PREPROCESS_INPUT_DIR"
echo "       run directory:    $RUN_DIR"
