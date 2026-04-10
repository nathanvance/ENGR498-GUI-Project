#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  run_pose_recovery_camera_gps.sh [bag_path] [--output-root DIR] [--image-topic TOPIC] [--gps-topic TOPIC] [--camera-time-offset-sec OFFSET] [--master-port PORT] [--remap RULE] [--rviz]

Examples:
  run_pose_recovery_camera_gps.sh ~/ws_livox/bags/my_run.bag
  run_pose_recovery_camera_gps.sh ~/ws_livox/bags/my_run.bag --image-topic /camera/image/compressed --gps-topic /fix
  run_pose_recovery_camera_gps.sh ~/ws_livox/bags/my_run.bag --rviz
EOF
}

RUNTIME_HOME="${PORTABLE_ROS_HOME:-/home/portable}"
if [[ ! -d "${RUNTIME_HOME}/ws_livox" ]]; then
  RUNTIME_HOME="/home/portable"
fi

resolve_bag() {
  local bag="$1"
  if [[ "$bag" != /* ]]; then
    [[ -f "$PWD/$bag" ]] && { echo "$PWD/$bag"; return 0; }
    [[ -f "$RUNTIME_HOME/ws_livox/bags/$bag" ]] && { echo "$RUNTIME_HOME/ws_livox/bags/$bag"; return 0; }
  fi
  echo "$bag"
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

wait_for_file() {
  local path="$1"
  local timeout_s="${2:-20}"
  local start
  start="$(date +%s)"
  while true; do
    [[ -f "$path" ]] && return 0
    if (( $(date +%s) - start >= timeout_s )); then
      return 1
    fi
    sleep 0.2
  done
}

wait_for_rosnode_exit() {
  local node_name="$1"
  local timeout_s="${2:-20}"
  local start
  start="$(date +%s)"
  while true; do
    if ! rosnode list 2>/dev/null | grep -Fxq "$node_name"; then
      return 0
    fi
    if (( $(date +%s) - start >= timeout_s )); then
      return 1
    fi
    sleep 0.2
  done
}

score_image_topic() {
  local topic="$1"
  local score=0
  [[ "$topic" == *"/compressed"* ]] && score=$((score + 100))
  [[ "$topic" == *compress* ]] && score=$((score + 90))
  [[ "$topic" == *image* ]] && score=$((score + 20))
  [[ "$topic" == *camera* ]] && score=$((score + 10))
  printf '%d\n' "$score"
}

score_gps_topic() {
  local topic="$1"
  local score=0
  [[ "$topic" == */fix || "$topic" == */fix/* ]] && score=$((score + 100))
  [[ "$topic" == *gps* ]] && score=$((score + 40))
  [[ "$topic" == *gnss* ]] && score=$((score + 30))
  [[ "$topic" == *navsat* ]] && score=$((score + 20))
  printf '%d\n' "$score"
}

choose_best_topic() {
  local kind="$1"
  shift || true
  local best=""
  local best_score=-1
  local topic score
  for topic in "$@"; do
    [[ -n "$topic" ]] || continue
    if [[ "$kind" == "image" ]]; then
      score="$(score_image_topic "$topic")"
    else
      score="$(score_gps_topic "$topic")"
    fi
    if (( score > best_score )); then
      best="$topic"
      best_score="$score"
    fi
  done
  printf '%s\n' "$best"
}

detect_bag_topic() {
  local bag="$1"
  local kind="$2"
  local -a candidates=()
  if [[ "$kind" == "image" ]]; then
    mapfile -t candidates < <(rosbag info "$bag" | awk '/sensor_msgs\/CompressedImage|sensor_msgs\/Image/ {print $1}')
  else
    mapfile -t candidates < <(rosbag info "$bag" | awk '/sensor_msgs\/NavSatFix/ {print $1}')
  fi

  if ((${#candidates[@]} == 0)); then
    printf '\n'
    return 0
  fi

  choose_best_topic "$kind" "${candidates[@]}"
}

inspect_topic_message() {
  local bag="$1"
  local topic="$2"
  python3 - "$bag" "$topic" <<'PY'
import rosbag
import sys

bag_path, topic = sys.argv[1:3]
message_type = ""
field_names = []

with rosbag.Bag(bag_path) as bag:
    for _, msg, _ in bag.read_messages(topics=[topic]):
        message_type = getattr(msg, "_type", "")
        fields = getattr(msg, "fields", None)
        if fields is not None:
            field_names = [field.name for field in fields]
        break

print(message_type)
print(",".join(field_names))
PY
}

port_is_available() {
  local port="$1"
  python3 - "$port" <<'PY'
import socket
import sys

port = int(sys.argv[1])
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
try:
    sock.bind(("127.0.0.1", port))
except OSError:
    sys.exit(1)
finally:
    sock.close()
PY
}

choose_master_port() {
  local requested="$1"
  local candidate
  for candidate in $(seq "$requested" $((requested + 20))); do
    if port_is_available "$candidate"; then
      printf '%s\n' "$candidate"
      return 0
    fi
  done
  return 1
}

resolve_fastlio_command() {
  local package_root=""
  local launch_path=""
  if package_root="$(rospack find "$FASTLIO_PKG" 2>/dev/null)"; then
    if [[ -f "$package_root/launch/$FASTLIO_LAUNCH" ]]; then
      FASTLIO_CMD=(roslaunch "$FASTLIO_PKG" "$FASTLIO_LAUNCH" "${RVIZ_ARG_NAME}:=${RVIZ_ARG_VALUE}")
      return 0
    fi
  fi

  launch_path="$RUNTIME_HOME/ws_livox/src/FAST_LIO/launch/$FASTLIO_LAUNCH"
  if [[ -f "$launch_path" ]]; then
    FASTLIO_CMD=(roslaunch "$launch_path" "${RVIZ_ARG_NAME}:=${RVIZ_ARG_VALUE}")
    return 0
  fi

  return 1
}

BAG_IN="$RUNTIME_HOME/ws_livox/bags/movingtest1.bag"
OUTPUT_ROOT="${PORTABLE_ROS_POSE_OUTPUT_ROOT:-${PORTABLE_ROS_OUTPUT_ROOT:-$RUNTIME_HOME/ws_livox/pose_recovery_outputs}}"
IMAGE_TOPIC_OVERRIDE=""
GPS_TOPIC_OVERRIDE=""
MASTER_PORT="${MASTER_PORT:-11311}"
CAMERA_TIME_OFFSET_SEC="${CAMERA_TIME_OFFSET_SEC:-0.0}"
UNPAUSE_DELAY_MS="${UNPAUSE_DELAY_MS:-3000}"
DISCOVERY_TIMEOUT_WALL_SEC="${DISCOVERY_TIMEOUT_WALL_SEC:-30}"
REMAPS=()
ENABLE_RVIZ=0

while [[ $# -gt 0 ]]; do
  case "$1" in
    -h|--help)
      usage
      exit 0
      ;;
    --output-root)
      OUTPUT_ROOT="$2"
      shift 2
      ;;
    --image-topic)
      IMAGE_TOPIC_OVERRIDE="$2"
      shift 2
      ;;
    --gps-topic)
      GPS_TOPIC_OVERRIDE="$2"
      shift 2
      ;;
    --master-port)
      MASTER_PORT="$2"
      shift 2
      ;;
    --camera-time-offset-sec)
      CAMERA_TIME_OFFSET_SEC="$2"
      shift 2
      ;;
    --remap)
      REMAPS+=("$2")
      shift 2
      ;;
    --rviz)
      ENABLE_RVIZ=1
      shift
      ;;
    --*)
      echo "ERROR: Unknown option: $1" >&2
      exit 2
      ;;
    *)
      if [[ "$BAG_IN" == "$RUNTIME_HOME/ws_livox/bags/movingtest1.bag" ]]; then
        BAG_IN="$1"
      else
        echo "ERROR: Unexpected positional argument: $1" >&2
        exit 2
      fi
      shift
      ;;
  esac
done

set --
export ROS_DISTRO="${ROS_DISTRO:-noetic}"
set +u
source /opt/ros/${ROS_DISTRO}/setup.bash
source "$RUNTIME_HOME/ws_livox/devel/setup.bash"
export ROS_PACKAGE_PATH="$RUNTIME_HOME/ws_livox/src${ROS_PACKAGE_PATH:+:$ROS_PACKAGE_PATH}"
set -u

BAG="$(resolve_bag "$BAG_IN")"
[[ -f "$BAG" ]] || { echo "ERROR: Bag file not found: $BAG" >&2; exit 2; }
[[ -r "$BAG" ]] || { echo "ERROR: Bag file not readable: $BAG" >&2; exit 2; }

command -v expect >/dev/null 2>&1 || {
  echo "ERROR: expect not installed. Run: sudo apt-get install -y expect" >&2
  exit 2
}

BAG_BASE="$(basename "$BAG")"
BAG_STEM="${BAG_BASE%.bag}"
RUN_ID="$(date +%Y%m%d_%H%M%S)_$$"
RUN_OUT_DIR="$OUTPUT_ROOT/${BAG_STEM}_${RUN_ID}"
LOG_DIR="$RUN_OUT_DIR/logs"
PCD_OUT_DIR="$RUN_OUT_DIR/pcd"
IMAGE_OUT_DIR="$RUN_OUT_DIR/images"
mkdir -p "$LOG_DIR" "$PCD_OUT_DIR" "$IMAGE_OUT_DIR"

CAMERA_OUT_CSV="$RUN_OUT_DIR/tf_camera_out.csv"
GPS_OUT_CSV="$RUN_OUT_DIR/tf_gps_out.csv"
IMAGE_TIMESTAMPS_CSV="$RUN_OUT_DIR/image_timestamps.csv"
DENSE_TRAJ_CSV="$RUN_OUT_DIR/tf_dense_trajectory.csv"
DENSE_TRAJ_INTERVAL_SEC="${DENSE_TRAJ_INTERVAL_SEC:-0.010}"
FASTLIO_LOG="$LOG_DIR/fastlio.log"
ROSBAG_LOG="$LOG_DIR/rosbag.log"
SAMPLER_LOG="$LOG_DIR/sampler.log"
ROSCORE_LOG="$LOG_DIR/roscore.log"
CONVERTER_LOG="$LOG_DIR/livox_converter.log"

FASTLIO_PKG="fast_lio"
FASTLIO_LAUNCH="mapping_horizon.launch"
FASTLIO_NODE_NAME="/laserMapping"
RVIZ_ARG_NAME="rviz"
RVIZ_ARG_VALUE="false"
if (( ENABLE_RVIZ == 1 )); then
  RVIZ_ARG_VALUE="true"
fi

FASTLIO_PCD_DIR="$RUNTIME_HOME/ws_livox/src/FAST_LIO/PCD"
FASTLIO_PCD_FILE="$FASTLIO_PCD_DIR/scans.pcd"
FASTLIO_LOG_DIR="$RUNTIME_HOME/ws_livox/src/FAST_LIO/Log"
LEGACY_FASTLIO_PCD_FILE="/home/leif/ws_livox/src/FAST_LIO/PCD/scans.pcd"
LEGACY_FASTLIO_LOG_DIR="/home/leif/ws_livox/src/FAST_LIO/Log"
mkdir -p "$FASTLIO_PCD_DIR"
mkdir -p "$FASTLIO_LOG_DIR"
mkdir -p "$LEGACY_FASTLIO_LOG_DIR"
rm -f "$FASTLIO_PCD_FILE"
rm -f "$LEGACY_FASTLIO_PCD_FILE"

IMAGE_TOPIC="$IMAGE_TOPIC_OVERRIDE"
GPS_TOPIC="$GPS_TOPIC_OVERRIDE"
LIDAR_TOPIC="${LIDAR_TOPIC:-/livox/lidar}"
LIDAR_RAW_TOPIC="${LIDAR_RAW_TOPIC:-${LIDAR_TOPIC}_raw}"
USE_LIVOX_POINTCLOUD2_RELAY=0

if [[ -z "$IMAGE_TOPIC" ]]; then
  IMAGE_TOPIC="$(detect_bag_topic "$BAG" "image")"
fi
if [[ -z "$GPS_TOPIC" ]]; then
  GPS_TOPIC="$(detect_bag_topic "$BAG" "gps")"
fi

mapfile -t LIDAR_TOPIC_INFO < <(inspect_topic_message "$BAG" "$LIDAR_TOPIC")
LIDAR_MSG_TYPE="${LIDAR_TOPIC_INFO[0]:-}"
LIDAR_FIELD_NAMES="${LIDAR_TOPIC_INFO[1]:-}"

if [[ "$LIDAR_MSG_TYPE" == "sensor_msgs/PointCloud2" ]]; then
  if [[ "$LIDAR_FIELD_NAMES" == *"tag"* && "$LIDAR_FIELD_NAMES" == *"line"* ]]; then
    USE_LIVOX_POINTCLOUD2_RELAY=1
    REMAPS+=("${LIDAR_TOPIC}:=${LIDAR_RAW_TOPIC}")
  else
    echo "ERROR: LiDAR topic ${LIDAR_TOPIC} is sensor_msgs/PointCloud2, but it does not expose Livox tag/line fields for automatic conversion." >&2
    echo "  detected fields: ${LIDAR_FIELD_NAMES:-<none>}" >&2
    exit 2
  fi
elif [[ -n "$LIDAR_MSG_TYPE" && "$LIDAR_MSG_TYPE" != "livox_ros_driver/CustomMsg" ]]; then
  echo "ERROR: Unsupported LiDAR topic type on ${LIDAR_TOPIC}: ${LIDAR_MSG_TYPE}" >&2
  exit 2
fi

echo "[preflight] bag:              $BAG"
echo "[preflight] output dir:       $RUN_OUT_DIR"
echo "[preflight] image topic:      ${IMAGE_TOPIC:-auto-runtime-detect}"
echo "[preflight] gps topic:        ${GPS_TOPIC:-auto-runtime-detect}"
echo "[preflight] lidar topic:      ${LIDAR_TOPIC}"
echo "[preflight] lidar type:       ${LIDAR_MSG_TYPE:-unknown}"
echo "[preflight] lidar fields:     ${LIDAR_FIELD_NAMES:-unknown}"
echo "[preflight] livox relay:      $([[ "$USE_LIVOX_POINTCLOUD2_RELAY" == "1" ]] && echo enabled || echo disabled)"
echo "[preflight] images dir:       $IMAGE_OUT_DIR"
echo "[preflight] rviz enabled:     ${RVIZ_ARG_VALUE}"

SELECTED_MASTER_PORT="$(choose_master_port "$MASTER_PORT")" || {
  echo "ERROR: Could not find a free ROS master port starting at $MASTER_PORT" >&2
  exit 2
}
if [[ "$SELECTED_MASTER_PORT" != "$MASTER_PORT" ]]; then
  echo "[preflight] requested ROS master port $MASTER_PORT is busy; using $SELECTED_MASTER_PORT instead"
fi
MASTER_PORT="$SELECTED_MASTER_PORT"
export ROS_MASTER_URI="http://127.0.0.1:${MASTER_PORT}"
export ROS_HOME="${ROS_HOME:-$LOG_DIR/ros_home_${RUN_ID}}"
mkdir -p "$ROS_HOME"

cleanup() {
  echo "[cleanup] stopping sampler/bag/fastlio/roscore..."
  [[ -n "${SAMPLER_PID:-}" ]] && kill "$SAMPLER_PID" 2>/dev/null || true
  [[ -n "${ROSBAG_EXPECT_PID:-}" ]] && kill "$ROSBAG_EXPECT_PID" 2>/dev/null || true
  [[ -n "${CONVERTER_PID:-}" ]] && kill "$CONVERTER_PID" 2>/dev/null || true
  [[ -n "${FASTLIO_PID:-}" ]] && kill "$FASTLIO_PID" 2>/dev/null || true

  rosnode kill -a >/dev/null 2>&1 || true
  pkill -f "roscore.*${MASTER_PORT}" >/dev/null 2>&1 || true
  pkill -f "rosmaster.*${MASTER_PORT}" >/dev/null 2>&1 || true
  [[ -n "${ROSCORE_PID:-}" ]] && kill "$ROSCORE_PID" 2>/dev/null || true

  pkill -f rosbag >/dev/null 2>&1 || true
  pkill -f fast_lio >/dev/null 2>&1 || true
  pkill -f tf_sample_camera_gps.py >/dev/null 2>&1 || true
  pkill -f roscore >/dev/null 2>&1 || true
  pkill -f rosmaster >/dev/null 2>&1 || true
}
trap cleanup EXIT

echo "[preflight] Restarting ROS master and ensuring a clean slate..."
pkill -f rosbag >/dev/null 2>&1 || true
pkill -f fast_lio >/dev/null 2>&1 || true
pkill -f tf_sample_camera_gps.py >/dev/null 2>&1 || true
pkill -f roscore >/dev/null 2>&1 || true
pkill -f rosmaster >/dev/null 2>&1 || true
sleep 1

: > "$ROSCORE_LOG"
roscore -p "$MASTER_PORT" >"$ROSCORE_LOG" 2>&1 &
ROSCORE_PID=$!

if ! wait_for_master 10; then
  echo "ERROR: roscore did not come up. See: $ROSCORE_LOG" >&2
  exit 2
fi

echo "  roscore log: $ROSCORE_LOG"
echo "  ROS_MASTER_URI=$ROS_MASTER_URI"
echo "  ROS_HOME=$ROS_HOME"

echo "[1/6] Reset sim time param..."
rosparam set /use_sim_time false
rosparam set /use_sim_time true

echo "[2/6] Start FAST-LIO..."
if (( USE_LIVOX_POINTCLOUD2_RELAY == 1 )); then
  : > "$CONVERTER_LOG"
  python3 "$RUNTIME_HOME/ws_livox/scripts/livox_pointcloud2_to_custommsg.py" \
    _input_topic:="$LIDAR_RAW_TOPIC" \
    _output_topic:="$LIDAR_TOPIC" >"$CONVERTER_LOG" 2>&1 &
  CONVERTER_PID=$!
  echo "  livox relay log: $CONVERTER_LOG"
  sleep 1
  if ! kill -0 "$CONVERTER_PID" 2>/dev/null; then
    echo "ERROR: Livox PointCloud2 relay exited before rosbag replay. See: $CONVERTER_LOG" >&2
    exit 2
  fi
fi
if ! resolve_fastlio_command; then
  echo "ERROR: Could not resolve FAST-LIO launch file '${FASTLIO_LAUNCH}'." >&2
  echo "  RUNTIME_HOME=$RUNTIME_HOME" >&2
  echo "  ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH:-}" >&2
  exit 2
fi
: > "$FASTLIO_LOG"
{
  echo "[fastlio] RUNTIME_HOME=$RUNTIME_HOME"
  echo "[fastlio] ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH:-}"
  if rospack find "$FASTLIO_PKG" >/dev/null 2>&1; then
    echo "[fastlio] rospack find $FASTLIO_PKG => $(rospack find "$FASTLIO_PKG")"
  else
    echo "[fastlio] rospack find $FASTLIO_PKG => <not found>"
  fi
  echo "[fastlio] launch command: ${FASTLIO_CMD[*]}"
} >>"$FASTLIO_LOG"
"${FASTLIO_CMD[@]}" >>"$FASTLIO_LOG" 2>&1 &
FASTLIO_PID=$!
echo "  fast-lio log: $FASTLIO_LOG"
sleep 2
if ! kill -0 "$FASTLIO_PID" 2>/dev/null; then
  echo "ERROR: FAST-LIO exited before rosbag replay. See: $FASTLIO_LOG" >&2
  exit 2
fi

echo "[3/6] Start camera/GPS TF sampler..."
SAMPLER_CMD=(
  python3
  "$RUNTIME_HOME/ws_livox/scripts/tf_sample_camera_gps.py"
  --camera-out-csv "$CAMERA_OUT_CSV"
  --gps-out-csv "$GPS_OUT_CSV"
  --image-output-dir "$IMAGE_OUT_DIR"
  --image-timestamps-csv "$IMAGE_TIMESTAMPS_CSV"
  --discovery-timeout-wall-sec "$DISCOVERY_TIMEOUT_WALL_SEC"
  --camera-time-offset-sec "$CAMERA_TIME_OFFSET_SEC"
  --dense-traj-out-csv "$DENSE_TRAJ_CSV"
  --dense-traj-interval-sec "$DENSE_TRAJ_INTERVAL_SEC"
)
if [[ -n "$IMAGE_TOPIC" ]]; then
  SAMPLER_CMD+=(--image-topic "$IMAGE_TOPIC")
fi
if [[ -n "$GPS_TOPIC" ]]; then
  SAMPLER_CMD+=(--gps-topic "$GPS_TOPIC")
fi

: > "$SAMPLER_LOG"
"${SAMPLER_CMD[@]}" >"$SAMPLER_LOG" 2>&1 &
SAMPLER_PID=$!
echo "  sampler log:  $SAMPLER_LOG"

echo "[4/6] Start rosbag paused, then auto-unpause after ${UNPAUSE_DELAY_MS} ms..."
: > "$ROSBAG_LOG"

ROSBAG_CMD=(rosbag play "$BAG" --clock --pause)
if ((${#REMAPS[@]} > 0)); then
  ROSBAG_CMD+=("${REMAPS[@]}")
fi
ROSBAG_CMD_Q="$(printf '%q ' "${ROSBAG_CMD[@]}")"
export ROSBAG_CMD_Q
export ROSBAG_LOG
export UNPAUSE_DELAY_MS

expect <<'EXP' > /dev/null 2>&1 &
log_user 0
set timeout 120
log_file -a $env(ROSBAG_LOG)

spawn -noecho bash -lc $env(ROSBAG_CMD_Q)

expect {
  -re {Hit space to toggle paused} {}
  -re {\[PAUSED\]} {}
  timeout { exit 3 }
}

after $env(UNPAUSE_DELAY_MS)
send " "

expect eof
EXP

ROSBAG_EXPECT_PID=$!
echo "  rosbag log:   $ROSBAG_LOG"

echo "[5/6] Running. Waiting for sampler to finish..."
wait "$SAMPLER_PID"

echo "[6/6] Stopping rosbag replay and FAST-LIO..."
if [[ -n "${ROSBAG_EXPECT_PID:-}" ]]; then
  kill "$ROSBAG_EXPECT_PID" 2>/dev/null || true
  for _ in {1..30}; do
    if ! kill -0 "$ROSBAG_EXPECT_PID" 2>/dev/null; then
      break
    fi
    sleep 0.1
  done
fi

if rosnode list 2>/dev/null | grep -Fxq "$FASTLIO_NODE_NAME"; then
  echo "[shutdown] Requesting graceful shutdown of $FASTLIO_NODE_NAME..."
  rosnode kill "$FASTLIO_NODE_NAME" >/dev/null 2>&1 || true
  wait_for_rosnode_exit "$FASTLIO_NODE_NAME" 25 || true
fi

if [[ -n "${FASTLIO_PID:-}" ]]; then
  kill -INT "$FASTLIO_PID" 2>/dev/null || true
  for _ in {1..100}; do
    if ! kill -0 "$FASTLIO_PID" 2>/dev/null; then
      break
    fi
    sleep 0.2
  done
fi

PCD_OUT_PATH="$PCD_OUT_DIR/scans.pcd"
if wait_for_file "$FASTLIO_PCD_FILE" 60; then
  cp "$FASTLIO_PCD_FILE" "$PCD_OUT_PATH"
  echo "[done] PCD copied to: $PCD_OUT_PATH"
elif wait_for_file "$LEGACY_FASTLIO_PCD_FILE" 10; then
  cp "$LEGACY_FASTLIO_PCD_FILE" "$PCD_OUT_PATH"
  echo "[done] PCD copied to: $PCD_OUT_PATH"
else
  echo "WARNING: FAST-LIO PCD file was not found at $FASTLIO_PCD_FILE" >&2
fi

python3 "$RUNTIME_HOME/ws_livox/scripts/sanitize_pose_recovery_outputs.py" \
  --gps-csv "$GPS_OUT_CSV" \
  --dense-traj-csv "$DENSE_TRAJ_CSV"

echo "[done] images dir:        $IMAGE_OUT_DIR"
echo "[done] image csv:         $IMAGE_TIMESTAMPS_CSV"
echo "[done] camera csv:        $CAMERA_OUT_CSV"
echo "[done] gps csv:           $GPS_OUT_CSV"
echo "[done] dense traj csv:    $DENSE_TRAJ_CSV"
echo "[done] logs dir:          $LOG_DIR"
