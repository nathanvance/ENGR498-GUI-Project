#!/usr/bin/env bash
set -euo pipefail

export ROS_DISTRO="${ROS_DISTRO:-noetic}"
set +u
source /opt/ros/${ROS_DISTRO}/setup.bash
source "$HOME/ws_livox/devel/setup.bash"
export ROS_PACKAGE_PATH="$HOME/ws_livox/src${ROS_PACKAGE_PATH:+:$ROS_PACKAGE_PATH}"
set -u

BAG_IN="${1:-$HOME/ws_livox/bags/movingtest1.bag}"
IN_CSV="${2:-$HOME/ws_livox/scripts/test_times.csv}"
OUT_CSV="${3:-$HOME/ws_livox/scripts/tf_out.csv}"

# Optional remaps, examples:
# REMAPS=(/livox/lidar:=/livox/lidar_points /imu/data:=/livox/imu)
REMAPS=()

FASTLIO_PKG="fast_lio"
FASTLIO_LAUNCH="mapping_horizon.launch"
RVIZ_ARG_NAME="rviz"
RVIZ_ARG_VALUE="false"
FASTLIO_CMD=(roslaunch "$FASTLIO_PKG" "$FASTLIO_LAUNCH" "${RVIZ_ARG_NAME}:=${RVIZ_ARG_VALUE}")

SAMPLER_CMD=("$HOME/ws_livox/scripts/tf_sample_csv.py" --in_csv "$IN_CSV" --out_csv "$OUT_CSV")

LOG_DIR="${LOG_DIR:-$HOME/ws_livox/scripts/run_logs}"
mkdir -p "$LOG_DIR"
FASTLIO_LOG="$LOG_DIR/fastlio.log"
ROSBAG_LOG="$LOG_DIR/rosbag.log"
SAMPLER_LOG="$LOG_DIR/sampler.log"
ROSCORE_LOG="$LOG_DIR/roscore.log"

resolve_bag() {
  local bag="$1"
  if [[ "$bag" != /* ]]; then
    [[ -f "$PWD/$bag" ]] && { echo "$PWD/$bag"; return 0; }
    [[ -f "$HOME/ws_livox/bags/$bag" ]] && { echo "$HOME/ws_livox/bags/$bag"; return 0; }
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

wait_for_clock_tick() {
  # Wait until /clock exists and publishes non-zero time at least once.
  local timeout_s="${1:-10}"
  local start
  start="$(date +%s)"
  while true; do
    # rostopic echo returns quickly with -n 1 if /clock is active.
    if rostopic echo -n 1 /clock >/dev/null 2>&1; then
      return 0
    fi
    if (( $(date +%s) - start >= timeout_s )); then
      return 1
    fi
    sleep 0.1
  done
}

# ---- Per-run isolated ROS master + ROS_HOME ----
RUN_ID="$(date +%Y%m%d_%H%M%S)_$$"
MASTER_PORT="${MASTER_PORT:-11311}"   # can override if you want
export ROS_MASTER_URI="http://127.0.0.1:${MASTER_PORT}"
export ROS_HOME="${ROS_HOME:-$LOG_DIR/ros_home_${RUN_ID}}"
mkdir -p "$ROS_HOME"

cleanup() {
  echo "[cleanup] stopping sampler/bag/fastlio/roscore..."

  [[ -n "${SAMPLER_PID:-}" ]] && kill "$SAMPLER_PID" 2>/dev/null || true
  [[ -n "${ROSBAG_EXPECT_PID:-}" ]] && kill "$ROSBAG_EXPECT_PID" 2>/dev/null || true
  [[ -n "${FASTLIO_PID:-}" ]] && kill "$FASTLIO_PID" 2>/dev/null || true

  # Try to stop all nodes registered to this master cleanly, then hard kill.
  rosnode kill -a >/dev/null 2>&1 || true
  pkill -f "roscore.*${MASTER_PORT}" >/dev/null 2>&1 || true
  pkill -f "rosmaster.*${MASTER_PORT}" >/dev/null 2>&1 || true

  [[ -n "${ROSCORE_PID:-}" ]] && kill "$ROSCORE_PID" 2>/dev/null || true

  # Last resort: kill any remaining roscore/rosmaster (scorched earth).
  pkill -f roscore >/dev/null 2>&1 || true
  pkill -f rosmaster >/dev/null 2>&1 || true
}
trap cleanup EXIT

BAG="$(resolve_bag "$BAG_IN")"
[[ -f "$BAG" ]] || { echo "ERROR: Bag file not found: $BAG" >&2; exit 2; }
[[ -r "$BAG" ]] || { echo "ERROR: Bag file not readable: $BAG" >&2; exit 2; }
[[ -f "$IN_CSV" ]] || { echo "ERROR: Input CSV not found: $IN_CSV" >&2; exit 2; }

command -v expect >/dev/null 2>&1 || {
  echo "ERROR: expect not installed. Run: sudo apt-get install -y expect" >&2
  exit 2
}

echo "[preflight] Restarting ROS master and ensuring a clean slate..."

# Kill any existing master on this port (and other leftovers)
pkill -f rosbag >/dev/null 2>&1 || true
pkill -f fast_lio >/dev/null 2>&1 || true
pkill -f tf_sample_csv.py >/dev/null 2>&1 || true
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

echo "[2/6] Start FAST-LIO (RViz disabled via ${RVIZ_ARG_NAME}:=${RVIZ_ARG_VALUE})..."
: > "$FASTLIO_LOG"
"${FASTLIO_CMD[@]}" >"$FASTLIO_LOG" 2>&1 &
FASTLIO_PID=$!
echo "  fast-lio log: $FASTLIO_LOG"

echo "[3/6] Start rosbag PAUSED, then auto-unpause (space) via expect..."
: > "$ROSBAG_LOG"

ROSBAG_CMD=(rosbag play "$BAG" --clock --pause)
if ((${#REMAPS[@]} > 0)); then
  ROSBAG_CMD+=("${REMAPS[@]}")
fi
ROSBAG_CMD_Q="$(printf '%q ' "${ROSBAG_CMD[@]}")"
export ROSBAG_CMD_Q
export ROSBAG_LOG

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

after 300
send " "

expect eof
EXP

ROSBAG_EXPECT_PID=$!
echo "  rosbag log:   $ROSBAG_LOG"

# Determinism gate: don’t start sampler until simulated time is definitely flowing
if ! wait_for_clock_tick 10; then
  echo "ERROR: /clock did not tick after starting rosbag. Check: $ROSBAG_LOG" >&2
  exit 3
fi

echo "[4/6] Start sampler (output suppressed to log)..."
: > "$SAMPLER_LOG"
"${SAMPLER_CMD[@]}" >"$SAMPLER_LOG" 2>&1 &
SAMPLER_PID=$!
echo "  sampler log:  $SAMPLER_LOG"

echo "[5/6] Running. Waiting for sampler to finish..."
wait "$SAMPLER_PID"

echo "[6/6] Sampler finished - stopping rosbag replay..."
if [[ -n "${ROSBAG_EXPECT_PID:-}" ]]; then
  # Killing expect will also terminate the rosbag process running in its PTY.
  kill "$ROSBAG_EXPECT_PID" 2>/dev/null || true

  for _ in {1..30}; do
    if ! kill -0 "$ROSBAG_EXPECT_PID" 2>/dev/null; then
      break
    fi
    sleep 0.1
  done
fi

echo "[done] Output:        $OUT_CSV"
echo "       fast-lio log:  $FASTLIO_LOG"
echo "       sampler log:   $SAMPLER_LOG"
echo "       rosbag log:    $ROSBAG_LOG"
echo "       roscore log:   $ROSCORE_LOG"
