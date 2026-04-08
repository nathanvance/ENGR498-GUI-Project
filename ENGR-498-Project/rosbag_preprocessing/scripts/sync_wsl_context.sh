#!/usr/bin/env bash
set -euo pipefail

# Maintainer-only helper.
# This refreshes the committed runtime staged in rt/ from a known-
# good local WSL development environment. Normal users should not need to run
# this before building the Docker image.

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RT_ROOT="${PROJECT_ROOT}/rt"
RUNTIME_USER="${PORTABLE_ROS_RUNTIME_USER:-portable}"
CONTAINER_HOME_ROOT="/home/${RUNTIME_USER}"
STAGE_CALIB_ROOT="${RT_ROOT}/calib"
STAGE_LIVOX_ROOT="${RT_ROOT}/livox"
STAGE_IRI_ROOT="${RT_ROOT}/iri"
STAGE_LIB_ROOT="${RT_ROOT}/lib"
STAGE_USR_ROOT="${RT_ROOT}/usr"
OVERRIDES_ROOT="${PROJECT_ROOT}/overrides"
DEFAULT_WSL_STAGING_ROOT="${HOME}/Senior_Design_Docker_Image"
WSL_HOME_ROOT="${WSL_HOME_ROOT:-${DEFAULT_WSL_STAGING_ROOT}}"
WS_CALIB_ROOT="${WS_CALIB_ROOT:-${WSL_HOME_ROOT}/ws_calib}"
WS_LIVOX_ROOT="${WS_LIVOX_ROOT:-${WSL_HOME_ROOT}/ws_livox}"
IRIDESCENCE_ROOT="${IRIDESCENCE_ROOT:-${WSL_HOME_ROOT}/iridescence}"
GLFW_SHIM_PATH="${GLFW_SHIM_PATH:-${WSL_HOME_ROOT}/lib/libglfw_hint_shim.so}"
USR_LOCAL_PREFIX="${USR_LOCAL_PREFIX:-/usr/local}"

mkdir -p "${WSL_HOME_ROOT}"

rm -rf "${RT_ROOT}"
mkdir -p \
  "${STAGE_CALIB_ROOT}" \
  "${STAGE_LIVOX_ROOT}" \
  "${STAGE_IRI_ROOT}/build" \
  "${STAGE_LIB_ROOT}" \
  "${STAGE_USR_ROOT}/lib" \
  "${STAGE_USR_ROOT}/share"

need_cmd() {
  command -v "$1" >/dev/null 2>&1 || {
    echo "ERROR: required command not found: $1" >&2
    exit 2
  }
}

need_cmd rsync

require_dir() {
  local path="$1"
  local label="$2"
  [[ -d "$path" ]] || MISSING_INPUTS+=("${label}: ${path}")
}

require_file() {
  local path="$1"
  local label="$2"
  [[ -f "$path" ]] || MISSING_INPUTS+=("${label}: ${path}")
}

sync_tree() {
  local src="$1"
  local dst="$2"
  shift 2

  mkdir -p "$(dirname "$dst")"
  rsync -a --delete "$@" "${src}/" "${dst}/"
}

sync_file() {
  local src="$1"
  local dst="$2"
  mkdir -p "$(dirname "$dst")"
  cp -f "$src" "$dst"
}

rewrite_catkin_marker() {
  local marker_path="$1"
  local source_root="$2"

  if [[ ! -f "$marker_path" ]]; then
    return 0
  fi

  printf '%s\n' "$source_root" >"$marker_path"
}

patch_container_shell_scripts() {
  python3 - "$RT_ROOT" "$CONTAINER_HOME_ROOT" <<'PY'
from pathlib import Path
import sys

rt_root = Path(sys.argv[1])
container_home = Path(sys.argv[2])
calib_root = rt_root / "calib"
livox_root = rt_root / "livox"


def replace_or_raise(text: str, needle: str, replacement: str, *, path: Path) -> str:
    if needle in text:
        return text.replace(needle, replacement)
    if replacement in text:
        return text
    raise RuntimeError(f"Expected text not found while patching {path}: {needle!r}")

safe_blocks = {
    calib_root / "scripts/run_direct_visual_lidar_calibration_workflow.sh": (
        'source /opt/ros/noetic/setup.bash\nsource "$HOME/ws_calib/devel/setup.bash"',
        'export ROS_DISTRO="${ROS_DISTRO:-noetic}"\n'
        'set +u\n'
        'source /opt/ros/${ROS_DISTRO}/setup.bash\n'
        'source "$HOME/ws_calib/devel/setup.bash"\n'
        'export ROS_PACKAGE_PATH="$HOME/ws_calib/src${ROS_PACKAGE_PATH:+:$ROS_PACKAGE_PATH}"\n'
        'set -u',
    ),
    livox_root / "scripts/run_pose_recovery_camera_gps.sh": (
        'source /opt/ros/noetic/setup.bash\nsource "$HOME/ws_livox/devel/setup.bash"',
        'export ROS_DISTRO="${ROS_DISTRO:-noetic}"\n'
        'set +u\n'
        'source /opt/ros/${ROS_DISTRO}/setup.bash\n'
        'source "$HOME/ws_livox/devel/setup.bash"\n'
        'export ROS_PACKAGE_PATH="$HOME/ws_livox/src${ROS_PACKAGE_PATH:+:$ROS_PACKAGE_PATH}"\n'
        'set -u',
    ),
}

for path, (needle, replacement) in safe_blocks.items():
    text = path.read_text(encoding="utf-8")
    text = replace_or_raise(text, needle, replacement, path=path)
    path.write_text(text, encoding="utf-8")

calibration_workflow = calib_root / "scripts/run_direct_visual_lidar_calibration_workflow.sh"
text = calibration_workflow.read_text(encoding="utf-8")
text = replace_or_raise(
    text,
    'DATASET_DIR="$(readlink -f "$DATASET_DIR")"',
    'if [[ "$DATASET_DIR" != /* ]]; then\n'
    '  DATASET_DIR="$PWD/$DATASET_DIR"\n'
    'fi',
    path=calibration_workflow,
)
text = replace_or_raise(
    text,
    'RUN_ROOT="$HOME/calibration_runs"',
    'RUN_ROOT="${PORTABLE_ROS_CALIBRATION_OUTPUT_ROOT:-${PORTABLE_ROS_OUTPUT_ROOT:-$HOME/calibration_runs}}"',
    path=calibration_workflow,
)
text = replace_or_raise(
    text,
    '      Default: ~/calibration_runs',
    '      Default: $PORTABLE_ROS_CALIBRATION_OUTPUT_ROOT, otherwise\n'
    '      $PORTABLE_ROS_OUTPUT_ROOT, otherwise ~/calibration_runs',
    path=calibration_workflow,
)
text = replace_or_raise(
    text,
    '  GUIK_FORCE_SOFTWARE_CURSOR="${GUIK_FORCE_SOFTWARE_CURSOR:-1}" \\\n'
    '  LD_LIBRARY_PATH="$gui_ld_path" \\\n'
    '  LD_PRELOAD="$SHIM_PATH" \\\n'
    '  "$@"\n',
    '  LD_LIBRARY_PATH="$gui_ld_path" \\\n'
    '  LD_PRELOAD="$SHIM_PATH" \\\n'
    '  "$@"\n',
    path=calibration_workflow,
)
calibration_workflow.write_text(text, encoding="utf-8")

pose_recovery_camera_gps = livox_root / "scripts/run_pose_recovery_camera_gps.sh"
text = pose_recovery_camera_gps.read_text(encoding="utf-8")
text = replace_or_raise(
    text,
    'OUTPUT_ROOT="$HOME/ws_livox/pose_recovery_outputs"',
    'OUTPUT_ROOT="${PORTABLE_ROS_POSE_OUTPUT_ROOT:-${PORTABLE_ROS_OUTPUT_ROOT:-$HOME/ws_livox/pose_recovery_outputs}}"',
    path=pose_recovery_camera_gps,
)
text = replace_or_raise(
    text,
    '  run_pose_recovery_camera_gps.sh ~/ws_livox/bags/my_run.bag',
    '  run_pose_recovery_camera_gps.sh ./data/my_run.bag',
    path=pose_recovery_camera_gps,
)
pose_recovery_camera_gps.write_text(text, encoding="utf-8")

pose_recovery = livox_root / "scripts/run_pose_recovery.sh"
text = pose_recovery.read_text(encoding="utf-8")
text = replace_or_raise(
    text,
    'source /opt/ros/noetic/setup.bash\nsource ~/ws_livox/devel/setup.bash\n\n#!/usr/bin/env bash\n',
    '#!/usr/bin/env bash\n',
    path=pose_recovery,
)
marker = 'set -euo pipefail\n\n'
insertion = (
    'set -euo pipefail\n\n'
    'export ROS_DISTRO="${ROS_DISTRO:-noetic}"\n'
    'set +u\n'
    'source /opt/ros/${ROS_DISTRO}/setup.bash\n'
    'source "$HOME/ws_livox/devel/setup.bash"\n'
    'export ROS_PACKAGE_PATH="$HOME/ws_livox/src${ROS_PACKAGE_PATH:+:$ROS_PACKAGE_PATH}"\n'
    'set -u\n\n'
)
if marker in text and 'source "$HOME/ws_livox/devel/setup.bash"' not in text:
    text = text.replace(marker, insertion, 1)
pose_recovery.write_text(text, encoding="utf-8")
PY
}

sync_glob() {
  local pattern="$1"
  local dst_dir="$2"
  mkdir -p "$dst_dir"
  shopt -s nullglob
  for path in $pattern; do
    cp -a "$path" "$dst_dir/"
  done
  shopt -u nullglob
}

MISSING_INPUTS=()
require_dir "${WS_CALIB_ROOT}" "Required WSL calibration workspace root"
require_dir "${WS_CALIB_ROOT}/devel" "Required WSL calibration workspace devel space"
require_dir "${WS_CALIB_ROOT}/src/direct_visual_lidar_calibration" "Required WSL calibration source package"
require_dir "${WS_CALIB_ROOT}/scripts" "Required WSL calibration scripts"
require_dir "${WS_LIVOX_ROOT}" "Required WSL Livox workspace root"
require_dir "${WS_LIVOX_ROOT}/devel" "Required WSL Livox workspace devel space"
require_dir "${WS_LIVOX_ROOT}/src/FAST_LIO" "Required WSL FAST_LIO source tree"
require_dir "${WS_LIVOX_ROOT}/src/livox_ros_driver" "Required WSL livox_ros_driver source tree"
require_dir "${WS_LIVOX_ROOT}/scripts" "Required WSL Livox scripts"
require_dir "${IRIDESCENCE_ROOT}" "Required WSL iridescence root"
require_dir "${IRIDESCENCE_ROOT}/build" "Required WSL iridescence build output directory"
require_file "${GLFW_SHIM_PATH}" "Required GLFW shim library"

if [[ ${#MISSING_INPUTS[@]} -gt 0 ]]; then
  echo "ERROR: sync_wsl_context.sh cannot refresh the embedded Docker runtime on this machine." >&2
  echo "This script is maintainer-only and requires an existing local WSL development runtime." >&2
  echo >&2
  echo "Missing prerequisites:" >&2
  for item in "${MISSING_INPUTS[@]}"; do
    echo "  - ${item}" >&2
  done
  echo >&2
  echo "Default maintainer staging root:" >&2
  echo "  ${WSL_HOME_ROOT}" >&2
  echo >&2
  echo "Normal users do not need this script. Use build_image_wsl.sh to build from the" >&2
  echo "runtime already committed to the repo." >&2
  exit 2
fi

sync_tree "${WS_CALIB_ROOT}/devel" "${STAGE_CALIB_ROOT}/devel"
rewrite_catkin_marker "${STAGE_CALIB_ROOT}/devel/.catkin" "${CONTAINER_HOME_ROOT}/ws_calib/src"

sync_tree "${WS_CALIB_ROOT}/src/direct_visual_lidar_calibration" "${STAGE_CALIB_ROOT}/src/direct_visual_lidar_calibration" \
  --exclude .git \
  --exclude build \
  --exclude devel \
  --exclude docker \
  --exclude docs \
  --exclude thirdparty

sync_tree "${WS_CALIB_ROOT}/scripts" "${STAGE_CALIB_ROOT}/scripts" \
  --exclude __pycache__

sync_tree "${WS_LIVOX_ROOT}/devel" "${STAGE_LIVOX_ROOT}/devel"
rewrite_catkin_marker "${STAGE_LIVOX_ROOT}/devel/.catkin" "${CONTAINER_HOME_ROOT}/ws_livox/src"

sync_tree "${WS_LIVOX_ROOT}/src/FAST_LIO" "${STAGE_LIVOX_ROOT}/src/FAST_LIO" \
  --exclude .git \
  --exclude doc \
  --exclude Log \
  --exclude PCD \
  --exclude build \
  --exclude devel

sync_tree "${WS_LIVOX_ROOT}/src/livox_ros_driver" "${STAGE_LIVOX_ROOT}/src/livox_ros_driver" \
  --exclude .git

sync_tree "${WS_LIVOX_ROOT}/scripts" "${STAGE_LIVOX_ROOT}/scripts" \
  --exclude __pycache__ \
  --exclude run_logs \
  --exclude pose_recovery_outputs

if [[ -d "${OVERRIDES_ROOT}/ws_livox/scripts" ]]; then
  rsync -a "${OVERRIDES_ROOT}/ws_livox/scripts/" "${STAGE_LIVOX_ROOT}/scripts/"
fi

sync_glob "${IRIDESCENCE_ROOT}/build/libiridescence.so*" "${STAGE_IRI_ROOT}/build"
sync_file "${GLFW_SHIM_PATH}" "${STAGE_LIB_ROOT}/libglfw_hint_shim.so"

sync_glob "${USR_LOCAL_PREFIX}/lib/libgtsam.so*" "${STAGE_USR_ROOT}/lib"
sync_glob "${USR_LOCAL_PREFIX}/lib/libgtsam_unstable.so*" "${STAGE_USR_ROOT}/lib"
sync_glob "${USR_LOCAL_PREFIX}/lib/libmetis-gtsam.so*" "${STAGE_USR_ROOT}/lib"
sync_glob "${USR_LOCAL_PREFIX}/lib/libiridescence.so*" "${STAGE_USR_ROOT}/lib"
sync_glob "${USR_LOCAL_PREFIX}/lib/libceres.so*" "${STAGE_USR_ROOT}/lib"
sync_tree "${USR_LOCAL_PREFIX}/share/iridescence" "${STAGE_USR_ROOT}/share/iridescence"
patch_container_shell_scripts

echo "[done] synced WSL runtime artifacts into ${RT_ROOT}"
