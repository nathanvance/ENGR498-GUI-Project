#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CONTEXT_ROOT="${PROJECT_ROOT}/context/runtime"
RUNTIME_USER="${PORTABLE_ROS_RUNTIME_USER:-portable}"
HOME_ROOT="${CONTEXT_ROOT}/home/${RUNTIME_USER}"
USR_LOCAL_ROOT="${CONTEXT_ROOT}/usr_local"
OVERRIDES_ROOT="${PROJECT_ROOT}/overrides"
WSL_HOME_ROOT="${WSL_HOME_ROOT:-$HOME}"
WS_CALIB_ROOT="${WS_CALIB_ROOT:-${WSL_HOME_ROOT}/ws_calib}"
WS_LIVOX_ROOT="${WS_LIVOX_ROOT:-${WSL_HOME_ROOT}/ws_livox}"
IRIDESCENCE_ROOT="${IRIDESCENCE_ROOT:-${WSL_HOME_ROOT}/iridescence}"
GLFW_SHIM_PATH="${GLFW_SHIM_PATH:-${WSL_HOME_ROOT}/lib/libglfw_hint_shim.so}"
USR_LOCAL_PREFIX="${USR_LOCAL_PREFIX:-/usr/local}"

rm -rf "${CONTEXT_ROOT}"
mkdir -p "${HOME_ROOT}" "${USR_LOCAL_ROOT}/lib" "${USR_LOCAL_ROOT}/share"

need_cmd() {
  command -v "$1" >/dev/null 2>&1 || {
    echo "ERROR: required command not found: $1" >&2
    exit 2
  }
}

need_cmd rsync

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
  python3 - "$HOME_ROOT" <<'PY'
from pathlib import Path
import sys

home_root = Path(sys.argv[1])


def replace_or_raise(text: str, needle: str, replacement: str, *, path: Path) -> str:
    if needle in text:
        return text.replace(needle, replacement)
    if replacement in text:
        return text
    raise RuntimeError(f"Expected text not found while patching {path}: {needle!r}")

safe_blocks = {
    home_root / "ws_calib/scripts/run_direct_visual_lidar_calibration_workflow.sh": (
        'source /opt/ros/noetic/setup.bash\nsource "$HOME/ws_calib/devel/setup.bash"',
        'export ROS_DISTRO="${ROS_DISTRO:-noetic}"\n'
        'set +u\n'
        'source /opt/ros/${ROS_DISTRO}/setup.bash\n'
        'source "$HOME/ws_calib/devel/setup.bash"\n'
        'export ROS_PACKAGE_PATH="$HOME/ws_calib/src${ROS_PACKAGE_PATH:+:$ROS_PACKAGE_PATH}"\n'
        'set -u',
    ),
    home_root / "ws_livox/scripts/run_pose_recovery_camera_gps.sh": (
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

calibration_workflow = home_root / "ws_calib/scripts/run_direct_visual_lidar_calibration_workflow.sh"
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

pose_recovery_camera_gps = home_root / "ws_livox/scripts/run_pose_recovery_camera_gps.sh"
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

pose_recovery = home_root / "ws_livox/scripts/run_pose_recovery.sh"
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

sync_tree "${WS_CALIB_ROOT}/devel" "${HOME_ROOT}/ws_calib/devel"
rewrite_catkin_marker "${HOME_ROOT}/ws_calib/devel/.catkin" "${HOME_ROOT}/ws_calib/src"

sync_tree "${WS_CALIB_ROOT}/src/direct_visual_lidar_calibration" "${HOME_ROOT}/ws_calib/src/direct_visual_lidar_calibration" \
  --exclude .git \
  --exclude build \
  --exclude devel \
  --exclude docker \
  --exclude docs \
  --exclude thirdparty

sync_tree "${WS_CALIB_ROOT}/scripts" "${HOME_ROOT}/ws_calib/scripts" \
  --exclude __pycache__

sync_tree "${WS_LIVOX_ROOT}/devel" "${HOME_ROOT}/ws_livox/devel"
rewrite_catkin_marker "${HOME_ROOT}/ws_livox/devel/.catkin" "${HOME_ROOT}/ws_livox/src"

sync_tree "${WS_LIVOX_ROOT}/src/FAST_LIO" "${HOME_ROOT}/ws_livox/src/FAST_LIO" \
  --exclude .git \
  --exclude doc \
  --exclude Log \
  --exclude PCD \
  --exclude build \
  --exclude devel

sync_tree "${WS_LIVOX_ROOT}/src/livox_ros_driver" "${HOME_ROOT}/ws_livox/src/livox_ros_driver" \
  --exclude .git

sync_tree "${WS_LIVOX_ROOT}/scripts" "${HOME_ROOT}/ws_livox/scripts" \
  --exclude __pycache__ \
  --exclude run_logs \
  --exclude pose_recovery_outputs

if [[ -d "${OVERRIDES_ROOT}/ws_livox/scripts" ]]; then
  rsync -a "${OVERRIDES_ROOT}/ws_livox/scripts/" "${HOME_ROOT}/ws_livox/scripts/"
fi

sync_glob "${IRIDESCENCE_ROOT}/build/libiridescence.so*" "${HOME_ROOT}/iridescence/build"
sync_file "${GLFW_SHIM_PATH}" "${HOME_ROOT}/lib/libglfw_hint_shim.so"

sync_glob "${USR_LOCAL_PREFIX}/lib/libgtsam.so*" "${USR_LOCAL_ROOT}/lib"
sync_glob "${USR_LOCAL_PREFIX}/lib/libgtsam_unstable.so*" "${USR_LOCAL_ROOT}/lib"
sync_glob "${USR_LOCAL_PREFIX}/lib/libmetis-gtsam.so*" "${USR_LOCAL_ROOT}/lib"
sync_glob "${USR_LOCAL_PREFIX}/lib/libiridescence.so*" "${USR_LOCAL_ROOT}/lib"
sync_glob "${USR_LOCAL_PREFIX}/lib/libceres.so*" "${USR_LOCAL_ROOT}/lib"
sync_tree "${USR_LOCAL_PREFIX}/share/iridescence" "${USR_LOCAL_ROOT}/share/iridescence"
patch_container_shell_scripts

echo "[done] synced WSL runtime artifacts into ${CONTEXT_ROOT}"
