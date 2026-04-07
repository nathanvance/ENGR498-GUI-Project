#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE_TAG="senior-design/portable-ros-stack:noetic"
REFRESH_RUNTIME=0

usage() {
  cat <<'EOF'
Usage:
  build_image_wsl.sh [--refresh-runtime] [--image-tag TAG]

Description:
  Build the rosbag_preprocessing Docker image from the committed runtime staged
  in context/runtime/. This works on a fresh machine without host-side
  ws_calib/ws_livox folders.

Options:
  --refresh-runtime
      Maintainer-only option. Refresh context/runtime/ from the current WSL
      development environment before building.

  --image-tag TAG
      Human-readable tag shown in the completion message.
      Default: senior-design/portable-ros-stack:noetic
EOF
}

find_docker_cmd() {
  if command -v docker >/dev/null 2>&1 && docker version >/dev/null 2>&1; then
    echo "docker"
    return 0
  fi
  if command -v docker.exe >/dev/null 2>&1; then
    echo "docker.exe"
    return 0
  fi
  echo "ERROR: neither 'docker' nor 'docker.exe' is available." >&2
  echo "Start Docker Desktop and make sure the Docker CLI is accessible." >&2
  exit 2
}

require_docker_daemon() {
  local docker_cmd="$1"
  if ! "${docker_cmd}" info >/dev/null 2>&1; then
    echo "ERROR: Docker Desktop is not ready yet." >&2
    echo "Start Docker Desktop and wait until the Linux engine is running, then retry." >&2
    exit 2
  fi
}

run_compose_build() {
  local docker_cmd="$1"
  if [[ "${docker_cmd}" == *.exe ]]; then
    local compose_file_windows
    compose_file_windows="$(wslpath -w "${PROJECT_ROOT}/compose.yaml")"
    DOCKER_BUILDKIT=1 \
      "${docker_cmd}" compose -f "${compose_file_windows}" build portable-ros-stack
  else
    COMPOSE_DOCKER_CLI_BUILD=1 DOCKER_BUILDKIT=1 \
      "${docker_cmd}" compose -f "${PROJECT_ROOT}/compose.yaml" build portable-ros-stack
  fi
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --refresh-runtime)
      REFRESH_RUNTIME=1
      shift
      ;;
    --image-tag)
      IMAGE_TAG="${2:?Missing value for --image-tag}"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "ERROR: unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

mkdir -p \
  "${PROJECT_ROOT}/context/runtime" \
  "${PROJECT_ROOT}/outputs/calibration" \
  "${PROJECT_ROOT}/outputs/pose_recovery" \
  "${PROJECT_ROOT}/dist"

if [[ "${REFRESH_RUNTIME}" == "1" ]]; then
  "${PROJECT_ROOT}/scripts/sync_wsl_context.sh"
fi

if [[ ! -d "${PROJECT_ROOT}/context/runtime/home/portable/ws_calib" ]] || \
   [[ ! -d "${PROJECT_ROOT}/context/runtime/home/portable/ws_livox" ]]; then
  echo "ERROR: committed runtime is missing from context/runtime/." >&2
  echo "This repo is expected to include the staged Docker runtime so fresh" >&2
  echo "machines can build without local ws_calib/ws_livox folders." >&2
  echo >&2
  echo "If you are a maintainer refreshing the embedded runtime, rerun with:" >&2
  echo "  bash scripts/build_image_wsl.sh --refresh-runtime" >&2
  exit 2
fi

DOCKER_CMD="$(find_docker_cmd)"
require_docker_daemon "${DOCKER_CMD}"
run_compose_build "${DOCKER_CMD}"

echo "[done] built ${IMAGE_TAG}"
