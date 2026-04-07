#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE_TAG="${1:-senior-design/portable-ros-stack:noetic}"
DIST_DIR="${PROJECT_ROOT}/dist"
OUT_FILE="${2:-${DIST_DIR}/portable-ros-stack-noetic.tar}"

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

mkdir -p "${DIST_DIR}"
DOCKER_CMD="$(find_docker_cmd)"
require_docker_daemon "${DOCKER_CMD}"
if [[ "${DOCKER_CMD}" == *.exe ]]; then
  OUT_FILE="$(wslpath -w "${OUT_FILE}")"
fi
"${DOCKER_CMD}" save -o "${OUT_FILE}" "${IMAGE_TAG}"

echo "[done] saved ${IMAGE_TAG} to ${OUT_FILE}"
