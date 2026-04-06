#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE_TAG="${1:-senior-design/portable-ros-stack:noetic}"
DIST_DIR="${PROJECT_ROOT}/dist"
OUT_FILE="${2:-${DIST_DIR}/portable-ros-stack-noetic.tar}"

mkdir -p "${DIST_DIR}"
docker save -o "${OUT_FILE}" "${IMAGE_TAG}"

echo "[done] saved ${IMAGE_TAG} to ${OUT_FILE}"
