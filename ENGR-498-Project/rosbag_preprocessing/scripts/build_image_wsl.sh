#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE_TAG="${1:-senior-design/portable-ros-stack:noetic}"

"${PROJECT_ROOT}/scripts/sync_wsl_context.sh"

COMPOSE_DOCKER_CLI_BUILD=1 DOCKER_BUILDKIT=1 \
docker compose -f "${PROJECT_ROOT}/compose.yaml" build portable-ros-stack

echo "[done] built ${IMAGE_TAG}"
