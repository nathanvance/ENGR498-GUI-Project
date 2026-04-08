#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

cat <<'EOF'
[info] Updating the rosbag_preprocessing Docker image from the committed runtime.
[info] Docker will reuse cached layers when available, so repeat updates are usually much faster than the first build.
EOF

exec "${PROJECT_ROOT}/scripts/build_image_wsl.sh" "$@"
