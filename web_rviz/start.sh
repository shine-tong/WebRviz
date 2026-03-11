#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)"

HOST="${WEBRVIZ_HOST:-0.0.0.0}"
PORT="${WEBRVIZ_PORT:-8080}"

mount_args=()
if [ -n "${WEBRVIZ_MOUNTS:-}" ]; then
  # shellcheck disable=SC2206
  mount_args=(${WEBRVIZ_MOUNTS})
fi

exec python3 "$SCRIPT_DIR/tools/serve_webrviz.py" \
  --dist "$SCRIPT_DIR/dist" \
  --host "$HOST" \
  --port "$PORT" \
  "${mount_args[@]}"