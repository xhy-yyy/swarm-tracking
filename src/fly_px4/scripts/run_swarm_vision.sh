#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
VISION_PYTHON="${VISION_PYTHON:-python3}"
if ! command -v "$VISION_PYTHON" >/dev/null 2>&1; then
  echo "Vision Python not found: $VISION_PYTHON; configure VISION_PYTHON." >&2
  exit 1
fi
exec "$VISION_PYTHON" -B "$SCRIPT_DIR/swarm_vision.py" "$@"
