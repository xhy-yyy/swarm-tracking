#!/usr/bin/env bash
# Operates only on this workspace's disposable external/ checkout.
set -euo pipefail
SWARM_WS="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
SIM_DIR="$SWARM_WS/external/PX4-Autopilot"
if ! command -v vcs >/dev/null 2>&1; then
  echo "Install python3-vcstool first (see docs/installation.md)." >&2
  exit 1
fi
mkdir -p "$SWARM_WS/external"
if [[ ! -e "$SIM_DIR" ]]; then
  vcs import "$SWARM_WS/external" < "$SWARM_WS/dependencies.repos"
fi
[[ "$(git -C "$SIM_DIR" rev-parse HEAD)" == 46a12a09bf11c8cbafc5ad905996645b4fe1a9df ]] || {
  echo "Unexpected PX4 revision in $SIM_DIR; use the pinned checkout." >&2; exit 1;
}
git -C "$SIM_DIR" submodule update --init --recursive
SITL_DIR="$SIM_DIR/Tools/sitl_gazebo"
if [[ "$(git -C "$SITL_DIR" rev-parse HEAD)" != 48440d7b5c78a21182415266334981f1163f4b2c ]]; then
  if [[ -n "$(git -C "$SITL_DIR" status --porcelain)" ]]; then
    echo "Refusing to change a modified simulator revision: $SITL_DIR" >&2; exit 1
  fi
  git -C "$SITL_DIR" fetch origin 48440d7b5c78a21182415266334981f1163f4b2c
  git -C "$SITL_DIR" checkout --detach 48440d7b5c78a21182415266334981f1163f4b2c
fi
apply_once() {
  local checkout="$1" patch="$2"
  if git -C "$checkout" apply --reverse --check "$patch" 2>/dev/null; then
    echo "Already applied: $(basename "$patch")"
  else
    git -C "$checkout" apply --check "$patch"
    git -C "$checkout" apply "$patch"
  fi
}
apply_once "$SIM_DIR" "$SWARM_WS/patches/px4-v1.13.2.patch"
apply_once "$SITL_DIR" "$SWARM_WS/patches/sitl-gazebo.patch"
echo "Pinned simulator sources are ready: $SIM_DIR"
