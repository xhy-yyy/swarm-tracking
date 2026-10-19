#!/usr/bin/env bash
set -eo pipefail
SWARM_WS="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$SWARM_WS/scripts/env.sh"
exec bash "$SWARM_WS/src/fly_px4/test/start_judge_and_actors.sh" "$@"
