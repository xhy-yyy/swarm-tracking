#!/usr/bin/env bash
set -eo pipefail
SWARM_WS="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$SWARM_WS/scripts/env.sh"
MAP="${1:-base1}"
case "$MAP" in
  base1|base2|base3) shift "$(( $# > 0 ? 1 : 0 ))" ;;
  *) echo "Usage: $0 [base1|base2|base3] [roslaunch arguments]" >&2; exit 2 ;;
esac
if [[ ! -x "$PX4_DIR/build/px4_sitl_default/bin/px4" ]]; then
  echo "Build the pinned simulator first; see docs/installation.md. PX4_DIR=$PX4_DIR" >&2
  exit 1
fi
python3 "$SWARM_WS/scripts/check_assets.py" --models-only
exec roslaunch swarm_simulation six_vehicles.launch \
  "world:=$SWARM_WS/src/swarm_simulation/worlds/competition/$MAP.world" "$@"
