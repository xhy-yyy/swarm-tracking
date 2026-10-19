#!/usr/bin/env bash
set -eo pipefail
SWARM_WS="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$SWARM_WS/scripts/env.sh"
SIM_BUILD_DIR="$SWARM_WS/external/PX4-Autopilot"
if [[ ! -f "$SIM_BUILD_DIR/Makefile" ]]; then
  echo "Run scripts/fetch_simulator.sh first." >&2; exit 1
fi
# DONT_RUN builds PX4 and the Gazebo plugins without starting a vehicle.
DONT_RUN=1 make -C "$SIM_BUILD_DIR" -j "${BUILD_JOBS:-2}" px4_sitl_default gazebo
