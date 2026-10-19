#!/usr/bin/env bash
set -eo pipefail
SWARM_WS="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$SWARM_WS/scripts/env.sh"
if [[ ! -f "$SWARM_WS/build/fly_px4/CTestTestfile.cmake" ]]; then
  echo "Run scripts/build.sh first." >&2; exit 1
fi
export PYTHONDONTWRITEBYTECODE=1
/usr/bin/python3 -m unittest discover -s "$SWARM_WS/src/fly_px4/test" -p 'test_*.py' -v
cd "$SWARM_WS/build/fly_px4"
ctest --output-on-failure
