#!/usr/bin/env bash
# Source this file from a fresh shell before running the simulation.
SWARM_WS="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ -f "$SWARM_WS/config/runtime.env" ]]; then
  source "$SWARM_WS/config/runtime.env"
fi
ROS_DISTRO="${ROS_DISTRO:-noetic}"
if [[ ! -f "/opt/ros/$ROS_DISTRO/setup.bash" ]]; then
  echo "ROS setup not found: /opt/ros/$ROS_DISTRO/setup.bash" >&2
  return 1
fi
# Remove other workspace overlays, whose EGO libraries can have incompatible ABIs.
_swarm_filtered=""
IFS=: read -ra _swarm_paths <<< "${LD_LIBRARY_PATH:-}"
for _swarm_path in "${_swarm_paths[@]}"; do
  [[ -z "$_swarm_path" ]] && continue
  case "$_swarm_path" in
    */devel/lib*|*/install/lib*) continue ;;
  esac
  _swarm_filtered="${_swarm_filtered:+$_swarm_filtered:}$_swarm_path"
done
export LD_LIBRARY_PATH="$_swarm_filtered"
export CMAKE_PREFIX_PATH="/opt/ros/$ROS_DISTRO"
export ROS_PACKAGE_PATH="/opt/ros/$ROS_DISTRO/share"
source "/opt/ros/$ROS_DISTRO/setup.bash"
if [[ -f "$SWARM_WS/devel/setup.bash" ]]; then
  source "$SWARM_WS/devel/setup.bash"
fi
export SWARM_WS
export VISION_PYTHON="${VISION_PYTHON:-python3}"
export PX4_DIR="${PX4_DIR:-$SWARM_WS/external/PX4-Autopilot}"
export SWARM_WEIGHTS="${SWARM_WEIGHTS:-$SWARM_WS/assets/weights/best_11s_new.pt}"
export SWARM_MODEL_SDF="$SWARM_WS/src/swarm_simulation/models/typhoon_h480_xhyyy/typhoon_h480_xhyyy.sdf"
export SWARM_REFERENCE_WORLDS="$SWARM_WS/src/swarm_simulation/worlds/reference"
export ROS_PACKAGE_PATH="$SWARM_WS/src:$PX4_DIR:$PX4_DIR/Tools/sitl_gazebo:$ROS_PACKAGE_PATH"
export GAZEBO_MODEL_PATH="$SWARM_WS/src/swarm_simulation/models:$SWARM_WS/assets/models:$PX4_DIR/Tools/sitl_gazebo/models${GAZEBO_MODEL_PATH:+:$GAZEBO_MODEL_PATH}"
export GAZEBO_PLUGIN_PATH="$SWARM_WS/devel/lib:$PX4_DIR/build/px4_sitl_default/build_gazebo${GAZEBO_PLUGIN_PATH:+:$GAZEBO_PLUGIN_PATH}"
export LD_LIBRARY_PATH="$SWARM_WS/devel/lib:$PX4_DIR/build/px4_sitl_default/build_gazebo:$LD_LIBRARY_PATH"
unset _swarm_filtered _swarm_path _swarm_paths
