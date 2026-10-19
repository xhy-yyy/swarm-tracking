#!/usr/bin/env bash
set -eo pipefail
SWARM_WS="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "/opt/ros/${ROS_DISTRO:-noetic}/setup.bash"
# Explicitly mark this workspace; catkin otherwise discovers a parent's metadata.
mkdir -p "$SWARM_WS/.catkin_tools/profiles/default"
catkin config --workspace "$SWARM_WS" --extend "/opt/ros/${ROS_DISTRO:-noetic}" \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3
catkin build --workspace "$SWARM_WS" --no-status --summarize \
  -j "${BUILD_JOBS:-2}" -p 1 \
  fly_px4 ego_planner ros_actor_cmd_pose_plugin swarm_simulation actor_collisions "$@"
