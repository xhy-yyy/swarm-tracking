# Swarm Tracking

[中文说明](README.md)

A six-UAV visual search and cooperative target tracking simulation workspace
using ROS Noetic, PX4 v1.13.2, Gazebo Classic 11, and a modified EGO-Swarm planner.

The system combines preset patrol routes, batched YOLO inference, depth and
monocular localization, image-time camera pose compensation, multi-UAV target
assignment, obstacle avoidance, and PX4 flight control.

## Build and test

Target platform: Ubuntu 20.04 with ROS Noetic and Gazebo Classic 11.

```bash
git clone https://github.com/xhy-yyy/swarm-tracking.git ~/swarm_tracking_ws
cd ~/swarm_tracking_ws
bash scripts/install_dependencies.sh
bash scripts/build.sh
bash scripts/test.sh
```

Install the pinned PX4 sources, the vision environment and both resource archives
as described in [installation.md](docs/installation.md). Simulator meshes and
weights are separate Release assets; they are not stored in Git.

Run these commands in separate terminals, in order:

```bash
bash scripts/run_simulation.sh base1
bash scripts/run_actors.sh both
bash scripts/run_mission.sh base1
```

Use the same map (`base1`, `base2`, or `base3`) for simulation and mission launch.
The judge starts its timer immediately. The current source was prepared and
tested locally; a complete six-UAV scored flight has not been rerun for this
release. See [validation.md](docs/validation.md).

Original contributions are BSD-3-Clause licensed. EGO-Swarm and other imported components
retain their respective licenses; see [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).
