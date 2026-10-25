<h1 align="center">Swarm Tracking</h1>

<p align="center">
  <strong>Multi-UAV Visual Search &amp; Cooperative Target Tracking</strong><br/>
  Six UAVs · Shared target observations · Local trajectory planning
</p>

<p align="center">
  <a href="docs/installation.md"><img src="https://img.shields.io/badge/ROS-Noetic-22314E?style=flat-square&amp;logo=ros" alt="ROS Noetic"/></a>
  <a href="dependencies.repos"><img src="https://img.shields.io/badge/PX4-v1.13.2-005A9C?style=flat-square" alt="PX4 v1.13.2"/></a>
  <a href="docs/installation.md"><img src="https://img.shields.io/badge/Gazebo-Classic%2011-E87500?style=flat-square" alt="Gazebo Classic 11"/></a>
  <a href="LICENSE"><img src="https://img.shields.io/badge/License-BSD--3--Clause-2E7D62?style=flat-square" alt="Original code: BSD-3-Clause"/></a>
</p>

<p align="center">
  <a href="#demos">Demos</a> ·
  <a href="#capabilities">Capabilities</a> ·
  <a href="#architecture">Architecture</a> ·
  <a href="#quick-start">Quick start</a> ·
  <a href="#documentation">Documentation</a> ·
  <a href="README.md">中文</a>
</p>

Swarm Tracking connects perception, target fusion, task assignment, local planning, and flight control in a six-UAV simulation workspace. UAVs patrol predefined areas, detect and localize pedestrians through onboard cameras, share target observations, and execute tracking tasks with obstacle avoidance and trajectory control.

Built on **ROS Noetic, PX4 SITL, Gazebo Classic, and a modified EGO-Swarm planner**, the project supports research and reproduction of cooperative search, visual localization, and target tracking workflows.

## Demos

<p align="center">
  <img src="docs/media/swarm-overview.jpg" width="100%" alt="Six UAV detection views alongside the Gazebo scene and mission scoring interface"/>
  <br/>
  <sub>Six onboard detection views during a simulation mission</sub>
</p>

| Visual detection and target tracking | Swarm search: overhead view |
| :---: | :---: |
| [![Animated target tracking preview](docs/media/target-tracking.gif)](docs/media/target-tracking.mp4?raw=1) | [![Animated overhead search preview](docs/media/swarm-search.gif)](docs/media/swarm-search.mp4?raw=1) |
| [Watch / download full MP4 · 20 s](docs/media/target-tracking.mp4?raw=1) | [Watch / download full MP4 · 20 s](docs/media/swarm-search.mp4?raw=1) |

These recordings show the original development environment; GIFs are short previews. See the [validation record](docs/validation.md) for the reproduction scope of the open-source workspace and the [media notes](docs/media/README.md) for asset details.

## Capabilities

| Module | Implementation |
| --- | --- |
| **Cooperative search** | Six-UAV patrol with three competition maps and seven legacy route presets. |
| **Visual perception** | Batched YOLO inference across six camera streams, depth and monocular localization, and image-time camera pose compensation. |
| **Target fusion** | Multi-UAV observation fusion, target track maintenance, task assignment, shared discoveries, and tracking handovers. |
| **Trajectory planning** | Modified EGO-Swarm 2D local planning with depth height-band sensing and obstacle avoidance. |
| **Flight control** | MAVROS/PX4 integration for takeoff, transit, patrol, tracking, and landing, with altitude and yaw control. |
| **Simulation evaluation** | Pedestrian controllers, competition scoring, offline regression tests, and ROS control tests. |

## Architecture

```mermaid
flowchart LR
    SIM["Gazebo Classic<br/>Six UAVs and scene"] -->|RGB / depth / IMU| V["Visual perception<br/>swarm_vision"]
    SIM -->|MAVROS odometry| B["World-frame relay"]
    B --> V
    V --> F["Target fusion and assignment<br/>swarm_fusion"]
    F --> C["Six mission controllers<br/>fly_px4_1"]
    B --> E["Modified EGO-Swarm<br/>Planning and avoidance"]
    C -->|Local goal| E
    E -->|Trajectory| C
    C -->|Setpoint| P["MAVROS / PX4"]
    P --> SIM
```

The mission entry point, [`start_swarm.sh`](src/fly_px4/scripts/start_swarm.sh), starts:

1. `swarm_relay_ego.launch`: world-frame relays, point clouds, EGO planning, and trajectory servers.
2. `swarm_track.launch`: detection, target fusion, assignment, and annotated views.
3. `swarm_fly_px4_1.launch`: the six UAV mission controllers.

Gazebo, pedestrians, and the judge run separately. Target fusion uses visual observations and odometry. See [architecture](docs/architecture.md) and [topics and frames](docs/topics_and_frames.md) for node responsibilities and data flow.

## Quick start

### 1. Prepare the environment and build

| Component | Project environment |
| --- | --- |
| Operating system | Ubuntu 20.04 |
| ROS / simulation | ROS Noetic · Gazebo Classic 11 · PX4 v1.13.2 |
| ROS Python | Python 3.8 |
| Vision inference | Separate Python 3.10 environment; see [`environment.yml`](environment.yml). |

```bash
git clone https://github.com/xhy-yyy/swarm-tracking.git ~/swarm_tracking_ws
cd ~/swarm_tracking_ws
bash scripts/install_dependencies.sh
bash scripts/build.sh
```

Before running, follow the [installation guide](docs/installation.md) to **build PX4 and its plugins, install model and weight assets, and create the vision environment**. Meshes, textures, and detector weights are distributed separately from the source repository.

```bash
# First-time setup: set the actual VISION_PYTHON path in runtime.env.
cp -n config/runtime.env.example config/runtime.env

# Verify assets after installing the matching model and weight archives.
python3 scripts/check_assets.py
```

### 2. Start the simulation and mission

Run the following from the repository root in three separate terminals, in order. This example uses `base1`.

**Terminal 1 · Simulation**

```bash
bash scripts/run_simulation.sh base1
```

Wait for Gazebo to load and all six MAVROS connections to become ready.

**Terminal 2 · Pedestrians and judge**

```bash
bash scripts/run_actors.sh both
```

The judge starts its timer immediately. Start it before mission takeoff.

**Terminal 3 · Cooperative search and tracking**

```bash
source scripts/env.sh
bash src/fly_px4/scripts/start_swarm.sh base1
```

`scripts/env.sh` loads this workspace's ROS environment, vision interpreter, and asset paths. `start_swarm.sh` manages the three mission launch processes; `Ctrl+C` stops the mission processes it started.

[`scripts/run_mission.sh`](scripts/run_mission.sh) wraps the same entry point and loads the environment automatically. The shorter equivalent is `bash scripts/run_mission.sh base1`.

### 3. Select a map and route

| Simulation and mission argument | Competition world | Current route preset |
| --- | --- | --- |
| `base1` | `base1.world` | `competition_base1.yaml` |
| `base2` | `base2.world` | `competition_base2.yaml` |
| `base3` | `base3.world` | `competition_base3.yaml` |

Use the same map for simulation and mission. Pass `base1`, `base2`, or `base3` explicitly: without a map argument, `start_swarm.sh` selects the legacy `base0` route.

```bash
# List competition maps and legacy route1..route7 presets.
bash src/fly_px4/scripts/start_swarm.sh --list-routes

# Show entry-point arguments.
bash src/fly_px4/scripts/start_swarm.sh --help
```

## Configuration

| File | Purpose |
| --- | --- |
| [`config/runtime.env.example`](config/runtime.env.example) | Local vision interpreter, PX4 directory, and detector weight paths. |
| [`start_positions.yaml`](src/fly_px4/config/start_positions.yaml) | Six UAV start positions and route directory; currently `patrol_maps_2`. |
| [`patrol_maps_2/`](src/fly_px4/config/patrol_maps_2/) | Competition and legacy routes for the current start configuration. |
| [`swarm_track.launch`](src/fly_px4/launch/swarm_track.launch) | Detection, synchronization, localization, fusion, and assignment. |
| [`swarm_relay_ego.launch`](src/fly_px4/launch/swarm_relay_ego.launch) | World-frame relays, planning plane, sensing, and kinematics. |
| [`swarm_fly_px4_1.launch`](src/fly_px4/launch/swarm_fly_px4_1.launch) | Patrol, tracking, altitude, and yaw control. |

When changing start positions, update both `spawn_poses` in [`six_vehicles.launch`](src/swarm_simulation/launch/six_vehicles.launch) and `start_positions.yaml`, then select the matching route directory.

## Validation

```bash
bash scripts/test.sh
```

This runs offline Python regressions and registered CTests; some control tests use an isolated ROS master. See the [validation record](docs/validation.md) for recorded build and test results and the scope of full six-UAV scored-flight reproduction.

## Documentation

The detailed technical guides are currently in Chinese.

| Guide | Contents |
| --- | --- |
| [Installation and operation](docs/installation.md) | Dependencies, PX4, vision environment, assets, and startup. |
| [Architecture](docs/architecture.md) | Packages, nodes, and mission startup. |
| [Topics and frames](docs/topics_and_frames.md) | ROS interfaces, messages, and coordinate conventions. |
| [Troubleshooting](docs/troubleshooting.md) | Vision, models, libraries, MAVROS, and route configuration. |
| [Detector model card](docs/model_card.md) | Class order, inference configuration, and model provenance. |
| [Control and planning design](src/fly_px4/docs/) | Altitude control, depth height-band avoidance, tracking yaw, and gimbal recovery. |
| [Contributing](CONTRIBUTING.md) | Development conventions and checks. |
| [Publishing](docs/publishing.md) | Asset packaging and release preparation. |

<details>
<summary><strong>Repository layout</strong></summary>

```text
swarm-tracking/
├── src/
│   ├── fly_px4/                  Mission control, vision, fusion, and entry point
│   ├── track_msgs/               Detection and tracking messages
│   ├── ego_planner/              Modified EGO-Swarm planner
│   ├── gazebo_ros_actor_plugin/  Pedestrian plugins and messages
│   ├── actor_collisions/         Actor collision plugin
│   └── swarm_simulation/         Scenes, model configuration, and scoring
├── scripts/                      Installation, build, test, and simulation scripts
├── config/                       Environment template and asset checksums
├── patches/                      Pinned PX4 / Gazebo patches
├── docs/                         Technical guides and demo media
├── assets/                       Locally installed models and weights
└── external/                     Locally fetched PX4 sources
```

</details>

## Acknowledgments and license

Thanks to **PX4, XTDrone, EGO-Swarm, Gazebo**, and the authors of the upstream components. Source provenance and version records are available in the [source snapshot](docs/source_snapshot.json).

Original code is licensed under [BSD-3-Clause](LICENSE). Third-party components retain their respective licenses; see [third-party notices](THIRD_PARTY_NOTICES.md). See the [model card](docs/model_card.md) for weight and training-data provenance and licensing information.
