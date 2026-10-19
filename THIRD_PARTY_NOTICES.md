# Component licenses and attribution

The root BSD-3-Clause license covers original contributions by xhy, including the release
scripts and project documentation. It does not replace the licenses of imported
code, simulator resources, dependencies, or trained model weights.

| Component | Origin | License / scope |
| --- | --- | --- |
| `src/ego_planner` | [EGO-Swarm](https://github.com/ZJU-FAST-Lab/ego-planner-swarm), with local planning/mapping modifications | GPL-3.0; preserve individual BSD/LGPL notices in utilities and Fast-Planner files. The exact upstream baseline of the historical import is not recorded; the imported snapshot is identified in `docs/source_snapshot.json`. |
| `src/ego_planner/Utils` | Maintainers listed in each package and source header | BSD or LGPLv3 as declared in the respective packages; original source notices retained. |
| `src/ego_planner/drone_detect` | Original package README and source notices | BSD; its existing LICENSE is retained. |
| `src/gazebo_ros_actor_plugin` | XTDrone / OSRF Gazebo ActorPlugin derivatives | Apache-2.0; original source headers and license retained. |
| `src/actor_collisions` | OSRF Gazebo actor collision example, copyright 2018 OSRF | Apache-2.0; this release adds a catkin wrapper. |
| `src/swarm_simulation/src/gazebo_ros_imu_sensor.cpp` and its header | [gazebo_ros_pkgs](https://github.com/ros-simulation/gazebo_ros_pkgs), local checkout `cd9cc952402375be5f274f39b29399c4087aa839` | Apache-2.0, copyright 2015 Alessandro Settimi. Built under a distinct library filename; source headers retained. |
| Competition scripts and worlds | [XTDrone](https://github.com/robin-shaun/XTDrone), local base `eac5093dbd9a635fbe37f6a29ffed8bad96ffe45` with local changes | MIT, copyright 2021 Kun Xiao; see `LICENSES/XTDrone-MIT.txt`. The included score script is the local competition variant, not a claim of an unmodified official judge. |
| PX4 patches and vehicle resources | [PX4](https://github.com/PX4/PX4-Autopilot), v1.13.2 | BSD-3-Clause; `LICENSES/PX4-BSD-3-Clause.txt`. |
| Gazebo gimbal/camera plugin patches | [PX4-SITL_gazebo](https://github.com/PX4/PX4-SITL_gazebo) | Preserve the Apache-2.0/BSD source-file notices in the pinned external checkout. |
| Gazebo model payload | PX4-SITL_gazebo, XTDrone and the local Gazebo model collection | Per-file/per-model notices remain in the payload. The model collection includes the Nathan Koenig CC-BY-3.0 notice copied to `LICENSES/Gazebo-Models-CC-BY-3.0.txt`; see `docs/model_sources.json` for each model's source. Mesh or texture files with more specific notices retain those notices. |
| `track_msgs` | Existing local message package | BSD as declared in its original manifest; see its package LICENSE. |
| Ultralytics inference dependency | [Ultralytics](https://github.com/ultralytics/ultralytics) | Upstream AGPL-3.0 / commercial licensing terms apply independently; it is installed as a dependency, not relicensed by this repository's BSD-3-Clause text. |
| `best_11s_new.pt` | Existing local trained checkpoint | Distributed separately from source. The root BSD-3-Clause license does not grant rights to the checkpoint or its underlying training data; see `docs/model_card.md`. |

The original `fly_px4` package lists lxlink as maintainer. This attribution is
retained as an author entry; the release maintainer is xhy. Existing copyright
headers take precedence over the general license summary above.

Copies of GPL-3.0, LGPL-3.0, Apache-2.0 and upstream project notices are in
`LICENSES/`. Modifications to imported EGO files retain their original licensing.
