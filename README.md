# Swarm Tracking

[English](README_en.md) · [安装说明](docs/installation.md) · [系统结构](docs/architecture.md) · [发布说明](docs/publishing.md)

基于 **ROS Noetic、PX4 v1.13.2、Gazebo Classic 11 和定制 EGO-Swarm** 的六机协同视觉搜索与目标跟踪仿真工程。

六架无人机按地图预设巡航，利用相机图像与深度检测、定位行人；融合节点维护目标航迹并分配跟踪任务，飞控节点执行巡航、跟踪、避障与降落。此版本保留既有任务逻辑，整理依赖、资源和启动入口。

## 功能

- 六机巡航与固定地图航线配置。
- YOLO 批量检测、深度/单目定位及图像时刻姿态补偿。
- 多机目标融合、任务分配、共享发现与跟踪交接。
- 二维 EGO 规划、深度高度带避障及高度/偏航控制。
- 独立的行人控制和比赛计分脚本。
- 离线 Python 回归测试、C++ 控制测试和 ROS 冒烟测试。

## 快速开始

支持环境：Ubuntu 20.04 + ROS Noetic + Gazebo Classic 11。源码可以放在任意目录，下面以 `~/swarm_tracking_ws` 为例。

```bash
git clone https://github.com/xhy-yyy/swarm-tracking.git ~/swarm_tracking_ws
cd ~/swarm_tracking_ws
bash scripts/install_dependencies.sh
bash scripts/build.sh
```

视觉环境、PX4 和资源包的完整安装步骤见 [安装说明](docs/installation.md)。Git 仓库不包含模型纹理/网格大资源和检测权重；发布包中的两个资源压缩包需要一起安装。

```bash
# 从已下载的 Release 资源所在目录安装，并校验 SHA-256。
python3 scripts/download_assets.py /path/to/release-assets
python3 scripts/check_assets.py
```

安装完毕后，在三个独立终端依次执行，地图选择保持一致：

```bash
# 终端 1：Gazebo + 六架 PX4 + MAVROS
bash scripts/run_simulation.sh base1

# 终端 2：行人与裁判。计时从裁判启动开始。
bash scripts/run_actors.sh both

# 终端 3：规划、视觉融合与飞行任务
bash scripts/run_mission.sh base1
```

`base1/base2/base3` 对应三张比赛地图。旧航线使用 `route1..route7`，具体对应关系可通过 `bash scripts/run_mission.sh --list-routes` 查看。当前推荐起点在 `src/fly_px4/config/start_positions.yaml`，同时匹配所附仿真启动文件。

## 验证

```bash
bash scripts/test.sh
```

验证范围和本地运行结果见 [验证记录](docs/validation.md)。本次整理未执行完整的六机计分飞行，不承诺尚未复测的任务成绩。

## 目录

```text
src/fly_px4/                  巡航/跟踪主控、感知、融合与测试
src/track_msgs/               检测、航迹与跟踪消息
src/ego_planner/              保留定制修改的 EGO-Swarm
src/gazebo_ros_actor_plugin/  行人控制插件与消息
src/actor_collisions/         Actor 碰撞插件
src/swarm_simulation/         地图、模型配置、计分脚本、相机 IMU 插件
scripts/                     构建、测试、启动与打包入口
patches/                     固定版 PX4 / Gazebo 插件补丁
config/                      环境示例和资源校验清单
assets/                      本地安装的大资源，不进入 Git
external/                    下载的 PX4 源码，不进入 Git
docs/                        安装、架构、接口和发布文档
```

详细的高度控制、避障、云台和路线设计文档保存在 `src/fly_px4/docs/`。

## 来源与许可

本工程基于既有本地工作空间整理，来源版本记录在 [source_snapshot.json](docs/source_snapshot.json)。感谢 PX4、XTDrone、EGO-Swarm、Gazebo 和各原始组件作者。

原创部分采用 BSD-3-Clause；EGO 及其他第三方组件保留各自许可证，具体范围见 [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)。检测权重另见 [模型说明](docs/model_card.md)。
