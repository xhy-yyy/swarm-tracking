# 安装与运行

## 环境

本次源码构建基于 Ubuntu 20.04、Python 3.8、ROS Noetic、Gazebo Classic 11。
检测节点单独使用 Python 3.10 环境，版本来自原有视觉环境，保存在
`requirements-vision.txt`。GPU/CUDA/PyTorch 组合取决于显卡；所列版本是
原环境记录，不代表已在其他显卡上验证。

ROS Noetic 和 Gazebo Classic 已结束官方支持，本项目保留这一历史运行环境：
[ROS 公告](https://discourse.ros.org/t/ros-noetic-end-of-life-may-31-2025/43160)、
[Gazebo 说明](https://classic.gazebosim.org/)。

## ROS 工作空间

```bash
cd /path/to/swarm_tracking_ws
bash scripts/install_dependencies.sh
bash scripts/build.sh
source scripts/env.sh
```

构建脚本显式扩展 `/opt/ros/noetic`，不扩展旧 `catkin_ws` 或 RACER 工作空间。
默认使用两个编译任务，内存充足时可设置 `BUILD_JOBS=4`。
构建覆盖主控、规划、消息、行人插件、碰撞插件和仿真资源包；保留的其他
EGO 工具包不属于默认构建目标。

## 安装资源包

发布者需把 `dist/` 中下列两个压缩包附到与源码匹配的 GitHub Release：

- `swarm-tracking-models-v0.1.0.tar.gz`
- `swarm-tracking-weights-v0.1.0.tar.gz`

下载到一个目录后执行：

```bash
python3 scripts/download_assets.py /path/to/downloaded-assets
python3 scripts/check_assets.py
```

也可以把实际 Release 的下载目录 URL 传给脚本：

```bash
python3 scripts/download_assets.py https://github.com/OWNER/REPO/releases/download/v0.1.0
```

`OWNER/REPO` 在创建仓库后替换。下载器按源码携带的压缩包 SHA-256 验证，
解压前检查路径；`check_assets.py` 再逐个验证资源文件。
本机整理出的工作空间已放入所需资源，可以直接检查。

## PX4 与 Gazebo 插件

```bash
bash scripts/fetch_simulator.sh
```

源码清单固定 PX4 v1.13.2 commit，并固定其 Gazebo 子模块版本；脚本在
`external/PX4-Autopilot` 内应用所附补丁。补丁保存 MAVLink 启动参数、
云台控制和相关插件构建修改，不修改原开发目录。

PX4 所需系统依赖另按该固定版本的
[Ubuntu 安装说明](https://docs.px4.io/v1.13/en/dev_setup/dev_env_linux_ubuntu.html)
安装，然后构建：

```bash
bash scripts/build_simulator.sh
```

脚本通过 `DONT_RUN=1` 只构建，不启动飞行器。获取和完整重编译 PX4 需要
网络和较长时间；本次整理验证了补丁和 ROS 工程，未重新下载并全量编译 PX4。

如使用自行准备的同版本仿真目录，可在本地配置 `PX4_DIR`；该目录需要具备
相同补丁和已编译的 SITL 插件。不要对原工作目录直接运行补丁脚本。

MAVROS 如提示缺少 GeographicLib 数据，执行：

```bash
sudo bash scripts/install_geographiclib_datasets.sh
```

## 视觉环境

```bash
conda env create -f environment.yml
```

如果需要其他 CUDA 构建，先按 PyTorch 对应硬件的安装方式安装 `torch` 和
`torchvision`，再安装其余依赖。系统 Python 3.8 负责 ROS/C++ 测试；视觉
Python 通过 `PYTHONPATH` 导入 ROS 的 Python 消息模块。当前检测节点直接解析
图像数组，不在这个 Python 3.10 环境中使用系统 Python 3.8 的 cv_bridge 扩展。

创建本地配置：

```bash
cp config/runtime.env.example config/runtime.env
```

在其中填写真实解释器路径：

```bash
VISION_PYTHON="/path/to/conda/envs/swarm-vision/bin/python"
```

也可设置 `PX4_DIR` 和 `SWARM_WEIGHTS`。`config/runtime.env` 已被 Git 忽略。
权重若换成其他训练结果，需要保持检测类别顺序与 `swarm_vision.py` 一致。

## 启动

每个终端从仓库根目录执行。先启动仿真，等待六机 MAVROS 连接；然后启动
行人与裁判，最后启动任务。裁判在启动时开始计时。

```bash
# 终端 1
bash scripts/run_simulation.sh base1
# 终端 2
bash scripts/run_actors.sh both
# 终端 3
bash scripts/run_mission.sh base1
```

主线默认显示六路标注图，需要图形桌面。无显示器调试时，单独启动
`roslaunch fly_px4 swarm_track.launch annotated:=false`，并按架构文档启动
其他部分。不要在同一个 ROS master 重复启动同名节点。

当前仿真启动文件以 world 副本内联六架飞机，保留既有传感器初始化顺序。
起点为 x=-14/-17、y=-3/0/3，配 `patrol_maps_2`；修改起点时，仿真
`spawn_poses` 和 `start_positions.yaml` 必须同步修改。

## 测试

```bash
bash scripts/test.sh
```

此命令运行离线 Python 测试和已注册的 CTest；部分 CTest 启动隔离的本机
ROS master。`smoke_*.py` 中其他完整仿真测试为手动测试，不由这个命令全量运行。
