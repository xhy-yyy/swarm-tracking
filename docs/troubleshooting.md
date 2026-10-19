# 常见问题

## 找不到视觉 Python 或模型

复制 `config/runtime.env.example` 为 `config/runtime.env`，填写
`VISION_PYTHON`。确认 `python3 scripts/check_assets.py --weights-only` 通过。
检测环境需能导入 `torch`、`ultralytics` 和 ROS Python 消息。

## EGO 节点出现符号或共享库错误

打开新终端，只加载 `source scripts/env.sh`。其他工作空间可能带有同名
EGO 库，不能混用。本工程默认构建仅扩展 `/opt/ros/noetic`。

## Gazebo 缺少模型、材质或插件

安装对应版本的模型资源包，运行 `python3 scripts/check_assets.py --models-only`。
通过 `scripts/run_simulation.sh` 启动，保证模型路径和插件路径被设置。
PX4 的 Gazebo 插件来自固定版本 PX4 构建；Actor 和相机 IMU 插件来自本工作空间。

## MAVROS 无法启动

检查六机端口是否已被另一组仿真占用，以及 GeographicLib 数据是否安装。
`run_simulation.sh` 中的 PX4 目录必须有 `build/px4_sitl_default/bin/px4`。

## 路线整体偏移或进场错误

比较 `six_vehicles.launch` 的 `spawn_poses` 和
`fly_px4/config/start_positions.yaml`。当前使用 `patrol_maps_2`。
同时核对 `world_frame.yaml` 中的尺度与仿真 GPS 插件。

## 行人避障与场景不一致

附带的 `black_box.txt`、`obstacle.txt` 是原开发环境的数据快照。更换场景时
需要同步更新行人控制的避障数据；本次没有重新生成每张地图的行人行为数据，
也没有重跑完整计分任务。计分脚本是本地修改版本。

## 没有显示器

视觉主启动默认显示标注图；单独启动 `swarm_track.launch annotated:=false`。
Docker 基础镜像用于构建/测试，GUI、GPU 和设备映射需由宿主按实际环境配置。
