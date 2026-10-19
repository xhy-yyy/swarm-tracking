# 比赛中按故障触发的云台恢复

`swarm_fly_px4_1.launch` 默认启动独立节点 `swarm_gimbal_recovery.py`，覆盖六架飞机。
正常飞行时只监测，不发布云台命令。节点没有飞行控制接口，也没有启动、悬停、
降落、模式切换或任务暂停逻辑；不修改 `fly_px4_1.cpp`、视觉融合或官方插件。

## 检测和恢复条件

只使用现有的传感器消息：

- `/<drone>/imu_gazebo`：相机 IMU 的姿态及角速度。
- `/<drone>/mavros/imu/data`：机体 IMU 的姿态及角速度。

`imu_gazebo` 虽然名字带 gazebo，消息来自模型已有的 IMU 传感器。
节点没有使用 `/gazebo/model_states`、`link_states`、关节真值、Gazebo 服务、
插件内部变量或模型文件。运行时的相机标定是静态配置，与视觉节点一致：
初始参考姿态及固定安装角均为 `[0, 25, 0]` 度；先做旋转矩阵变换再检查云台
横滚和俯仰，不把相机正常朝下 25° 当成故障。修改相机安装或 IMU 参考方式后，
需同步配置，而不是在飞行中用未知姿态自行校准。

当前 YAML 使用持续偏角模式：

- `require_stationary: false`，不要求机体平稳、相机低角速度或偏角变化范围小于 2°。
- 扣除固定安装角后，云台横滚或俯仰偏离水平至少 20°，连续保持 3 秒，即允许一次回中。
- 两轴偏角都跌回 20° 内，或反馈失效，就取消本次计时；再次超出阈值需重新保持 3 秒。
- 两路 IMU 均有效，ROS 时间和实际接收时间均未超过 0.3 秒，时间差不超过 0.15 秒。

连续时间要求 ROS 时间和实际时间均达到 `fault_sec`；检测频率是 5 Hz，满足条件后
在下一次检测时发布。`require_stationary: true` 可恢复原来的额外限制，此时
`body_tilt_max_deg`、`body_rate_max_deg_s`、`camera_rate_max_deg_s` 和 `stuck_span_deg`
才参与判定。启动日志会打印实际加载的模式、偏角阈值和持续时间。

动作使用标准 MAVROS `/<drone>/mavros/mount_control/command`：模式 2，
roll/pitch/yaw 均为有限的 0°。它重写云台目标，不改变机体航向；固定安装角
仍然保留。回中不绕过 PX4，也不修改 MAVLink 通道或官方控制代码。

每次故障最多发 3 条回中指令，间隔至少 1 秒；每条重试都要求重新确认偏角持续 3 秒。
实际云台角度恢复到 3° 内并保持 0.7 秒才记为成功。6 秒未恢复则记录失败。
恢复超时从首次指令开始计算，当前 3 秒确认时间配合 6 秒超时，通常最多发 2 条；
`max_attempts` 是上限，不保证发满。反馈有效性检查、重试上限和冷却仍然保留。
成功或失败后进入 30 秒冷却，期间不发命令；冷却后持续故障可以重新触发。
反馈断流或时间倒退时不盲目发指令。默认模式下，运动中只要偏角持续超出阈值，
仍会回中；短暂偏角以及正常下倾 25° 不触发。

这是一条针对当前固定云台目标的恢复措施。
共享 MAVLink 解析状态的根因仍在官方插件中，回中消息也可能受到该故障影响，
因此不能保证每次自动恢复。后续如果主动控制云台指向其他角度，需要将检测目标
改为实际期望角度，避免把主动指向当成卡住。

## 使用

在 `config/gimbal_recovery.yaml` 中设置检测与自动恢复总开关：

```yaml
enabled: true   # true：开启；false：关闭
```

启动时读取该配置，修改后需重启云台检测节点才生效。关闭时节点保持空闲，
不订阅 IMU、不运行检测、不发布云台命令或诊断状态；飞行节点照常运行。

正常启动命令保持原样，不需要另外启动恢复节点：

```bash
roslaunch fly_px4 swarm_fly_px4_1.launch
```

也可通过启动参数完全不启动独立恢复节点：

```bash
roslaunch fly_px4 swarm_fly_px4_1.launch gimbal_recovery_enabled:=false
```

已在飞行且没有启动此节点时，也可以单独启动一次：

```bash
roslaunch fly_px4 swarm_gimbal_recovery.launch
```

不要与主 launch 重复启动。配置在 `config/gimbal_recovery.yaml`。
状态为 `/gimbal_recovery/status_1` 到 `status_6`，JSON 包含状态、原因、测量角度和
本次尝试次数；这些状态不接入飞行控制。正常状态 `MONITORING`，故障确认中
`SUSPECT`，恢复中 `RECOVERING`，成功/失败后的冷却为 `COOLDOWN`。

## 验证

```bash
/usr/bin/python3 test/test_gimbal_recovery.py
# 在已 source ROS/catkin 环境的终端运行；测试自行启动隔离的 ROS master。
/usr/bin/python3 test/smoke_gimbal_recovery.py
```

策略测试检查正常飞行不发指令、两轴偏角持续触发、两种模式下的运动判定、瞬态排除、标定、传感器断流、
时钟暂停/倒退、重试和冷却。ROS 测试使用合成 IMU，检查六机隔离、实际消息接口、
启动集成及节点只注册传感器订阅和云台/诊断发布；不会连接正常仿真。
这些测试不能替代六机飞行中自然故障的验证。
