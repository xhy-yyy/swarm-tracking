# 跟踪机头与图像居中控制

跟踪时优先朝向本机已经关联、认领的行人框中心视线。框中心射线使用成像时刻的真实相机 IMU 姿态转换到世界系，不依赖深度定位中的 `bearing`，也不使用裁判真值。融合完成身份关联后立即发布 `/tracking/view_N`，不再等待 10 Hz 派活定时器。

`fly_px4_1` 在 50 Hz 控制循环中估计视线角速度，只把图像观测补偿到当前时刻。静止时滤波抑制框抖动，连续运动时采用较快滤波；检测到折返时撤掉旧方向角速度。连续偏航参考受角速度与角加速度限制，`yaw_rate` 始终等于最终参考角的离散导数。实际姿态闭环仍由 PX4 完成。

EGO PVA、近距 XY 速度跟随、跟踪悬停与回退均使用同一对 `yaw/yaw_rate`。有有效跟踪方向时，脱困搜索自旋不会覆盖朝向行人的参考。图像过期后平滑转回当前目标坐标方向；坐标观测也过期时停止旧速度预测，保持重捕朝向。目标切换、位姿断流与仿真时钟回退会清除旧视线状态。

## 默认参数

参数在 `launch/swarm_fly_px4_1.launch` 中向六机统一下发。

| 参数 | 默认值 | 作用 |
|---|---:|---|
| `track_yaw_rate_max` | 60 deg/s | 参考偏航转速上限 |
| `track_yaw_acc_max` | 180 deg/s² | 参考转速变化上限 |
| `track_yaw_kp` | 5 /s | 视线方向与参考角之间的追赶增益 |
| `track_yaw_deadband_deg` | 5 deg | 视线与机头参考角的死区，设为 0 关闭 |
| `track_yaw_stationary_tau` | 0.12 s | 静止/未确认运动时的滤波时间常数 |
| `track_yaw_moving_tau` | 0.04 s | 连续运动确认后的滤波时间常数 |
| `track_view_timeout` | 0.35 s | 按成像时间判断图像反馈过期 |
| `track_view_conf_min` | 0.4 | 最小检测置信度 |
| `track_yaw_velocity_timeout` | 0.25 s | 坐标辅助前馈所用本机速度时效 |
| `track_stale_sec` | 0.3 s | 坐标辅助速度前馈时效 |

视线方向与当前机头参考角的最短角度差在 ±5°以内时，不再追赶，也不叠加视线角速度前馈；已有转速按角加速度上限减至零后保持参考角。超过 5°恢复追赶。死区对图像方向和坐标辅助方向均生效，并按过滤、延迟补偿后的当前视线判断；它作用于发送给 PX4 的参考角，实际机头由 PX4 跟随该参考。巡航转向的对准完成阈值不受此参数影响。

`track_lead_sec` 保留启动命令兼容，当前算法不使用额外未来位置前推。`track_yaw_kp` 是参考生成器的增益，不是 PX4 的 `MC_YAW_P`。

图像不可用时，辅助角速度用 `(目标速度 - 本机实测速度)` 计算，并按观测年龄和目标速度协方差降权。本机速度必须来自新鲜的 MAVROS `velocity_local`；缺失时只跟随坐标方向，不把速度指令当作实测速度。

## 启动与检查

消息接口有更新：`Detection` 增加独立图像视线字段，新增 `TrackingView`。需要重新编译、加载同一套生成消息，并同时重启视觉、融合和主控；旧消息 MD5 不能与新节点混用。

```bash
cd $SWARM_WS
catkin build track_msgs fly_px4 --jobs 2 --parallel-packages 1
source devel/setup.bash
```

随后按原来的顺序重新启动 Gazebo/PX4、定位/规划、视觉融合及 `swarm_fly_px4_1.launch`。

2026-10-05 按参赛部署要求撤销本地云台 FOLLOW 的 60 ms 预测；云台恢复使用最近一次飞控航向，两份本地同名插件均重新构建。比赛使用官方插件，不需要复制本地插件。机头的图像反馈、平滑参考和 `yaw_rate` 控制仍在算法包内保留。

PX4 与 XTDrone 两处 RealSense SDF 均恢复为 XTDrone 仓库原件：不添加 IMU 姿态参考设置，完整保留左右双目相机，深度/彩色参数不变。不要用删除相机或修改官方 IMU 的方式适配算法。

原始 IMU 的参考系由 ROS 插件版本决定。算法启动参数 `camera_imu_reference` 支持：

- `initial`（当前默认）：本机 XTDrone 原始插件输出相对传感器加载姿态的旋转。算法用已知初始姿态左乘恢复世界姿态：`R_world = R_initial @ R_reported`。
- `world`：标准 Noetic 插件直接输出世界姿态，算法不再叠加安装角。

`camera_imu_initial_rpy_deg` 默认 `[0, 25, 0]`，表示传感器加载时相机物理坐标系的世界 roll/pitch/yaw；与当前六机零初始航向、25° 安装俯角匹配。出生航向、初始机体倾角或相机安装角改变时必须同步更新。该值是固定初始参考，不能使用当前机体航向，也不随视觉节点重启重新估计。以上是算法坐标变换，不会写入传感器或 Gazebo 插件。

官方机器若使用标准 Noetic IMU 插件，启动视觉时选择：

```bash
roslaunch fly_px4 swarm_track.launch camera_imu_reference:=world
```

在起飞前运行 `preflight_check.py`，查看转换后光轴俯角是否接近 -25°。原始话题里接近 0° 对应 `initial`，接近 -25° 对应 `world`；选错模式会漏加或重复加安装角。默认一键启动按本机的 `initial` 模式运行。

每机可检查：

```bash
rostopic echo /tracking/view_1
rostopic echo /tracking/yaw_diag_1
```

`view_1` 包含成像时间、负责机号、认领凭证、框中心世界方位和水平像素误差。反馈只有在现有身份关联接受观测之后发布；双红关联歧义、非负责机、重复或乱序图像不会授权新偏航。

`yaw_diag_1.data` 顺序为：实际机头角度(deg)、参考角度(deg)、参考转速(deg/s)、图像反馈有效标记、框中心水平误差(px)、图像年龄(s)、坐标辅助方位(deg)。无可用图像时像素误差和年龄为 NaN。

评估时记录行人在画面中的水平偏移、折返后恢复时间、静止抖动，以及现有相机诊断中的 `camera_body_yaw_diff`。偏航解决水平居中；垂直位置仍由云台俯仰、跟随距离与高度共同决定。检测延迟与有限角加速度会造成短暂折返偏移，居中也不能代替相机标定和图像/深度/姿态同步。

## 回归检查

```bash
cd $SWARM_WS
(cd build/fly_px4 && ctest -R track_yaw_controller --output-on-failure)
python3 src/fly_px4/test/test_tracking_view.py
python3 src/fly_px4/test/smoke_tracking_yaw.py
```

纯 C++ 测试覆盖实际 10 Hz 图像与管线延迟、静止噪声、折返、角度环绕、变周期与视线过期；ROS 测试在独立 localhost master 下验证主控的真实消息输入/输出，不连接正在运行的飞行系统。它们不替代 Gazebo/PX4 飞行中的画面居中与定位误差测量。

先前算法回归：偏航 ROS 集成 20 项通过、图像视线及身份关联 11 项通过、原坐标时序/位姿守卫 18 项通过，偏航参考 CTest 通过。在 ROS 模拟输入/输出测试中，20 deg/s 匀速阶段的参考角误差末段最大约 0.008°；这是指令参考误差，不是实际飞行机头误差。云台预测已经撤销，先前云台补偿测试不再适用于当前版本；实际画面居中需重新实跑评估。

原始 IMU 适配回归：

```bash
python3 src/fly_px4/test/test_camera_pose_provider.py
bash src/fly_px4/test/run_camera_imu_reference_smoke.sh
```
