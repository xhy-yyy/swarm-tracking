# 固定高度控制修复（2026-10-10）

本次修复针对 `fly_px4_1` 的高度指令链，不调整 PX4 高度环参数、水平速度、跟踪 PID、机头控制或六机分配策略。本文保留原开发阶段的设计记录；本次发布的验证范围见根目录 docs/validation.md。

## 已确认的根因

1. `forwardEgoCmd()` 巡逻时只把位置限制在高度区间，没有锁定 YAML 的 `z`；跟踪时虽然钳成 `track_z`，但仍透传某些垂直速度和加速度。在本机 PX4 的 `PositionControl::_positionControl/_velocityControl` 中，位置反馈、速度前馈和加速度前馈会相加，钳位置不能消除冲突。
2. EGO PVA 直接发布高度，不更新 `pos_lpf_z_`；PID、悬停、脱困又使用该滤波器的旧值。例如滤波历史停在 4.5 m，EGO 已下发 2.7 m，切到 PID 首拍约为 4.299 m。这是代码即可复现的高度指令回跳。
3. 巡逻脱困使用每拍实测 `cz` 作高度目标，正常巡逻等待使用 `route_z`；跟踪目标却发给 EGO `route_z`，PX4 使用 `track_z`。释放滞回还会撤销位置锁高。
4. EGO 的短距离全局轨迹分支使用未投影的起终点；全局规划的垂直导数和急停位置也没有完整遵守 `plane_mode`。因此配置为二维仍可能出现带 Z 分量的轨迹。

## 现在的执行方式

`flightAltitude()` 仅依据飞行状态选择目标，避免依赖另一个可能延迟更新的 `tracking_active_` 标志。

| 状态/分支 | 目标高度 |
| --- | --- |
| CLIMB、TRANSIT、PATROL、巡逻原地转向和脱困 | YAML 巡航 `z` |
| TRACK、近距 PID、远距 EGO、观测暂停、短暂目标释放和跟踪回退 | YAML `track_z` |
| 位姿无效、未 OFFBOARD 或未解锁 | 原有三轴零速降级，关闭高度参考 |
| 软限高恢复 | 当前状态高度与 `max_expl_z` 中较低者，零垂直前馈 |
| 硬限高 | 原有 AUTO.LAND 优先级 |

`AltitudeHold` 只生成 Z/VZ/AZ，不限制或暂停水平运动。状态高度改变时使用五次曲线，位置、速度和加速度来自同一参考；进入目标后输出精确目标高度和零垂直前馈。重复目标、PID/EGO/悬停切换均不重启曲线。默认沿用现有 `max_vz=0.5 m/s`、`max_az=1.0 m/s²`，4.5↔2.7 m 参考过渡为 6.75 s。运动中目标反向会先按加速度约束制动，再规划新升降，避免速度瞬间反向。

`ControlPx4::publish_local_target()` 是唯一 Z 出口。主控启用高度参考后，它统一覆盖 PZ/VZ/AFZ 并启用对应 mask；各水平模式绕过旧 Z LPF，防止重新输出历史高度。XY 数值、XY 滤波、偏航及其 mask 保持各模式既有语义。释放滞回仍使用原有 XY 零速滤波及忽略 yaw 的模式，出口补上高度控制。共享控制类默认不启用此功能，`fly_px4_2`/RACER 的三维行为保持原样。

起飞保持原来的位置目标方式；空中位姿/模式恢复后从有效实测高度重新生成参考。配置不再被静默钳成另一高度：巡航 `z` 必须满足原有 `[min_expl_z,max_expl_z]` 范围，飞行高度须低于软限高，航点显式 `z` 必须等于巡航高度，不合法时启动报错。

## EGO 平面和实际高度

`publishEgoGoal()` 对外记录状态对应的真实目标高度，EGO 接收到后统一投影到它的固定虚拟平面。保留 `plane_mode=true`、默认 `plane_z=4.5`，因为二维雷达点云也投影到该平面，六机共享轨迹避让使用同一平面。仅动态把规划平面改成 2.7 m，会让原平面上的雷达障碍不可见。

EGO 的 FSM 保存投影后的终点，全局轨迹的长短分支均投影位置并清除垂直导数，急停同样投影。主控只采纳 EGO 的水平 PVA，实际高度不会随水平避障规划变化。launch 仅更新说明，没有修改运行参数或地图。

## 验证与使用

已编译 `fly_px4_1`、共享底层调用者 `fly_px4_2` 和 `ego_planner_node`。验证包括纯高度参考的导数与边界、真实 ROS 发布的 PositionTarget/mask/水平字段、实际主控的状态切换、EGO 平面路径以及现有机头、跟踪导航和水平避障测试。隔离测试使用自己的 loopback ROS master、假 MAVROS/里程计，不连接现有仿真或飞控。

可复跑：

```bash
source /opt/ros/noetic/setup.bash
source $SWARM_WS/devel/setup.bash
cd $SWARM_WS/build/fly_px4
ctest --output-on-failure
/usr/bin/python3 $SWARM_WS/src/fly_px4/test/smoke_altitude_hold.py
/usr/bin/python3 $SWARM_WS/src/fly_px4/test/smoke_patrol_waypoint_handoff.py
```

代码与二进制已更新，下一轮飞行需重新启动主控及 EGO 规划节点才能使用修复。没有实际飞行日志，不能据此保证机体无剩余超调，也不能确定 PX4 高度环、估计器或推力/姿态耦合是否还有问题。

标准 swarm launch 没有启动额外的 `ego_to_mavros`/`ego_trigger`，但无法排除手动启动的竞争发布者。实飞若仍波动，应同时记录六机的 `setpoint_raw/local`、`setpoint_raw/target_local`、`local_position/pose`、`local_position/velocity_local`、`state`、EGO `pos_cmd` 和 tracking `mode`，以及 PX4 ULog。检查各机 `rostopic info /typhoon_h480_N/mavros/setpoint_raw/local` 是否只有预期的主控 publisher；据指令与实测响应区分剩余原因，再决定是否调整飞控参数。
