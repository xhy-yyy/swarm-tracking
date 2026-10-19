# 跟踪与归航的升降交接

`fly_px4_1` 在巡航与跟踪之间分阶段飞行：

1. 保持进入阶段时的实际 XY 和高度，水平速度、垂直速度连续稳定后才开始升降。
2. 以刹稳时的实际 XY 为保持点，用五次多项式生成 Z、Vz、Az，先完成升降。
3. 参考到达终点后，继续等待实际高度和 XYZ 速度稳定。
4. 跟踪发送 `/ego_track_goal_N`，归航实际偏航对准后发送 `/ego_return_goal_N`，请求 EGO 从实际里程计生成新轨迹。
5. 收到本次请求的 ACK，并收到对应首条轨迹的有效、邻近参考点后，才开启水平 EGO 或近距 PID。

升降阶段不会发布跟踪站位目标、执行旧 EGO 指令或进入 PID。中途释放、改派、暂停、观测 STOP、位姿/速度断流和 OFFBOARD 中断都会撤销旧交接；恢复时根据实际位置继续。运行中明显偏离任务高度也会重新升降、重新交接。ROS 时钟回拨撤销主控和 EGO 的旧请求与授权。

## 默认限制

主控私有参数：

| 参数 | 默认值 | 用途 |
| --- | --- | --- |
| `max_vz` | 0.5 m/s | 升降参考最大速度 |
| `max_az` | 1.0 m/s² | 升降参考最大加速度 |
| `altitude_xy_speed_tolerance` | 0.2 m/s | 刹稳/稳定判据 |
| `altitude_z_speed_tolerance` | 0.1 m/s | 垂直稳定判据 |
| `altitude_z_tolerance` | 0.15 m | 实测到高判据 |
| `altitude_stable_time` | 0.3 s | 判据连续成立时间 |
| `altitude_reentry_tolerance` | 0.3 m | 水平飞行中偏离高度时重新交接 |
| `pose_timeout` | 0.5 s | 位姿和本地三轴速度的接收/采样时效 |

高度仍取航线 YAML 的 `z`/航点 `z` 和 `track_z`。默认 4.5 → 2.7 m 的参考升降耗时约 6.75 s，刹停和实际稳定等待另外计算。反馈来自 MAVROS 的 `local_position/pose` 和 `local_position/velocity_local`，XY 使用项目的物理米坐标变换，Z 不缩放。

## EGO 接口与地图

首次跟踪请求使用 `nav_msgs/Odometry`：`header.stamp` 标识请求，`pose.pose.position` 携带世界坐标目标，`twist.twist.linear` 原子绑定目标速度。`/ego_track_ready_N` 和 `/ego_return_ready_N` 的 ACK 格式均为 `[请求时间戳 ns, 首条轨迹 ID, 轨迹起始时间 ns]`。默认每 2 s 无可接管轨迹时重新请求，不沿旧参考点接轨。

完成升降后的目标高度同步到 EGO 全局轨迹、A* 和样条优化器。水平轨迹与执行高度保持一致；主控拒绝高度不匹配的参考。共享控制层在交接时重置速度滤波历史，PID 高度直接保持已完成的跟踪高度。

`single_relay_ego.launch` 开启 `grid_map/column_mode`。雷达点仍存于固定 `column_plane_z` 层，各允许飞行高度按同一 XY 列查询占据，真实 XYZ 地图边界仍有效。这样切换到 2.7 m 不会让固定存储层的墙体消失。

该地图表示二维雷达当前扫描截面的 XY 障碍；它没有新增上、下方的三维感知。分阶段动作减少同时升降和水平切墙角的风险，不能据此断言屋檐、横梁等未被扫描到的垂直障碍已经检测到。

## 使用与验证

已编译主控、共享控制层和 EGO 相关库。应用时同时重启 `swarm_relay_ego.launch` 和 `swarm_fly_px4_1.launch`，保证双方使用新的目标/ACK接口。

验证包括 C++ 状态机测试 `test_altitude_transition`，隔离 ROS 的 `smoke_altitude_handoff.py`、`smoke_return_handoff.py` 以及十图 `smoke_patrol_waypoint_handoff.py`。ROS 回归使用随机 loopback master、fake MAVROS 和实际 EGO 节点，不连接在运行的飞行/仿真 master；这些检查不替代 Gazebo/PX4 实际飞行验证。
