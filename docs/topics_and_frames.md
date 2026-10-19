# 话题、编号和坐标系

## 编号

主控与 `track_msgs` 使用机号 **1..6**。仿真命名空间使用
`typhoon_h480_0` 到 `typhoon_h480_5`，因此 `namespace_index = drone_id - 1`。

## 主要话题

| 话题 | 类型 / 作用 |
| --- | --- |
| `/typhoon_h480_N/mavros/local_position/odom` | `nav_msgs/Odometry`，MAVROS 本地里程计 |
| `/typhoon_h480_N/odom_world_relay` | `nav_msgs/Odometry`，世界系里程计 |
| `/typhoon_h480_N/pose_relay` | `geometry_msgs/PoseStamped`，世界系位姿 |
| `/vision/detections` | `track_msgs/DetectionArray`，图像时刻检测与定位 |
| `/tracking/target_I` | `nav_msgs/Odometry`，兼容的跟踪目标输出 |
| `/tracking/state_I` | `track_msgs/TrackingTarget`，状态、量测年龄和任务令牌 |
| `/tracking/view_I` | `track_msgs/TrackingView`，当前任务的图像时刻偏航反馈 |
| `/tracking/mode_I` | `std_msgs/Int8`，模式通知 |
| `/tracking/active_I` | `std_msgs/Header`，主控跟踪回执及任务令牌 |
| `/tracking/reject_I` | `std_msgs/Int8`，主控拒绝或释放目标 |
| `/tracking/targets` | `track_msgs/TargetArray`，全局目标集合 |
| `/tracking/mission_complete` | `std_msgs/Bool`，任务完成通知 |
| `/ego/pos_cmd_I` | `quadrotor_msgs/PositionCommand`，规划轨迹指令 |
| `/typhoon_h480_N/mavros/setpoint_raw/local` | `mavros_msgs/PositionTarget`，本地飞控指令 |

`N=0..5`，`I=1..6`。相机、深度和 IMU 话题后缀可通过 launch 参数配置，
以模型 SDF 和实际 `rostopic list` 为准。

## 时间与坐标

- 检测 `header.stamp` 是成像时刻，`frame_id` 为 `world`。
- 视觉定位结合图像时刻的机体位姿、相机 IMU 和云台几何；不能用节点当前姿态替代。
- MAVROS 本地 XY 先按 `local_xy_scale` 缩放，再加对应机号的起点偏移。
- 规划指令回到本机飞控前执行对应的逆变换。
- `TrackingTarget.state.header.stamp` 表示状态外推时刻；`measurement_stamp`
  与 `depth_stamp` 单独记录观测时刻，`assignment` 用来识别任务归属。
- `TrackingView` 仅来自被接受的真实图像，不通过定时器重发旧图像反馈。
- 红色目标使用两个航迹槽 `red1/red2`，不按 Gazebo actor 编号关联。

`world_frame.yaml` 当前尺度为 `6353000 / 6371000`，对应原模型的 GPS 半径设置；
如果更换 GPS 插件参数，需要同步调整。机体 FLU、相机光轴、图像内参和
`camera_imu_reference` 的详细约定见 `src/fly_px4/docs/`。
