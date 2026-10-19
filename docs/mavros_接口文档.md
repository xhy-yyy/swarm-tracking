# MAVROS 接口文档（Typhoon H480 命名空间）

> 命名空间：`/typhoon_h480_0/mavros/`
> 所有接口均在此基础上加前缀，如 `/typhoon_h480_0/mavros/state`

---

## 一、核心控制接口（你的程序在用）

| 话题/服务 | 类型 | 方向 | 用途 |
|-----------|------|------|------|
| `state` | Topic | PX4→ROS | 飞控状态：连接、解锁、模式 |
| `local_position/pose` | Topic | PX4→ROS | 本地位置（NED坐标系） |
| `setpoint_raw/local` | Topic | ROS→PX4 | **速度/位置控制指令** |
| `cmd/arming` | Service | ROS→PX4 | 解锁/上锁电机 |
| `set_mode` | Service | ROS→PX4 | 切换飞行模式（如 OFFBOARD） |
| `param/set` | Service | ROS→PX4 | 设置飞控参数 |

### 使用示例

```cpp
// 订阅状态
nh.subscribe<mavros_msgs::State>("mavros/state", 10, cb);

// 订阅位置
nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, cb);

// 发布速度指令
nh.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 10);

// 解锁服务
nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");

// 切模式服务
nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
```

---

## 二、系统状态类

| 话题 | 消息类型 | 用途 |
|------|----------|------|
| `state` | `mavros_msgs/State` | 飞控核心状态：connected、armed、guided、mode |
| `extended_state` | `mavros_msgs/ExtendedState` | 扩展状态：landed_state（地面/起飞/降落中） |
| `sys_status` | `mavros_msgs/Status` | 系统状态：电池、传感器健康 |
| `battery` | `sensor_msgs/BatteryState` | 电池电压、电流、剩余电量 |
| `radio_status` | `mavros_msgs/RadioStatus` | 遥控器信号强度 |
| `rc/in` | `mavros_msgs/RCIn` | 遥控器输入通道值 |
| `rc/out` | `mavros_msgs/RCOut` | 飞控输出的 PWM 值 |
| `rc/override` | Topic（发布）| 覆盖遥控器通道值 |

---

## 三、位置与姿态类

| 话题 | 消息类型 | 用途 |
|------|----------|------|
| `local_position/pose` | `geometry_msgs/PoseStamped` | NED 坐标系本地位置（x/y/z + 四元数姿态） |
| `local_position/velocity_local` | `geometry_msgs/TwistStamped` | NED 速度（vx/vy/vz） |
| `local_position/velocity_body` | `geometry_msgs/TwistStamped` | 机体坐标系速度 |
| `local_position/accel` | `geometry_msgs/AccelStamped` | 机体坐标系加速度 |
| `local_position/odom` | `nav_msgs/Odometry` | 里程计（位置+速度） |
| `global_position/global` | `sensor_msgs/NavSatFix` | GPS 经纬度 |
| `global_position/local` | `nav_msgs/Odometry` | GPS→本地坐标系转换 |
| `global_position/rel_alt` | `std_msgs/Float64` | 相对地面高度 |
| `global_position/compass_hdg` | `std_msgs/Float64` | 罗盘航向角 |
| `altitude` | `mavros_msgs/Altitude` | 多种高度（相对/海平面/地形） |
| `home_position/home` | `mavros_msgs/HomePosition` | 返航点位置 |
| `home_position/set` | Topic（发布） | 设置返航点 |
| `vision_pose/pose` | `geometry_msgs/PoseStamped` | 视觉定位输入（VIO/外部定位） |

---

## 四、IMU / 传感器类

| 话题 | 消息类型 | 用途 |
|------|----------|------|
| `imu/data` | `sensor_msgs/Imu` | 融合后的 IMU 数据（加速度+角速度） |
| `imu/data_raw` | `sensor_msgs/Imu` | 原始 IMU 数据 |
| `imu/mag` | `sensor_msgs/MagneticField` | 磁力计 |
| `imu/static_pressure` | `sensor_msgs/FluidPressure` | 气压计 |
| `imu/temperature_imu` | `sensor_msgs/Temperature` | IMU 温度 |
| `imu/temperature_baro` | `sensor_msgs/Temperature` | 气压计温度 |

---

## 五、控制指令类（发布给 PX4）

### 速度控制

| 话题 | 消息类型 | 用途 |
|------|----------|------|
| `setpoint_raw/local` | `mavros_msgs/PositionTarget` | **通用控制指令**：可以控制速度/位置/加速度/姿态 |
| `setpoint_velocity/cmd_vel` | `geometry_msgs/TwistStamped` | 速度指令（带时间戳） |
| `setpoint_velocity/cmd_vel_unstamped` | `geometry_msgs/Twist` | 速度指令（无时间戳） |

> **你的程序用的是 `setpoint_raw/local`**：type_mask 配置为 IGNORE 位置、YAW，只控制 vx/vy/vz。

### 位置控制

| 话题 | 消息类型 | 用途 |
|------|----------|------|
| `setpoint_position/local` | `geometry_msgs/PoseStamped` | 本地位置指令（NED坐标） |
| `setpoint_position/global` | `mavros_msgs/GlobalPositionTarget` | GPS 位置指令 |
| `setpoint_raw/global` | `mavros_msgs/GlobalPositionTarget` | 通用 GPS 控制指令 |

### 姿态控制

| 话题 | 消息类型 | 用途 |
|------|----------|------|
| `setpoint_raw/attitude` | `mavros_msgs/AttitudeTarget` | 姿态+油门指令 |
| `setpoint_attitude/cmd_vel` | `geometry_msgs/TwistStamped` | 角速度 + 油门指令 |
| `setpoint_attitude/thrust` | `mavros_msgs/Thrust` | 推力向量指令 |

---

## 六、任务类

| 话题/服务 | 类型 | 用途 |
|-----------|------|------|
| `mission/waypoints` | Topic | 当前任务航点列表 |
| `mission/reached` | Topic | 指示最后一个到达的航点序号 |
| `geofence/waypoints` | Topic | 地理围栏航点 |

---

## 七、参数类

| 接口 | 类型 | 用途 |
|------|------|------|
| `param/set` | Service | 设置单个参数（PARAM_SET） |
| `param/get` | Service | 读取单个参数（PARAM_GET） |
| `param/param_value` | Topic | 参数变更通知 |

### 使用示例

```bash
# 命令行
rosservice call /typhoon_h480_0/mavros/param/set "param_id: 'COM_RCL_EXCEPT' value: {integer: 4, real: 0.0}"
```

---

## 八、遥控器 / 手动控制

| 话题 | 用途 |
|------|------|
| `rc/in` | 遥控器输入的原始通道值 |
| `rc/out` | 飞控输出的 PWM 值 |
| `rc/override` | 强制覆盖遥控器通道值（仿真中模拟遥控器） |
| `manual_control/control` | 手动控制杆量 |
| `manual_control/send` | 发送手动控制指令（可用于虚拟摇杆） |

---

## 九、VFR HUD（飞行仪表）

| 话题 | 消息类型 | 用途 |
|------|----------|------|
| `vfr_hud` | `mavros_msgs/VfrHUD` | 空速、地速、航向、油门、高度、爬升率 |

常用于 QGC 地面站的 HUD 显示。

---

## 十、常用查看命令速查

```bash
# 实时查看飞控状态
rostopic echo /typhoon_h480_0/mavros/state

# 实时查看位置
rostopic echo /typhoon_h480_0/mavros/local_position/pose

# 查看话题类型
rostopic type /typhoon_h480_0/mavros/state

# 查看消息字段定义
rosmsg show mavros_msgs/State

# 查看全部服务
rosservice list | grep typhoon_h480_0/mavros

# 查看电池
rostopic echo /typhoon_h480_0/mavros/battery

# 查看 VFR HUD
rostopic echo /typhoon_h480_0/mavros/vfr_hud
```
