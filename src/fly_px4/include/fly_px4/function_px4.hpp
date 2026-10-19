#pragma once

#include <ros/ros.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/PoseStamped.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <functional>

#include "fly_px4/control_px4.hpp"

// 主流程 EXPLORE → LANDING；TAKEOFF/TRACKING 接口保留，解锁与 OFFBOARD 由外部守卫处理。
enum class FlightState {
    TAKEOFF  = 0,   // (保留, 不进入) 旧式独立起飞阶段
    EXPLORE  = 1,   // 探索: 转发 RACER PositionCommand (PVA+yaw) 到 mavros
    TRACKING = 2,   // (预留) 跟踪嫌疑犯, 跟丢后回 EXPLORE
    LANDING  = 3,   // 切 AUTO.LAND 由 PX4 着地+上锁
    DONE     = 4    // 完成
};

// 回退模式：HOVER 保持当前位置，YAW 停止 XY 并转向，RETREAT 沿历史轨迹回退。
struct ExplFallback {
    enum class Mode { HOVER, YAW_SPIN, RETREAT };
    Mode mode = Mode::HOVER;
    bool valid = false;       // false -> handle_explore 退回原行为(起飞点悬停)
    double x = 0.0;           // 目标位置(本地帧). HOVER/YAW_SPIN 用当前位置
    double y = 0.0;
    double yaw_deg = 0.0;     // YAW_SPIN 用递增航向; 其余用锁存航向
    bool use_yaw = false;     // true 时用上面的 yaw_deg, false 用 lockedYawDeg()
};

// handle_* 返回 true 表示阶段完成，指令通过 ControlPx4 下发。
class FunctionPx4 {
public:
    using GetYawCb = std::function<double()>;   // 返回当前航向(rad)

    FunctionPx4(ros::NodeHandle& nh,
                ros::ServiceClient& set_mode_cli,
                ros::ServiceClient& arming_cli,
                ControlPx4* control,
                GetYawCb get_yaw);

    // 状态机访问
    FlightState getFlightState() const { return flight_state_; }
    void setFlightState(FlightState s) { flight_state_ = s; }
    void setArmed(bool a) { armed_ = a; }

    // 新任务复位(清起飞/探索/降落内部状态, 航向锁存会重锁)
    void resetForNewTask();

    // 设置 spawn 偏移(世界帧下该机本地原点坐标). RACER pos_cmd 是世界帧,
    // PX4 FRAME_LOCAL_NED 是本地帧(原点=spawn), 转发前需减去此偏移.
    void setSpawnOffset(double x, double y) { spawn_offset_x_ = x; spawn_offset_y_ = y; }

    // 各阶段处理函数

    // 备用起飞流程锁定 XY/偏航，要求高度持续稳定。
    bool handle_takeoff(double cx, double cy, double cz, double target_height);

    // 位置控制辅助：XY PD、Z 保持，到达判断带滞回。
    bool handle_fly_to_point_3d_pid(double cx, double cy, double cz,
                                   double tx, double ty, double tz, double speed);

    // 有新鲜指令且未回退时转发 PVA，否则执行回退；完成返回 true。
    // 跟踪仅覆盖偏航。
    bool handle_explore(double cx, double cy, double cz,
                        bool has_cmd,
                        const quadrotor_msgs::PositionCommand& cmd,
                        bool exploration_done,
                        double min_expl_z, double max_expl_z,
                        const ExplFallback& fb = ExplFallback(),
                        bool yaw_override = false,
                        double yaw_override_deg = 0.0);

    // 降落: 切 AUTO.LAND 由 PX4 完成着地+上锁(只发一次模式切换请求, land_pub_state_ 防重发)
    void handle_landing_pub();

    // 地面待命心跳: 三轴零速, 满足 PX4 进 OFFBOARD 前置条件. 飞行中禁用(纯速度环会被 bias 拉偏)
    void hold_for_offboard();

private:
    // 起飞时锁存的航向(度), 全程打点沿用, 避免飞行中航向乱转
    double lockedYawDeg();

    ros::NodeHandle& nh_;
    ros::ServiceClient& set_mode_cli_;
    ros::ServiceClient& arming_cli_;
    ControlPx4* control_;
    GetYawCb get_yaw_;

    FlightState flight_state_ = FlightState::EXPLORE;
    bool armed_ = false;

    // 起飞锁存
    bool takeoff_inited_ = false;
    double takeoff_x_ = 0.0, takeoff_y_ = 0.0;
    double takeoff_yaw_deg_ = 0.0;

    // 3D PID 内部状态(每拍微分滤波 + 指令限幅)
    bool pd3_active_ = false;
    ros::Time pd3_last_time_;
    double pd3_last_x_ = 0.0, pd3_last_y_ = 0.0, pd3_last_z_ = 0.0;
    double pd3_vx_filt_ = 0.0, pd3_vy_filt_ = 0.0, pd3_vz_filt_ = 0.0;
    double pd3_cmd_vx_ = 0.0, pd3_cmd_vy_ = 0.0;

    // 到达判据滞回计时(takeoff / 3d_pid 共用)
    bool delay_started_ = false;
    ros::Time delay_start_time_;

    // landing 防重发
    bool land_pub_state_ = false;

    // EXPLORE 内部状态
    // 进入 EXPLORE 后首拍锁存悬停高度(RACER 未出指令时锁此高度, 防掉高)
    bool expl_inited_ = false;
    double expl_hover_z_ = 0.0;
    bool expl_first_cmd_logged_ = false;

    // spawn 偏移(世界帧): RACER pos_cmd(世界) 转发到 PX4(本地) 时需减去
    double spawn_offset_x_ = 0.0, spawn_offset_y_ = 0.0;
};
