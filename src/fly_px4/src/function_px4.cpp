#include "fly_px4/function_px4.hpp"

FunctionPx4::FunctionPx4(ros::NodeHandle& nh,
                         ros::ServiceClient& set_mode_cli,
                         ros::ServiceClient& arming_cli,
                         ControlPx4* control,
                         GetYawCb get_yaw)
    : nh_(nh),
      set_mode_cli_(set_mode_cli),
      arming_cli_(arming_cli),
      control_(control),
      get_yaw_(std::move(get_yaw)) {}

// 起飞锁定 XY 和偏航，高度到达判断带稳定时间与滞回。
bool FunctionPx4::handle_takeoff(double cx, double cy, double cz, double target_height) {
    constexpr double ARRIVE_DIST = 0.1;         // 到达阈值 m (Z 方向)
    constexpr double ARRIVE_RESET_DIST = 0.15;  // 滞回复位阈值 m

    // 首次进入: 锁存起飞点 XY + 起飞航向(整个爬升+后续打点航向保持不变)
    if (!takeoff_inited_) {
        takeoff_x_ = cx;
        takeoff_y_ = cy;
        takeoff_yaw_deg_ = get_yaw_() * 180.0 / M_PI;
        ROS_INFO("[起飞] 锁定起飞点 (%.2f, %.2f), 目标高度 %.2f, 航向 %.1f°",
                 cx, cy, target_height, takeoff_yaw_deg_);
        takeoff_inited_ = true;
    }

    // 位置锁(起飞点 XY, 目标高度), 速度前馈=0, 航向锁
    control_->fly_posvel_yaw(takeoff_x_, takeoff_y_, target_height,
                             0.0, 0.0, 0.0, takeoff_yaw_deg_);

    double dist = std::fabs(target_height - cz);
    ROS_INFO_THROTTLE(0.5, "[起飞] 高度 %.2f / %.2f, 差 %.2f",
                      cz, target_height, dist);

    // 到达判据: Z 差稳定 0.3s, 带滞回
    auto now = ros::Time::now();
    bool in_pos  = dist < ARRIVE_DIST;
    bool out_pos = dist > ARRIVE_RESET_DIST;
    if (delay_started_) {
        if (out_pos) {
            delay_started_ = false;
        } else if ((now - delay_start_time_).toSec() >= 0.3) {
            delay_started_ = false;
            ROS_INFO("[起飞] 起飞完成, 高度 %.2f", cz);
            return true;
        }
    } else if (in_pos) {
        delay_start_time_ = now;
        delay_started_ = true;
    }
    return false;
}

// 降落: 切 AUTO.LAND 由 PX4 完成着地+上锁 (只发一次模式切换请求, land_pub_state_ 防重发)
void FunctionPx4::handle_landing_pub() {
    if (!land_pub_state_) {
        mavros_msgs::SetMode cmd;
        cmd.request.custom_mode = "AUTO.LAND";
        set_mode_cli_.call(cmd);
        land_pub_state_ = true;
        ROS_INFO("[降落] 已发送模式切换: OFFBOARD -> AUTO.LAND");
    }
}

// 地面待命心跳: 三轴零速, 满足 PX4 进 OFFBOARD 前须有 >2Hz setpoint 流的前置条件.
// 飞行中禁用 —— 纯速度环会被里程计速度 bias 拉偏, 只用于未解锁地面等待期.
void FunctionPx4::hold_for_offboard() {
    control_->fly_velxyz(0.0, 0.0, 0.0);
}

void FunctionPx4::resetForNewTask() {
    flight_state_ = FlightState::EXPLORE;  // 跳过独立起飞: 初始即 EXPLORE
    takeoff_inited_ = false;
    pd3_active_ = false;
    delay_started_ = false;
    land_pub_state_ = false;
    armed_ = false;
    // EXPLORE 状态复位
    expl_inited_ = false;
    expl_hover_z_ = 0.0;
    expl_first_cmd_logged_ = false;
}

// 转发新鲜 RACER PVA；无有效指令时执行回退，完成后返回 true。
bool FunctionPx4::handle_explore(double cx, double cy, double cz,
                                 bool has_cmd,
                                 const quadrotor_msgs::PositionCommand& cmd,
                                 bool exploration_done,
                                 double min_expl_z, double max_expl_z,
                                 const ExplFallback& fb,
                                 bool yaw_override,
                                 double yaw_override_deg) {
    // 进入 EXPLORE 首拍: 锁存当前高度作为 RACER 未出指令时的悬停高度.
    // 跳过独立起飞: 进 EXPLORE 时可能还在低空(甚至刚离地), 悬停高度钳到至少
    // min_expl_z, 保证 RACER 未出指令时飞机自行爬升进探索盒, 不掉低空.
    if (!expl_inited_) {
        expl_hover_z_ = std::max(cz, min_expl_z);
        expl_inited_ = true;
        ROS_INFO("[探索] 进入 EXPLORE, 悬停高度锁存 %.2f (钳位≥%.2f), 起飞点 (%.2f, %.2f)",
                 expl_hover_z_, min_expl_z, takeoff_x_, takeoff_y_);
    }

    // 兜底动作优先: fb.valid 时说明 Task 判定"没有新指令"或"有指令但飞机冻住了",
    // 这一拍不转发 RACER 指令, 改执行 HOVER / YAW_SPIN / RETREAT。
    // 三种动作都不做盲飞平移, 详见 ExplFallback 的注释。
    if (fb.valid) {
        // 兜底动作的航向: 跟踪时优先把机头留在目标上(除非兜底动作本身就是要转 yaw
        // 刷新感知, 那时 fb.use_yaw 为 true, 以它为准)。
        // 这样"悬停/沿航迹回退"这两层脱困期间, 目标仍在画面里, 锁定不会断。
        const double yaw_deg = fb.use_yaw ? fb.yaw_deg
                             : (yaw_override ? yaw_override_deg : lockedYawDeg());
        control_->fly_posvel_yaw(fb.x, fb.y, expl_hover_z_, 0.0, 0.0, 0.0, yaw_deg);
        const char* mode_str = (fb.mode == ExplFallback::Mode::YAW_SPIN) ? "原地自转"
                             : (fb.mode == ExplFallback::Mode::RETREAT)  ? "沿航迹回退"
                                                                         : "原地悬停";
        ROS_WARN_THROTTLE(1.0, "[脱困] %s -> (%.2f, %.2f, %.2f) yaw=%.0f°, 当前(%.2f,%.2f,%.2f)",
                          mode_str, fb.x, fb.y, expl_hover_z_, yaw_deg, cx, cy, cz);
        if (exploration_done) {
            ROS_WARN("[探索] 收到 RACER FINISH 信号, 探索完成");
            return true;
        }
        return false;
    }

    if (has_cmd) {
        if (!expl_first_cmd_logged_) {
            ROS_INFO("[探索] 收到首条 RACER 指令: pos(%.2f,%.2f,%.2f) yaw=%.2f",
                     cmd.position.x, cmd.position.y, cmd.position.z, cmd.yaw);
            expl_first_cmd_logged_ = true;
        }
        // z 钳位到 [min_expl_z, max_expl_z]: RACER box 已约束, 这里做安全兜底
        // (防 RACER 用地面 stale odom 规划期把飞机压向低空/超高)
        double z_cmd = cmd.position.z;
        if (z_cmd < min_expl_z) z_cmd = min_expl_z;
        if (z_cmd > max_expl_z) z_cmd = max_expl_z;
        // 坐标转换: RACER pos_cmd 是世界帧, PX4 FRAME_LOCAL_NED 是本地帧(原点=spawn).
        // 位置减去 spawn 偏移; 速度/加速度/航向不变(本地帧与世界帧仅有平移, 无旋转).
        double lx = cmd.position.x - spawn_offset_x_;
        double ly = cmd.position.y - spawn_offset_y_;
        // yaw: 跟踪时用覆写值(机头压住目标), 否则用 RACER 规划的航向
        double yaw_deg = yaw_override ? yaw_override_deg : (cmd.yaw * 180.0 / M_PI);
        control_->fly_pva_yaw(
            lx, ly, z_cmd,
            cmd.velocity.x, cmd.velocity.y, cmd.velocity.z,
            cmd.acceleration.x, cmd.acceleration.y, cmd.acceleration.z,
            yaw_deg);
        ROS_INFO_THROTTLE(0.5,
            "[%s] 跟踪 RACER 指令 世界(%.2f,%.2f,%.2f)->本地(%.2f,%.2f,%.2f) 当前(%.2f,%.2f,%.2f) yaw=%.0f%s",
            yaw_override ? "跟踪" : "探索",
            cmd.position.x, cmd.position.y, cmd.position.z, lx, ly, z_cmd, cx, cy, cz,
            yaw_deg, yaw_override ? "(锁目标)" : "");
    } else {
        // RACER 尚未出指令: 在起飞点 XY + 锁存高度悬停, 航向锁
        control_->fly_posvel_yaw(takeoff_x_, takeoff_y_, expl_hover_z_,
                                 0.0, 0.0, 0.0, lockedYawDeg());
        ROS_INFO_THROTTLE(0.5, "[探索] 等待 RACER 指令, 悬停 (%.2f, %.2f, %.2f)",
                          takeoff_x_, takeoff_y_, expl_hover_z_);
    }

    if (exploration_done) {
        ROS_WARN("[探索] 收到 RACER FINISH 信号, 探索完成");
        return true;
    }
    return false;
}

double FunctionPx4::lockedYawDeg() {
    // 起飞航向已锁则沿用, 否则用当前航向(takeoff 尚未跑到的边界情况)
    return takeoff_inited_ ? takeoff_yaw_deg_ : (get_yaw_() * 180.0 / M_PI);
}

// 3D PID 打点
// 位置->速度 PD 控制(三轴, 无 I), XY 限速 + 刹车曲线 + 加速度限幅, Z 走位置环锁高, 航向锁定.
// 对应 fly_a9 的 handle_fly_to_point_3d_pid (vcz 不参与下发, 只用 XY 速度环 + Z 位置环).
bool FunctionPx4::handle_fly_to_point_3d_pid(double cx, double cy, double cz,
                                             double tx, double ty, double tz, double speed) {
    constexpr double VEL_KP = 0.80;        // 位置->速度 P 增益 1/s (足够大让巡航段被限速器封顶)
    constexpr double VEL_KD = 0.25;        // 速度阻尼 D 增益 (乘滤波速度, 压过冲主旋钮)
    constexpr double VEL_D_LPF_FC = 2.0;   // Hz, 速度估计低通截止 (D 项 + 到达速度门共用)
    constexpr double VEL_BRAKE_DEC = 0.50; // m/s^2, 刹车封顶(太小→速度上不去+冲过头; 太大→磨蹭)
    constexpr double VEL_ACCEL = 1.5;      // m/s^2, 速度指令爬升加速度上限 (防从0突跳抢推力)
    constexpr double VEL_STALE = 0.2;     // s, 距上拍超时视为重入, 滤波/指令清零重启
    constexpr double ARRIVE_DIST = 0.1;         // 到达阈值 m (三维距离)
    constexpr double ARRIVE_RESET_DIST = 0.15;  // 滞回复位阈值 m
    constexpr double ARRIVE_VEL = 0.10;          // m/s, 到达速度门 (超 2 倍复位计时)
    constexpr double DEFAULT_FLY_SPEED = 0.4;   // speed<=0 时的默认逼近限速 m/s
    double fly_speed = (speed > 0.0) ? speed : DEFAULT_FLY_SPEED;

    auto now = ros::Time::now();
    double loop_dt = (now - pd3_last_time_).toSec();   // 上拍->本拍 (刷新前抓, 供加速度限幅)
    if (!pd3_active_ || loop_dt > VEL_STALE) {
        // 首次进入/状态机离开后重入: 速度估计清零, 本拍退化为纯 P (无 D 阶跃)
        pd3_vx_filt_ = 0.0;
        pd3_vy_filt_ = 0.0;
        pd3_vz_filt_ = 0.0;
        pd3_cmd_vx_ = 0.0;
        pd3_cmd_vy_ = 0.0;
        pd3_active_ = true;
    } else if (loop_dt > 1e-6) {
        // 速度估计: 位置数值微分 + 一阶低通 (喂 D 项 + 到达速度门)
        double alpha = loop_dt / (loop_dt + 1.0 / (2.0 * M_PI * VEL_D_LPF_FC));
        pd3_vx_filt_ += alpha * ((cx - pd3_last_x_) / loop_dt - pd3_vx_filt_);
        pd3_vy_filt_ += alpha * ((cy - pd3_last_y_) / loop_dt - pd3_vy_filt_);
        pd3_vz_filt_ += alpha * ((cz - pd3_last_z_) / loop_dt - pd3_vz_filt_);
    }
    pd3_last_time_ = now;
    pd3_last_x_ = cx;
    pd3_last_y_ = cy;
    pd3_last_z_ = cz;

    // 控制律(位置->速度): v_cmd = Kp·e − Kd·v_est (三轴, 无 I)
    double ex = tx - cx;
    double ey = ty - cy;
    double ez = tz - cz;
    double dist = std::sqrt(ex * ex + ey * ey + ez * ez);   // 三维: 到达判据/日志用
    double dist_xy = std::hypot(ex, ey);                    // 水平: 限幅只看这个
    double vcx = VEL_KP * ex - VEL_KD * pd3_vx_filt_;
    double vcy = VEL_KP * ey - VEL_KD * pd3_vy_filt_;
    double vcz = VEL_KP * ez - VEL_KD * pd3_vz_filt_;
    // 限幅: 巡航限速 + 刹车曲线 (近区 √(2ad) 自然趋零 -> 平滑刹停). 只约束 XY
    // (下发通道只有 XY 速度, Z 走位置环 —— 把 vcz 计进速度模长等于让不下发的量抢 XY 预算)
    double v_lim = std::min(fly_speed, std::sqrt(2.0 * VEL_BRAKE_DEC * dist_xy));
    double v_mag = std::hypot(vcx, vcy);
    if (v_mag > v_lim && v_mag > 1e-9) {
        double k = v_lim / v_mag;
        vcx *= k;
        vcy *= k;
    }
    // 加速度限幅(仅 XY): 平滑指令爬升/反向, 防突跳抢推力 (首拍 pd3_cmd_v*=0)
    if (loop_dt > 1e-6 && loop_dt < 0.1) {
        double dv_max = VEL_ACCEL * loop_dt;
        double dvx = vcx - pd3_cmd_vx_;
        double dvy = vcy - pd3_cmd_vy_;
        double dv_mag = std::hypot(dvx, dvy);
        if (dv_mag > dv_max) {
            vcx = pd3_cmd_vx_ + dvx * dv_max / dv_mag;
            vcy = pd3_cmd_vy_ + dvy * dv_max / dv_mag;
        }
    }
    pd3_cmd_vx_ = vcx;
    pd3_cmd_vy_ = vcy;
    // 下发: XY 走速度环(PD 逼近), Z 锁位置(tz 走 PX4 位置环, 不给速度), 锁定航向.
    control_->fly_velxy_posez_yaw(vcx, vcy, tz, lockedYawDeg(), 0.0);
    (void)vcz;   // vcz 不参与下发, 保留只为三轴控制律写法完整

    double speed_est = std::sqrt(pd3_vx_filt_ * pd3_vx_filt_ +
                                 pd3_vy_filt_ * pd3_vy_filt_ +
                                 pd3_vz_filt_ * pd3_vz_filt_);
    ROS_INFO_THROTTLE(0.5,
        "[PD打点3D] 目标(%.2f,%.2f,%.2f) 当前(%.2f,%.2f,%.2f) 距离=%.2fm 速度=%.2fm/s 指令=(%.2f,%.2f)",
        tx, ty, tz, cx, cy, cz, dist, speed_est, vcx, vcy);

    // 到达判据: 3D 距离 + z 差 + 速度门 稳定 0.3s, 带滞回
    bool in_pos  = dist < ARRIVE_DIST && speed_est < ARRIVE_VEL &&
                   std::fabs(cz - tz) < 0.1;
    bool out_pos = dist > ARRIVE_RESET_DIST || speed_est > 2.0 * ARRIVE_VEL ||
                   std::fabs(cz - tz) > 0.15;
    if (delay_started_) {
        if (out_pos) {
            delay_started_ = false;
        } else if ((now - delay_start_time_).toSec() >= 0.3) {
            delay_started_ = false;
            ROS_INFO("[PD打点3D] 已到达 (%.2f, %.2f, %.2f)", tx, ty, tz);
            return true;
        }
    } else if (in_pos) {
        delay_start_time_ = now;
        delay_started_ = true;
    }
    return false;
}
