// RACER 探索主控：EXPLORE → LANDING，解锁和 OFFBOARD 由独立守卫处理。
// 命名空间、机号和机数由参数指定。

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/ParamSet.h>
#include <mavros_msgs/ParamValue.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>

#include <array>
#include <clocale>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "fly_px4/control_px4.hpp"
#include "fly_px4/function_px4.hpp"

// RACER FSM 状态枚举(与 fast_exploration_fsm.h 的 EXPL_STATE 对齐)
static constexpr int8_t RACER_STATE_INIT         = 0;
static constexpr int8_t RACER_STATE_WAIT_TRIGGER = 1;
static constexpr int8_t RACER_STATE_FINISH       = 5;

// MAVROS 状态订阅封装
class MavrosSubscriber {
public:
    MavrosSubscriber(ros::NodeHandle& nh, const std::string& topic) {
        sub_ = nh.subscribe<mavros_msgs::State>(topic, 10, &MavrosSubscriber::cb, this);
    }
    mavros_msgs::State getCurrentState() const {
        std::lock_guard<std::mutex> lock(mtx_);
        return current_;
    }
private:
    void cb(const mavros_msgs::State::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(mtx_);
        current_ = *msg;
    }
    mutable std::mutex mtx_;
    mavros_msgs::State current_;
    ros::Subscriber sub_;
};

// 本地位置订阅封装
// 对应 fly_a9 的 SlamOdomSubscriber, 这里订阅 mavros 本地位置并提供航向换算
class PoseSubscriber {
public:
    PoseSubscriber(ros::NodeHandle& nh, const std::string& topic) {
        sub_ = nh.subscribe<geometry_msgs::PoseStamped>(topic, 10, &PoseSubscriber::cb, this);
    }
    geometry_msgs::PoseStamped getCurrentPose() const {
        std::lock_guard<std::mutex> lock(mtx_);
        return current_;
    }
    bool hasPose() const {
        std::lock_guard<std::mutex> lock(mtx_);
        return received_;
    }
    // 四元数 -> 航向(rad)
    double getCurrentYaw() const {
        std::lock_guard<std::mutex> lock(mtx_);
        const auto& q = current_.pose.orientation;
        double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
        double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        return std::atan2(siny_cosp, cosy_cosp);
    }
private:
    void cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(mtx_);
        current_ = *msg;
        received_ = true;
    }
    mutable std::mutex mtx_;
    geometry_msgs::PoseStamped current_;
    bool received_ = false;
    ros::Subscriber sub_;
};

// RACER PositionCommand 订阅封装
// 订阅 /planning/pos_cmd_{id}, 缓存最新指令(EXPLORE 状态转发用)
class PosCmdSubscriber {
public:
    PosCmdSubscriber(ros::NodeHandle& nh, const std::string& topic) {
        sub_ = nh.subscribe<quadrotor_msgs::PositionCommand>(
            topic, 50, &PosCmdSubscriber::cb, this);
    }
    bool hasCmd() const {
        std::lock_guard<std::mutex> lock(mtx_);
        return received_;
    }
    // 指令租约独立检查；新鲜 traj_server 消息不代表规划器仍正常工作。
    double secsSinceCmd() const {
        std::lock_guard<std::mutex> lock(mtx_);
        if (!received_) return 1e9;
        return (ros::Time::now() - last_rx_).toSec();
    }
    quadrotor_msgs::PositionCommand getCmd() const {
        std::lock_guard<std::mutex> lock(mtx_);
        return current_;
    }
private:
    void cb(const quadrotor_msgs::PositionCommand::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(mtx_);
        current_ = *msg;
        received_ = true;
        last_rx_ = ros::Time::now();
    }
    mutable std::mutex mtx_;
    quadrotor_msgs::PositionCommand current_;
    bool received_ = false;
    ros::Time last_rx_;
    ros::Subscriber sub_;
};

// RACER 探索状态订阅封装
// 订阅 /exploration/state_{id}(latched Int8), 检测 FINISH=5
class ExplStateSubscriber {
public:
    ExplStateSubscriber(ros::NodeHandle& nh, const std::string& topic) {
        sub_ = nh.subscribe<std_msgs::Int8>(topic, 10, &ExplStateSubscriber::cb, this);
    }
    bool isFinished() const {
        std::lock_guard<std::mutex> lock(mtx_);
        return finished_;
    }
    // RACER FSM 状态 0..6，-1 表示尚未收到；锁存状态用于检测重启。
    int state() const {
        std::lock_guard<std::mutex> lock(mtx_);
        return state_;
    }
private:
    void cb(const std_msgs::Int8::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(mtx_);
        state_ = static_cast<int>(msg->data);
        if (msg->data == RACER_STATE_FINISH) finished_ = true;
    }
    mutable std::mutex mtx_;
    bool finished_ = false;
    int state_ = -1;
    ros::Subscriber sub_;
};

// 跨工作空间通过通用 ROS 消息交换世界坐标目标和跟踪模式。
class TrackTargetSubscriber {
public:
    TrackTargetSubscriber(ros::NodeHandle& nh, const std::string& topic) {
        sub_ = nh.subscribe<nav_msgs::Odometry>(topic, 4,
            &TrackTargetSubscriber::cb, this, ros::TransportHints().tcpNoDelay());
    }
    // 目标世界坐标 + 速度; 没收到过返回 false
    bool get(double& x, double& y, double& vx, double& vy, double& age) const {
        std::lock_guard<std::mutex> lock(mtx_);
        if (!received_) return false;
        x = x_; y = y_; vx = vx_; vy = vy_;
        age = (ros::Time::now() - last_rx_).toSec();
        return true;
    }
private:
    void cb(const nav_msgs::Odometry::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(mtx_);
        x_ = msg->pose.pose.position.x;
        y_ = msg->pose.pose.position.y;
        vx_ = msg->twist.twist.linear.x;
        vy_ = msg->twist.twist.linear.y;
        received_ = true;
        last_rx_ = ros::Time::now();
    }
    mutable std::mutex mtx_;
    double x_ = 0.0, y_ = 0.0, vx_ = 0.0, vy_ = 0.0;
    bool received_ = false;
    ros::Time last_rx_;
    ros::Subscriber sub_;
};

class TrackModeSubscriber {
public:
    TrackModeSubscriber(ros::NodeHandle& nh, const std::string& topic) {
        sub_ = nh.subscribe<std_msgs::Int8>(topic, 4, &TrackModeSubscriber::cb, this);
    }
    // 0 OFF / 1 APPROACH / 2 LOCKED / 3 REACQUIRE / 4 SEARCH
    int mode() const {
        std::lock_guard<std::mutex> lock(mtx_);
        return mode_;
    }
private:
    void cb(const std_msgs::Int8::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(mtx_);
        mode_ = static_cast<int>(msg->data);
    }
    mutable std::mutex mtx_;
    int mode_ = 0;
    ros::Subscriber sub_;
};

// 任务节点
class Task {
public:
    Task(ros::NodeHandle& nh)
        : nh_(nh) {
        nh_.param<std::string>("drone_ns",       drone_ns_,       "typhoon_h480_0");
        nh_.param<int>        ("drone_id",       drone_id_,       1);
        nh_.param<int>        ("drone_num",      drone_num_,      6);
        nh_.param<double>     ("min_expl_z",     min_expl_z_,     4.6);
        nh_.param<double>     ("max_expl_z",     max_expl_z_,     5.4);
        // 限高失效保护: 裁判 score_cal.py 在 z>6.5 时直接判废整轮, 故留两道防线
        nh_.param<double>     ("z_soft_limit",   z_soft_limit_,   5.7);
        nh_.param<double>     ("z_hard_limit",   z_hard_limit_,   6.0);
        // spawn 偏移(世界帧下该机本地原点坐标): RACER pos_cmd(世界)->PX4(本地) 转换用
        nh_.param<double>     ("init_x",         init_x_,         0.0);
        nh_.param<double>     ("init_y",         init_y_,         0.0);
        // 卡死脱困 (三层, 全程不做任何盲飞平移, 见 handleExploreWithEscape 的说明)
        nh_.param<double>("cmd_timeout",       cmd_timeout_,       2.0);
        nh_.param<double>("stuck_yaw_sec",     stuck_yaw_sec_,     6.0);
        nh_.param<double>("stuck_retreat_sec", stuck_retreat_sec_, 12.0);
        nh_.param<double>("retreat_dist",      retreat_dist_,      8.0);
        nh_.param<double>("frozen_sec",        frozen_sec_,        25.0);
        nh_.param<double>("spin_rate_deg",     spin_rate_deg_,     30.0);
        nh_.param<double>("retreat_hold",      retreat_hold_,      8.0);
        // 坠机判定: 探索中高度掉到 crash_z 以下并持续 crash_hold 秒 -> 认定坠毁
        // (巡航 5.0m, 探索盒下界 4.6m, 所以 1.5m 只可能是掉下来了, 不会误判)
        nh_.param<double>("crash_z",           crash_z_,           1.5);
        nh_.param<double>("crash_hold",        crash_hold_,        5.0);
        // 目标跟踪 (yaw 覆写)
        // 20 deg/s: 云台 yaw 是跟随机体的 PID, 机体转太快相机姿态会滞后、检测被丢,
        // 机头会反复追目标来回抖。降到 20 接近 vision 侧 15deg/s 的丢帧门, 转动更平稳。
        nh_.param<double>("track_yaw_rate_max", track_yaw_rate_max_, 20.0);
        // 3 度死区: 目标估计有噪声, 不设死区会让机头一直小幅抖, 云台跟着晃
        nh_.param<double>("track_yaw_deadband_deg", track_yaw_deadband_, 3.0);
        nh_.param<double>("track_lead_sec",    track_lead_sec_,    0.2);
        nh_.param<double>("track_stale_sec",   track_stale_sec_,   0.5);
        // 搜索模式的机头扫掠: 把有效扫幅从 9.1m 提到约 12m (+33%)
        // search_sweep_enable=false 时机头固定对准搜索目标, 不做左右扫掠(消除"摆头")
        nh_.param<bool>  ("search_sweep_enable", search_sweep_enable_, true);
        nh_.param<double>("search_sweep_deg",  search_sweep_deg_,  30.0);
        nh_.param<double>("search_sweep_rate", search_sweep_rate_, 40.0);
        // 任务兜底: 到点无条件降落, 把降落分先落袋(600s 上限, 留 15s 余量)
        nh_.param<double>("mission_deadline_sec", mission_deadline_sec_, 585.0);
        // 融合节点压根没启动时的兜底: RACER 报 FINISH 且这么久没收到 mission_complete
        // 就按旧行为降落, 免得六架机悬停到仿真被杀
        nh_.param<double>("mission_wait_sec", mission_wait_sec_, 30.0);

        const std::string mavros_ns = "/" + drone_ns_ + "/mavros";
        const std::string id_str = std::to_string(drone_id_);

        // 订阅
        mavros_sub_   = std::make_unique<MavrosSubscriber>(nh_, mavros_ns + "/state");
        pose_sub_     = std::make_unique<PoseSubscriber>(nh_, mavros_ns + "/local_position/pose");
        pos_cmd_sub_  = std::make_unique<PosCmdSubscriber>(nh_, "/planning/pos_cmd_" + id_str);
        expl_state_sub_ = std::make_unique<ExplStateSubscriber>(nh_, "/exploration/state_" + id_str);

        // 目标跟踪
        track_target_sub_ = std::make_unique<TrackTargetSubscriber>(
            nh_, "/tracking/target_" + id_str);
        track_mode_sub_ = std::make_unique<TrackModeSubscriber>(
            nh_, "/tracking/mode_" + id_str);
        // 任务完成信号(latched): 6 个行人全部消除后才允许因 RACER FINISH 而降落。
        // latched 很关键 —— 本节点通常比 fusion 后启动, 一订阅就要能拿到当前值。
        mission_complete_sub_ = nh_.subscribe<std_msgs::Bool>(
            "/tracking/mission_complete", 2, &Task::missionCompleteCb, this);
        mission_start_ = ros::Time::now();

        // 发布器
        // setpoint (OFFBOARD 下飞控依此飞行)
        set_raw_pub_ = nh_.advertise<mavros_msgs::PositionTarget>(
            mavros_ns + "/setpoint_raw/local", 10);
        // 锁存触发消息，使重启的 RACER 能重新获得目标。
        trigger_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(
            "/move_base_simple/goal", 1, /*latch=*/true);

        // 服务
        arming_cli_  = nh_.serviceClient<mavros_msgs::CommandBool>(mavros_ns + "/cmd/arming");
        set_mode_cli_ = nh_.serviceClient<mavros_msgs::SetMode>(mavros_ns + "/set_mode");
        param_cli_   = nh_.serviceClient<mavros_msgs::ParamSet>(mavros_ns + "/param/set");

        // 低层控制 + 飞行函数层
        control_ = std::make_unique<ControlPx4>(set_raw_pub_);
        function_ = std::make_unique<FunctionPx4>(
            nh_, set_mode_cli_, arming_cli_, control_.get(),
            [this]() { return pose_sub_->getCurrentYaw(); });

        function_->resetForNewTask();   // 初始状态即 EXPLORE: 跳过独立起飞阶段
        // 注入 spawn 偏移: RACER pos_cmd(世界帧) 转发到 PX4(本地帧) 时减去
        function_->setSpawnOffset(init_x_, init_y_);

        // 用 0 作哨兵: runtask 里 "now - last >= 3.0" 首拍必然成立(立即请求).
        // 不用 now()-Duration(5): use_sim_time 且 /clock 未到时 now()=0, 0-5 得负时间会抛
        // "Time is out of dual 32-bit range" 崩溃 (main 已在启动时等 /clock, 此处仅作兜底).
        last_request_time_ = ros::Time(0);

        // 50Hz 状态机定时器
        timer_ = nh_.createTimer(ros::Duration(0.02), &Task::runtask, this);

        ROS_INFO("[fly_px4_2] drone_ns=%s drone_id=%d drone_num=%d expl_z=[%.2f,%.2f] "
                 "z_limit=[soft %.2f, hard %.2f] spawn=(%.1f,%.1f)",
                 drone_ns_.c_str(), drone_id_, drone_num_,
                 min_expl_z_, max_expl_z_, z_soft_limit_, z_hard_limit_, init_x_, init_y_);
    }

private:
    ros::NodeHandle& nh_;
    std::string drone_ns_;
    int drone_id_   = 1;
    int drone_num_  = 6;
    double min_expl_z_     = 4.6;   // = RACER box_min_z
    double max_expl_z_     = 5.4;   // = RACER box_max_z
    double z_soft_limit_   = 5.7;   // 超此高度 0.3s -> 强制下压到 max_expl_z_
    double z_hard_limit_   = 6.0;   // 超此高度 -> 立即 AUTO.LAND (裁判判废线 6.5)
    ros::Time z_over_since_;        // 首次越过软限的时刻 (0 表示未越限)
    double init_x_         = 0.0;   // spawn 偏移(世界帧): pos_cmd 世界->本地 转换用
    double init_y_         = 0.0;

    std::unique_ptr<MavrosSubscriber>     mavros_sub_;
    std::unique_ptr<PoseSubscriber>       pose_sub_;
    std::unique_ptr<PosCmdSubscriber>     pos_cmd_sub_;
    std::unique_ptr<ExplStateSubscriber>  expl_state_sub_;

    // 目标跟踪
    std::unique_ptr<TrackTargetSubscriber> track_target_sub_;
    std::unique_ptr<TrackModeSubscriber>   track_mode_sub_;
    ros::Subscriber mission_complete_sub_;
    bool mission_complete_ = false;        // 6 人全消除
    bool mission_msg_seen_ = false;        // 是否收到过 mission_complete (判 fusion 在不在)
    ros::Time mission_start_;              // 本节点启动时刻(deadline 起算点)
    double track_yaw_rate_max_ = 20.0;     // 机头转速上限 deg/s
    double track_yaw_deadband_ = 3.0;      // 死区 deg
    double track_lead_sec_ = 0.2;          // 目标前推时间 s
    double track_stale_sec_ = 0.5;         // 目标信息超过这么旧就不再覆写 yaw
    bool search_sweep_enable_ = false;      // 搜索时机头是否左右扫掠(false=固定对准搜索目标)
    double search_sweep_deg_ = 30.0;        // 搜索时机头左右扫掠幅度 deg
    double search_sweep_rate_ = 40.0;       // 扫掠角速度 deg/s
    double mission_deadline_sec_ = 585.0;  // 到点无条件降落
    double mission_wait_sec_ = 30.0;       // fusion 缺席的容忍时间
    double yaw_cmd_prev_deg_ = 0.0;        // 上一拍下发的机头角(做速率限幅用)
    bool yaw_cmd_inited_ = false;
    bool tracking_active_ = false;         // 当前是否在跟踪(供 frozen 判据加门)

    ros::Publisher  set_raw_pub_;
    ros::Publisher  trigger_pub_;
    ros::ServiceClient arming_cli_;
    ros::ServiceClient set_mode_cli_;
    ros::ServiceClient param_cli_;
    ros::Timer timer_;

    std::unique_ptr<ControlPx4>  control_;
    std::unique_ptr<FunctionPx4> function_;

    ros::Time last_request_time_;
    bool param_set_done_ = false;

    // RACER 触发
    bool trigger_sent_ = false;         // 是否已发 RACER 触发目标
    bool racer_started_ = false;        // 是否已收到首条 RACER 指令(收到即停止重发 trigger)
    ros::Time last_trigger_republish_;

    // 卡死脱困
    double cmd_timeout_       = 2.0;   // 超过这么久没收到新 pos_cmd 就当"没有指令"
    double stuck_yaw_sec_     = 6.0;   // 无指令持续这么久 -> 原地自转(不平移)
    double stuck_retreat_sec_ = 12.0;  // 无指令持续这么久 -> 沿自身历史航迹回退
    double retreat_dist_      = 8.0;   // 最多沿航迹回退多少米
    double frozen_sec_        = 25.0;  // 有指令但位置几乎不动持续这么久 -> 也做原地自转
    double spin_rate_deg_     = 30.0;  // 自转角速度(度/秒)
    double retreat_hold_      = 8.0;   // 自转完仍不动时, 沿航迹回退动作保持多少秒
    double crash_z_           = 1.5;   // 探索中低于此高度并持续 crash_hold 秒 -> 判定坠毁
    double crash_hold_        = 5.0;
    ros::Time crash_since_;            // 首次低于 crash_z 的时刻 (0 = 未低于)
    bool has_flown_ = false;           // 本机是否曾飞起来过(EXPLORE 中高度曾 ≥ crash_z).
                                       // 必须先飞起过才判坠机, 否则起飞前地面待命会被误判

    // 本地历史航迹每 0.5m 记录一点，用于沿原路回退。
    std::vector<std::array<double, 3>> trail_;
    static constexpr size_t kTrailMax = 120;   // 120 x 0.5m = 60m 记忆

    ros::Time cmd_lost_since_;   // 首次判定"无新指令"的时刻 (0 = 当前有指令)
    bool escape_target_valid_ = false;
    bool retreat_picked_ = false;   // 本轮回退目标是否已选定(必须锁住, 否则会一路退回起飞点)
    ros::Time retreat_picked_t_;
    double escape_x_ = 0.0, escape_y_ = 0.0;   // 回退目标(本地帧)
    double escape_yaw0_deg_ = 0.0;             // 自转起始航向

    // 有指令但位置不动的卡死判定
    std::array<double, 3> frozen_ref_{ { 0.0, 0.0, 0.0 } };
    ros::Time frozen_ref_t_;
    bool frozen_ref_valid_ = false;
    double frozen_spin_yaw0_deg_ = 0.0;      // 这一轮自转的起始航向
    bool frozen_spin_yaw0_valid_ = false;
    bool frozen_retreat_picked_ = false;     // "指令正常但冻住"这一路的回退目标是否已选定
    bool escape_active_ = false;             // 当前是否在执行兜底动作(用于暂停航迹记录)

    void missionCompleteCb(const std_msgs::Bool::ConstPtr& msg) {
        mission_msg_seen_ = true;
        if (msg->data && !mission_complete_) {
            ROS_WARN("[任务] 收到 mission_complete: 6 个目标已全部消除, 允许降落");
        }
        mission_complete_ = msg->data;
    }

    // 跟踪仅覆盖偏航，位置、速度、加速度继续使用 RACER 指令。
    bool computeTrackYaw(double cx, double cy, double& yaw_deg) {
        const int mode = track_mode_sub_->mode();
        if (mode < 1) return false;

        // !! 本地帧 -> 世界帧 !!  pose_sub_ 是 PX4 的本地帧(原点=spawn),
        // 而目标是世界帧。这里最容易错: 忘了加 init_ 就会算出一个偏了十几米的方位。
        const double wx = cx + init_x_;
        const double wy = cy + init_y_;

        double tx, ty, tvx, tvy, age;
        const bool have = track_target_sub_->get(tx, ty, tvx, tvy, age);
        if (!have || age > track_stale_sec_) return false;

        double want;
        if (mode == 4) {
            // 搜索模式: 机头对准"当前位置 -> 搜索目标点"方向(比用速度方向稳, 拐弯时不甩)。
            // search_sweep_enable=true 时在此基础上左右扫掠(±search_sweep_deg, 三角波),
            // 把有效扫幅从 9.1m 提到约 12m; 默认关闭, 消除"摆头"。
            const double base = std::atan2(ty - wy, tx - wx) * 180.0 / M_PI;
            double tri = 0.0;
            if (search_sweep_enable_) {
                const double phase = std::fmod(ros::Time::now().toSec() * search_sweep_rate_,
                                               4.0 * search_sweep_deg_);
                // 三角波: 0 -> +A -> 0 -> -A -> 0
                tri = phase <= 2.0 * search_sweep_deg_
                          ? (phase - search_sweep_deg_)
                          : (3.0 * search_sweep_deg_ - phase);
            }
            want = base + tri;
        } else {
            // 跟踪模式: 机头压住目标, 目标位置按速度前推(补规划+执行延迟)
            const double px = tx + tvx * track_lead_sec_;
            const double py = ty + tvy * track_lead_sec_;
            want = std::atan2(py - wy, px - wx) * 180.0 / M_PI;
        }

        if (!yaw_cmd_inited_) {
            yaw_cmd_prev_deg_ = pose_sub_->getCurrentYaw() * 180.0 / M_PI;
            yaw_cmd_inited_ = true;
        }
        double d = wrapDeg(want - yaw_cmd_prev_deg_);
        if (std::fabs(d) < track_yaw_deadband_) {
            d = 0.0;   // 死区: 目标估计有噪声, 不设死区机头会一直小幅抖
        }
        // 速率限幅: 50Hz 下每拍最多转 rate/50 度
        const double step = track_yaw_rate_max_ * 0.02;
        if (d > step) d = step;
        if (d < -step) d = -step;
        yaw_cmd_prev_deg_ = wrapDeg(yaw_cmd_prev_deg_ + d);
        yaw_deg = yaw_cmd_prev_deg_;
        return true;
    }

    // 状态机主循环
    void runtask(const ros::TimerEvent&) {
        auto state = mavros_sub_->getCurrentState();
        auto pose  = pose_sub_->getCurrentPose();
        double cx = pose.pose.position.x;
        double cy = pose.pose.position.y;
        double cz = pose.pose.position.z;

        // 一次性: 设 COM_RCL_EXCEPT=4 (sim 下 OFFBOARD 不需遥控器, 防遥控器保护抢占)
        if (!param_set_done_) {
            mavros_msgs::ParamSet srv;
            mavros_msgs::ParamValue v; v.integer = 4; v.real = 0.0;
            srv.request.param_id = "COM_RCL_EXCEPT";
            srv.request.value = v;
            if (param_cli_.call(srv))
                ROS_INFO("COM_RCL_EXCEPT 已设为 4, OFFBOARD 下遥控器保护已禁用");
            else
                ROS_WARN("设置 COM_RCL_EXCEPT 失败");
            param_set_done_ = true;
        }

        const auto fs = function_->getFlightState();
        // LANDING/DONE 自行管理模式(切 AUTO.LAND 后不再依赖 OFFBOARD, 也不重解锁)
        const bool landing_manages_mode =
            (fs == FlightState::LANDING || fs == FlightState::DONE);

        // 仅在曾飞起后，低高度持续超过门限才判坠机。
        if (fs == FlightState::EXPLORE) {
            if (cz >= crash_z_) {
                has_flown_ = true;
                crash_since_ = ros::Time(0);
            } else if (has_flown_) {
                if (crash_since_.isZero()) crash_since_ = ros::Time::now();
                if (ros::Time::now() - crash_since_ >= ros::Duration(crash_hold_)) {
                    ROS_ERROR("[坠机] z=%.2f 已持续低于 %.2f 米 %.0f 秒, 判定本机已坠毁/失控, "
                              "停止一切指令下发并转 DONE (不再空占一个机位)",
                              cz, crash_z_, crash_hold_);
                    function_->setFlightState(FlightState::DONE);
                    return;
                }
            } else {
                crash_since_ = ros::Time(0);   // 从未飞起(起飞前地面阶段): 不计时
            }
        }

        // 注: 设成 DONE 之后不需要在这里拦截 —— landing_manages_mode 已经包含 DONE,
        // 所以 OFFBOARD/解锁/限高三个守卫都会跳过, 直接走到 switch 的 DONE 分支,
        // 那里会 timer_.stop() + ros::shutdown() 干净退出, 不再下发任何设定点。

        // OFFBOARD 守卫: 非 LANDING 时必须 OFFBOARD, 否则喂心跳 + 请求切模式
        if (state.mode != "OFFBOARD" && !landing_manages_mode) {
            function_->hold_for_offboard();
            if (ros::Time::now() - last_request_time_ >= ros::Duration(3.0)) {
                mavros_msgs::SetMode cmd; cmd.request.custom_mode = "OFFBOARD";
                set_mode_cli_.call(cmd);
                ROS_INFO("尝试切换到 OFFBOARD... (当前 %s)", state.mode.c_str());
                last_request_time_ = ros::Time::now();
            }
            return;
        }

        // 解锁守卫: 未解锁则请求解锁 (3s 重试), 期间持续喂心跳
        if (!state.armed && !landing_manages_mode) {
            function_->hold_for_offboard();
            if (ros::Time::now() - last_request_time_ >= ros::Duration(3.0)) {
                mavros_msgs::CommandBool cmd; cmd.request.value = true;
                arming_cli_.call(cmd);
                ROS_INFO("尝试解锁...");
                function_->setArmed(true);
                last_request_time_ = ros::Time::now();
            }
            return;
        }

        // 按本地估计高度执行软限下压和硬限降落。
        if (!landing_manages_mode) {
            if (cz > z_hard_limit_) {
                ROS_ERROR("[限高] z=%.2f 超硬限 %.2f, 立即降落保命 (裁判线 6.5)", cz, z_hard_limit_);
                function_->setFlightState(FlightState::LANDING);
                function_->handle_landing_pub();
                return;
            }
            if (cz > z_soft_limit_) {
                if (z_over_since_.isZero()) z_over_since_ = ros::Time::now();
                // 软硬高度限之间余量较小，软限使用较短持续时间。
                if (ros::Time::now() - z_over_since_ >= ros::Duration(0.3)) {
                    ROS_WARN_THROTTLE(1.0, "[限高] z=%.2f 超软限 %.2f 已 0.5s, 强制下压到 %.2f",
                                      cz, z_soft_limit_, max_expl_z_);
                    // 越过 RACER 直接下压: 锁住当前水平位置, 只改高度
                    control_->fly_posvel_yaw(cx, cy, max_expl_z_, 0.0, 0.0, 0.0,
                                             pose_sub_->getCurrentYaw() * 180.0 / M_PI);
                    return;  // 本拍不执行常规状态机, 避免 RACER 指令又把它拉上去
                }
            } else {
                z_over_since_ = ros::Time(0);
            }
        }

        // 记录自身航迹(脱困时沿它回退, 见 computeFallback)
        // 只在探索阶段记: 起飞/降落是垂直运动, 记进去没意义。
        if (fs == FlightState::EXPLORE) recordTrail(cx, cy, cz);

        // 状态机: 各阶段调对应 handle_xxx
        switch (fs) {
        case FlightState::EXPLORE: {
            // 触发 RACER: 解锁+OFFBOARD 后立即发 /move_base_simple/goal (2s 重发直到出首条指令)
            checkAndFireTrigger();

            // 只认"新鲜"的指令: RACER 崩了/卡住时不能抱着旧指令一直飞 (见 secsSinceCmd 注释)
            const double cmd_age = pos_cmd_sub_->secsSinceCmd();
            bool has_cmd = pos_cmd_sub_->hasCmd() && cmd_age <= cmd_timeout_;
            auto cmd = pos_cmd_sub_->getCmd();  // 无指令时为零值, has_cmd=false 不用
            if (has_cmd) racer_started_ = true;

            // RACER 回到 INIT/WAIT_TRIGGER 时重发触发，即使 traj_server 仍有新消息。
            const int racer_state = expl_state_sub_->state();
            if (racer_started_ &&
                (racer_state == RACER_STATE_INIT || racer_state == RACER_STATE_WAIT_TRIGGER)) {
                ROS_ERROR_THROTTLE(5.0,
                    "[脱困] RACER 状态回到 %d (INIT/WAIT_TRIGGER), 判定它重启过, 重发 trigger",
                    racer_state);
                racer_started_ = false;   // -> checkAndFireTrigger 恢复每 2s 重发
            }
            // 任务完成、时限或融合缺失回退可触发降落；不单独依据 RACER FINISH。
            const double mission_elapsed = (ros::Time::now() - mission_start_).toSec();
            const bool racer_finished = expl_state_sub_->isFinished();
            const bool deadline_hit = mission_elapsed >= mission_deadline_sec_;
            const bool fusion_absent =
                !mission_msg_seen_ && mission_elapsed >= mission_wait_sec_;
            bool done = false;
            if (deadline_hit) {
                ROS_WARN_THROTTLE(5.0, "[任务] 已用 %.0fs 到达兜底时限, 准备降落",
                                  mission_elapsed);
                done = true;
            } else if (racer_finished && (mission_complete_ || fusion_absent)) {
                if (fusion_absent) {
                    ROS_WARN_THROTTLE(5.0,
                        "[任务] %.0fs 内未收到 mission_complete (融合节点未启动?), "
                        "按旧行为: RACER FINISH 即降落", mission_wait_sec_);
                }
                done = true;
            } else if (racer_finished) {
                ROS_WARN_THROTTLE(10.0,
                    "[任务] RACER 报探索完成, 但目标未全部消除, 继续待命/巡逻 (不降落)");
            }

            // 跟踪时覆写机头朝向(位置/速度/加速度仍照抄 RACER)
            double track_yaw_deg = 0.0;
            const bool yaw_override = computeTrackYaw(cx, cy, track_yaw_deg);
            tracking_active_ = (track_mode_sub_->mode() >= 1);

            // 无指令(或有指令但飞机冻住)时的兜底动作: 悬停 / 原地自转 / 沿航迹回退
            const ExplFallback fb = computeFallback(cx, cy, cz, has_cmd, cmd_age);
            escape_active_ = fb.valid;   // 脱困期间暂停航迹记录, 见 recordTrail
            if (function_->handle_explore(cx, cy, cz, has_cmd, cmd, done,
                                          min_expl_z_, max_expl_z_, fb,
                                          yaw_override, track_yaw_deg)) {
                ROS_WARN("[状态切换] EXPLORE -> LANDING (任务结束)");
                function_->setFlightState(FlightState::LANDING);
            }
            break;
        }

        case FlightState::TRACKING:
            // 跟踪在 EXPLORE 内处理并保留安全守卫，保留状态入口用于兼容。
            ROS_WARN_ONCE("[状态切换] TRACKING 未使用(跟踪在 EXPLORE 内以 yaw 覆写实现), 回 EXPLORE");
            function_->setFlightState(FlightState::EXPLORE);
            break;

        case FlightState::LANDING:
            function_->handle_landing_pub();
            // AUTO.LAND 着地后 PX4 自动上锁 -> 完成
            if (!state.armed) {
                ROS_WARN("[状态切换] LANDING -> DONE (已上锁, 着地完成)");
                function_->setFlightState(FlightState::DONE);
            }
            break;

        case FlightState::DONE:
            ROS_WARN("任务执行完毕, 程序退出");
            timer_.stop();
            ros::shutdown();
            break;
        }
    }

    // 把角度规整到 (-180, 180]: 自转的目标航向是累加出来的, 不规整会一路涨到几千度
    static double wrapDeg(double d) {
        while (d > 180.0) d -= 360.0;
        while (d <= -180.0) d += 360.0;
        return d;
    }

    // 航迹记录: 每 0.5m 存一个本地帧位置, 供"沿航迹回退"脱困用
    void recordTrail(double x, double y, double z) {
        // 脱困期间绝对不能记: 回退时飞机是沿着航迹倒着走的, 记进去的话航迹尾部就变成了
        // "刚才退过来的那段", 下一次 pickRetreatPoint 会顺着它又走回去, 来回拉锯。
        if (escape_active_) return;
        if (!trail_.empty()) {
            const auto& b = trail_.back();
            const double d = std::hypot(x - b[0], y - b[1]);
            if (d < 0.5) return;
            if (d > 5.0) trail_.clear();   // EKF 跳变: 之前的点不再可信, 整条丢掉
        }
        trail_.push_back({ { x, y, z } });
        if (trail_.size() > kTrailMax) trail_.erase(trail_.begin());
    }

    // 指令过期后依次悬停、转向和历史回退；新鲜但冻结的指令另行检测。
    ExplFallback computeFallback(double cx, double cy, double cz, bool has_cmd, double cmd_age) {
        ExplFallback fb;
        const auto now = ros::Time::now();

        if (has_cmd) {
            // 指令新鲜: 清掉丢指令计时, 只检查"有指令却不动"
            cmd_lost_since_ = ros::Time(0);
            escape_target_valid_ = false;
            retreat_picked_ = false;

            // 跟踪允许合法悬停，跳过探索冻结判据。
            if (tracking_active_) {
                frozen_ref_valid_ = false;
                return fb;   // valid=false -> 正常转发 RACER 指令
            }

            if (!frozen_ref_valid_ ||
                std::hypot(cx - frozen_ref_[0], cy - frozen_ref_[1]) > 1.0) {
                frozen_ref_ = { { cx, cy, cz } };
                frozen_ref_t_ = now;
                frozen_ref_valid_ = true;
                return fb;   // valid=false -> 正常转发
            }
            const double still = (now - frozen_ref_t_).toSec();
            if (still < frozen_sec_) return fb;

            // 新鲜指令下冻结时转向或历史回退，恢复后交还规划器。
            const double spin_dur = 360.0 / std::max(spin_rate_deg_, 1.0);
            const double spun = still - frozen_sec_;

            if (spun >= spin_dur + retreat_hold_) {
                frozen_ref_t_ = now;          // 重新计时, 这一拍起交回 RACER
                frozen_ref_ = { { cx, cy, cz } };
                frozen_spin_yaw0_valid_ = false;
                frozen_retreat_picked_ = false;
                return fb;                    // valid=false
            }

            fb.valid = true;
            if (spun < spin_dur) {
                if (!frozen_spin_yaw0_valid_) {
                    frozen_spin_yaw0_deg_ = pose_sub_->getCurrentYaw() * 180.0 / M_PI;
                    frozen_spin_yaw0_valid_ = true;
                    ROS_ERROR("[脱困] 指令正常但 %.0fs 内移动不足 1m, 先原地自转一圈刷新感知",
                              frozen_sec_);
                }
                fb.mode = ExplFallback::Mode::YAW_SPIN;
                fb.x = cx;
                fb.y = cy;
                fb.use_yaw = true;
                fb.yaw_deg = wrapDeg(frozen_spin_yaw0_deg_ + spin_rate_deg_ * spun);
                return fb;
            }

            // 自转完还是不动 -> 沿航迹回退。目标只选一次然后锁住
            if (!frozen_retreat_picked_) {
                if (!pickRetreatPoint(cx, cy)) {
                    fb.mode = ExplFallback::Mode::HOVER;   // 没航迹可退, 绝不盲飞
                    fb.x = cx;
                    fb.y = cy;
                    return fb;
                }
                frozen_retreat_picked_ = true;
            }
            fb.mode = ExplFallback::Mode::RETREAT;
            fb.x = escape_x_;
            fb.y = escape_y_;
            return fb;
        }

        // 以下: 没有新鲜指令
        frozen_ref_valid_ = false;
        // 还没收到过任何指令(刚起飞, RACER 还没被触发): 保持原行为, 起飞点悬停
        if (!pos_cmd_sub_->hasCmd()) return fb;

        if (cmd_lost_since_.isZero()) {
            cmd_lost_since_ = now;
            escape_target_valid_ = false;
            escape_yaw0_deg_ = pose_sub_->getCurrentYaw() * 180.0 / M_PI;
            // RACER 可能已经崩了; 清掉 racer_started_ 让 checkAndFireTrigger 恢复重发 trigger,
            // 配合 exploration_node 的 respawn 实现自愈
            racer_started_ = false;
            ROS_ERROR("[脱困] 已 %.1fs 没收到新的 RACER 指令, 转入兜底(先原地悬停)", cmd_age);
        }
        const double lost = (now - cmd_lost_since_).toSec();

        // 锁住刚丢指令那一刻的位置, 作为第 1/2 层的定点(免得被漂移一点点带走)
        if (!escape_target_valid_) {
            escape_x_ = cx;
            escape_y_ = cy;
            escape_target_valid_ = true;
        }

        fb.valid = true;
        fb.x = escape_x_;
        fb.y = escape_y_;

        if (lost < stuck_yaw_sec_) {
            fb.mode = ExplFallback::Mode::HOVER;   // 第 1 层: 定点悬停(不是回起飞点)
            return fb;
        }
        if (lost < stuck_retreat_sec_) {
            fb.mode = ExplFallback::Mode::YAW_SPIN;  // 第 2 层: 位置不动, 只转 yaw
            fb.use_yaw = true;
            fb.yaw_deg = wrapDeg(escape_yaw0_deg_ + spin_rate_deg_ * (lost - stuck_yaw_sec_));
            return fb;
        }

        // 每段回退缓存一个目标，避免控制循环反复选择。
        if (!retreat_picked_ || (now - retreat_picked_t_).toSec() > stuck_retreat_sec_) {
            if (pickRetreatPoint(cx, cy)) {
                retreat_picked_ = true;
                retreat_picked_t_ = now;
            } else {
                // 没有可用航迹(刚起飞就丢指令): 退化成原地悬停, 绝不盲飞
                fb.mode = ExplFallback::Mode::HOVER;
                return fb;
            }
        }
        fb.mode = ExplFallback::Mode::RETREAT;
        fb.x = escape_x_;
        fb.y = escape_y_;
        return fb;
    }

    // 从航迹尾部往回累加, 找到距当前位置约 retreat_dist_ 米的那个点。
    // 找到后写进 escape_x_/escape_y_, 并把该点之后的航迹截掉, 这样下一次回退会接着往前退。
    bool pickRetreatPoint(double cx, double cy) {
        if (trail_.size() < 2) return false;
        double acc = 0.0;
        double px = cx, py = cy;
        for (int i = static_cast<int>(trail_.size()) - 1; i >= 0; --i) {
            acc += std::hypot(trail_[i][0] - px, trail_[i][1] - py);
            px = trail_[i][0];
            py = trail_[i][1];
            if (acc >= retreat_dist_ || i == 0) {
                escape_x_ = trail_[i][0];
                escape_y_ = trail_[i][1];
                escape_target_valid_ = true;
                trail_.resize(static_cast<size_t>(i) + 1);
                ROS_ERROR("[脱困] 沿航迹回退 %.1fm 到 (%.2f, %.2f)", acc, escape_x_, escape_y_);
                return true;
            }
        }
        return false;
    }

    // RACER 触发: 解锁+OFFBOARD 后立即发 /move_base_simple/goal
    // 已去掉 6 机就绪屏障: 先起飞的机先触发本机 RACER, 不再等最慢的机 arm。
    // 2s 周期重发直到 RACER 出首条指令(或探索完成), 幂等无害。
    void checkAndFireTrigger() {
        if (racer_started_) return;
        // 飞机必须先爬到探索高度(min_expl_z)再触发 RACER:
        // 否则 RACER 的 A* 起点 z 还在探索盒外(地面爬升中), isInBox 检查失败,
        // 严格+乐观都无解 -> 飞机停飞。等爬到 min_expl_z 再发 trigger。
        if (pose_sub_->getCurrentPose().pose.position.z < min_expl_z_) return;
        if (trigger_sent_ &&
            ros::Time::now() - last_trigger_republish_ < ros::Duration(2.0)) {
            return;
        }
        geometry_msgs::PoseStamped goal;
        goal.header.stamp = ros::Time::now();
        goal.header.frame_id = "map";
        goal.pose = pose_sub_->getCurrentPose().pose;
        trigger_pub_.publish(goal);
        last_trigger_republish_ = ros::Time::now();
        if (!trigger_sent_) {
            trigger_sent_ = true;
            ROS_WARN("[触发] 本机 (drone_id=%d) 已就绪, 立即发 /move_base_simple/goal 触发 RACER 探索",
                     drone_id_);
        }
    }
};

// 主函数
int main(int argc, char** argv) {
    ros::init(argc, argv, "fly_px4_node");
    setlocale(LC_ALL, "zh_CN.UTF-8");
    ros::NodeHandle nh("~");

    // 等待 /clock 有效后再做 ROS 时间运算。
    bool use_sim = false;
    ros::param::param<bool>("/use_sim_time", use_sim, false);
    if (use_sim) {
        ROS_INFO("[启动] use_sim_time=true, 等待 Gazebo 仿真时钟 /clock ...");
        ros::WallTime wt0 = ros::WallTime::now();
        while (ros::ok() && !(ros::Time::now() > ros::Time(0))) {
            if ((ros::WallTime::now() - wt0).toSec() > 60.0) {
                ROS_ERROR("[启动] 等待 /clock 超时 60s. 请先启动终端1:");
                ROS_ERROR("         roslaunch px4 multi_vehicle_6.launch");
                return 1;
            }
            ros::spinOnce();           // 处理 /clock 回调
            ros::WallDuration(0.2).sleep();
        }
        if (ros::ok())
            ROS_INFO("[启动] 仿真时钟就绪 (now=%.3f), 继续启动状态机", ros::Time::now().toSec());
    }

    Task task(nh);

    ROS_INFO("fly_px4_2 启动 (多机探索状态机版: 探索(RACER) -> 降落, 跳过独立起飞)");
    ROS_INFO("· 状态流: EXPLORE -> LANDING -> DONE (TAKEOFF/TRACKING 保留定义, 当前不进入)");

    ros::spin();
    return 0;
}
