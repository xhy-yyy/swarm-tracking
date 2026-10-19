#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/ParamSet.h>
#include <mavros_msgs/ParamGet.h>
#include <mavros_msgs/ParamValue.h>
#include <mavros_msgs/EstimatorStatus.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Header.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32MultiArray.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <track_msgs/TrackingTarget.h>
#include <track_msgs/TrackingView.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <clocale>
#include <cmath>
#include <deque>
#include <memory>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include "fly_px4/control_px4.hpp"
#include "fly_px4/altitude_hold.hpp"
#include "fly_px4/track_obstacle_avoidance.hpp"
#include "fly_px4/track_yaw_controller.hpp"
#include "fly_px4/tracking_navigation_policy.hpp"

// 固定可飞边界；局部避障使用 EGO 感知地图。
static const fly_px4::TrackingMapBounds kTrackingMapBounds;

// base0 的历史兜底航线。其他地图缺失/损坏时禁止套用这些坐标。
// 索引 = drone_id - 1；正常启动始终从 YAML 读取。
struct DefaultRoute {
    double z;
    double start_delay;
    double wp[4][2];
};
static const DefaultRoute kDefaultRoutes[6] = {
    // 1 SC(南中)
    { 4.5, 0.0, { { -13.5,  -1.5 }, {  43.5,  -1.5 }, {  43.5, -43.5 }, { -13.5, -43.5 } } },
    // 2 NC(北中)
    { 4.5, 0.0, { { -13.5,   1.5 }, {  43.5,   1.5 }, {  43.5,  43.5 }, { -13.5,  43.5 } } },
    // 3 NE(北东)
    { 4.5, 0.0, { {  46.5,   1.5 }, { 118.5,   1.5 }, { 118.5,  43.5 }, {  46.5,  43.5 } } },
    // 4 SW(南西)，逆时针，与 base0 YAML 的闭环一致
    { 4.5, 0.0, { { -16.5,  -1.5 }, { -43.5,  -1.5 }, { -43.5, -43.5 }, { -16.5, -43.5 } } },
    // 5 SE(南东)
    { 4.5, 0.0, { {  46.5,  -1.5 }, { 118.5,  -1.5 }, { 118.5, -43.5 }, {  46.5, -43.5 } } },
    // 6 NW(北西)
    { 4.5, 0.0, { { -43.5,   1.5 }, { -16.5,   1.5 }, { -16.5,  43.5 }, { -43.5,  43.5 } } },
};

// 状态机
enum class PatrolState {
    CLIMB   = 0,   // 先垂直离地，再按配置边爬边进场；start_delay 控制 EGO 接管
    TRANSIT = 1,   // 进场: 飞到航线第一个航点
    PATROL  = 2,   // 沿闭合航线循环
    TRACK   = 3,   // 跟踪行人 (站位点交给 ego, 机头锁目标)
    LANDING = 4,
    DONE    = 5,
};
static const char* stateName(PatrolState s) {
    switch (s) {
    case PatrolState::CLIMB:   return "CLIMB";
    case PatrolState::TRANSIT: return "TRANSIT";
    case PatrolState::PATROL:  return "PATROL";
    case PatrolState::TRACK:   return "TRACK";
    case PatrolState::LANDING: return "LANDING";
    default:                   return "DONE";
    }
}

// 机头模式
enum class YawMode { COURSE = 0, COURSE_OFFSET = 1, FIXED = 2 };

struct Waypoint {
    double x = 0.0, y = 0.0;
    double z = -1.0;               // <0 = 用航线默认高度
    double speed = -1.0;           // <0 = 用 cruise_speed
    YawMode yaw_mode = YawMode::COURSE;
    double yaw_offset_deg = 0.0;
    double yaw_deg = 0.0;
};

// 脱困动作(与 fly_px4_2 的 ExplFallback 同构, 这里自带一份免得动 function_px4.*)
struct Fallback {
    enum class Mode { HOVER, YAW_SPIN, RETREAT };
    bool valid = false;
    Mode mode = Mode::HOVER;
    double x = 0.0, y = 0.0;
    bool use_yaw = false;
    double yaw_deg = 0.0;
};

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

// 起飞前设置并回读 GPS 高度参数；估计器就绪不等于已确认 GPS 融合。
// 飞行中不重设高度参考。
class Px4GpsHeightSetup {
public:
    Px4GpsHeightSetup(ros::NodeHandle& nh, const std::string& mavros_ns,
                      const MavrosSubscriber& state)
        : state_(state), spinner_(1, &queue_) {
        nh.param<bool>("use_gps_height", enabled_, true);
        nh.param<double>("gps_height_settle_sec", settle_sec_, 3.0);
        if (!std::isfinite(settle_sec_) || settle_sec_ < 0.0) settle_sec_ = 3.0;
        if (!enabled_) {
            ready_.store(true);
            return;
        }

        // 参数服务可能等待 MAVLink 超时，独立队列保证 50Hz 心跳继续发布。
        ros::NodeHandle worker_nh(nh);
        worker_nh.setCallbackQueue(&queue_);
        get_cli_ = worker_nh.serviceClient<mavros_msgs::ParamGet>(mavros_ns + "/param/get");
        set_cli_ = worker_nh.serviceClient<mavros_msgs::ParamSet>(mavros_ns + "/param/set");
        gps_sub_ = worker_nh.subscribe(mavros_ns + "/global_position/raw/fix", 1,
                                       &Px4GpsHeightSetup::gpsCb, this);
        estimator_sub_ = worker_nh.subscribe(mavros_ns + "/estimator_status", 1,
                                             &Px4GpsHeightSetup::estimatorCb, this);
        timer_ = worker_nh.createWallTimer(ros::WallDuration(0.25),
                                           &Px4GpsHeightSetup::configure, this);
        spinner_.start();
    }

    ~Px4GpsHeightSetup() {
        timer_.stop();
        spinner_.stop();
    }

    bool ready() const { return ready_.load(); }

private:
    void gpsCb(const sensor_msgs::NavSatFix::ConstPtr& msg) {
        gps_valid_ = msg->status.status >= sensor_msgs::NavSatStatus::STATUS_FIX &&
                     std::isfinite(msg->altitude);
        gps_rx_ = ros::Time::now();
    }

    void estimatorCb(const mavros_msgs::EstimatorStatus::ConstPtr& msg) {
        estimator_valid_ = msg->pos_vert_abs_status_flag &&
                           !msg->gps_glitch_status_flag && !msg->accel_error_status_flag;
        estimator_rx_ = ros::Time::now();
    }

    bool readGpsMode() {
        mavros_msgs::ParamGet srv;
        srv.request.param_id = "EKF2_HGT_MODE";
        return get_cli_.call(srv) && srv.response.success &&
               srv.response.value.integer == 1 && srv.response.value.real == 0.0;
    }

    void configure(const ros::WallTimerEvent&) {
        const auto state = state_.getCurrentState();
        if (!state.connected) {
            ready_.store(false);
            verified_ = false;
            required_settle_sec_ = 0.0;
            healthy_since_ = ros::Time(0);
            return;
        }
        if (state.armed) {
            if (!ready())
                ROS_WARN_THROTTLE(5.0, "[高度源] 已解锁，保持当前高度源；上锁后再配置 GPS");
            return;
        }

        if (!verified_) {
            const ros::WallTime now = ros::WallTime::now();
            if (!last_try_.isZero() && (now - last_try_).toSec() < 2.0) return;
            last_try_ = now;
            if (!get_cli_.exists() || !set_cli_.exists()) return;
            if (!readGpsMode()) {
                // 等待服务期间可能发生外部解锁；写参数前再检查一次。
                const auto current = state_.getCurrentState();
                if (!current.connected || current.armed) return;
                mavros_msgs::ParamSet srv;
                srv.request.param_id = "EKF2_HGT_MODE";
                srv.request.value.integer = 1;
                srv.request.value.real = 0.0;
                // 写入后读回超时，下一次读到 1 也仍属于本节点请求过切换。
                required_settle_sec_ = settle_sec_;
                if (!set_cli_.call(srv) || !srv.response.success || !readGpsMode()) {
                    ROS_WARN("[高度源] EKF2_HGT_MODE=1 写入/读回失败，保持待命，2s 后重试");
                    return;
                }
            }
            verified_ = true;
            verified_at_ = ros::Time::now();
            healthy_since_ = ros::Time(0);
            // 已经是 GPS 时没有发生高度参考切换；只等参数读回后的新鲜有效数据。
            // 真正写入/切换高度源时，仍保留配置的连续有效等待。
            ROS_INFO("[高度源] 已读回 EKF2_HGT_MODE=1，%s，等待新的有效 GPS/高度估计 %.1fs",
                     required_settle_sec_ > 0.0 ? "本次设置高度源" : "无需稳定等待",
                     required_settle_sec_);
        }

        const ros::Time now = ros::Time::now();
        const double gps_age = (now - gps_rx_).toSec();
        const double estimator_age = (now - estimator_rx_).toSec();
        const bool healthy = gps_valid_ && estimator_valid_ &&
                             gps_rx_ >= verified_at_ && estimator_rx_ >= verified_at_ &&
                             gps_age >= 0.0 && gps_age < 1.5 &&
                             estimator_age >= 0.0 && estimator_age < 3.0;
        if (!healthy || (!healthy_since_.isZero() && now < healthy_since_)) {
            ready_.store(false);
            healthy_since_ = ros::Time(0);
            return;
        }
        if (healthy_since_.isZero()) healthy_since_ = now;
        if ((now - healthy_since_).toSec() >= required_settle_sec_ && !ready()) {
            ROS_INFO("[高度源] GPS 参数读回及估计就绪检查通过，允许起飞");
            ready_.store(true);
        }
    }

    const MavrosSubscriber& state_;
    bool enabled_ = true;
    double settle_sec_ = 3.0;
    double required_settle_sec_ = 0.0;
    std::atomic<bool> ready_{false};
    bool verified_ = false;
    bool gps_valid_ = false, estimator_valid_ = false;
    ros::Time verified_at_, healthy_since_, gps_rx_, estimator_rx_;
    ros::WallTime last_try_;
    ros::CallbackQueue queue_;
    ros::ServiceClient get_cli_, set_cli_;
    ros::Subscriber gps_sub_, estimator_sub_;
    ros::WallTimer timer_;
    ros::AsyncSpinner spinner_;
};

// 本地位置订阅封装
class PoseSubscriber {
public:
    PoseSubscriber(ros::NodeHandle& nh, const std::string& topic, double local_xy_scale)
        : local_xy_scale_(local_xy_scale) {
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
    bool getFreshPose(geometry_msgs::PoseStamped& pose, double max_age) const {
        std::lock_guard<std::mutex> lock(mtx_);
        if (!received_) return false;
        const ros::Time now = ros::Time::now();
        const double rx_age = (now - last_rx_).toSec();
        const double source_age = (now - current_.header.stamp).toSec();
        if (rx_age < 0.0 || rx_age > max_age || source_age < -0.05 ||
            source_age > max_age) return false;
        pose = current_;
        return true;
    }
    double getCurrentYaw() const {
        std::lock_guard<std::mutex> lock(mtx_);
        const auto& q = current_.pose.orientation;
        double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
        double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        return std::atan2(siny_cosp, cosy_cosp);
    }
private:
    void cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        const auto& p = msg->pose.position;
        const auto& q = msg->pose.orientation;
        const double qnorm = std::hypot(std::hypot(q.x, q.y), std::hypot(q.z, q.w));
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
            !std::isfinite(qnorm) || qnorm < 1e-6) {
            ROS_WARN_THROTTLE(1.0, "[位姿] 丢弃非有限位置或无效四元数");
            return;
        }
        std::lock_guard<std::mutex> lock(mtx_);
        current_ = *msg;
        current_.pose.position.x *= local_xy_scale_;
        current_.pose.position.y *= local_xy_scale_;
        current_.pose.orientation.x /= qnorm;
        current_.pose.orientation.y /= qnorm;
        current_.pose.orientation.z /= qnorm;
        current_.pose.orientation.w /= qnorm;
        received_ = true;
        last_rx_ = ros::Time::now();
    }
    mutable std::mutex mtx_;
    geometry_msgs::PoseStamped current_;
    double local_xy_scale_;
    bool received_ = false;
    ros::Time last_rx_;
    ros::Subscriber sub_;
};

// ego PositionCommand 订阅封装
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
    // 指令租约独立检查：traj_server 持续发新消息不能证明规划器仍正常工作。
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

// 跟踪目标 / 模式订阅封装
// swarm_fusion 的原子目标：坐标时刻、真实观测时刻与通信租约分别保存。
class TrackTargetSubscriber {
public:
    TrackTargetSubscriber(ros::NodeHandle& nh, const std::string& topic,
                          double state_timeout, double observation_timeout,
                          double future_tolerance)
        : state_timeout_(state_timeout), observation_timeout_(observation_timeout),
          future_tolerance_(future_tolerance) {
        sub_ = nh.subscribe<track_msgs::TrackingTarget>(topic, 4,
            &TrackTargetSubscriber::cb, this, ros::TransportHints().tcpNoDelay());
    }
    bool get(double& x, double& y, double& vx, double& vy, double& age,
             std::string* assignment = nullptr, double* observation_age = nullptr,
             bool* bearing_only = nullptr, double* prediction_age = nullptr,
             double* velocity_variance = nullptr) const {
        std::lock_guard<std::mutex> lock(mtx_);
        if (!received_) return false;
        const ros::Time now = ros::Time::now();
        const double state_age = (now - state_stamp_).toSec();
        const double meas_age = (now - measurement_stamp_).toSec();
        age = (now - last_rx_).toSec();
        if (age < 0.0 || state_age < -future_tolerance_ ||
            meas_age < -future_tolerance_) return false;
        // state 的位置已经由融合预测到 state_stamp，这里只补接收后的剩余时间。
        // 单目方位锚点和陈旧观测不按缓存速度继续推进，也不注入速度前馈。
        const bool predict = !bearing_only_ && meas_age <= observation_timeout_ &&
                             state_age <= state_timeout_;
        vx = predict ? vx_ : 0.0;
        vy = predict ? vy_ : 0.0;
        const double dt = std::max(0.0, state_age);
        x = x_ + vx * dt;
        y = y_ + vy * dt;
        if (!std::isfinite(x) || !std::isfinite(y)) return false;
        if (assignment) *assignment = assignment_;
        if (observation_age) *observation_age = std::max(0.0, meas_age);
        if (bearing_only) *bearing_only = bearing_only_;
        if (prediction_age) *prediction_age = std::max(0.0, state_age);
        if (velocity_variance) *velocity_variance = velocity_variance_;
        return true;
    }
private:
    void cb(const track_msgs::TrackingTarget::ConstPtr& msg) {
        const auto& state = msg->state;
        const auto& p = state.pose.pose.position;
        const auto& v = state.twist.twist.linear;
        const ros::Time now = ros::Time::now();
        if (state.header.frame_id != "world" || msg->assignment.empty() ||
            state.child_frame_id != msg->assignment || state.header.stamp.isZero() ||
            msg->measurement_stamp.isZero() || !std::isfinite(p.x) ||
            !std::isfinite(p.y) || !std::isfinite(p.z) || !std::isfinite(v.x) ||
            !std::isfinite(v.y) || !std::isfinite(v.z) ||
            (state.header.stamp - now).toSec() > future_tolerance_ ||
            (msg->measurement_stamp - state.header.stamp).toSec() > future_tolerance_ ||
            (!msg->depth_stamp.isZero() &&
             (msg->depth_stamp - msg->measurement_stamp).toSec() > future_tolerance_)) {
            ROS_WARN_THROTTLE(1.0, "[目标] 丢弃坐标、时间戳或认领凭证不合法的目标");
            return;
        }
        std::lock_guard<std::mutex> lock(mtx_);
        // 深度预测状态切换到单目锚点时state_stamp可回到观测时刻；
        // 用真实观测时间拒绝乱序，不能把这种正常模式切换判成旧消息。
        if (received_ && last_rx_ <= now && assignment_ == msg->assignment &&
            msg->measurement_stamp < measurement_stamp_) return;
        x_ = p.x; y_ = p.y;
        vx_ = v.x; vy_ = v.y;
        state_stamp_ = state.header.stamp;
        measurement_stamp_ = msg->measurement_stamp;
        depth_stamp_ = msg->depth_stamp;
        assignment_ = msg->assignment;
        bearing_only_ = msg->bearing_only;
        const double var_x = state.twist.covariance[0];
        const double var_y = state.twist.covariance[7];
        velocity_variance_ = (std::isfinite(var_x) && std::isfinite(var_y) &&
                              var_x >= 0.0 && var_y >= 0.0)
            ? std::max(var_x, var_y) : std::numeric_limits<double>::infinity();
        received_ = true;
        last_rx_ = now;
    }
    mutable std::mutex mtx_;
    double x_ = 0.0, y_ = 0.0, vx_ = 0.0, vy_ = 0.0;
    std::string assignment_;
    bool received_ = false;
    bool bearing_only_ = false;
    double velocity_variance_ = std::numeric_limits<double>::infinity();
    ros::Time state_stamp_, measurement_stamp_, depth_stamp_;
    double state_timeout_, observation_timeout_, future_tolerance_;
    ros::Time last_rx_;
    ros::Subscriber sub_;
};

// 本机已关联行人的图像视线。只接受本机+当前认领凭证，不从颜色猜测身份。
class TrackViewSubscriber {
public:
    TrackViewSubscriber(ros::NodeHandle& nh, const std::string& topic, int drone_id,
                        double timeout, double future_tolerance, double min_confidence)
        : drone_id_(drone_id), timeout_(timeout), future_tolerance_(future_tolerance),
          min_confidence_(min_confidence) {
        sub_ = nh.subscribe<track_msgs::TrackingView>(topic, 1,
            &TrackViewSubscriber::cb, this, ros::TransportHints().tcpNoDelay());
    }
    bool get(const std::string& assignment, track_msgs::TrackingView& view) const {
        std::lock_guard<std::mutex> lock(mtx_);
        if (!received_ || view_.assignment != assignment) return false;
        const double age = (ros::Time::now() - view_.header.stamp).toSec();
        const double rx_age = (ros::Time::now() - last_rx_).toSec();
        if (age < -future_tolerance_ || age > timeout_ || rx_age < 0.0 ||
            rx_age > timeout_) return false;
        view = view_;
        return true;
    }
private:
    void cb(const track_msgs::TrackingView::ConstPtr& msg) {
        const ros::Time now = ros::Time::now();
        if (msg->drone_id != drone_id_ || msg->assignment.empty() ||
            msg->header.frame_id != "world" || msg->header.stamp.isZero() ||
            !std::isfinite(msg->bearing) || !std::isfinite(msg->horizontal_error_px) ||
            !std::isfinite(msg->confidence) || msg->confidence < min_confidence_ ||
            msg->confidence > 1.0 ||
            (msg->header.stamp - now).toSec() > future_tolerance_ ||
            (now - msg->header.stamp).toSec() > timeout_) return;
        std::lock_guard<std::mutex> lock(mtx_);
        if (received_ && last_rx_ <= now && msg->header.stamp <= view_.header.stamp)
            return;
        view_ = *msg;
        last_rx_ = now;
        received_ = true;
    }
    mutable std::mutex mtx_;
    track_msgs::TrackingView view_;
    ros::Time last_rx_;
    bool received_ = false;
    int drone_id_;
    double timeout_, future_tolerance_, min_confidence_;
    ros::Subscriber sub_;
};

// MAVROS velocity_local 是 ENU 水平速度；与本地位置使用相同尺度换算。
class LocalVelocitySubscriber {
public:
    LocalVelocitySubscriber(ros::NodeHandle& nh, const std::string& topic, double scale)
        : scale_(scale) {
        sub_ = nh.subscribe<geometry_msgs::TwistStamped>(topic, 1,
            &LocalVelocitySubscriber::cb, this, ros::TransportHints().tcpNoDelay());
    }
    bool get(double& vx, double& vy, double max_age) const {
        std::lock_guard<std::mutex> lock(mtx_);
        if (!received_) return false;
        const ros::Time now = ros::Time::now();
        const double source_age = (now - stamp_).toSec();
        const double rx_age = (now - last_rx_).toSec();
        if (source_age < -0.05 || source_age > max_age || rx_age < 0.0 ||
            rx_age > max_age) return false;
        vx = vx_; vy = vy_;
        return true;
    }
private:
    void cb(const geometry_msgs::TwistStamped::ConstPtr& msg) {
        const double vx = msg->twist.linear.x * scale_;
        const double vy = msg->twist.linear.y * scale_;
        if (msg->header.stamp.isZero() || !std::isfinite(vx) || !std::isfinite(vy)) return;
        std::lock_guard<std::mutex> lock(mtx_);
        if (received_ && last_rx_ <= ros::Time::now() && msg->header.stamp < stamp_) return;
        vx_ = vx; vy_ = vy; stamp_ = msg->header.stamp;
        last_rx_ = ros::Time::now(); received_ = true;
    }
    mutable std::mutex mtx_;
    double scale_, vx_ = 0.0, vy_ = 0.0;
    bool received_ = false;
    ros::Time stamp_, last_rx_;
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
        if (last_rx_.isZero() || (ros::Time::now() - last_rx_).toSec() > 1.0) return 0;
        return mode_;
    }
private:
    void cb(const std_msgs::Int8::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(mtx_);
        mode_ = static_cast<int>(msg->data);
        last_rx_ = ros::Time::now();
    }
    mutable std::mutex mtx_;
    int mode_ = 0;
    ros::Time last_rx_;
    ros::Subscriber sub_;
};

// 友机世界坐标用于站位间距检查。
class PeerOdomSubscriber {
public:
    PeerOdomSubscriber(ros::NodeHandle& nh, const std::string& topic) {
        sub_ = nh.subscribe<nav_msgs::Odometry>(topic, 1,
            &PeerOdomSubscriber::cb, this, ros::TransportHints().tcpNoDelay());
    }
    // 世界帧位置; 从未收到或超过 max_age 秒没更新则返回 false
    bool get(double& x, double& y, double max_age = 2.0) const {
        std::lock_guard<std::mutex> lock(mtx_);
        if (!received_) return false;
        if ((ros::Time::now() - last_rx_).toSec() > max_age) return false;
        x = x_; y = y_;
        return true;
    }
private:
    void cb(const nav_msgs::Odometry::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(mtx_);
        x_ = msg->pose.pose.position.x;
        y_ = msg->pose.pose.position.y;
        received_ = true;
        last_rx_ = ros::Time::now();
    }
    mutable std::mutex mtx_;
    double x_ = 0.0, y_ = 0.0;
    bool received_ = false;
    ros::Time last_rx_;
    ros::Subscriber sub_;
};

// 用新鲜的局部膨胀地图检查航点；地图缺失或过期时不在此否决。
class ObstacleMapSubscriber {
public:
    ObstacleMapSubscriber(ros::NodeHandle& nh, const std::string& topic) {
        sub_ = nh.subscribe<sensor_msgs::PointCloud2>(topic, 1,
            &ObstacleMapSubscriber::cb, this, ros::TransportHints().tcpNoDelay());
    }
    // 航点 (wx, wy) 是否落在膨胀障碍内: 有新鲜地图(<=max_age s)且最近障碍点
    // 距离 < clear 时判 true。
    bool blocked(double wx, double wy, double clear, double max_age) const {
        std::lock_guard<std::mutex> lock(mtx_);
        if (!received_) return false;
        if ((ros::Time::now() - last_rx_).toSec() > max_age) return false;
        for (const auto& p : pts_) {
            const double dx = wx - p.first, dy = wy - p.second;
            if (dx * dx + dy * dy < clear * clear) return true;
        }
        return false;
    }
    // 提取局部膨胀障碍 XY；缺图或过期返回 false，由调用方决定回退。
    bool nearby(double cx, double cy, double radius,
                std::vector<std::pair<float, float>>& out) const {
        std::lock_guard<std::mutex> lock(mtx_);
        out.clear();
        if (!received_) return false;
        if ((ros::Time::now() - last_rx_).toSec() > 1.0) return false;
        const double r2 = radius * radius;
        for (const auto& p : pts_) {
            const double dx = cx - p.first, dy = cy - p.second;
            if (dx * dx + dy * dy <= r2) out.emplace_back(p.first, p.second);
        }
        return true;
    }
private:
    void cb(const sensor_msgs::PointCloud2::ConstPtr& msg) {
        std::vector<std::pair<float, float>> tmp;
        tmp.reserve(msg->width * msg->height);
        sensor_msgs::PointCloud2ConstIterator<float> ix(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iy(*msg, "y");
        for (; ix != ix.end(); ++ix, ++iy) tmp.emplace_back(*ix, *iy);
        std::lock_guard<std::mutex> lock(mtx_);
        pts_.swap(tmp);
        received_ = true;
        last_rx_ = ros::Time::now();
    }
    mutable std::mutex mtx_;
    std::vector<std::pair<float, float>> pts_;
    bool received_ = false;
    ros::Time last_rx_;
    ros::Subscriber sub_;
};

// 原始雷达记忆用于净空判断；机体高度带仅参与碰撞检查，不遮挡视线。
// 上方结构证据另用于排除棚下站位。
class LidarSurfaceSubscriber {
public:
    LidarSurfaceSubscriber(ros::NodeHandle& nh, const std::string& topic,
                           const std::string& band_topic, const std::string& overhead_topic,
                           double res, double memory_sec)
        : mem_(res), memory_sec_(memory_sec) {
        sub_ = nh.subscribe<sensor_msgs::PointCloud2>(topic, 2,
            &LidarSurfaceSubscriber::cb, this, ros::TransportHints().tcpNoDelay());
        band_sub_ = nh.subscribe<sensor_msgs::PointCloud2>(band_topic, 2,
            &LidarSurfaceSubscriber::bandCb, this, ros::TransportHints().tcpNoDelay());
        overhead_sub_ = nh.subscribe<sensor_msgs::PointCloud2>(overhead_topic, 2,
            &LidarSurfaceSubscriber::overheadCb, this, ros::TransportHints().tcpNoDelay());
    }
    bool fresh(double max_age) const {
        std::lock_guard<std::mutex> lock(mtx_);
        return received_ && (ros::Time::now() - last_rx_).toSec() <= max_age;
    }
    // 碰撞记忆与较长的雷达历史分开，避免把普通墙面误判为上方结构。
    void buildField(fly_px4::ClearanceField& field, double cx, double cy, double half,
                    double memory_sec, double thin_extent) {
        std::lock_guard<std::mutex> lock(mtx_);
        const double now = ros::Time::now().toSec();
        mem_.prune(now, std::max(memory_sec, overhead_lidar_sec_), cx, cy, half + 8.0);
        field.build(mem_, cx, cy, half, now, memory_sec, thin_extent, overhead_lidar_sec_);
    }
    void setOverheadLidarSec(double sec) {
        std::lock_guard<std::mutex> lock(mtx_);
        overhead_lidar_sec_ = std::isfinite(sec) && sec > 0.0 ? sec : 0.0;
    }
private:
    void bandCb(const sensor_msgs::PointCloud2::ConstPtr& msg) { insertBand(msg, false); }
    void overheadCb(const sensor_msgs::PointCloud2::ConstPtr& msg) { insertBand(msg, true); }
    void insertBand(const sensor_msgs::PointCloud2::ConstPtr& msg, bool overhead) {
        const double now = ros::Time::now().toSec();
        double stamp = msg->header.stamp.isZero() ? now : msg->header.stamp.toSec();
        if (stamp > now + 0.05) stamp = now;
        std::lock_guard<std::mutex> lock(mtx_);
        sensor_msgs::PointCloud2ConstIterator<float> ix(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iy(*msg, "y");
        for (; ix != ix.end(); ++ix, ++iy)
            mem_.insert(*ix, *iy, stamp, /*occludes=*/false, overhead);
        if (!band_received_ && msg->width * msg->height > 0) {
            band_received_ = true;
            ROS_INFO("[避障] 收到机体高度带障碍点云 (深度相机/雷达高度记忆), 只算碰撞不挡视线");
        }
    }

    void cb(const sensor_msgs::PointCloud2::ConstPtr& msg) {
        const ros::Time now = ros::Time::now();
        double stamp = msg->header.stamp.isZero() ? now.toSec() : msg->header.stamp.toSec();
        if (stamp > now.toSec() + 0.05) stamp = now.toSec();
        std::lock_guard<std::mutex> lock(mtx_);
        sensor_msgs::PointCloud2ConstIterator<float> ix(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iy(*msg, "y");
        for (; ix != ix.end(); ++ix, ++iy) mem_.insert(*ix, *iy, stamp);
        received_ = true;
        last_rx_ = now;
        // 巡逻时不建场, 也要按时间清, 否则整圈航线的墙面都攒在内存里
        if (last_prune_ < 0.0 || stamp < last_prune_ || stamp - last_prune_ >= 1.0) {
            mem_.prune(stamp, std::max(memory_sec_, overhead_lidar_sec_), 0.0, 0.0,
                       std::numeric_limits<double>::infinity());
            last_prune_ = stamp;
        }
    }
    mutable std::mutex mtx_;
    fly_px4::SurfaceMemory mem_;
    double memory_sec_ = 3.0;
    double overhead_lidar_sec_ = 0.0;
    double last_prune_ = -1.0;
    bool received_ = false;
    bool band_received_ = false;
    ros::Time last_rx_;
    ros::Subscriber sub_, band_sub_, overhead_sub_;
};

// 任务节点
class Task {
public:
    explicit Task(ros::NodeHandle& nh)
        : nh_(nh) {
        // 基本参数
        nh_.param<std::string>("drone_ns",  drone_ns_,  "typhoon_h480_0");
        nh_.param<int>        ("drone_id",  drone_id_,  1);
        nh_.param<int>        ("drone_num", drone_num_, 6);
        // 巡航高度配置范围与限高保护；实际指令不再采用 EGO 的 Z。
        nh_.param<double>("min_expl_z",   min_expl_z_,   4.1);
        nh_.param<double>("max_expl_z",   max_expl_z_,   4.9);
        nh_.param<double>("z_soft_limit", z_soft_limit_, 5.7);
        nh_.param<double>("z_hard_limit", z_hard_limit_, 6.0);
        // 起点只改变 world/local 换算；马路航点始终保持地图坐标。
        // 六机 launch 与 odom_world_relay 读取同一个 start_positions.yaml。
        std::string start_key;
        nh_.param<std::string>("start_position_key", start_key, "");
        if (!start_key.empty()) {
            XmlRpc::XmlRpcValue starts;
            if (!nh_.getParam("start_positions", starts) ||
                starts.getType() != XmlRpc::XmlRpcValue::TypeStruct ||
                (starts.size() != 6 &&
                 !(starts.size() == 7 && starts.hasMember("patrol_maps_dir"))))
                throw std::invalid_argument("start_positions must contain drone_1..drone_6");
            if (starts.hasMember("patrol_maps_dir")) {
                auto& directory = starts["patrol_maps_dir"];
                if (directory.getType() != XmlRpc::XmlRpcValue::TypeString)
                    throw std::invalid_argument("patrol_maps_dir must be patrol_maps or patrol_maps_2");
                const std::string name = static_cast<std::string>(directory);
                if (name != "patrol_maps" && name != "patrol_maps_2")
                    throw std::invalid_argument("patrol_maps_dir must be patrol_maps or patrol_maps_2");
            }
            for (int id = 1; id <= 6; ++id) {
                const std::string key = "drone_" + std::to_string(id);
                if (!starts.hasMember(key))
                    throw std::invalid_argument("Missing start position: " + key);
                auto& entry = starts[key];
                if (entry.getType() != XmlRpc::XmlRpcValue::TypeStruct || entry.size() != 2 ||
                    !entry.hasMember("x") || !entry.hasMember("y"))
                    throw std::invalid_argument(key + " must contain only x/y");
                for (const char* axis : {"x", "y"}) {
                    auto& value = entry[axis];
                    if ((value.getType() != XmlRpc::XmlRpcValue::TypeInt &&
                         value.getType() != XmlRpc::XmlRpcValue::TypeDouble) ||
                        !std::isfinite(xmlD(value)))
                        throw std::invalid_argument(key + "." + axis + " must be a finite number");
                }
            }
            if (start_key != "drone_" + std::to_string(drone_id_) || !starts.hasMember(start_key))
                throw std::invalid_argument("start_position_key must match drone_id");
            init_x_ = xmlD(starts[start_key]["x"]);
            init_y_ = xmlD(starts[start_key]["y"]);
            ROS_INFO("[起点配置] %s: x=%.3f, y=%.3f (start_positions.yaml)",
                     start_key.c_str(), init_x_, init_y_);
        } else {
            // 保留单机调试命令的旧参数兼容；六机 launch 总是加载起点 YAML。
            nh_.param<double>("init_x", init_x_, 0.0);
            nh_.param<double>("init_y", init_y_, 0.0);
        }
        ros::param::param<double>("/swarm_coordinates/local_xy_scale", local_xy_scale_, 1.0);
        if (!std::isfinite(local_xy_scale_) || local_xy_scale_ <= 0.0)
            throw std::invalid_argument("local_xy_scale must be finite and positive");
        ROS_INFO("[坐标] local_xy_scale=%.12f (EKF XY -> physical local metres)", local_xy_scale_);
        nh_.param<double>("pose_timeout", pose_timeout_, 0.5);
        if (!std::isfinite(pose_timeout_) || pose_timeout_ <= 0.0) pose_timeout_ = 0.5;

        // 脱困(与 fly_px4_2 同参数同默认值)
        nh_.param<double>("cmd_timeout",       cmd_timeout_,       2.0);
        nh_.param<double>("stuck_yaw_sec",     stuck_yaw_sec_,     6.0);
        nh_.param<double>("stuck_retreat_sec", stuck_retreat_sec_, 12.0);
        nh_.param<double>("retreat_dist",      retreat_dist_,      8.0);
        nh_.param<double>("frozen_sec",        frozen_sec_,        25.0);
        nh_.param<double>("spin_rate_deg",     spin_rate_deg_,     30.0);
        nh_.param<double>("retreat_hold",      retreat_hold_,      8.0);
        nh_.param<double>("crash_z",           crash_z_,           1.5);
        nh_.param<double>("crash_hold",        crash_hold_,        5.0);

        // 跟踪机头
        // 图像框中心视线优先；yaw_rate 是连续参考角的导数，PX4 负责实际姿态闭环。
        nh_.param<double>("track_yaw_rate_max",     track_yaw_rate_max_, 60.0);
        nh_.param<double>("track_yaw_acc_max",      track_yaw_acc_max_, 180.0);
        nh_.param<double>("track_yaw_kp",           track_yaw_kp_, 5.0);
        nh_.param<double>("track_yaw_deadband_deg", track_yaw_deadband_deg_, 5.0);
        nh_.param<double>("track_view_timeout",     track_view_timeout_, 0.35);
        nh_.param<double>("track_view_conf_min",    track_view_conf_min_, 0.4);
        nh_.param<double>("track_yaw_stationary_tau", track_yaw_stationary_tau_, 0.12);
        nh_.param<double>("track_yaw_moving_tau",   track_yaw_moving_tau_, 0.04);
        nh_.param<double>("track_yaw_velocity_timeout", track_yaw_velocity_timeout_, 0.25);
        nh_.param<double>("track_stale_sec",        track_stale_sec_, 0.3);
        const auto positive = [](double& v, double fallback) {
            if (!std::isfinite(v) || v <= 0.0) v = fallback;
        };
        positive(track_yaw_rate_max_, 60.0);
        positive(track_yaw_acc_max_, 180.0);
        positive(track_yaw_kp_, 5.0);
        if (!std::isfinite(track_yaw_deadband_deg_) || track_yaw_deadband_deg_ < 0.0)
            track_yaw_deadband_deg_ = 5.0;
        track_yaw_deadband_deg_ = std::min(track_yaw_deadband_deg_, 180.0);
        positive(track_view_timeout_, 0.35);
        positive(track_yaw_stationary_tau_, 0.12);
        positive(track_yaw_moving_tau_, 0.04);
        positive(track_yaw_velocity_timeout_, 0.25);
        positive(track_stale_sec_, 0.3);
        if (!std::isfinite(track_view_conf_min_) || track_view_conf_min_ < 0.0 ||
            track_view_conf_min_ > 1.0) track_view_conf_min_ = 0.4;
        fly_px4::TrackYawConfig yaw_config;
        yaw_config.rate_max = track_yaw_rate_max_ * M_PI / 180.0;
        yaw_config.acc_max = track_yaw_acc_max_ * M_PI / 180.0;
        yaw_config.kp = track_yaw_kp_;
        yaw_config.deadband = track_yaw_deadband_deg_ * M_PI / 180.0;
        yaw_config.stationary_tau = track_yaw_stationary_tau_;
        yaw_config.moving_tau = track_yaw_moving_tau_;
        yaw_config.bearing_timeout = track_view_timeout_;
        track_yaw_controller_.configure(yaw_config);
        // 兼容旧前推参数名称；只补偿图像延迟，不额外瞄未来位置。
        double legacy_lead = 0.0;
        nh_.param<double>("track_lead_sec", legacy_lead, 0.0);
        if (legacy_lead != 0.0)
            ROS_WARN("[机头] track_lead_sec 已停用：只补偿到当前视线时刻，不瞄未来位置");
        nh_.param<double>("target_state_timeout", target_state_timeout_, 1.0);
        nh_.param<double>("track_measurement_timeout", track_measurement_timeout_, 1.0);
        nh_.param<double>("target_future_tolerance", target_future_tolerance_, 0.05);
        nh_.param<double>("track_reacquire_nav_timeout", track_reacquire_nav_timeout_, 25.0);
        nh_.param<double>("track_planner_start_grace", track_planner_start_grace_, 3.0);
        positive(track_reacquire_nav_timeout_, 25.0);
        positive(track_planner_start_grace_, 3.0);
        if (!std::isfinite(target_state_timeout_) || target_state_timeout_ <= 0.0)
            target_state_timeout_ = 1.0;
        if (!std::isfinite(track_measurement_timeout_) || track_measurement_timeout_ <= 0.0)
            track_measurement_timeout_ = 1.0;
        if (!std::isfinite(target_future_tolerance_) || target_future_tolerance_ < 0.0)
            target_future_tolerance_ = 0.05;
        // 跟踪站位: 7.0m 配 3.8m 跟踪高度 + 相机 20 度下倾, 人正好在画面正中
        nh_.param<double>("standoff_peer_clear", standoff_peer_clear_, 3.5);
        nh_.param<double>("standoff_return_margin", standoff_return_margin_, 0.5);
        if (!std::isfinite(standoff_return_margin_) || standoff_return_margin_ < 0.0)
            standoff_return_margin_ = 0.5;
        nh_.param<double>("standoff_reject_delay", standoff_reject_delay_, 2.0);
        nh_.param<double>("standoff_band_hold_sec", standoff_band_hold_sec_, 10.0);
        nh_.param<double>("standoff_band_hold_dist", standoff_band_hold_dist_, 12.0);
        nh_.param<double>("standoff_band_max_radius", standoff_band_max_radius_, 8.0);
        nh_.param<double>("track_release_sec",   track_release_sec_,   1.0);
        // 起飞守卫重试间隔 (s): 越小 OFFBOARD/解锁请求越勤, 转浆起飞越快
        nh_.param<double>("takeoff_retry_sec", takeoff_retry_sec_, 0.5);
        // 跟随目标点高频更新 (0.3s/0.25m), 内部过滤阈值要小于它才不会被拦
        nh_.param<double>("goal_move_thresh",    goal_move_thresh_,    0.2);
        // 等 ego 首条指令超过这么久就从 WARN 升级成 ERROR (上游全崩 vs 还在建图)
        nh_.param<double>("ego_wait_warn_sec",   ego_wait_warn_sec_,   15.0);
        // 状态切换的垂直参考约束；EGO 仅负责 XY，不采用它的 Z 前馈。
        nh_.param<double>("max_vz", max_vz_, 0.5);
        nh_.param<double>("max_az", max_az_, 1.0);
        altitude_hold_.configure(max_vz_, max_az_);

        // patrol_collision_check 控制巡逻和初始进场检查；跟踪后归航保持避障。
        nh_.param<bool>("patrol_collision_check", patrol_collision_check_, false);

        loadRoute();

        const std::string mavros_ns = "/" + drone_ns_ + "/mavros";
        const std::string id_str = std::to_string(drone_id_);

        // 订阅
        mavros_sub_  = std::make_unique<MavrosSubscriber>(nh_, mavros_ns + "/state");
        pose_sub_    = std::make_unique<PoseSubscriber>(
            nh_, mavros_ns + "/local_position/pose", local_xy_scale_);
        local_velocity_sub_ = std::make_unique<LocalVelocitySubscriber>(
            nh_, mavros_ns + "/local_position/velocity_local", local_xy_scale_);
        pos_cmd_sub_ = std::make_unique<PosCmdSubscriber>(nh_, "/ego/pos_cmd_" + id_str);

        track_target_sub_ = std::make_unique<TrackTargetSubscriber>(
            nh_, "/tracking/state_" + id_str, target_state_timeout_,
            track_measurement_timeout_, target_future_tolerance_);
        track_mode_sub_ = std::make_unique<TrackModeSubscriber>(
            nh_, "/tracking/mode_" + id_str);
        track_view_sub_ = std::make_unique<TrackViewSubscriber>(
            nh_, "/tracking/view_" + id_str, drone_id_, track_view_timeout_,
            target_future_tolerance_, track_view_conf_min_);
        // 任务完成信号(latched): 本节点通常比 fusion 后启动, 一订阅就要能拿到当前值
        mission_complete_sub_ = nh_.subscribe<std_msgs::Bool>(
            "/tracking/mission_complete", 2, &Task::missionCompleteCb, this);

        // 友机位置(世界帧). ns 索引是 0-based, drone_id 是 1-based
        for (int i = 0; i < drone_num_; ++i) {
            if (i == drone_id_ - 1) continue;   // 跳过自己
            peer_subs_.push_back(std::make_unique<PeerOdomSubscriber>(
                nh_, "/typhoon_h480_" + std::to_string(i) + "/odom_world_relay"));
        }

        // 本机 ego 的膨胀障碍地图(世界帧). ego 节点名 = ego_planner_<drone_id>,
        // 地图话题在它命名空间下: /ego_planner_<id>/grid_map/occupancy_inflate
        obstacle_map_ = std::make_unique<ObstacleMapSubscriber>(
            nh_, "/ego_planner_" + id_str + "/grid_map/occupancy_inflate");
        // 原始雷达表面 (跟踪避障: 离墙净空/视线), 与 ego 建图同一路点云;
        // 机体高度带点 (深度相机/雷达高度记忆) 只算碰撞
        if (avoid_enable_)
            lidar_surface_ = std::make_unique<LidarSurfaceSubscriber>(
                nh_, "/" + drone_ns_ + "/lidar_cloud_world",
                "/" + drone_ns_ + "/body_band_cloud", "/" + drone_ns_ + "/body_band_overhead",
                avoid_res_, avoid_memory_sec_);
        if (lidar_surface_) lidar_surface_->setOverheadLidarSec(avoid_overhead_lidar_sec_);


        // 发布
        set_raw_pub_ = nh_.advertise<mavros_msgs::PositionTarget>(
            mavros_ns + "/setpoint_raw/local", 10);
        // 目标点: latch=true. ego_planner_node 若 respawn, 新订阅者一连上就能拿到
        // 最后一个目标, 不用等我们的重发周期(套路同 fly_px4_2 的 trigger latch)。
        ego_goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(
            "/ego_goal_" + id_str, 1, /*latch=*/true);
        // 目标速度: 跟踪移动目标时发给 ego, 让轨迹终点速度非零(消除打点)。
        // latch 同理, ego respawn 后立刻能拿到当前目标速度。
        ego_goal_vel_pub_ = nh_.advertise<geometry_msgs::TwistStamped>(
            "/ego_goal_vel_" + id_str, 1, /*latch=*/true);
        // 限速同理 latch: ego 重启后必须立刻知道当前该用哪档速度, 否则会拿
        // launch 里的初值(巡航 12)去做锁定跟踪, 一头撞进行人把人吓跑。
        ego_speed_pub_ = nh_.advertise<std_msgs::Float32MultiArray>(
            "/ego_speed_" + id_str, 1, /*latch=*/true);
        // 巡逻时关闭 ego 的碰撞急停 (latch 同理, ego 重启后要立刻知道当前是否巡逻)
        disable_collision_pub_ = nh_.advertise<std_msgs::Bool>(
            "/ego_disable_collision_" + id_str, 1, /*latch=*/true);
        reject_pub_ = nh_.advertise<std_msgs::Int8>("/tracking/reject_" + id_str, 2);
        tracking_active_pub_ = nh_.advertise<std_msgs::Header>(
            "/tracking/active_" + id_str, 2);
        // 机体高度带请求 [本状态目标高度(本地帧), 跟踪相关阶段 0/1] -> body_band_cloud.py:
        // 只在跟踪/跟踪后归航时开深度并发布高度带障碍, 升降途中高度带延伸到目标高度
        band_request_pub_ = nh_.advertise<std_msgs::Float32MultiArray>(
            "/" + drone_ns_ + "/body_band_request", 2);
        // 站位方位链诊断 (只读, 50Hz, 不参与控制; 见 stepTrack 里的发布点)
        standoff_diag_pub_ = nh_.advertise<std_msgs::Float32MultiArray>(
            "/tracking/standoff_diag_" + id_str, 50);
        // [实际yaw_deg, 参考yaw_deg, 参考rate_deg/s, 图像反馈有效,
        //  框中心水平误差px, 图像年龄s, 坐标LOS_deg]。无图像时误差/年龄为NaN。
        yaw_diag_pub_ = nh_.advertise<std_msgs::Float32MultiArray>(
            "/tracking/yaw_diag_" + id_str, 10);

        // 服务
        arming_cli_   = nh_.serviceClient<mavros_msgs::CommandBool>(mavros_ns + "/cmd/arming");
        set_mode_cli_ = nh_.serviceClient<mavros_msgs::SetMode>(mavros_ns + "/set_mode");
        param_cli_    = nh_.serviceClient<mavros_msgs::ParamSet>(mavros_ns + "/param/set");

        control_ = std::make_unique<ControlPx4>(set_raw_pub_, local_xy_scale_);
        gps_height_setup_ = std::make_unique<Px4GpsHeightSetup>(nh_, mavros_ns, *mavros_sub_);

        // 用零时间作未初始化标记，避免 /clock 启动前构造负 ROS 时间。
        last_offboard_t_ = ros::Time(0);
        last_arm_t_ = ros::Time(0);

        timer_ = nh_.createTimer(ros::Duration(0.02), &Task::runtask, this);
        tracking_feedback_timer_ = nh_.createTimer(
            ros::Duration(0.1), &Task::trackingFeedback, this);

        ROS_INFO("[fly_px4_1] ns=%s id=%d 航线 %zu 点 z=%.2f 巡航=%.1fm/s "
                 "z_limit=[钳位 %.2f, 软 %.2f, 硬 %.2f] spawn=(%.1f,%.1f) start_delay=%.1fs",
                 drone_ns_.c_str(), drone_id_, route_.size(), route_z_, cruise_speed_,
                 max_expl_z_, z_soft_limit_, z_hard_limit_, init_x_, init_y_, start_delay_);
    }

private:
    // 公共参数由 patrol_routes.yaml 提供，各机航线可覆盖。
    void loadRoute() {
        const std::string def_ns = "patrol/defaults/";
        const std::string rt_ns  = "patrol/routes/drone_" + std::to_string(drone_id_) + "/";
        nh_.param<std::string>("patrol_map", patrol_map_, "base0");
        if (!(nh_.getParam(rt_ns + "climb_in_place", climb_in_place_) ||
              nh_.getParam(def_ns + "climb_in_place", climb_in_place_)))
            climb_in_place_ = false;

        // 速度档
        cruise_speed_ = dparam(def_ns, rt_ns, "cruise_speed", 12.0);
        cruise_accel_ = dparam(def_ns, rt_ns, "cruise_accel", 8.0);
        // 兼容旧配置: 未设置独立速度/加速度时沿用 transit_speed/transit_accel。
        const double transit_speed = dparam(def_ns, rt_ns, "transit_speed", 8.0);
        const double transit_accel = dparam(def_ns, rt_ns, "transit_accel", 6.0);
        entry_speed_ = dparam(def_ns, rt_ns, "entry_speed", transit_speed);
        entry_accel_ = dparam(def_ns, rt_ns, "entry_accel", transit_accel);
        return_speed_ = dparam(def_ns, rt_ns, "return_speed", transit_speed);
        return_accel_ = dparam(def_ns, rt_ns, "return_accel", transit_accel);
        // 远/中/近距离分档带滞回，以下为 YAML 未覆盖时的默认值。
        track_speed_far_  = dparam(def_ns, rt_ns, "track_speed_far",  6.0);
        track_acc_far_    = dparam(def_ns, rt_ns, "track_acc_far",    4.0);
        // mid 档 (>9m 接近途中): 1.8 m/s —— 要能追上走动的人 (跟随目标点以
        // 行人 1m/s 移动, 上限得留余量), 又别太快冲过站位。
        track_speed_near_ = dparam(def_ns, rt_ns, "track_speed_near", 3.0);
        track_acc_near_   = dparam(def_ns, rt_ns, "track_acc_near",   3.0);
        // creep 档 (<9m 跟随段): 2.0 m/s / 0.5 m/s²。
        track_speed_creep_= dparam(def_ns, rt_ns, "track_speed_creep", 2.0);
        track_acc_creep_  = dparam(def_ns, rt_ns, "track_acc_creep",  0.5);
        // 档位切换的滞回边界: far<->mid 用 20/30m, mid<->creep 用 9/11m。
        track_near_dist_  = dparam(def_ns, rt_ns, "track_near_dist", 20.0);
        track_far_dist_   = dparam(def_ns, rt_ns, "track_far_dist",  30.0);
        track_creep_in_   = dparam(def_ns, rt_ns, "track_creep_in",   9.0);
        track_creep_out_  = dparam(def_ns, rt_ns, "track_creep_out", 11.0);
        // far 档 EGO 引导点距离: 须小于 EGO planning_horizon(25m) 留余量,
        // 大于 8m/s 的刹车距离 v^2/2a(≈6.4m)。0 = 关闭, 直接发站位点。
        track_far_goal_dist_ = dparam(def_ns, rt_ns, "track_far_goal_dist", 22.0);
        // 跟踪高度以 YAML 为准：本机覆盖优先，其次公共默认值；不另设固定高度或钳位。
        const std::string track_z_key = nh_.hasParam(rt_ns + "track_z")
            ? rt_ns + "track_z" : def_ns + "track_z";
        if (!nh_.getParam(track_z_key, track_z_) ||
            !std::isfinite(track_z_) || track_z_ <= 0.0)
            throw std::invalid_argument("track_z must be a finite positive number in YAML: " +
                                        track_z_key);
        ROS_INFO("[航线] 跟踪高度 %.2fm，来自 YAML 参数 %s",
                 track_z_, track_z_key.c_str());
        // 站位参数：半径、速度前馈、近身退让，以及重规划的位移和时间门限。
        standoff_radius_     = dparam(def_ns, rt_ns, "standoff_radius",       5.0);
        standoff_move_       = dparam(def_ns, rt_ns, "standoff_move_thresh",  0.25);
        standoff_min_dt_     = dparam(def_ns, rt_ns, "standoff_min_interval", 0.3);
        standoff_force_d_    = dparam(def_ns, rt_ns, "standoff_force_dist",   2.5);
        // lead=0 时站位保持在目标周围的距离环上。
        standoff_lead_sec_   = dparam(def_ns, rt_ns, "standoff_lead_sec",     0.0);
        standoff_lead_max_   = dparam(def_ns, rt_ns, "standoff_lead_max",     1.5);
        standoff_lead_tau_   = dparam(def_ns, rt_ns, "standoff_lead_tau",     0.5);
        standoff_retreat_    = dparam(def_ns, rt_ns, "standoff_retreat_dist", 4.0);
        // 跟随段动态限速: cap = clamp(行人速度 + margin, floor, 档位上限)
        track_follow_margin_ = dparam(def_ns, rt_ns, "track_follow_margin",   0.35);
        track_follow_floor_  = dparam(def_ns, rt_ns, "track_follow_floor",    1.0);
        // 接近段/跟随段的分界: 距站位超过这个余量用档位上限全速接近
        track_follow_gap_    = dparam(def_ns, rt_ns, "track_follow_gap",      3.0);
        // 限速平滑 (治跟踪时速度剧烈变化): cap 斜率限制, 升快降慢
        track_cap_slew_up_   = dparam(def_ns, rt_ns, "track_cap_slew_up",     1.2);
        track_cap_slew_down_ = dparam(def_ns, rt_ns, "track_cap_slew_down",   0.6);
        track_acc_soft_      = dparam(def_ns, rt_ns, "track_acc_soft",        0.8);
        // 跟踪 PID 速度环 (近距离平滑跟随, 详见类内成员与 stepTrack)
        {
            bool b = true;
            nh_.param<bool>(def_ns + "track_vel_loop_enable", b, true);
            nh_.param<bool>(rt_ns + "track_vel_loop_enable", b, b);
            vel_loop_enable_ = b;
        }
        vel_loop_kp_         = dparam(def_ns, rt_ns, "track_vel_loop_kp",          1.2);
        vel_loop_ki_         = dparam(def_ns, rt_ns, "track_vel_loop_ki",          0.10);
        vel_loop_kd_         = dparam(def_ns, rt_ns, "track_vel_loop_kd",          0.20);
        vel_loop_err_tau_    = dparam(def_ns, rt_ns, "track_vel_loop_err_tau",     0.3);
        vel_loop_ff_dead_    = dparam(def_ns, rt_ns, "track_vel_loop_ff_dead",     0.35);
        vel_loop_acc_max_    = dparam(def_ns, rt_ns, "track_vel_loop_acc_max",     1.5);
        vel_loop_vmax_       = dparam(def_ns, rt_ns, "track_vel_loop_vmax",        3.0);
        vel_loop_enter_dist_ = dparam(def_ns, rt_ns, "track_vel_loop_enter_dist",  9.0);
        vel_loop_exit_dist_  = dparam(def_ns, rt_ns, "track_vel_loop_exit_dist",  12.0);
        vel_loop_enter_err_  = dparam(def_ns, rt_ns, "track_vel_loop_enter_err",   3.5);
        vel_loop_exit_err_   = dparam(def_ns, rt_ns, "track_vel_loop_exit_err",    5.0);
        vel_loop_lookahead_  = dparam(def_ns, rt_ns, "track_vel_loop_lookahead",   2.5);
        vel_loop_clear_      = dparam(def_ns, rt_ns, "track_vel_loop_clear",       0.55);
        vel_loop_samples_    = (int)dparam(def_ns, rt_ns, "track_vel_loop_samples", 7.0);
        vel_loop_w_turn_     = dparam(def_ns, rt_ns, "track_vel_loop_w_turn",      0.6);
        // 跟踪避障 (原始雷达净空场; 只在有障碍风险时介入, 不改上面 PID 参数)
        {
            bool b = true;
            nh_.param<bool>(def_ns + "track_avoid_enable", b, true);
            nh_.param<bool>(rt_ns + "track_avoid_enable", b, b);
            avoid_enable_ = b;
        }
        avoid_res_          = dparam(def_ns, rt_ns, "track_avoid_resolution",   0.2);
        avoid_memory_sec_   = dparam(def_ns, rt_ns, "track_avoid_memory_sec",   3.0);
        // = body_band_cloud 的 memory_sec: 雷达这么久内扫到过的格不算屋檐/顶棚
        avoid_overhead_lidar_sec_ = dparam(def_ns, rt_ns, "track_avoid_overhead_lidar_sec", 15.0);
        avoid_field_half_   = dparam(def_ns, rt_ns, "track_avoid_field_half",  14.0);
        avoid_thin_extent_  = dparam(def_ns, rt_ns, "track_avoid_thin_extent",  0.8);
        guard_.d_hard       = dparam(def_ns, rt_ns, "track_avoid_d_hard",       1.5);
        guard_.d_safe       = dparam(def_ns, rt_ns, "track_avoid_d_safe",       2.5);
        guard_.lag          = dparam(def_ns, rt_ns, "track_avoid_lag",          0.3);
        guard_.horizon      = dparam(def_ns, rt_ns, "track_avoid_horizon",      1.2);
        guard_.brake_acc    = dparam(def_ns, rt_ns, "track_avoid_brake_acc",    2.0);
        guard_.acc          = vel_loop_acc_max_;   // 与 PID 末级斜率限幅一致
        avoid_block_sec_    = dparam(def_ns, rt_ns, "track_avoid_block_sec",    0.6);
        avoid_holdoff_sec_  = dparam(def_ns, rt_ns, "track_avoid_holdoff_sec",  1.5);
        standoff_obs_.wall_clear = dparam(def_ns, rt_ns, "standoff_wall_clear", 2.8);
        standoff_obs_.los_clear  = dparam(def_ns, rt_ns, "standoff_los_clear",  0.35);
        {
            bool b = true;
            nh_.param<bool>(def_ns + "standoff_corridor_enable", b, true);
            nh_.param<bool>(rt_ns + "standoff_corridor_enable", b, b);
            standoff_obs_.corridor = b;
        }
        standoff_obs_.corridor_clear = dparam(def_ns, rt_ns, "standoff_corridor_clear", 1.6);
        standoff_obs_.cross_radius   = dparam(def_ns, rt_ns, "standoff_cross_radius",   3.5);
        if (!(avoid_res_ >= 0.05 && avoid_res_ <= 0.5)) avoid_res_ = 0.2;
        if (!(guard_.d_hard > 0.3)) guard_.d_hard = 1.5;
        if (!(guard_.d_safe > guard_.d_hard)) guard_.d_safe = guard_.d_hard + 1.0;
        standoff_obs_.wall_min = std::min(standoff_obs_.wall_clear, guard_.d_hard + 0.6);
        // 走廊档下限不低于速度过滤的硬净空 + 0.3, 降级悬崖介于两者之间
        standoff_obs_.corridor_clear = std::min(standoff_obs_.wall_clear,
            std::max(standoff_obs_.corridor_clear, guard_.d_hard + 0.3));
        standoff_obs_.min_radius = standoff_retreat_;
        standoff_obs_.corridor_cliff = std::max(guard_.d_hard + 0.2,
            std::min(standoff_obs_.corridor_clear - 0.3, standoff_obs_.wall_min));
        if (avoid_enable_)
            ROS_INFO("[避障] 跟踪避障: 硬净空 %.2fm 期望 %.2fm 前视 %.1fs+滞后 %.2fs, "
                     "站位离墙 %.2fm, 记忆 %.1fs",
                     guard_.d_hard, guard_.d_safe, guard_.horizon, guard_.lag,
                     standoff_obs_.wall_clear, avoid_memory_sec_);
        // 跟踪卡死自愈: 指令新鲜但每级这么久没动且没到站位 -> 缩半径 -> 缩到 0
        // -> 贴身悬停/reject (三级阶梯, 见 stepTrack)
        track_stuck_sec_     = dparam(def_ns, rt_ns, "track_stuck_sec",       4.0);

        arrive_radius_     = dparam(def_ns, rt_ns, "arrive_radius", 1.5);
        goal_repub_period_ = dparam(def_ns, rt_ns, "goal_repub_period", 2.0);
        // 航点余量作用于已膨胀地图，避免重复按原始障碍半径理解。
        waypoint_obstacle_clear_ = dparam(def_ns, rt_ns, "waypoint_obstacle_clear", 0.5);
        patrol_yaw_rate_max_ = dparam(def_ns, rt_ns, "patrol_yaw_rate_max", 45.0);
        start_delay_ = dparam(def_ns, rt_ns, "start_delay", 0.0);

        const int idx = drone_id_ - 1;
        const double fallback_z = (idx >= 0 && idx < 6) ? kDefaultRoutes[idx].z : 4.5;
        route_z_ = dparam(def_ns, rt_ns, "z", fallback_z);

        route_loop_ = true;
        {
            bool b;
            if (nh_.getParam(rt_ns + "loop", b) || nh_.getParam(def_ns + "loop", b))
                route_loop_ = b;
        }

        // 航线默认机头模式
        YawMode def_yaw_mode = YawMode::COURSE;
        {
            std::string s;
            if (nh_.getParam(rt_ns + "yaw_mode", s) || nh_.getParam(def_ns + "yaw_mode", s))
                def_yaw_mode = parseYawMode(s);
        }
        const double def_yaw_off = dparam(def_ns, rt_ns, "yaw_offset_deg", 0.0);

        // entry_waypoints 只进场一次，不参与闭环，也不作为跟踪后的归航目标。
        auto parsePoints = [&](XmlRpc::XmlRpcValue& wps, std::vector<Waypoint>& out) {
            if (wps.getType() != XmlRpc::XmlRpcValue::TypeArray) return false;
            for (int i = 0; i < wps.size(); ++i) {
                XmlRpc::XmlRpcValue& w = wps[i];
                if (w.getType() != XmlRpc::XmlRpcValue::TypeStruct ||
                    !w.hasMember("x") || !w.hasMember("y")) return false;
                auto numeric = [](const XmlRpc::XmlRpcValue& v) {
                    return v.getType() == XmlRpc::XmlRpcValue::TypeInt ||
                           v.getType() == XmlRpc::XmlRpcValue::TypeDouble;
                };
                if (!numeric(w["x"]) || !numeric(w["y"])) return false;
                Waypoint p;
                p.x = xmlD(w["x"]);
                p.y = xmlD(w["y"]);
                p.z     = w.hasMember("z")     ? xmlD(w["z"])     : -1.0;
                p.speed = w.hasMember("speed") ? xmlD(w["speed"]) : -1.0;
                if (!std::isfinite(p.x) || !std::isfinite(p.y) ||
                    !std::isfinite(p.z) || !std::isfinite(p.speed)) return false;
                p.yaw_mode = def_yaw_mode;
                if (w.hasMember("yaw_mode")) {
                    p.yaw_mode = parseYawMode(static_cast<std::string>(w["yaw_mode"]));
                }
                p.yaw_offset_deg = w.hasMember("yaw_offset_deg") ? xmlD(w["yaw_offset_deg"])
                                                                : def_yaw_off;
                if (w.hasMember("yaw_deg")) {
                    p.yaw_deg = xmlD(w["yaw_deg"]);
                    if (!w.hasMember("yaw_mode")) p.yaw_mode = YawMode::FIXED;
                }
                out.push_back(p);
            }
            return true;
        };
        XmlRpc::XmlRpcValue wps;
        bool ok = nh_.getParam(rt_ns + "waypoints", wps) && parsePoints(wps, route_) &&
                  route_.size() >= 2;

        if (!ok || route_.size() < 2) {
            route_.clear();
            if (patrol_map_ != "base0") {
                throw std::invalid_argument("Invalid patrol route for " + patrol_map_ +
                                            "; refusing the base0 fallback: " + rt_ns);
            }
            // !! 绝不能"读不到航线就静默悬停" !!  一架机在空中什么都不干比崩掉更难发现。
            ROS_ERROR("[航线] 参数 %swaypoints 缺失或不合法, 改用内置兜底航线 (drone_id=%d)",
                      rt_ns.c_str(), drone_id_);
            if (idx < 0 || idx >= 6) {
                ROS_FATAL("[航线] drone_id=%d 超出 1..6, 无兜底航线可用", drone_id_);
                return;
            }
            for (int i = 0; i < 4; ++i) {
                Waypoint p;
                p.x = kDefaultRoutes[idx].wp[i][0];
                p.y = kDefaultRoutes[idx].wp[i][1];
                p.yaw_mode = YawMode::COURSE;
                route_.push_back(p);
            }
            route_z_    = kDefaultRoutes[idx].z;
            start_delay_ = kDefaultRoutes[idx].start_delay;
        }

        XmlRpc::XmlRpcValue entry_wps;
        if (nh_.getParam(rt_ns + "entry_waypoints", entry_wps)) {
            std::vector<Waypoint> entry;
            if (!parsePoints(entry_wps, entry))
                throw std::invalid_argument("Invalid entry_waypoints: " + rt_ns);
            route_loop_start_ = entry.size();
            route_.insert(route_.begin(), entry.begin(), entry.end());
        }
        ROS_INFO("[航线] map=%s 进场=%zu点 巡航=%zu点 原地爬升=%d",
                 patrol_map_.c_str(), route_loop_start_, route_.size()-route_loop_start_,
                 (int)climb_in_place_);

        // 固定高度任务不静默改写 YAML，也不接受航点各自改变巡航高度。
        if (!std::isfinite(route_z_) || route_z_ <= 0.0 ||
            route_z_ < min_expl_z_ || route_z_ > max_expl_z_)
            throw std::invalid_argument("YAML patrol z must be finite, positive and within "
                                        "[min_expl_z, max_expl_z]");
        if (track_z_ >= z_soft_limit_ || route_z_ >= z_soft_limit_)
            throw std::invalid_argument("YAML flight heights must be below z_soft_limit");
        for (auto& p : route_) {
            if (p.z >= 0.0 && std::fabs(p.z - route_z_) > 1e-6)
                throw std::invalid_argument("waypoint z must match the YAML patrol z "
                                            "in fixed-altitude patrol mode");
            p.z = route_z_;
        }
    }

    double dparam(const std::string& def_ns, const std::string& rt_ns,
                  const std::string& key, double fallback) const {
        double v;
        if (nh_.getParam(rt_ns + key, v)) return v;       // 本机覆盖优先
        if (nh_.getParam(def_ns + key, v)) return v;
        return fallback;
    }
    static double xmlD(XmlRpc::XmlRpcValue& v) {
        if (v.getType() == XmlRpc::XmlRpcValue::TypeInt)    return static_cast<int>(v);
        if (v.getType() == XmlRpc::XmlRpcValue::TypeDouble) return static_cast<double>(v);
        return 0.0;
    }
    static YawMode parseYawMode(const std::string& s) {
        if (s == "fixed")         return YawMode::FIXED;
        if (s == "course_offset") return YawMode::COURSE_OFFSET;
        return YawMode::COURSE;
    }

    // 回调
    void missionCompleteCb(const std_msgs::Bool::ConstPtr& msg) {
        if (msg->data && !mission_complete_) {
            ROS_INFO("[任务] 收到内部 mission_complete，仅作状态记录；"
                     "不主动降落，继续运行直到裁判结束比赛");
        }
        mission_complete_ = msg->data;
    }

    // 小工具
    // 角度规整到 (-180, 180]: 自转目标航向是累加出来的, 不规整会一路涨到几千度
    static double wrapDeg(double d) {
        while (d > 180.0) d -= 360.0;
        while (d <= -180.0) d += 360.0;
        return d;
    }

    // 角度规整到 (-pi, pi] (rad). 速度环方向采样用。
    static double wrapRad(double r) {
        while (r > M_PI) r -= 2.0 * M_PI;
        while (r <= -M_PI) r += 2.0 * M_PI;
        return r;
    }

    // 巡逻沿用原速率限幅。跟踪使用独立的视线参考生成器，并同步更新
    // yaw_cmd_prev_deg_，保证退出跟踪后的巡逻角度参考连续。
    double rateLimitYaw(double want_deg, double rate_max, double deadband_deg) {
        if (!yaw_cmd_inited_) {
            yaw_cmd_prev_deg_ = pose_sub_->getCurrentYaw() * 180.0 / M_PI;
            yaw_cmd_inited_ = true;
        }
        double d = wrapDeg(want_deg - yaw_cmd_prev_deg_);
        if (std::fabs(d) < deadband_deg) d = 0.0;
        const double step = rate_max * 0.02;        // 巡逻50Hz
        if (d >  step) d =  step;
        if (d < -step) d = -step;
        yaw_cmd_prev_deg_ = wrapDeg(yaw_cmd_prev_deg_ + d);
        return yaw_cmd_prev_deg_;
    }

    // 发给 ego 的两个出口
    void publishEgoSpeed(double vel, double acc) {
        // 速度按 0.5m/s 量化，减少速度轻微变化导致的重规划。
        const double q_vel = std::round(vel / 0.5) * 0.5;
        if (std::fabs(q_vel - ego_speed_v_) < 1e-6 &&
            std::fabs(acc - ego_speed_a_) < 1e-3) return;
        std_msgs::Float32MultiArray m;
        m.data.push_back(static_cast<float>(q_vel));
        m.data.push_back(static_cast<float>(acc));
        ego_speed_pub_.publish(m);
        ego_speed_v_ = q_vel;
        ego_speed_a_ = acc;
        ROS_INFO("[ego] 限速 -> %.2f m/s (acc %.2f)", q_vel, acc);
    }

    // 逼近使用距离档限速，伴飞限制到目标速度；滞回与非对称斜率抑制跳变。
    void publishTrackSpeed(int tier, double v_person, double d_t,
                           double wx, double wy) {
        if (tier <= 0) {
            publishEgoSpeed(track_speed_far_, track_acc_far_);
            track_cap_init_ = false;   // 回 far 档后重新起步, 下次进 mid 从当前值起坡
            return;
        }
        const double tier_cap = (tier == 1) ? track_speed_near_ : track_speed_creep_;
        double acc      = (tier == 1) ? track_acc_near_ : track_acc_creep_;
        double cap;
        // 以到实际站位点的距离判断逼近，不能只看与目标的径向距离差。
        double gap_cur;
        if (standoff_valid_) {
            gap_cur = std::hypot(wx - goal_x_, wy - goal_y_);
        } else {
            gap_cur = d_t - standoff_radius_;   // 站位点还没算出来时退回旧近似
        }
        if (!track_follow_seg_) {
            if (gap_cur < track_follow_gap_) track_follow_seg_ = true;
        } else {
            if (gap_cur > track_follow_gap_ + 1.5) track_follow_seg_ = false;
        }
        if (!track_follow_seg_) {
            cap = tier_cap;                          // 接近段: 档位上限全速
        } else {
            const double margin   = (tier == 1) ? 0.8 : track_follow_margin_;
            const double floor_v  = (tier == 1) ? 1.4 : track_follow_floor_;
            cap = std::min(tier_cap, v_person + margin);
            cap = std::max(cap, floor_v);            // 跟随段: 贴人速
        }
        // cap 斜率限制 (不对称: 升快降慢) —— 50Hz
        if (!track_cap_init_) {
            track_cap_smooth_ = cap;
            track_cap_init_ = true;
        } else {
            const double slew = (cap < track_cap_smooth_) ? track_cap_slew_down_
                                                          : track_cap_slew_up_;
            const double max_dv = slew * 0.02;
            double dv = cap - track_cap_smooth_;
            if (dv >  max_dv) dv =  max_dv;
            if (dv < -max_dv) dv = -max_dv;
            track_cap_smooth_ += dv;
            if (dv < -1e-6) acc = std::min(acc, track_acc_soft_);  // 降速用软加速度
        }
        publishEgoSpeed(track_cap_smooth_, acc);
    }

    // 巡逻/归航时发 true(关闭碰撞急停, 马路净空), 跟踪时发 false(恢复避障)。
    void publishDisableCollision(bool disable) {
        if (disable_collision_inited_ && disable_collision_cur_ == disable) return;
        std_msgs::Bool m;
        m.data = disable;
        disable_collision_pub_.publish(m);
        disable_collision_cur_ = disable;
        disable_collision_inited_ = true;
    }

    double flightAltitude() const {
        return state_ == PatrolState::TRACK ? track_z_ : route_z_;
    }

    // 以 10Hz 发布目标高度和跟踪激活状态；巡逻 active=0，停用深度与高度带输出。
    void publishBandRequest() {
        const ros::Time now = ros::Time::now();
        if (!last_band_request_.isZero() && now >= last_band_request_ &&
            (now - last_band_request_).toSec() < 0.1) return;
        last_band_request_ = now;
        const bool active = state_ == PatrolState::TRACK ||
                            (state_ == PatrolState::TRANSIT && transit_from_track_);
        std_msgs::Float32MultiArray m;
        m.data = {static_cast<float>(flightAltitude()), active ? 1.0f : 0.0f};
        band_request_pub_.publish(m);
    }

    // 每拍只生成一次 Z PVA。水平模式切换不重启它，目标只随飞行状态改变。
    void updateAltitudeReference(double cz) {
        const double now = ros::Time::now().toSec();
        if (!altitude_reference_ready_) {
            // 保持原起飞行为；失联/位姿恢复后从实测高度重新平滑接管。
            altitude_hold_.reset(now, has_flown_ ? cz : route_z_);
            altitude_reference_ready_ = true;
        }
        altitude_hold_.setTarget(now, flightAltitude());
        const auto reference = altitude_hold_.step(now);
        control_->set_altitude_reference(reference.z, reference.vz, reference.az);
    }

    // 发送世界坐标目标和末端速度。
    void publishEgoGoal(double wx, double wy, double wz, bool force,
                        double vel_x = 0.0, double vel_y = 0.0) {
        // 外部目标记录真实飞行高度；plane_mode 的 EGO 将它投影到固定虚拟
        // 二维平面，与雷达地图/六机轨迹共享平面一致，仅提供水平规划。
        wz = flightAltitude();
        if (!force && goal_valid_ &&
            std::fabs(wz - goal_z_) < 1e-6 &&
            std::hypot(wx - goal_x_, wy - goal_y_) < goal_move_thresh_) {
            return;
        }
        geometry_msgs::PoseStamped g;
        g.header.stamp = ros::Time::now();
        g.header.frame_id = "world";
        g.pose.position.x = wx;
        g.pose.position.y = wy;
        g.pose.position.z = wz;
        g.pose.orientation.w = 1.0;
        ego_goal_pub_.publish(g);
        goal_x_ = wx; goal_y_ = wy; goal_z_ = wz;
        goal_valid_ = true;
        last_goal_pub_ = ros::Time::now();
        if (state_ == PatrolState::TRACK)
            track_planner_wait_.goalSent(last_goal_pub_.toSec());

        // 目标速度同步发 (latch), ego 用它当轨迹终点速度。
        geometry_msgs::TwistStamped tv;
        tv.header.stamp = g.header.stamp;
        tv.header.frame_id = "world";
        tv.twist.linear.x = vel_x;
        tv.twist.linear.y = vel_y;
        tv.twist.linear.z = 0.0;
        ego_goal_vel_pub_.publish(tv);
    }

    // 优先使用关联框的图像视线；失检时回退坐标视线，yaw 与 yaw_rate 同步。
    bool computeTrackYaw(double cx, double cy, double& yaw_deg) {
        track_yaw_rate_rad_ = 0.0;
        const int mode = track_mode_sub_->mode();
        if (mode < 1) {
            track_yaw_ready_ = false;
            return false;
        }

        // !! 本地帧 -> 世界帧 !!  pose_sub_ 是 PX4 本地帧(原点=spawn), 目标是世界帧。
        // 这里最容易错: 忘了加 init_ 就会算出一个偏十几米的方位(1 号机偏 143 度)。
        const double wx = cx + init_x_;
        const double wy = cy + init_y_;

        double tx, ty, tvx, tvy, age, observation_age, state_age, velocity_variance;
        bool bearing_only = false;
        std::string assignment;
        if (!track_target_sub_->get(tx, ty, tvx, tvy, age, &assignment,
                                   &observation_age, &bearing_only, &state_age,
                                   &velocity_variance) || age > 1.0) {
            track_yaw_ready_ = false;
            return false;
        }

        const double now = ros::Time::now().toSec();
        const double actual_yaw = pose_sub_->getCurrentYaw();
        if (!track_yaw_ready_ || track_yaw_assignment_ != assignment ||
            now < track_yaw_last_t_ || now - track_yaw_last_t_ > 0.5) {
            track_yaw_controller_.reset(actual_yaw, now);
            track_yaw_assignment_ = assignment;
            track_yaw_ready_ = true;
        }
        track_yaw_last_t_ = now;

        double px = tx, py = ty;
        const bool stale = observation_age > track_measurement_timeout_ ||
                           state_age > target_state_timeout_;
        if (stale) {
            // 观测断流时保留当前认领，朝最后可信重捕点转头；不持续外推旧速度。
            if (!reacquire_target_valid_ || reacquire_target_assignment_ != assignment) {
                reacquire_target_x_ = tx;
                reacquire_target_y_ = ty;
                reacquire_target_assignment_ = assignment;
                reacquire_target_valid_ = true;
            }
            px = reacquire_target_x_;
            py = reacquire_target_y_;
        } else {
            reacquire_target_valid_ = false;
        }

        const double rx = px - wx, ry = py - wy;
        const double range2 = rx * rx + ry * ry;
        const double fallback_bearing = range2 > 0.25 ? std::atan2(ry, rx) : actual_yaw;
        double fallback_rate = 0.0, dvx = 0.0, dvy = 0.0;
        const bool have_rate = !stale && !bearing_only && range2 > 0.25 &&
            observation_age < track_stale_sec_ && velocity_variance < 4.0 &&
            local_velocity_sub_->get(dvx, dvy, track_yaw_velocity_timeout_);
        if (have_rate) {
            // 降权陈旧/不确定的KF速度；必须使用目标相对于本机的水平速度。
            const double confidence = (1.0 - observation_age / track_stale_sec_) /
                                      (1.0 + velocity_variance);
            fallback_rate = confidence * (rx * (tvy - dvy) - ry * (tvx - dvx)) / range2;
        }

        track_msgs::TrackingView view;
        const bool have_view = track_view_sub_->get(assignment, view);
        if (have_view)
            track_yaw_controller_.observe(view.bearing, view.header.stamp.toSec());
        const auto command = track_yaw_controller_.step(
            now, fallback_bearing, fallback_rate, have_rate);
        yaw_deg = command.yaw * 180.0 / M_PI;
        track_yaw_rate_rad_ = command.rate;
        // 巡逻仍使用原限速器；保持从跟踪参考到巡逻的角度连续。
        yaw_cmd_prev_deg_ = yaw_deg;
        yaw_cmd_inited_ = true;

        const float missing = std::numeric_limits<float>::quiet_NaN();
        std_msgs::Float32MultiArray diag;
        diag.data = {static_cast<float>(actual_yaw * 180.0 / M_PI),
                     static_cast<float>(yaw_deg),
                     static_cast<float>(command.rate * 180.0 / M_PI),
                     command.visual_active ? 1.0f : 0.0f,
                     have_view ? static_cast<float>(view.horizontal_error_px) : missing,
                     have_view ? static_cast<float>(now - view.header.stamp.toSec()) : missing,
                     static_cast<float>(fallback_bearing * 180.0 / M_PI)};
        yaw_diag_pub_.publish(diag);
        return true;
    }

    // 站位 = 目标位置 + 速度前馈 + 距离环偏移，保持在当前同侧并保护近身退让。
    // 候选同时检查边界、友机间距及感知障碍。
    double standoffPeerClearance(double cxx, double cyy) const {
        double clearance = std::numeric_limits<double>::infinity();
        for (const auto& ps : peer_subs_) {
            double pxp, pyp;
            if (!ps->get(pxp, pyp)) continue;
            clearance = std::min(clearance, std::hypot(cxx - pxp, cyy - pyp));
        }
        return clearance;
    }

    bool selectStandoff(double wx, double wy, double tx, double ty,
                        double tvx, double tvy,
                        double& sx, double& sy) {
        static const double kOffsetsDeg[12] = { 0, 30, -30, 60, -60, 90,
                                               -90, 120, -120, 150, -150, 180 };
        const double d_person = std::hypot(wx - tx, wy - ty);
        // 卡住时缩小站位半径，仍保留近身退让保护。
        double r_eff = standoff_radius_ * standoff_r_scale_;
        if (d_person < standoff_retreat_) {
            r_eff += standoff_retreat_ - d_person;   // 人越近, 目标点越远, 飞机退得越快
        }
        // 沿目标指向本机的同侧选站位。
        diag_r_eff_ = r_eff;
        standoff_az_pick_ = -1;
        double az0 = std::atan2(wy - ty, wx - tx) +
                     reacquire_off_deg_ * M_PI / 180.0;
        // 先滤波基础方位再搜索候选，避免被否决的候选反复改变滤波状态。
        double az0_deg = wrapDeg(az0 * 180.0 / M_PI);
        diag_az0_raw_deg_ = az0_deg;      // 诊断用: 滤波前基准方位
        if (!standoff_az_valid_) {
            standoff_az_prev_ = az0_deg;
            standoff_az_valid_ = true;
        } else {
            const double d = wrapDeg(az0_deg - standoff_az_prev_);
            if (std::fabs(d) > standoff_az_deadband_deg_) {
                const double exceed = d - std::copysign(standoff_az_deadband_deg_, d);
                standoff_az_prev_ = wrapDeg(standoff_az_prev_ + exceed * standoff_az_alpha_);
            }
            // 死区内: 保持 standoff_az_prev_ 不变
        }
        az0 = standoff_az_prev_ * M_PI / 180.0;

        // 目标速度低通后用于站位前馈。
        const double lead_dt = lead_valid_
            ? std::min(1.0, (ros::Time::now() - lead_last_t_).toSec()) : 1e9;
        if (!lead_valid_ || lead_dt > 1.0) {
            lead_vx_ = tvx;
            lead_vy_ = tvy;
        } else {
            const double lead_alpha = lead_dt / (lead_dt + standoff_lead_tau_);
            lead_vx_ += lead_alpha * (tvx - lead_vx_);
            lead_vy_ += lead_alpha * (tvy - lead_vy_);
        }
        lead_valid_ = true;
        lead_last_t_ = ros::Time::now();

        // 按位移模长限制前馈，保持速度方向。
        double lead_x = lead_vx_ * standoff_lead_sec_;
        double lead_y = lead_vy_ * standoff_lead_sec_;
        const double lead_mag = std::hypot(lead_x, lead_y);
        if (lead_mag > standoff_lead_max_) {
            const double k = standoff_lead_max_ / lead_mag;
            lead_x *= k;
            lead_y *= k;
        }

        // 保持已选安全方位；恢复首选需额外净空，避免友机在门限附近路过时
        // 候选每拍跳回首选。已选点失去硬净空/越界则必须立刻重选。
        std::array<fly_px4::StandoffCandidate, 12> candidates;
        const auto fillRing = [&](double r) {
            for (int k = 0; k < 12; ++k) {
                const double az = az0 + kOffsetsDeg[k] * M_PI / 180.0;
                auto& c = candidates[k];
                c.x = tx + lead_x + r * std::cos(az);
                c.y = ty + lead_y + r * std::sin(az);
                c.inside_bounds = kTrackingMapBounds.contains(c.x, c.y);
                c.peer_clearance = standoffPeerClearance(c.x, c.y);
            }
        };
        int pick = -1;
        int ring = 0;
        diag_standoff_wall_ = diag_standoff_los_ = -1.0;
        if (refreshAvoidField(wx, wy)) {
            // 障碍感知: 站位点离墙够远、站位->行人视线不穿楼、优先选本机能直接
            // 飞到的一侧。附近无障碍时与下面原选法逐拍一致。
            std::array<fly_px4::StandoffObstacleInfo, 12> info;
            std::array<double, 12> offsets;
            for (int k = 0; k < 12; ++k) offsets[k] = kOffsetsDeg[k];
            bool degraded = false;
            // 外扩站位环受 standoff_band_max_radius 和深度可达距离限制。
            int rings = 1;
            if (avoid_field_.hasOverhead()) {
                while (rings < 3 && r_eff + kStandoffBandRingStep * rings
                                     <= standoff_band_max_radius_ + 1e-6) ++rings;
            }
            bool any_band_blocked = false;
            bool corridor_tier = false;
            for (ring = 0; ring < rings; ++ring) {
                const double r_ring = r_eff + kStandoffBandRingStep * ring;
                fillRing(r_ring);
                any_band_blocked = false;
                const int prev_k = ring == standoff_ring_ ? standoff_candidate_ : -1;
                for (int k = 0; k < 12; ++k) {
                    info[k] = fly_px4::evaluateStandoffObstacles(avoid_field_,
                        candidates[k].x, candidates[k].y, tx, ty, wx, wy, standoff_obs_);
                }
                // 首选附近没有常规候选时，才尝试沿走廊中线细化净空。
                if (standoff_obs_.corridor &&
                    !fly_px4::standoffNearATier(candidates, info, offsets, prev_k,
                                                standoff_peer_clear_, standoff_obs_)) {
                    for (int k = 0; k < 12; ++k) {
                        double rx, ry;
                        if (!fly_px4::refineStandoffPoint(avoid_field_, tx + lead_x, ty + lead_y,
                                r_ring, az0 + kOffsetsDeg[k] * M_PI / 180.0, wx, wy, tx, ty,
                                standoff_obs_, rx, ry)) continue;
                        auto& c = candidates[k];
                        c.x = rx;
                        c.y = ry;
                        c.inside_bounds = kTrackingMapBounds.contains(c.x, c.y);
                        c.peer_clearance = standoffPeerClearance(c.x, c.y);
                        info[k] = fly_px4::evaluateStandoffObstacles(avoid_field_,
                            c.x, c.y, tx, ty, wx, wy, standoff_obs_);
                    }
                }
                for (int k = 0; k < 12; ++k) any_band_blocked |= info[k].band_blocked;
                pick = fly_px4::chooseObstacleAwareStandoff(candidates, info, offsets,
                    prev_k, standoff_peer_clear_,
                    standoff_return_margin_, standoff_obs_, &degraded, &corridor_tier);
                if (pick >= 0 || !any_band_blocked) break;
            }
            if (pick >= 0) {
                diag_standoff_wall_ = std::min(99.0, info[pick].wall_clearance);
                diag_standoff_los_ = std::min(99.0, info[pick].los_clearance);
                if (pick != standoff_candidate_ && (pick != 0 || standoff_candidate_ > 0))
                    ROS_INFO_THROTTLE(0.5, "[避障] 站位换到 %+.0f° (离墙 %.1fm, 视线余量 %.2fm, 直飞净空 %.1fm)%s%s%s",
                             kOffsetsDeg[pick], std::min(99.0, info[pick].wall_clearance),
                             std::min(99.0, info[pick].los_clearance),
                             std::min(99.0, info[pick].path_clearance),
                             degraded ? " [无合格候选, 取最好]" : "",
                             corridor_tier ? " [走廊]" : "",
                             info[pick].crosses_person ? " [绕到人另一侧]" : "");
                if (corridor_tier && !standoff_corridor_tier_)
                    ROS_INFO("[避障] 进入走廊档: 站位离墙 %.1fm (下限 %.1fm), 沿通道中线跟随",
                             std::min(99.0, info[pick].wall_clearance), standoff_obs_.corridor_clear);
                if (ring > 0) {
                    diag_r_eff_ = r_eff + kStandoffBandRingStep * ring;
                    ROS_INFO_THROTTLE(1.0, "[避障] 半径 %.1fm 的站位都在高度带障碍(屋檐/顶棚)后, "
                                      "外扩到 %.1fm 在棚外观察", r_eff, diag_r_eff_);
                }
            } else {
                ring = 0;
            }
            standoff_band_blocked_ = pick < 0 && any_band_blocked;
            if (degraded && !standoff_obs_degraded_)
                ROS_WARN("[避障] 半径 %.1fm 上没有离墙/视线都合格的站位, 降级取最好候选", r_eff);
            standoff_obs_degraded_ = degraded;
            standoff_corridor_tier_ = pick >= 0 && corridor_tier;
        } else {
            standoff_corridor_tier_ = false;
            fillRing(r_eff);
            pick = fly_px4::chooseStandoffCandidate(candidates,
                standoff_candidate_, standoff_peer_clear_, standoff_return_margin_);
            standoff_band_blocked_ = false;
        }
        standoff_candidate_ = pick;
        standoff_ring_ = ring;
        if (pick < 0) return false;
        sx = candidates[pick].x;
        sy = candidates[pick].y;
        standoff_az_pick_ = pick;
        return true;
    }

    // 跟踪避障净空场: 雷达新鲜才可用, 10Hz 重建 (同一拍内站位/速度过滤共用)。
    // 返回 false = 不可用, 调用方走原有逻辑 (原 inflate 地图方向采样 / 原站位选法)。
    bool refreshAvoidField(double wx, double wy) {
        if (!avoid_enable_ || !lidar_surface_ || !lidar_surface_->fresh(0.5)) {
            avoid_field_.invalidate();
            return false;
        }
        const ros::Time now = ros::Time::now();
        if (!avoid_field_.valid() || avoid_field_t_.isZero() || now < avoid_field_t_ ||
            (now - avoid_field_t_).toSec() >= 0.1) {
            lidar_surface_->buildField(avoid_field_, wx, wy, avoid_field_half_,
                                       avoid_memory_sec_, avoid_thin_extent_);
            avoid_field_t_ = now;
        }
        return avoid_field_.valid();
    }

    void publishReject(int reason) {
        // /tracking/reject_N: 任意值都表示"这个目标我接不了", fusion 会拉黑 2s 并改派
        if (!last_reject_.isZero() && (ros::Time::now() - last_reject_).toSec() < 2.0) return;
        std_msgs::Int8 m;
        m.data = static_cast<int8_t>(reason);
        reject_pub_.publish(m);
        last_reject_ = ros::Time::now();
        ROS_ERROR("[跟踪] 放弃当前目标 (reason=%d), 已发 /tracking/reject_%d", reason, drone_id_);
    }

    // 归航投影到最近闭环线段，优先沿航线向前；不以当前机头决定方向。
    size_t nearestSegmentEnd(double wx, double wy) const {
        const size_t n = route_.size();
        if (n == 0) return 0;
        const size_t seg_n = route_loop_ ? n : (n - 1);
        const size_t patrol_n = n - route_loop_start_;

        size_t best = wp_idx_;
        double best_score = 1e18;

        for (size_t i = route_loop_start_; i < seg_n; ++i) {
            const size_t idx_end = i + 1 < n ? i + 1 : route_loop_start_;
            const Waypoint& a = route_[i];
            const Waypoint& b = route_[idx_end];
            const double ex = b.x - a.x, ey = b.y - a.y;
            const double len2 = ex * ex + ey * ey;
            double t = 0.0;
            if (len2 > 1e-9) t = ((wx - a.x) * ex + (wy - a.y) * ey) / len2;
            t = std::min(1.0, std::max(0.0, t));
            const double px = a.x + t * ex, py = a.y + t * ey;
            const double d = std::hypot(wx - px, wy - py);

            // 保持巡逻方向: 序号在当前航点之后(环形)的段更优先
            const size_t current = std::max(wp_idx_, route_loop_start_);
            const size_t ahead = (idx_end + patrol_n - current) % patrol_n;
            const double order_penalty = (ahead == 0) ? 0.0 : 0.5 * static_cast<double>(ahead);

            const double score = d + order_penalty;
            if (score < best_score) { best_score = score; best = idx_end; }
        }
        return best;
    }

    // 沿用 15652b4：切点时立即发布下一目标，机头随移动轨迹转向。
    void advanceWaypoint() {
        if (route_.empty()) return;
        // 最多检查一圈, 避免全部航点都被堵时死循环
        for (size_t tries = 0; tries < route_.size(); ++tries) {
            if (wp_idx_ + 1 < route_.size()) {
                ++wp_idx_;
            } else if (route_loop_) {
                wp_idx_ = route_loop_start_;
                ++lap_count_;
                ROS_INFO("[巡逻] 第 %d 圈完成, 回到首航点", lap_count_);
            } else {
                ROS_WARN_ONCE("[巡逻] 航线非循环且已到末点, 保持在末点悬停");
                return;
            }
            const Waypoint& w = route_[wp_idx_];
            // 护栏: 航点落在膨胀障碍内(ego 局部地图揭示)就跳过, 否则 ego 规划
            // 不到该点会卡死。地图陈旧/缺失时 blocked 返回 false, 不误伤。
            if (obstacle_map_ &&
                obstacle_map_->blocked(w.x, w.y, waypoint_obstacle_clear_, 2.0)) {
                ROS_WARN("[巡逻] ★ 航点 %zu/%zu (%.1f, %.1f) 在膨胀障碍内, 跳过",
                         wp_idx_ + 1, route_.size(), w.x, w.y);
                continue;
            }
            publishEgoGoal(w.x, w.y, w.z, /*force=*/true);
            ROS_INFO("[巡逻] -> 航点 %zu/%zu (%.1f, %.1f)", wp_idx_ + 1, route_.size(), w.x, w.y);
            return;
        }
        // 整圈航点都被堵: 保持当前位置, 别硬飞进障碍
        ROS_ERROR("[巡逻] ★ 所有航点都在膨胀障碍内, 无法选目标, 原地悬停");
    }

    // 指令转发
    // ego 指令的 XY PVA + yaw -> PX4 本地帧。高度由统一参考负责。
    void forwardEgoCmd(const quadrotor_msgs::PositionCommand& cmd,
                       bool yaw_override, double yaw_override_deg,
                       double yaw_rate_max) {
        // EGO 的虚拟规划平面只用于 XY。Z/PVA 由状态对应的同一个高度
        // 参考在 ControlPx4 出口下发，绝不混用 EGO 的垂直前馈。

        // 世界 -> physical local metres。ControlPx4 在 MAVROS 出口逆变换尺度。
        const double lx = cmd.position.x - init_x_;
        const double ly = cmd.position.y - init_y_;

        double yaw_deg;
        if (yaw_override) {
            yaw_deg = yaw_override_deg;
        } else {
            // 巡逻偏航限速，减少航点切换时的机头突转。
            const Waypoint* w = route_.empty() ? nullptr : &route_[wp_idx_];
            double want = cmd.yaw * 180.0 / M_PI;
            if (w) {
                if (w->yaw_mode == YawMode::FIXED)              want = w->yaw_deg;
                else if (w->yaw_mode == YawMode::COURSE_OFFSET)  want += w->yaw_offset_deg;
            }
            yaw_deg = rateLimitYaw(want, yaw_rate_max, 0.0);
        }

        control_->fly_pva_xy_posez_yaw(lx, ly, flightAltitude(),
                                      cmd.velocity.x, cmd.velocity.y,
                                      cmd.acceleration.x, cmd.acceleration.y,
                                      yaw_deg, yaw_override ? track_yaw_rate_rad_
                                          : std::numeric_limits<double>::quiet_NaN());
    }

    // 执行脱困动作(全程零平移或沿自身航迹, 绝不盲飞)
    void execFallback(const Fallback& fb,
                      bool yaw_override, double yaw_override_deg) {
        double yaw_deg;
        // 有有效跟踪方向时，脱困悬停/回退也继续看人；不被巡逻搜索自旋覆写。
        if (yaw_override)          yaw_deg = yaw_override_deg;
        else if (fb.use_yaw)        yaw_deg = fb.yaw_deg;
        else                       yaw_deg = pose_sub_->getCurrentYaw() * 180.0 / M_PI;
        // 搜索、自旋、回退均维持状态对应的 YAML 高度，不能每拍锁实测 cz。
        control_->fly_posvel_yaw(fb.x, fb.y, flightAltitude(), 0.0, 0.0, 0.0, yaw_deg,
            yaw_override ? track_yaw_rate_rad_ : std::numeric_limits<double>::quiet_NaN());
    }

    // 回显仍有有效位姿和通信租约的认领。观测冻结仍维持 REACQUIRE，
    // 位姿断流、任务撤销或通信失联则撤销回执，不能让旧凭证永久阻塞改派。
    void trackingFeedback(const ros::TimerEvent&) {
        std_msgs::Header msg;
        msg.stamp = ros::Time::now();
        const auto mav = mavros_sub_->getCurrentState();
        geometry_msgs::PoseStamped pose;
        double tx, ty, tvx, tvy, rx_age;
        std::string assignment;
        if (state_ == PatrolState::TRACK && mav.connected && mav.armed && mav.mode == "OFFBOARD"
                && pose_sub_->getFreshPose(pose, pose_timeout_)
                && track_mode_sub_->mode() >= 1
                && track_target_sub_->get(tx, ty, tvx, tvy, rx_age, &assignment)
                && rx_age >= 0.0 && rx_age <= 1.0
                && !assignment.empty() && assignment == tracking_assignment_)
            msg.frame_id = tracking_assignment_;
        tracking_active_pub_.publish(msg);
    }

    // 状态机主循环 (50Hz)
    void runtask(const ros::TimerEvent&) {
        auto state = mavros_sub_->getCurrentState();
        geometry_msgs::PoseStamped pose;
        const bool fresh_pose = pose_sub_->getFreshPose(pose, pose_timeout_);
        if (!fresh_pose && state_ != PatrolState::LANDING && state_ != PatrolState::DONE) {
            // 首帧尚未到、断流或无效测量时继续发送零速心跳，等待有效位姿恢复。
            // 不以默认 (0,0,0) 起飞，也不把最后一个陈旧位置继续作为移动目标。
            control_->clear_altitude_reference();
            altitude_reference_ready_ = false;
            control_->fly_velxyz(0.0, 0.0, 0.0);
            yaw_cmd_inited_ = false;
            track_yaw_ready_ = false;
            track_yaw_rate_rad_ = 0.0;
            vel_loop_active_ = false;
            vl_init_ = false;
            vl_cmd_init_ = false;
            ROS_WARN_THROTTLE(1.0, "[位姿] 等待有效本地位姿 (时效上限 %.2fs)，保持零速",
                              pose_timeout_);
            return;
        }
        if (!fresh_pose) pose = pose_sub_->getCurrentPose();
        const double cx = pose.pose.position.x;
        const double cy = pose.pose.position.y;
        const double cz = pose.pose.position.z;
        const double wx = cx + init_x_;     // 世界帧(航点/目标都在世界帧)
        const double wy = cy + init_y_;

        const PatrolState st = state_;
        // LANDING/DONE 自行管理模式(切 AUTO.LAND 后不再依赖 OFFBOARD, 也不重解锁)
        const bool landing_manages_mode =
            (st == PatrolState::LANDING || st == PatrolState::DONE);
        const bool airborne_state = (st == PatrolState::CLIMB || st == PatrolState::TRANSIT ||
                                     st == PatrolState::PATROL || st == PatrolState::TRACK);

        // 参数配置/高度参考稳定期间只发心跳，不切 OFFBOARD 或解锁。
        // 节点在空中重启时继续原控制，禁止为配置高度源中断已解锁的飞行。
        if (!state.armed && !landing_manages_mode &&
            (!state.connected || !gps_height_setup_->ready())) {
            control_->clear_altitude_reference();
            altitude_reference_ready_ = false;
            control_->fly_velxyz(0.0, 0.0, 0.0);
            ROS_WARN_THROTTLE(2.0, "[高度源] 等待 FCU 连接、GPS 参数读回及估计就绪，暂不解锁");
            return;
        }

        // 仿真 OFFBOARD 的遥控器例外设置；失败不阻挡飞行状态机。
        setOffboardRcException();

        // 仅曾经飞起的飞机参与坠机判定，避免把地面等待判为坠毁。
        if (airborne_state) {
            if (cz >= crash_z_) {
                has_flown_ = true;
                crash_since_ = ros::Time(0);
            } else if (has_flown_) {
                if (crash_since_.isZero()) crash_since_ = ros::Time::now();
                if (ros::Time::now() - crash_since_ >= ros::Duration(crash_hold_)) {
                    ROS_ERROR("[坠机] z=%.2f 已持续低于 %.2f 米 %.0f 秒, 判定本机已坠毁/失控, "
                              "停止一切指令下发并转 DONE", cz, crash_z_, crash_hold_);
                    setState(PatrolState::DONE);
                    return;
                }
            } else {
                crash_since_ = ros::Time(0);
            }
        }

        // 解锁和 OFFBOARD 分别重试；起飞前持续发送零速度心跳。
        if ((state.mode != "OFFBOARD" || !state.armed) && !landing_manages_mode) {
            control_->clear_altitude_reference();
            altitude_reference_ready_ = false;
            control_->fly_velxyz(0.0, 0.0, 0.0);   // 地面待命心跳(全零速)
            const double now = ros::Time::now().toSec();
            if (state.mode != "OFFBOARD" &&
                now - last_offboard_t_.toSec() >= takeoff_retry_sec_) {
                mavros_msgs::SetMode cmd; cmd.request.custom_mode = "OFFBOARD";
                set_mode_cli_.call(cmd);
                ROS_INFO("尝试切换到 OFFBOARD... (当前 %s)", state.mode.c_str());
                last_offboard_t_ = ros::Time::now();
            }
            if (!state.armed &&
                now - last_arm_t_.toSec() >= takeoff_retry_sec_) {
                mavros_msgs::CommandBool cmd; cmd.request.value = true;
                arming_cli_.call(cmd);
                ROS_INFO("尝试解锁...");
                last_arm_t_ = ros::Time::now();
            }
            return;
        }

        if (!landing_manages_mode) {
            updateAltitudeReference(cz);
            publishBandRequest();
        }

        // 本地估计高度超过硬限立即降落，超过软限持续下压。
        if (!landing_manages_mode) {
            if (cz > z_hard_limit_) {
                ROS_ERROR("[限高] z=%.2f 超硬限 %.2f, 立即降落保命 (裁判世界高度限 6.0)", cz, z_hard_limit_);
                setState(PatrolState::LANDING);
                landingPub();
                return;
            }
            if (cz > z_soft_limit_) {
                if (z_over_since_.isZero()) z_over_since_ = ros::Time::now();
                // 软限持续超时或越过软硬限中点时覆盖高度指令。
                const bool waited = (ros::Time::now() - z_over_since_) >= ros::Duration(0.3);
                const bool too_high = cz > 0.5 * (z_soft_limit_ + z_hard_limit_);
                if (waited || too_high) {
                    const double safe_z = std::min(flightAltitude(), max_expl_z_);
                    ROS_WARN_THROTTLE(1.0, "[限高] z=%.2f 超软限 %.2f (%s), 强制下压到 %.2f",
                                      cz, z_soft_limit_,
                                      too_high ? "上冲过快" : "已持续 0.3s", safe_z);
                    // 越过 ego 直接下压: 锁住当前水平位置, 只改高度
                    // 紧急恢复清除尚未完成的升降前馈，直接请求同一安全目标。
                    altitude_hold_.reset(ros::Time::now().toSec(), safe_z);
                    control_->set_altitude_reference(safe_z, 0.0, 0.0);
                    control_->fly_posvel_yaw(cx, cy, safe_z, 0.0, 0.0, 0.0,
                                             pose_sub_->getCurrentYaw() * 180.0 / M_PI);
                    return;   // 本拍不走常规状态机, 免得 ego 指令又把它拉上去
                }
            } else {
                z_over_since_ = ros::Time(0);
            }
        }

        // 记录自身航迹(脱困时沿它回退)
        if (st == PatrolState::TRANSIT || st == PatrolState::PATROL || st == PatrolState::TRACK) {
            recordTrail(cx, cy, cz);
        }

        // 内部消除和 mission_complete 仅记录状态，继续巡查等待裁判结束。

        // 分状态
        switch (state_) {
        case PatrolState::CLIMB:   stepClimb(cx, cy, cz);            break;
        case PatrolState::TRANSIT:
        case PatrolState::PATROL:  stepPatrol(cx, cy, cz, wx, wy);   break;
        case PatrolState::TRACK:   stepTrack(cx, cy, cz, wx, wy);    break;

        case PatrolState::LANDING:
            landingPub();
            if (!state.armed) {
                ROS_WARN("[状态切换] LANDING -> DONE (已上锁, 着地完成)");
                setState(PatrolState::DONE);
            }
            break;

        case PatrolState::DONE:
            ROS_WARN("任务执行完毕, 程序退出");
            timer_.stop();
            ros::shutdown();
            break;
        }
    }

    // climb_min_z 前垂直爬升，随后由 PX4 直接平移进场；
    // 满足 min_expl_z 和 start_delay 后才交给 EGO TRANSIT。
    void stepClimb(double cx, double cy, double cz) {
        if (!climb_inited_) {
            climb_yaw_deg_ = pose_sub_->getCurrentYaw() * 180.0 / M_PI;
            climb_t0_ = ros::Time::now();
            climb_inited_ = true;
            // 首航点(世界帧) -> 本地帧: 边爬边飞过去 (无航线时锁 spawn 原地爬)
            if (!route_.empty()) {
                climb_x_ = route_[0].x - init_x_;
                climb_y_ = route_[0].y - init_y_;
            } else {
                climb_x_ = cx;
                climb_y_ = cy;
            }
            ROS_INFO("[起飞] %s: 目标 (%.1f, %.1f) z=%.2f, %.1fs 后允许进场",
                     climb_in_place_ ? "原地爬升后规划进场" : "边爬升边进场",
                     route_.empty() ? cx + init_x_ : route_[0].x,
                     route_.empty() ? cy + init_y_ : route_[0].y,
                     route_z_, start_delay_);
        }
        // 七套预设均沿用 base0：2m 以上边爬边飞首个进场点/巡航点。
        // 各图首段已离线检查道路与净空；climb_in_place=true 可选原地爬升。
        const bool cruise = (cz >= climb_min_z_) && !climb_in_place_;
        control_->fly_posvel_yaw(cruise ? climb_x_ : cx, cruise ? climb_y_ : cy,
                                 route_z_, 0.0, 0.0, 0.0, climb_yaw_deg_);

        const double elapsed = (ros::Time::now() - climb_t0_).toSec();
        if (cz >= min_expl_z_ && elapsed >= start_delay_) {
            if (route_.empty()) {
                ROS_ERROR_THROTTLE(10.0, "[巡逻] 没有可用航线, 保持悬停");
                return;
            }
            setState(PatrolState::TRANSIT);
            wp_idx_ = 0;
            publishEgoSpeed(entry_speed_, entry_accel_);
            const Waypoint& w = route_[0];
            publishEgoGoal(w.x, w.y, w.z, /*force=*/true);
            ROS_WARN("[状态切换] CLIMB -> TRANSIT: 进场飞向首航点 (%.1f, %.1f)", w.x, w.y);
        }
    }

    // TRANSIT / PATROL
    void stepPatrol(double cx, double cy, double cz, double wx, double wy) {
        if (route_.empty()) {
            control_->fly_posvel_yaw(cx, cy, route_z_, 0.0, 0.0, 0.0,
                                     pose_sub_->getCurrentYaw() * 180.0 / M_PI);
            return;
        }

        // 巡逻检查由开关控制；跟踪后归航始终启用碰撞检查。
        publishDisableCollision(!patrol_collision_check_ && !transit_from_track_);

        // 发现行人 -> 立即转跟踪(用户要求"一看到人就飞过去", 不再等进场完成)。
        double target_x, target_y, target_vx, target_vy, target_age, observation_age, state_age;
        const int track_mode = track_mode_sub_->mode();
        if (track_mode >= 1
                && track_target_sub_->get(target_x, target_y, target_vx, target_vy, target_age,
                                           nullptr, &observation_age, nullptr, &state_age)
                && fly_px4::trackingNavigation(track_mode, target_age, observation_age,
                       state_age, track_measurement_timeout_, target_state_timeout_,
                       track_reacquire_nav_timeout_) != fly_px4::TrackingNavigation::STOP) {
            enterTrack();
            return;
        }

        const bool transit = (state_ == PatrolState::TRANSIT);
        const Waypoint& w = route_[wp_idx_];

        // 归航先对准: 跟踪刚结束机头还锁着行人, 直接飞会边飞边转(转弯弧线撞障碍)。
        // 先原地把机头转到航点方向, 对准后再发 ego 目标飞直线。
        if (transit && transit_from_track_ && transit_align_pending_) {
            const double want = std::atan2(w.y - wy, w.x - wx) * 180.0 / M_PI;
            const double cur = pose_sub_->getCurrentYaw() * 180.0 / M_PI;
            if (std::fabs(wrapDeg(want - cur)) > 20.0) {
                control_->fly_posvel_yaw(cx, cy, route_z_, 0.0, 0.0, 0.0, want);
                return;
            }
            transit_align_pending_ = false;
            // 对准归航方向后再发新轨迹，避免执行时间已过的旧轨迹。
            publishEgoGoal(w.x, w.y, w.z, /*force=*/true);
            return_traj_id_ = pos_cmd_sub_->getCmd().trajectory_id;
            return_traj_wait_t0_ = ros::Time::now();
            return_traj_wait_ = true;
            return_hold_yaw_deg_ = want;
        }

        // 速度档: 起飞进场和跟踪后归航分别限制速度/加速度, 正常巡逻用航点速度或 cruise
        if (transit) {
            publishEgoSpeed(transit_from_track_ ? return_speed_ : entry_speed_,
                            transit_from_track_ ? return_accel_ : entry_accel_);
        } else {
            publishEgoSpeed(w.speed > 0.0 ? w.speed : cruise_speed_, cruise_accel_);
        }

        // 初始进场使用普通到点半径，跟踪后归航使用更小半径。
        const double arrive_r = transit
            ? (transit_from_track_ ? std::min(arrive_radius_, 0.8) : arrive_radius_)
            : arrive_radius_;
        const double d_wp = std::hypot(wx - w.x, wy - w.y);

        // 护栏: 当前目标航点若已被 ego 局部地图揭示为"在膨胀障碍内", 实时跳过
        // (不用等飞到了才发现进不去)。只在接近到局部地图覆盖范围后才会触发。
        if (obstacle_map_ &&
            obstacle_map_->blocked(w.x, w.y, waypoint_obstacle_clear_, 2.0)) {
            ROS_WARN_THROTTLE(1.0, "[巡逻] 航点 %zu/%zu (%.1f, %.1f) 在膨胀障碍内, 跳过",
                              wp_idx_ + 1, route_.size(), w.x, w.y);
            advanceWaypoint();
            return;
        }

        // 每秒记录航线进度和横向偏差。
        {
            const size_t prev_idx = wp_idx_ == route_loop_start_ && !transit ?
                                    route_.size()-1 :
                                    (wp_idx_ + route_.size()-1) % route_.size();
            const Waypoint& prev_w = route_[prev_idx];
            const double seg_x = w.x - prev_w.x, seg_y = w.y - prev_w.y;
            const double seg_len = std::hypot(seg_x, seg_y);
            double s = 0.0, e = 0.0;
            if (seg_len > 1e-3) {
                const double ux = seg_x / seg_len, uy = seg_y / seg_len;
                s = (wx - prev_w.x) * ux + (wy - prev_w.y) * uy;   // 沿航段方向的进度(米)
                e = (wx - prev_w.x) * (-uy) + (wy - prev_w.y) * ux; // 垂直航段方向的偏移(米)
            }
            ROS_INFO_THROTTLE(1.0,
                "[巡逻轨迹] pos=(%.1f,%.1f) -> wp%zu(%.1f,%.1f) d=%.1f 沿航段进度=%.1f/%.1f 垂直偏移=%.1f transit=%d",
                wx, wy, wp_idx_ + 1, w.x, w.y, d_wp, s, seg_len, e, (int)transit);
        }

        if (d_wp < arrive_r) {
            if (transit && wp_idx_ >= route_loop_start_) {
                setState(PatrolState::PATROL);
                transit_from_track_ = false;   // 已回到航线, 之后按巡逻的净空假设关避障
                ROS_WARN("[状态切换] TRANSIT -> PATROL: 已入航线, 巡航速度 %.1f m/s", cruise_speed_);
            }
            advanceWaypoint();
            return;
        }

        // 指令新鲜度: ego 崩了/规划不出来时不能抱着旧指令一直飞
        const double cmd_age = pos_cmd_sub_->secsSinceCmd();
        const bool has_cmd = pos_cmd_sub_->hasCmd() && cmd_age <= cmd_timeout_;
        if (has_cmd) ego_started_ = true;

        // 仅缺少新鲜指令时重发目标。
        if (!has_cmd &&
            (last_goal_pub_.isZero() ||
             (ros::Time::now() - last_goal_pub_).toSec() >= goal_repub_period_)) {
            publishEgoGoal(w.x, w.y, w.z, /*force=*/true);
            if (pos_cmd_sub_->hasCmd()) {
                ROS_WARN_THROTTLE(5.0, "[巡逻] %.1fs 没收到 ego 指令, 重发目标 (%.1f, %.1f)",
                                  cmd_age, w.x, w.y);
            } else {
                // 单独诊断 EGO 尚未返回首条指令的情况。
                const double waited = (ros::Time::now() - patrol_entry_time_).toSec();
                if (waited < ego_wait_warn_sec_) {
                    ROS_WARN_THROTTLE(2.0, "[巡逻] 等 ego 首条指令 (%.0fs, 还在建图/规划?), "
                                           "重发目标 (%.1f, %.1f)", waited, w.x, w.y);
                } else {
                    ROS_ERROR_THROTTLE(5.0,
                        "[巡逻] ★ 已等 %.0fs 仍未收到任何 ego 指令 —— 本机只会原地悬停! "
                        "去查第 2 步(swarm_relay_ego.launch): "
                        "1) rosnode list | grep -cE 'ego_planner|traj_server' 是否为 12; "
                        "2) rostopic list | grep -c ego/pos_cmd 是否为 6; "
                        "3) 那屏有没有 'process has died ... exit code 127' "
                        "(撞 racer_ws 同名库, 见 with_catkin_libs.sh)",
                        waited);
                }
            }
        }

        tracking_active_ = false;
        const Fallback fb = computeFallback(cx, cy, cz, has_cmd, cmd_age);
        escape_active_ = fb.valid;
        if (fb.valid) {
            execFallback(fb, false, 0.0);
            return;
        }
        if (return_traj_wait_) {
            const bool fresh = has_cmd &&
                pos_cmd_sub_->getCmd().trajectory_id != return_traj_id_;
            if (!fresh && (ros::Time::now() - return_traj_wait_t0_).toSec() < 0.5) {
                control_->fly_posvel_yaw(cx, cy, route_z_, 0.0, 0.0, 0.0, return_hold_yaw_deg_);
                return;
            }
            return_traj_wait_ = false;
        }
        if (has_cmd) {
            forwardEgoCmd(pos_cmd_sub_->getCmd(), false, 0.0,
                          transit ? patrol_yaw_rate_max_ : patrol_yaw_rate_max_);
        } else {
            // 还没收到过任何指令(刚进场, ego 还在算第一条轨迹): 原地悬停等
            control_->fly_posvel_yaw(cx, cy, route_z_, 0.0, 0.0, 0.0,
                                     pose_sub_->getCurrentYaw() * 180.0 / M_PI);
        }
    }

    void enterTrack() {
        tracking_assignment_.clear();  // stepTrack 真正读到目标后才确认新任务
        track_planner_wait_.reset();
        track_yaw_ready_ = false;
        track_yaw_rate_rad_ = 0.0;
        setState(PatrolState::TRACK);
        // 跟踪阶段恢复 ego 碰撞急停: 行人会走进房区, 必须让 ego 带着避障。
        publishDisableCollision(false);
        // 切跟踪瞬间降到 creep 档: 巡航 12m/s 惯性下大角度转弯会画大弧线(转弯半径
        // v²/a≈18m), 弧线穿过房区撞障碍。先压到低速(半径降到~0.4m), 再按距离分档。
        publishEgoSpeed(track_speed_creep_, track_acc_creep_);
        track_tier_ = 2;             // 从 creep 档起步, 距离 >26m 才重新提 far
        track_tier_inited_ = true;
        track_lost_since_ = ros::Time(0);
        standoff_valid_ = false;
        standoff_calc_ok_ = false;
        standoff_fail_since_ = ros::Time(0);
        reacquire_off_deg_ = 0.0;
        standoff_r_scale_ = 1.0;
        standoff_az_valid_ = false;   // 新目标从当前方位重新收敛, 别沿用上个目标的站位方位
        standoff_candidate_ = -1;
        standoff_ring_ = 0;
        lead_valid_ = false;          // 前推速度低通器重新起步 (新目标用当前速度直接起)
        track_cap_init_ = false;      // 限速平滑器重新起步 (新目标从当前档位值起坡)
        track_follow_seg_ = false;    // 段状态复位 (新目标先按接近段走)
        track_retreat_until_ = ros::Time(0);
        last_standoff_pub_ = ros::Time(0);
        track_stuck_t_ = ros::Time(0);
        track_stuck_stage_ = 0;
        // 速度环状态复位: 新目标, 误差低通/积分/下发速度都重新起步
        vel_loop_active_ = false;
        vl_init_ = false;
        vl_cmd_init_ = false;
        guard_active_ = false;
        guard_blocked_since_ = ros::Time(0);
        vel_loop_holdoff_until_ = ros::Time(0);
        standoff_obs_degraded_ = false;
        standoff_corridor_tier_ = false;
        corridor_ref_until_ = ros::Time(0);
        prev_track_mode_ = track_mode_sub_->mode();
        ROS_WARN("[状态切换] PATROL -> TRACK: fusion 派活 (mode=%d)", prev_track_mode_);
    }

    // 近距 PID 控制 XY，共用高度和偏航参考；位置误差低通、目标速度前馈和加速度限幅。
    // 关闭近距 PID 时回退 EGO。
    bool stepVelLoop(double wx, double wy, double sx, double sy, double tx, double ty,
                     bool yaw_override, double yaw_deg) {
        double vx = 0.0, vy = 0.0;
        if (!computeVelLoopCmd(wx, wy, sx, sy, vx, vy)) return false;
        diag_guard_des_clear_ = diag_guard_out_clear_ = -1.0;
        if (refreshAvoidField(wx, wy)) {
            if (!guardVelLoop(wx, wy, tx, ty, vx, vy)) return false;
        } else if (!avoidBySampling(wx, wy, vx, vy, vx, vy)) {
            return false;   // 雷达表面不可用: 原 inflate 地图方向采样
        }

        // 加速度斜率限幅 (去抖最后一关; 50Hz 定时器)
        const double dt = 0.02;
        if (vl_cmd_init_) {
            const double max_dv = vel_loop_acc_max_ * dt;
            double dvx = vx - vl_cmd_vx_, dvy = vy - vl_cmd_vy_;
            const double dmag = std::hypot(dvx, dvy);
            if (dmag > max_dv && dmag > 1e-9) {
                const double k = max_dv / dmag;
                dvx *= k; dvy *= k;
            }
            vx = vl_cmd_vx_ + dvx;
            vy = vl_cmd_vy_ + dvy;
        }
        vl_cmd_vx_ = vx; vl_cmd_vy_ = vy; vl_cmd_init_ = true;

        const double yaw = yaw_override ? yaw_deg
                                        : pose_sub_->getCurrentYaw() * 180.0 / M_PI;
        control_->fly_velxy_posez_yaw(vx, vy, track_z_, yaw,
                                     yaw_override ? track_yaw_rate_rad_ : 0.0);
        return true;
    }

    // PID + 行人速度前馈 -> 期望 XY 速度 (世界帧). 含误差低通/积分抗饱和/限幅。
    bool computeVelLoopCmd(double wx, double wy,
                           double sx, double sy, double& vx, double& vy) {
        const double now = ros::Time::now().toSec();
        double dt = vl_init_ ? (now - vl_last_t_) : 0.02;
        if (!(dt > 1e-4) || dt > 0.5) dt = 0.02;   // 刚接管/长时间没进来: 用标称拍
        vl_last_t_ = now;

        const double ex = sx - wx, ey = sy - wy;
        if (!vl_init_) {
            vl_ex_x_ = ex;      vl_ex_y_ = ey;
            vl_ex_x_prev_ = ex; vl_ex_y_prev_ = ey;
            vl_ix_ = 0.0;       vl_iy_ = 0.0;
            vl_init_ = true;
        } else {
            const double a = dt / (dt + vel_loop_err_tau_);
            vl_ex_x_ += a * (ex - vl_ex_x_);
            vl_ex_y_ += a * (ey - vl_ex_y_);
        }

        // 积分 + 抗饱和(积分项本身钳位)
        vl_ix_ += vl_ex_x_ * dt;
        vl_iy_ += vl_ex_y_ * dt;
        const double i_lim = 2.0;
        vl_ix_ = std::min(i_lim, std::max(-i_lim, vl_ix_));
        vl_iy_ = std::min(i_lim, std::max(-i_lim, vl_iy_));

        // D 项对**低通后**误差做差分(不用原始误差, 否则又是噪声源)
        const double dex = (vl_ex_x_ - vl_ex_x_prev_) / dt;
        const double dey = (vl_ex_y_ - vl_ex_y_prev_) / dt;
        vl_ex_x_prev_ = vl_ex_x_;
        vl_ex_y_prev_ = vl_ex_y_;

        // 速度前馈死区抑制静止目标的速度噪声。
        double ff_x = lead_vx_, ff_y = lead_vy_;
        if (std::hypot(ff_x, ff_y) < vel_loop_ff_dead_) {
            ff_x = 0.0; ff_y = 0.0;
        }
        vx = ff_x + vel_loop_kp_ * vl_ex_x_ + vel_loop_ki_ * vl_ix_ + vel_loop_kd_ * dex;
        vy = ff_y + vel_loop_kp_ * vl_ex_y_ + vel_loop_ki_ * vl_iy_ + vel_loop_kd_ * dey;

        // 幅值限幅 (前馈+修正合成的总速度)
        const double mag = std::hypot(vx, vy);
        if (mag > vel_loop_vmax_ && mag > 1e-9) {
            const double k = vel_loop_vmax_ / mag;
            vx *= k; vy *= k;
        }
        return true;
    }

    // 速度守卫预测延迟、斜率限制、前视和制动距离；持续受阻时回退 EGO。
    bool guardVelLoop(double wx, double wy, double tx, double ty, double& vx, double& vy) {
        fly_px4::VelocityGuardState gs;
        gs.px = wx; gs.py = wy;
        gs.vdx = vx; gs.vdy = vy;
        gs.vpx = vl_cmd_init_ ? vl_cmd_vx_ : 0.0;
        gs.vpy = vl_cmd_init_ ? vl_cmd_vy_ : 0.0;
        double ax = 0.0, ay = 0.0;
        if (local_velocity_sub_->get(ax, ay, 0.3)) { gs.v0x = ax; gs.v0y = ay; }
        else { gs.v0x = gs.vpx; gs.v0y = gs.vpy; }
        gs.has_person = std::isfinite(tx) && std::isfinite(ty);
        gs.tx = tx; gs.ty = ty;
        gs.active = guard_active_;
        // 站位在走廊档: 期望净空降到站位自身离墙 (-0.2m), 沿通道中线飞不算贴墙;
        // 档位结束后再保持 2s, 让飞机先飞出通道
        const ros::Time t_now = ros::Time::now();
        if (standoff_obs_.corridor && standoff_corridor_tier_ && diag_standoff_wall_ > 0.0) {
            corridor_safe_ref_ = std::max(standoff_obs_.corridor_clear, diag_standoff_wall_ - 0.2);
            corridor_ref_until_ = t_now + ros::Duration(2.0);
        }
        if (!corridor_ref_until_.isZero() && t_now <= corridor_ref_until_)
            gs.safe_ref = corridor_safe_ref_;
        const fly_px4::VelocityGuardResult r = fly_px4::guardVelocity(avoid_field_, guard_, gs);
        diag_guard_des_clear_ = std::min(99.0, r.desired_clear);
        diag_guard_out_clear_ = std::min(99.0, r.chosen_clear);
        if (r.intervened != guard_active_)
            ROS_INFO("[避障] 速度过滤%s (期望路径净空 %.2fm)",
                     r.intervened ? "介入" : "解除, 恢复原 PID 速度", diag_guard_des_clear_);
        guard_active_ = r.intervened;
        if (r.intervened)
            ROS_INFO_THROTTLE(0.5, "[避障] 期望 (%.2f,%.2f) 净空 %.2fm -> 下发 (%.2f,%.2f) 净空 %.2fm%s",
                              vx, vy, diag_guard_des_clear_, r.vx, r.vy, diag_guard_out_clear_,
                              r.blocked ? " [前进方向被堵]" : "");
        vx = r.vx; vy = r.vy;
        const ros::Time now = ros::Time::now();
        if (!r.blocked) {
            guard_blocked_since_ = ros::Time(0);
            return true;
        }
        if (guard_blocked_since_.isZero()) guard_blocked_since_ = now;
        if ((now - guard_blocked_since_).toSec() < avoid_block_sec_) return true;
        // 本地已无前进通路 (站位在楼另一侧之类): 交给 EGO 全局绕, 短时不再抢回。
        // 5s 内反复被堵说明 EGO 还没绕出来, 让它多飞一会 (1.5 -> 3 -> 4.5s)。
        guard_blocked_since_ = ros::Time(0);
        const bool repeated = !avoid_last_block_.isZero() && now >= avoid_last_block_ &&
                              (now - avoid_last_block_).toSec() < 5.0;
        avoid_holdoff_cur_ = repeated ? std::min(3.0 * avoid_holdoff_sec_, 2.0 * avoid_holdoff_cur_)
                                      : avoid_holdoff_sec_;
        avoid_last_block_ = now;
        vel_loop_holdoff_until_ = now + ros::Duration(avoid_holdoff_cur_);
        ROS_WARN("[避障] %.1fs 找不到朝站位前进的安全速度, 交 EGO 绕行 %.1fs",
                 avoid_block_sec_, avoid_holdoff_cur_);
        return false;
    }

    // 方向采样避障. 保持速度幅值, 只旋转方向。返回 false = 全被堵(交回 EGO)。
    bool avoidBySampling(double wx, double wy, double dvx, double dvy,
                         double& ox, double& oy) {
        ox = dvx; oy = dvy;
        const double speed = std::hypot(dvx, dvy);
        if (speed < 1e-3) return true;      // 几乎不动, 无需避障

        const double L = std::max(vel_loop_lookahead_, speed * 1.2 + 0.8);
        std::vector<std::pair<float, float>> pts;
        if (!obstacle_map_ ||
            !obstacle_map_->nearby(wx, wy, L + vel_loop_clear_ + 0.5, pts))
            return true;                    // 地图缺失/陈旧: 不避障(EGO 仍在兜底)
        if (pts.empty()) return true;       // 附近无障碍: 直行

        const double base = std::atan2(dvy, dvx);
        const double cur  = (vl_cmd_init_ && std::hypot(vl_cmd_vx_, vl_cmd_vy_) > 1e-3)
                            ? std::atan2(vl_cmd_vy_, vl_cmd_vx_) : base;
        const int n = std::max(1, vel_loop_samples_);
        const double step = 15.0 * M_PI / 180.0;

        double best_cost = 1e18, best_ang = base;
        bool found = false;
        for (int i = -n; i <= n; ++i) {
            const double ang = base + i * step;
            if (rayBlocked(wx, wy, std::cos(ang), std::sin(ang), L, pts)) continue;
            const double d_dir = std::fabs(wrapRad(ang - base));
            const double d_cur = std::fabs(wrapRad(ang - cur));
            const double cost  = d_dir + vel_loop_w_turn_ * d_cur;
            if (cost < best_cost) { best_cost = cost; best_ang = ang; found = true; }
        }
        if (!found) return false;           // 全方向都被堵: 交回 EGO 绕行
        ox = speed * std::cos(best_ang);
        oy = speed * std::sin(best_ang);
        return true;
    }

    // 从 (wx,wy) 沿单位方向 (ux,uy) 前视 L 米, 走廊半宽 vel_loop_clear_ 内
    // 是否有膨胀障碍点 (点到线段距离 < 半宽即判堵)。
    bool rayBlocked(double wx, double wy, double ux, double uy, double L,
                    const std::vector<std::pair<float, float>>& pts) const {
        const double ax = ux * L, ay = uy * L;      // 线段向量
        const double len2 = ax * ax + ay * ay;
        const double c2 = vel_loop_clear_ * vel_loop_clear_;
        for (const auto& p : pts) {
            const double px = p.first - wx, py = p.second - wy;
            double t = (len2 > 1e-9) ? (px * ax + py * ay) / len2 : 0.0;
            t = std::min(1.0, std::max(0.0, t));
            const double dx = px - t * ax, dy = py - t * ay;
            if (dx * dx + dy * dy < c2) return true;
        }
        return false;
    }

    // TRACK
    void stepTrack(double cx, double cy, double cz, double wx, double wy) {
        const int mode = track_mode_sub_->mode();

        // 释放判据: mode 回 0 并持续 track_release_sec 秒才回巡逻。
        // 不立刻回是因为 fusion 的 emit_modes 只有 2Hz, 中间可能有一两拍空档。
        if (mode < 1) {
            track_yaw_ready_ = false;
            track_yaw_rate_rad_ = 0.0;
            if (track_lost_since_.isZero()) track_lost_since_ = ros::Time::now();
            if ((ros::Time::now() - track_lost_since_).toSec() >= track_release_sec_) {
                resumePatrol(wx, wy);
                return;
            }
            // 任务释放后在滞回区停止 XY，继续保持高度。
            control_->fly_velxyz(0.0, 0.0, 0.0);
            vel_loop_active_ = false;
            vl_init_ = false;
            vl_cmd_init_ = false;
            return;
        } else {
            track_lost_since_ = ros::Time(0);
        }

        // 短暂重捕期间保持目标和朝向，减少站位环无效旋转。
        prev_track_mode_ = mode;

        // 远距由距离档限速，近距进入 PID；分档滞回独立于融合模式。
        double tx, ty, tvx, tvy, age, observation_age, state_age;
        std::string assignment;
        const bool have_target = track_target_sub_->get(tx, ty, tvx, tvy, age, &assignment,
                                                        &observation_age, nullptr, &state_age);
        if (!have_target || age > 1.0) {
            // 调度断线/撤销后不无限维持旧目标；这也是独占改派的指令租约。
            resumePatrol(wx, wy);
            return;
        }
        if (mode >= 1) tracking_assignment_ = assignment;
        const auto navigation = fly_px4::trackingNavigation(mode, age, observation_age,
            state_age, track_measurement_timeout_, target_state_timeout_,
            track_reacquire_nav_timeout_);
        const bool saved_reacquire = navigation == fly_px4::TrackingNavigation::SAVED_REACQUIRE;
        if (navigation == fly_px4::TrackingNavigation::STOP) {
            // 单目和深度都没有新观测时停止站位更新/前馈，保留 TRACK 等融合重捕或释放。
            // 深度时间戳本身不作门槛：正常单目恢复仍能立即继续导航。
            tracking_active_ = true;
            standoff_calc_ok_ = false;
            vel_loop_active_ = false;
            vl_init_ = false;
            vl_cmd_init_ = false;
            lead_valid_ = false;
            double yaw_deg = pose_sub_->getCurrentYaw() * 180.0 / M_PI;
            computeTrackYaw(cx, cy, yaw_deg);
            control_->fly_velxy_posez_yaw(0.0, 0.0, track_z_, yaw_deg, track_yaw_rate_rad_);
            ROS_WARN_THROTTLE(1.0, "[跟踪] 观测/状态年龄 %.2f/%.2fs，暂停平移并保留重捕机头",
                              observation_age, state_age);
            return;
        }
        if (saved_reacquire) {
            // 融合明确派发的保存点只交给 EGO 沿规划路径重捕，不借旧速度外推，
            // 不进入近距 PID；真实观测时间保持不变，超过窗口自动停止。
            tvx = tvy = 0.0;
            lead_valid_ = false;
            vel_loop_active_ = false;
            vl_init_ = false;
            vl_cmd_init_ = false;
        }
        if (have_target && age <= 1.0) {
            const double d_t = std::hypot(wx - tx, wy - ty);
            const double v_person = std::hypot(tvx, tvy);
            int tier = track_tier_;
            if (!track_tier_inited_) {
                tier = (d_t > track_far_dist_) ? 0
                     : (d_t < track_creep_in_ ? 2 : 1);
                track_tier_inited_ = true;
            } else if (track_tier_ == 0) {
                if (d_t < track_near_dist_) tier = 1;
            } else if (track_tier_ == 1) {
                if (d_t > track_far_dist_)      tier = 0;
                else if (d_t < track_creep_in_) tier = 2;
            } else {
                if (d_t > track_creep_out_) tier = 1;
            }
            track_tier_ = tier;
            // 每拍都算(人速会变), publishEgoSpeed 内部有死区, 实际只在
            // 限速变化 >0.15m/s 时才真正发给 ego
            publishTrackSpeed(tier, v_person, d_t, wx, wy);
        }

        // 站位按最小周期和位移门限更新，落后于目标时强制刷新。
        if (have_target && age <= 1.0) {
            double sx, sy;
            if (selectStandoff(wx, wy, tx, ty, tvx, tvy, sx, sy)) {
                standoff_fail_since_ = ros::Time(0);
                // 每拍的真值给速度环 (与给 EGO 的阶梯 goal_x_ 分开, 见成员注释)
                standoff_x_ = sx; standoff_y_ = sy; standoff_calc_ok_ = true;
                // 远距引导点保持在规划范围内，避免单目测距抖动推动目标。
                double gx = sx, gy = sy;
                if (track_tier_ == 0)
                    fly_px4::farApproachGoal(wx, wy, sx, sy, track_far_goal_dist_, gx, gy);
                const bool first = !standoff_valid_;
                const double moved = standoff_valid_
                    ? std::hypot(gx - goal_x_, gy - goal_y_) : 1e9;
                const double since = last_standoff_pub_.isZero() ? 1e9
                    : (ros::Time::now() - last_standoff_pub_).toSec();
                const bool forced = moved > standoff_force_d_;
                if (first || forced || (since >= standoff_min_dt_ && moved > standoff_move_)) {
                    // 先检查局部目标更新门限；逼近与伴飞使用不同末端速度。
                    const double gvx = (track_tier_ == 0) ? 0.0 : tvx;
                    const double gvy = (track_tier_ == 0) ? 0.0 : tvy;
                    publishEgoGoal(gx, gy, track_z_, /*force=*/true, gvx, gvy);
                    last_standoff_pub_ = ros::Time::now();
                    standoff_valid_ = true;
                }
            } else {
                standoff_calc_ok_ = false;
                // 没有候选时先保持，超时后拒绝任务；上方结构附近在外侧等待。
                const bool band_hold = standoff_band_blocked_ &&
                    std::hypot(wx - tx, wy - ty) <= standoff_band_hold_dist_;
                if (standoff_fail_since_.isZero()) {
                    standoff_fail_since_ = ros::Time::now();
                    if (band_hold)
                        ROS_WARN("[避障] 站位都在屋檐/顶棚后, 在棚外观察 (最多 %.0fs)",
                                 standoff_band_hold_sec_);
                }
                const double reject_delay = band_hold
                    ? std::max(standoff_reject_delay_, standoff_band_hold_sec_)
                    : standoff_reject_delay_;
                if ((ros::Time::now() - standoff_fail_since_).toSec() >= reject_delay) {
                    publishReject(1);   // 持续被挡/城界: 真接不了, 才放弃改派
                    standoff_fail_since_ = ros::Time(0);
                }
            }

            // 诊断数组：[0]旧字段=999，[1]当前方位°，[2]原始方位°，[3]滤波方位°，
            // [4..5]旧字段=0，[6]候选序号(-1失败)，[7]有效半径m，[8]目标距离m，
            // [9]目标速度m/s，[10]墙面净空(-1不可用)，[11]视线净空，
            // [12]降级标志，[13]速度守卫，[14]期望路径净空，[15]指令路径净空，[16]走廊档。
            {
                std_msgs::Float32MultiArray dm;
                dm.data.resize(17);
                dm.data[0] = 999.0f;
                dm.data[1] = static_cast<float>(wrapDeg(
                    std::atan2(wy - ty, wx - tx) * 180.0 / M_PI));
                dm.data[2] = static_cast<float>(diag_az0_raw_deg_);
                dm.data[3] = static_cast<float>(standoff_az_prev_);
                dm.data[4] = 0.0f;
                dm.data[5] = 0.0f;
                dm.data[6] = static_cast<float>(standoff_az_pick_);
                dm.data[7] = static_cast<float>(diag_r_eff_);
                dm.data[8] = static_cast<float>(std::hypot(wx - tx, wy - ty));
                dm.data[9] = static_cast<float>(std::hypot(tvx, tvy));
                dm.data[10] = static_cast<float>(diag_standoff_wall_);
                dm.data[11] = static_cast<float>(diag_standoff_los_);
                dm.data[12] = standoff_obs_degraded_ ? 1.0f : 0.0f;
                dm.data[13] = (vel_loop_active_ && guard_active_) ? 1.0f : 0.0f;
                dm.data[14] = static_cast<float>(diag_guard_des_clear_);
                dm.data[15] = static_cast<float>(diag_guard_out_clear_);
                dm.data[16] = standoff_corridor_tier_ ? 1.0f : 0.0f;
                standoff_diag_pub_.publish(dm);
            }
        }

        // 机头压目标
        double track_yaw_deg = 0.0;
        const bool yaw_override = computeTrackYaw(cx, cy, track_yaw_deg);
        tracking_active_ = true;

        // 回退沿历史轨迹执行时覆盖规划指令和拒绝逻辑，并保持目标视线。
        if (!track_retreat_until_.isZero()) {
            if (ros::Time::now() < track_retreat_until_) {
                Fallback rfb;
                rfb.valid = true;
                rfb.mode = Fallback::Mode::RETREAT;
                rfb.x = track_retreat_x_;
                rfb.y = track_retreat_y_;
                execFallback(rfb, yaw_override, track_yaw_deg);
                escape_active_ = true;   // 不记航迹, 免得回退段变成新航迹来回拉锯
                return;
            }
            // 回退窗口结束: 从自由空间重新出发, 强制重发站位给 ego 一个干净目标
            track_retreat_until_ = ros::Time(0);
            standoff_valid_ = false;
            track_stuck_t_ = ros::Time(0);
            track_stuck_stage_ = 0;
            ROS_WARN("[跟踪] 回退完成, 重选站位继续跟踪");
        }

        const double cmd_age = pos_cmd_sub_->secsSinceCmd();
        const bool has_cmd = pos_cmd_sub_->hasCmd() && cmd_age <= cmd_timeout_;
        if (has_cmd) ego_started_ = true;

        // 首个跟踪 goal 后等待规划响应，旧巡逻缓存的年龄不能让新任务立即失败。
        // 后续 goal 重发不刷新首次等待；收到指令后以最后响应时刻检查断流。
        const double track_now = ros::Time::now().toSec();
        if (has_cmd) track_planner_wait_.commandReceived(track_now - cmd_age);

        // ego 长时间出不来指令 = 规划不出通往站位点的路 -> 请 fusion 改派
        if (track_planner_wait_.shouldReject(track_now, has_cmd, track_planner_start_grace_)) {
            publishReject(2);
            if (last_goal_pub_.isZero() ||
                (ros::Time::now() - last_goal_pub_).toSec() >= goal_repub_period_) {
                if (standoff_valid_) publishEgoGoal(goal_x_, goal_y_, goal_z_, /*force=*/true);
            }
        }

        // 指令新鲜但距站位较远且移动不足时判卡住：半径减半、归零、历史回退，最后拒绝任务。
        if (has_cmd && standoff_valid_) {
            const double d_goal = std::hypot(wx - goal_x_, wy - goal_y_);
            if (d_goal < 2.5) {
                track_stuck_t_ = ros::Time(0);      // 到位悬停盯人: 正常
                track_stuck_stage_ = 0;
                standoff_r_scale_ = 1.0;            // 站位可达了, 恢复正常半径
            } else {
                if (track_stuck_t_.isZero()) {
                    track_stuck_t_ = ros::Time::now();
                    track_stuck_x_ = wx;
                    track_stuck_y_ = wy;
                } else if ((ros::Time::now() - track_stuck_t_).toSec() >= track_stuck_sec_) {
                    const double moved = std::hypot(wx - track_stuck_x_, wy - track_stuck_y_);
                    track_stuck_t_ = ros::Time::now();
                    track_stuck_x_ = wx;
                    track_stuck_y_ = wy;
                    if (moved < 1.0) {
                        ++track_stuck_stage_;
                        if (track_stuck_stage_ == 1) {
                            standoff_r_scale_ = std::min(standoff_r_scale_, 0.5);
                            standoff_valid_ = false;   // 下一拍按新半径重选站位并强发
                            ROS_ERROR("[跟踪] 指令正常但 %.0fs 只动了 %.2fm 且距站位 %.1fm, "
                                      "站位半径缩到 %.1fm (落点可能在楼区里)",
                                      track_stuck_sec_, moved, d_goal,
                                      standoff_radius_ * standoff_r_scale_);
                        } else if (track_stuck_stage_ == 2) {
                            standoff_r_scale_ = 0.0;   // 目标点退到人身上
                            reacquire_off_deg_ = wrapDeg(reacquire_off_deg_ + 60.0);
                            standoff_valid_ = false;
                            ROS_ERROR("[跟踪] 缩半径后仍卡死, 目标点退到人身上 (半径 0)");
                        } else if (track_stuck_stage_ == 3) {
                            if (mode != 3 && d_goal < 5.0) {
                                // 贴到人附近(<5m)且检测还在: 就近悬停盯人, 15s 计数
                                // 继续攒 —— 比 reject 改派强 (人没丢, 不用重新找)
                                track_stuck_stage_ = 0;
                                ROS_WARN("[跟踪] 已贴到目标 %.1fm 内的最近可达点, 悬停盯人攒计数",
                                         d_goal);
                            } else if (pickRetreatPoint(cx, cy)) {
                                // 沿记录的历史轨迹逐段回退。
                                track_retreat_x_ = escape_x_;
                                track_retreat_y_ = escape_y_;
                                track_retreat_until_ = ros::Time::now()
                                    + ros::Duration(track_stuck_sec_ + 1.0);
                                standoff_r_scale_ = 0.5;   // 退完从半半径重试
                                ROS_ERROR("[跟踪] 半径 0 仍卡死(困在障碍区), 沿航迹回退 %.1fm "
                                          "到 (%.1f,%.1f) 脱困", retreat_dist_,
                                          escape_x_, escape_y_);
                            } else {
                                ROS_ERROR("[跟踪] 无航迹可退, 放弃本目标让 fusion 改派");
                                publishReject(4);
                                track_stuck_stage_ = 0;
                            }
                        } else {
                            ROS_ERROR("[跟踪] 回退后仍卡死(距站位 %.1fm, mode=%d), "
                                      "放弃本目标让 fusion 改派", d_goal, mode);
                            publishReject(4);
                            track_stuck_stage_ = 0;
                        }
                    } else {
                        track_stuck_stage_ = 0;        // 在动(慢也算), 只是慢
                    }
                }
            }
        } else {
            track_stuck_t_ = ros::Time(0);
        }

        // 正常规划启动宽限内不沿用巡逻阶段累积的断流回退；先悬停等新响应。
        const bool planning_grace = track_planner_wait_.waitingForResponse(
            track_now, has_cmd, track_planner_start_grace_);
        if (planning_grace) {
            cmd_lost_since_ = ros::Time(0);
            escape_target_valid_ = false;
            retreat_picked_ = false;
        }
        const Fallback fb = planning_grace ? Fallback{}
            : computeFallback(cx, cy, cz, has_cmd, cmd_age);
        escape_active_ = fb.valid;
        if (fb.valid) {
            execFallback(fb, yaw_override, track_yaw_deg);
            return;
        }

        // PID 进入/退出带滞回；使用连续站位，持续受阻则回退 EGO。
        const bool avoid_holdoff = !vel_loop_holdoff_until_.isZero() &&
                                   ros::Time::now() < vel_loop_holdoff_until_;
        if (vel_loop_enable_ && !saved_reacquire && standoff_calc_ok_ && have_target && age <= 1.0 &&
            !avoid_holdoff) {
            const double d_person = std::hypot(wx - tx, wy - ty);
            const double e_goal   = std::hypot(standoff_x_ - wx, standoff_y_ - wy);
            const bool in_cond    = (d_person < vel_loop_enter_dist_ &&
                                     e_goal   < vel_loop_enter_err_);
            const bool hold_cond  = (d_person < vel_loop_exit_dist_ &&
                                     e_goal   < vel_loop_exit_err_);
            if (vel_loop_active_ ? hold_cond : in_cond) {
                if (stepVelLoop(wx, wy, standoff_x_, standoff_y_, tx, ty,
                                yaw_override, track_yaw_deg)) {
                    if (!vel_loop_active_)
                        ROS_WARN("[跟踪] 切入 PID 速度环 (d_person=%.1fm, |e|=%.1fm)",
                                 d_person, e_goal);
                    vel_loop_active_ = true;
                    return;
                }
            }
        }
        if (vel_loop_active_) {
            ROS_WARN("[跟踪] 退出 PID 速度环(条件不满足或方向全被堵), 交回 EGO");
            vel_loop_active_ = false;
            guard_active_ = false;
            guard_blocked_since_ = ros::Time(0);
            vl_init_ = false;        // 下次接管重新起步: 误差低通/积分/斜率限幅都从当前值起
            vl_cmd_init_ = false;
        }

        if (has_cmd) {
            forwardEgoCmd(pos_cmd_sub_->getCmd(), yaw_override, track_yaw_deg,
                          track_yaw_rate_max_);
        } else {
            // 没指令但在跟踪: 原地悬停 + 机头保持压住目标, **绝不盲飞平移**。
            // 站着不动也在给 fusion 喂检测, 15 秒计数可以继续攒。
            const double yaw_deg = yaw_override ? track_yaw_deg
                                                : pose_sub_->getCurrentYaw() * 180.0 / M_PI;
            control_->fly_posvel_yaw(cx, cy, track_z_, 0.0, 0.0, 0.0, yaw_deg,
                yaw_override ? track_yaw_rate_rad_ : std::numeric_limits<double>::quiet_NaN());
        }
    }

    void resumePatrol(double wx, double wy) {
        wp_idx_ = nearestSegmentEnd(wx, wy);
        const Waypoint& w = route_[wp_idx_];
        setState(PatrolState::TRANSIT);   // 用 return_speed 归航, 回到航线后自动转 PATROL
        transit_from_track_ = true;   // 起点可能还在跟踪时飞进的房区里, 这段路要保留避障
        transit_align_pending_ = true;   // 先原地对准航点方向再飞, 对准后才发目标(见 stepPatrol)
        return_traj_wait_ = false;
        publishEgoSpeed(return_speed_, return_accel_);
        tracking_active_ = false;     // z 钳位恢复 [4.6,5.4], 爬回巡航高度
        track_stuck_t_ = ros::Time(0);
        track_stuck_stage_ = 0;
        ROS_WARN("[状态切换] TRACK -> TRANSIT: 目标已释放, 归航到航点 %zu (%.1f, %.1f)",
                 wp_idx_ + 1, w.x, w.y);
    }

    void setState(PatrolState s) {
        if (state_ == s) return;
        ROS_INFO("[状态] %s -> %s", stateName(state_), stateName(s));
        // 首次进入巡逻类状态时记录入口时间，后续状态切换不重置。
        if (patrol_entry_time_.isZero() &&
            (s == PatrolState::TRANSIT || s == PatrolState::PATROL)) {
            patrol_entry_time_ = ros::Time::now();
        }
        if (s != PatrolState::TRACK) {
            track_yaw_ready_ = false;
            track_yaw_rate_rad_ = 0.0;
        }
        state_ = s;
    }

    void landingPub() {
        if (land_sent_) return;
        mavros_msgs::SetMode cmd;
        cmd.request.custom_mode = "AUTO.LAND";
        if (set_mode_cli_.call(cmd)) {
            ROS_WARN("[降落] 已切 AUTO.LAND, 由 PX4 自行着地并上锁");
            land_sent_ = true;
        } else {
            ROS_WARN_THROTTLE(2.0, "[降落] 切 AUTO.LAND 失败, 重试");
        }
    }

    // 仿真下保留 OFFBOARD 遥控器例外，不调整速度或加速度限值。
    void setOffboardRcException() {
        if (rc_exception_done_) return;

        // 限制服务重试频率，避免阻塞 50Hz 控制循环。
        if (!rc_exception_last_try_.isZero() &&
            (ros::Time::now() - rc_exception_last_try_).toSec() < 2.0) {
            return;
        }
        rc_exception_last_try_ = ros::Time::now();

        mavros_msgs::ParamSet srv;
        srv.request.param_id = "COM_RCL_EXCEPT";
        srv.request.value.integer = 4;
        srv.request.value.real = 0.0;
        if (param_cli_.call(srv) && srv.response.success) {
            ROS_WARN("[PX4] COM_RCL_EXCEPT 已设为 4");
            rc_exception_done_ = true;
            rc_exception_failures_ = 0;
        } else {
            if (++rc_exception_failures_ >= 15) {
                // 30s 还设不上: mavros/PX4 有别的问题, 不要一直卡在这里刷日志。
                ROS_ERROR("[PX4] 设置 COM_RCL_EXCEPT 连续失败 %d 次，停止重试",
                          rc_exception_failures_);
                rc_exception_done_ = true;
            } else {
                ROS_WARN("[PX4] 设置 COM_RCL_EXCEPT 失败，2s 后重试 (%d/15)",
                         rc_exception_failures_);
            }
        }
    }

    // 航迹记录: 每 0.5m 存一个本地帧位置, 供"沿航迹回退"脱困用
    void recordTrail(double x, double y, double z) {
        // 脱困期间绝对不能记: 回退时飞机沿航迹倒着走, 记进去的话航迹尾部就变成
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

    // 无指令依次悬停、转向、沿历史轨迹回退；新鲜但冻结的指令另行检测。
    Fallback computeFallback(double cx, double cy, double cz, bool has_cmd, double cmd_age) {
        Fallback fb;
        const auto now = ros::Time::now();

        if (has_cmd) {
            cmd_lost_since_ = ros::Time(0);
            escape_target_valid_ = false;
            retreat_picked_ = false;

            // 跟踪期间允许合法悬停，不使用巡逻冻结判据。
            if (tracking_active_) {
                frozen_ref_valid_ = false;
                return fb;
            }

            if (!frozen_ref_valid_ ||
                std::hypot(cx - frozen_ref_[0], cy - frozen_ref_[1]) > 1.0) {
                frozen_ref_ = { { cx, cy, cz } };
                frozen_ref_t_ = now;
                frozen_ref_valid_ = true;
                return fb;
            }
            const double still = (now - frozen_ref_t_).toSec();
            if (still < frozen_sec_) return fb;

            // 新鲜 traj_server 消息不能证明飞机在移动或规划器仍活跃。
            const double spin_dur = 360.0 / std::max(spin_rate_deg_, 1.0);
            const double spun = still - frozen_sec_;

            if (spun >= spin_dur + retreat_hold_) {
                // 必须有"交回"这一步 —— 自转/回退期间飞机不会移动超过 1m, 静止判据会
                // 一直成立, 不给规划器机会就永远出不去了。
                frozen_ref_t_ = now;
                frozen_ref_ = { { cx, cy, cz } };
                frozen_spin_yaw0_valid_ = false;
                frozen_retreat_picked_ = false;
                return fb;
            }

            fb.valid = true;
            if (spun < spin_dur) {
                if (!frozen_spin_yaw0_valid_) {
                    frozen_spin_yaw0_deg_ = pose_sub_->getCurrentYaw() * 180.0 / M_PI;
                    frozen_spin_yaw0_valid_ = true;
                    ROS_ERROR("[脱困] 指令正常但 %.0fs 内移动不足 1m, 先原地自转一圈刷新感知",
                              frozen_sec_);
                    // 冻结恢复时强制刷新目标，重新启动全局规划。
                    if (!route_.empty()) {
                        const Waypoint& w = route_[wp_idx_];
                        publishEgoGoal(w.x, w.y, w.z, /*force=*/true);
                        ROS_WARN("[脱困] 已重发航点 %zu (%.1f, %.1f) 给 ego, 尝试唤醒规划器",
                                 wp_idx_ + 1, w.x, w.y);
                    }
                }
                fb.mode = Fallback::Mode::YAW_SPIN;
                fb.x = cx;
                fb.y = cy;
                fb.use_yaw = true;
                fb.yaw_deg = wrapDeg(frozen_spin_yaw0_deg_ + spin_rate_deg_ * spun);
                return fb;
            }

            if (!frozen_retreat_picked_) {
                if (!pickRetreatPoint(cx, cy)) {
                    fb.mode = Fallback::Mode::HOVER;   // 没航迹可退, 绝不盲飞
                    fb.x = cx;
                    fb.y = cy;
                    return fb;
                }
                frozen_retreat_picked_ = true;
            }
            fb.mode = Fallback::Mode::RETREAT;
            fb.x = escape_x_;
            fb.y = escape_y_;
            return fb;
        }

        // 以下: 没有新鲜指令
        frozen_ref_valid_ = false;
        if (!pos_cmd_sub_->hasCmd()) return fb;   // 从未收到过(刚进场): 交给调用方悬停

        if (cmd_lost_since_.isZero()) {
            cmd_lost_since_ = now;
            escape_target_valid_ = false;
            escape_yaw0_deg_ = pose_sub_->getCurrentYaw() * 180.0 / M_PI;
            ego_started_ = false;   // -> 恢复按周期重发目标, 配合 ego 的 respawn 自愈
            ROS_ERROR("[脱困] 已 %.1fs 没收到新的 ego 指令, 转入兜底(先原地悬停)", cmd_age);
        }
        const double lost = (now - cmd_lost_since_).toSec();

        // 锁住刚丢指令那一刻的位置, 作为第 1/2 层的定点(免得被漂移带走)
        if (!escape_target_valid_) {
            escape_x_ = cx;
            escape_y_ = cy;
            escape_target_valid_ = true;
        }

        fb.valid = true;
        fb.x = escape_x_;
        fb.y = escape_y_;

        if (lost < stuck_yaw_sec_) {
            fb.mode = Fallback::Mode::HOVER;
            return fb;
        }
        if (lost < stuck_retreat_sec_) {
            fb.mode = Fallback::Mode::YAW_SPIN;
            fb.use_yaw = true;
            fb.yaw_deg = wrapDeg(escape_yaw0_deg_ + spin_rate_deg_ * (lost - stuck_yaw_sec_));
            return fb;
        }

        // 每段回退只选择一次目标，避免 50Hz 重算使目标漂移。
        if (!retreat_picked_ || (now - retreat_picked_t_).toSec() > stuck_retreat_sec_) {
            if (pickRetreatPoint(cx, cy)) {
                retreat_picked_ = true;
                retreat_picked_t_ = now;
            } else {
                fb.mode = Fallback::Mode::HOVER;   // 没航迹(刚起飞就丢指令): 绝不盲飞
                return fb;
            }
        }
        fb.mode = Fallback::Mode::RETREAT;
        fb.x = escape_x_;
        fb.y = escape_y_;
        return fb;
    }

    // 从航迹尾部往回累加, 找到距当前位置约 retreat_dist_ 米的那个点。
    // 找到后写进 escape_x_/escape_y_, 并把该点之后的航迹截掉, 下一次回退会接着往前退。
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

    // 成员
    ros::NodeHandle& nh_;
    std::string drone_ns_;
    int drone_id_  = 1;
    int drone_num_ = 6;

    double min_expl_z_   = 4.1;
    double max_expl_z_   = 4.9;
    double z_soft_limit_ = 5.7;
    double z_hard_limit_ = 6.0;
    ros::Time z_over_since_;
    double init_x_ = 0.0, init_y_ = 0.0;
    double local_xy_scale_ = 1.0;

    // 航线
    std::vector<Waypoint> route_;
    size_t route_loop_start_ = 0;             // 前面的 entry_waypoints 只走一次
    std::string patrol_map_ = "base0";
    bool climb_in_place_ = false;
    size_t wp_idx_ = 0;
    bool route_loop_ = true;
    double route_z_ = 4.5;
    double start_delay_ = 0.0;
    int lap_count_ = 0;
    bool transit_from_track_ = false;         // 本次 TRANSIT 是不是跟踪结束后归航的(见 stepPatrol)
    double cruise_speed_ = 12.0, cruise_accel_ = 8.0;
    double entry_speed_ = 8.0, entry_accel_ = 6.0;
    double return_speed_ = 8.0, return_accel_ = 6.0;
    // 跟踪限速三档 (按与目标的水平距离滞回切换, 见 stepTrack):
    // far 快接近 / mid 制动+接近 / creep 跟随段 (只用于 EGO 远距逼近)
    double track_speed_far_ = 6.0,   track_acc_far_ = 4.0;
    double track_speed_near_ = 3.0,  track_acc_near_ = 3.0;
    double track_speed_creep_ = 2.0, track_acc_creep_ = 0.5;
    double track_near_dist_ = 20.0,  track_far_dist_ = 30.0;
    double track_creep_in_ = 9.0,    track_creep_out_ = 11.0;
    // far 档发给 EGO 的引导点距离 (m, 0=关闭; 见 stepTrack 跟随目标点)
    double track_far_goal_dist_ = 22.0;
    int track_tier_ = -1;            // -1=未定 0=far 1=mid 2=creep
    bool track_tier_inited_ = false;
    // 站位点参数 (跟随目标点)
    double standoff_radius_ = 5.0;   // 期望跟随距离
    double standoff_retreat_ = 4.0;  // 人冲近到此距离内时目标点外推 (盲区保护)
    double standoff_lead_sec_ = 0.0; // 站位前推秒数 (0=站位点严格在人 5m 圆上)
    double standoff_lead_max_ = 1.5; // 前推**距离**上限 (m): 2m/s 时压掉多余甩幅
    // 前推速度低通: 行人转向/急停时 KF 速度方向骤变, 前推项 (v*lead_sec) 1:1
    // 进目标点会让目标点沿弧线甩 (左右晃主因)。一阶时间常数平滑方向过渡。
    double standoff_lead_tau_ = 0.5;  // 前推速度低通时间常数 (s)
    double lead_vx_ = 0.0, lead_vy_ = 0.0;  // 平滑后的行人速度 (前推项用)
    bool   lead_valid_ = false;
    ros::Time lead_last_t_;
    double standoff_move_ = 0.25;
    double standoff_min_dt_ = 0.3;
    double standoff_force_d_ = 2.5;
    double standoff_r_scale_ = 1.0;     // 站位半径缩放 (卡死自愈逐级收缩, 到位恢复 1.0)
    double standoff_az_deadband_deg_ = 8.0;  // 方位死区 (deg)
    double standoff_az_alpha_ = 0.5;         // 方位低通系数 (0~1, 越小越平滑)
    double standoff_az_prev_ = 0.0;          // 上次方位角 (deg, 已 wrap)
    bool   standoff_az_valid_ = false;
    int    standoff_az_pick_ = -1;           // 诊断: 本拍选中的候选序号 (kOffsetsDeg 下标)
    // 以下两个只供 /tracking/standoff_diag_<id> 诊断话题用, 不参与任何控制
    double diag_az0_raw_deg_ = 0.0;
    double diag_r_eff_ = 0.0;
    ros::Time track_retreat_until_;     // 非零 = 正在沿航迹回退脱困 (卡死阶梯第 3 级)
    double track_retreat_x_ = 0.0, track_retreat_y_ = 0.0;  // 回退目标点 (本地帧)
    double track_follow_margin_ = 0.35; // 跟随段限速 = 行人速度 + 此余量
    double track_follow_floor_ = 1.0;   // 跟随段限速下限 (行人停下时不爬行)
    double track_follow_gap_ = 3.0;     // 距站位超过此余量 = 接近段(档位上限全速)
    double track_cap_slew_up_ = 1.2;    // 限速升速斜率 (m/s², 接近要快)
    double track_cap_slew_down_ = 0.6;  // 限速降速斜率 (m/s², 慢降不急刹)
    double track_acc_soft_ = 0.8;       // 降速时发给 ego 的加速度上限
    double track_cap_smooth_ = 0.0;     // 斜率限制后的实际下发限速
    bool   track_cap_init_ = false;     // 限速平滑器是否已起步
    bool   track_follow_seg_ = false;   // 当前在跟随段(距站位近)还是接近段

    // 近距 PID 与连续站位共用同一参考。
    bool   vel_loop_enable_ = true;
    double vel_loop_kp_ = 1.2, vel_loop_ki_ = 0.10, vel_loop_kd_ = 0.20;
    double vel_loop_err_tau_ = 0.3;     // 误差一阶低通 (s)
    // 速度前馈死区，单位 m/s。
    double vel_loop_ff_dead_ = 0.35;
    double vel_loop_acc_max_ = 1.5;     // 指令加速度斜率限幅 (m/s²)
    double vel_loop_vmax_ = 3.0;        // 速度指令幅值上限 (须 > 行人最高速)
    double vel_loop_enter_dist_ = 9.0,  vel_loop_exit_dist_ = 12.0;
    double vel_loop_enter_err_ = 3.5,   vel_loop_exit_err_ = 5.0;
    double vel_loop_lookahead_ = 2.5;   // 方向采样前视基线 (m)
    double vel_loop_clear_ = 0.55;      // 前视走廊半宽 (m)
    int    vel_loop_samples_ = 7;       // 每侧采样数 (15° 步进 -> ±105°)
    double vel_loop_w_turn_ = 0.6;      // 偏离当前速度方向的惩罚 (防左右横跳)
    // 跟踪避障 (原始雷达净空场, 见 track_obstacle_avoidance.hpp)
    bool   avoid_enable_ = true;
    double avoid_res_ = 0.2, avoid_memory_sec_ = 3.0, avoid_field_half_ = 14.0;
    double avoid_overhead_lidar_sec_ = 15.0;
    double avoid_thin_extent_ = 0.8;
    double avoid_block_sec_ = 0.6, avoid_holdoff_sec_ = 1.5;
    fly_px4::VelocityGuardParams guard_;
    fly_px4::StandoffObstacleParams standoff_obs_;
    fly_px4::ClearanceField avoid_field_;
    std::unique_ptr<LidarSurfaceSubscriber> lidar_surface_;
    ros::Time avoid_field_t_;           // 上次建场时刻 (10Hz 限频)
    bool   guard_active_ = false;       // 上一拍速度过滤在介入 (滞回)
    ros::Time guard_blocked_since_;     // 持续找不到前进方向的起点
    ros::Time vel_loop_holdoff_until_;  // 被堵交回 EGO 后暂不重进速度环
    ros::Time avoid_last_block_;
    double avoid_holdoff_cur_ = 1.5;
    bool   standoff_obs_degraded_ = false;
    double diag_standoff_wall_ = -1.0, diag_standoff_los_ = -1.0;
    double diag_guard_des_clear_ = -1.0, diag_guard_out_clear_ = -1.0;
    bool   vel_loop_active_ = false;    // 当前是否速度环在控 (滞回用)
    // 速度环内部状态
    bool   vl_init_ = false;
    double vl_ex_x_ = 0.0, vl_ex_y_ = 0.0;      // 低通后的位置误差
    double vl_ex_x_prev_ = 0.0, vl_ex_y_prev_ = 0.0;  // 上一拍低通误差 (D 项)
    double vl_ix_ = 0.0,   vl_iy_ = 0.0;        // 积分项 (有抗饱和)
    double vl_last_t_ = 0.0;                    // 上次 dt 计算时刻 (s)
    double vl_cmd_vx_ = 0.0, vl_cmd_vy_ = 0.0;  // 上一拍下发速度 (斜率限幅用)
    bool   vl_cmd_init_ = false;

    double arrive_radius_ = 1.5;
    double goal_repub_period_ = 2.0;
    double waypoint_obstacle_clear_ = 0.5;   // 航点距膨胀障碍点 < 此值判"在障碍内"
    double patrol_yaw_rate_max_ = 45.0;

    // 状态
    PatrolState state_ = PatrolState::CLIMB;
    bool climb_inited_ = false;
    double climb_x_ = 0.0, climb_y_ = 0.0, climb_yaw_deg_ = 0.0;
    double climb_min_z_ = 2.0;   // 离地安全高度: 之上才允许边爬边水平飞 (避开 <2m 低矮障碍)
    ros::Time climb_t0_;
    bool land_sent_ = false;
    bool transit_align_pending_ = false;  // 归航先对准机头(避免边飞边转撞障碍)
    // 归航目标发出后等 EGO 的新轨迹 (trajectory_id 变化, 最多 0.5s) 再转发, 期间原地保持
    bool return_traj_wait_ = false;
    uint32_t return_traj_id_ = 0;
    ros::Time return_traj_wait_t0_;
    double return_hold_yaw_deg_ = 0.0;

    // 订阅/发布
    std::unique_ptr<MavrosSubscriber>      mavros_sub_;
    std::unique_ptr<PoseSubscriber>        pose_sub_;
    std::unique_ptr<PosCmdSubscriber>      pos_cmd_sub_;
    std::unique_ptr<TrackTargetSubscriber> track_target_sub_;
    std::unique_ptr<TrackViewSubscriber>   track_view_sub_;
    std::unique_ptr<LocalVelocitySubscriber> local_velocity_sub_;
    std::unique_ptr<TrackModeSubscriber>   track_mode_sub_;
    std::vector<std::unique_ptr<PeerOdomSubscriber>> peer_subs_;
    std::unique_ptr<ObstacleMapSubscriber> obstacle_map_;
    ros::Subscriber mission_complete_sub_;

    ros::Publisher set_raw_pub_;
    ros::Publisher ego_goal_pub_;
    ros::Publisher ego_speed_pub_;
    ros::Publisher disable_collision_pub_;
    ros::Publisher ego_goal_vel_pub_;
    ros::Publisher reject_pub_;
    ros::Publisher tracking_active_pub_;
    std::string tracking_assignment_;
    ros::Publisher standoff_diag_pub_;   // /tracking/standoff_diag_<id> 诊断, 不参与控制
    ros::Publisher band_request_pub_;    // /<ns>/body_band_request
    ros::Time last_band_request_;
    ros::Publisher yaw_diag_pub_;
    ros::ServiceClient arming_cli_;
    ros::ServiceClient set_mode_cli_;
    ros::ServiceClient param_cli_;
    ros::Timer timer_;
    ros::Timer tracking_feedback_timer_;

    std::unique_ptr<ControlPx4> control_;
    std::unique_ptr<Px4GpsHeightSetup> gps_height_setup_;
    fly_px4::AltitudeHold altitude_hold_;
    bool altitude_reference_ready_ = false;

    ros::Time last_offboard_t_;        // 上次尝试切 OFFBOARD 的时刻 (0=未试过)
    ros::Time last_arm_t_;             // 上次尝试解锁的时刻 (0=未试过)
    double takeoff_retry_sec_ = 0.5;   // 起飞守卫重试间隔: 越小转浆/起飞越快 (launch: takeoff_retry_sec)
    double pose_timeout_ = 0.5;        // 本地位姿的接收和采样时间均须新鲜
    bool rc_exception_done_ = false;
    int rc_exception_failures_ = 0;
    ros::Time rc_exception_last_try_;

    // ego 交互
    bool ego_started_ = false;
    double ego_wait_warn_sec_ = 15.0;   // 等 ego 首条指令多久后升级成 ERROR
    double max_vz_ = 0.5;               // 转发 ego 指令时的垂直速度钳位 m/s
    double max_az_ = 1.0;               // 同上, 垂直加速度 m/s^2
    ros::Time patrol_entry_time_;       // 进入 TRANSIT 的时刻(等待计时起点)
    double ego_speed_v_ = -1.0, ego_speed_a_ = -1.0;
    bool disable_collision_inited_ = false;
    bool disable_collision_cur_ = false;
    bool patrol_collision_check_ = false;   // 巡逻阶段开 ego 完整避障 (launch 参数)
    bool goal_valid_ = false;
    double goal_x_ = 0.0, goal_y_ = 0.0, goal_z_ = 0.0;
    double goal_move_thresh_ = 1.0;
    ros::Time last_goal_pub_;

    // 跟踪
    bool mission_complete_ = false;
    double track_yaw_rate_max_ = 60.0;
    double track_yaw_acc_max_ = 180.0;
    double track_yaw_kp_ = 5.0;        // 参考角追赶增益，实际yaw闭环由PX4完成
    double track_yaw_deadband_deg_ = 5.0;  // 视线与机头参考角的死区；0关闭
    double track_view_timeout_ = 0.35;
    double track_view_conf_min_ = 0.4;
    double track_yaw_stationary_tau_ = 0.12;
    double track_yaw_moving_tau_ = 0.04;
    double track_yaw_velocity_timeout_ = 0.25;
    double track_stale_sec_ = 0.3;
    fly_px4::TrackYawController track_yaw_controller_;
    bool track_yaw_ready_ = false;
    std::string track_yaw_assignment_;
    double track_yaw_rate_rad_ = 0.0;
    double track_yaw_last_t_ = 0.0;
    double target_state_timeout_ = 1.0;
    double track_measurement_timeout_ = 1.0;
    double track_reacquire_nav_timeout_ = 25.0;
    double track_planner_start_grace_ = 3.0;
    fly_px4::TrackingPlannerWait track_planner_wait_;
    double target_future_tolerance_ = 0.05;
    bool reacquire_target_valid_ = false;
    double reacquire_target_x_ = 0.0, reacquire_target_y_ = 0.0;
    std::string reacquire_target_assignment_;
    double standoff_peer_clear_ = 3.5;
    double standoff_return_margin_ = 0.5;
    int standoff_candidate_ = -1;
    int standoff_ring_ = 0;              // 站位所在圈: 0=r_eff, 1/2=高度带障碍下外扩
    static constexpr double kStandoffBandRingStep = 2.5;
    double track_release_sec_ = 1.0;
    double reacquire_off_deg_ = 0.0;   // 站位方位偏转 (仅卡死自愈第 2 级用它换落点)
    bool standoff_valid_ = false;
    double standoff_x_ = 0.0, standoff_y_ = 0.0;
    bool   standoff_calc_ok_ = false;    // 本拍 selectStandoff 是否算出了可用站位点
    ros::Time last_standoff_pub_;
    ros::Time standoff_fail_since_;      // 首次选不到站位的时刻 (0=未在失败)
    double standoff_reject_delay_ = 2.0; // 连续选不到站位这么久才 reject (友机挡是暂时的)
    bool   standoff_band_blocked_ = false;
    bool   standoff_corridor_tier_ = false;  // 当前站位是走廊档 (离墙 < standoff_wall_clear)
    double corridor_safe_ref_ = 2.5;          // 走廊档下速度过滤的期望净空
    ros::Time corridor_ref_until_;  // 本拍选不到站位是因为全在屋檐/顶棚后
    double standoff_band_hold_sec_ = 10.0;  // 此时人在观察距离内则多看这么久再 reject
    double standoff_band_hold_dist_ = 12.0;
    double standoff_band_max_radius_ = 8.0;  // 屋檐/顶棚外扩站位的最大半径 (深度相机可测距离内)
    // 跟踪卡死自愈 (见 stepTrack)
    double track_z_ = std::numeric_limits<double>::quiet_NaN(); // 跟踪高度，必须从 YAML 加载
    double track_stuck_sec_ = 8.0;     // 判卡死的时间窗
    ros::Time track_stuck_t_;          // 卡死观测窗起点 (0=未在观测)
    double track_stuck_x_ = 0.0, track_stuck_y_ = 0.0;
    int track_stuck_stage_ = 0;        // 自愈阶梯: 1 重发目标 / 2 旋方位 / 3 reject
    int prev_track_mode_ = 0;
    ros::Time track_lost_since_;
    ros::Time last_reject_;
    double yaw_cmd_prev_deg_ = 0.0;
    bool yaw_cmd_inited_ = false;
    bool tracking_active_ = false;

    // 脱困
    double cmd_timeout_       = 2.0;
    double stuck_yaw_sec_     = 6.0;
    double stuck_retreat_sec_ = 12.0;
    double retreat_dist_      = 8.0;
    double frozen_sec_        = 25.0;
    double spin_rate_deg_     = 30.0;
    double retreat_hold_      = 8.0;
    double crash_z_           = 1.5;
    double crash_hold_        = 5.0;
    ros::Time crash_since_;
    bool has_flown_ = false;

    // 本地历史航迹每 0.5m 记录一点，用于沿原路回退。
    std::vector<std::array<double, 3>> trail_;
    static constexpr size_t kTrailMax = 120;   // 120 x 0.5m = 60m 记忆

    ros::Time cmd_lost_since_;
    bool escape_target_valid_ = false;
    bool retreat_picked_ = false;
    ros::Time retreat_picked_t_;
    double escape_x_ = 0.0, escape_y_ = 0.0;
    double escape_yaw0_deg_ = 0.0;

    std::array<double, 3> frozen_ref_{ { 0.0, 0.0, 0.0 } };
    ros::Time frozen_ref_t_;
    bool frozen_ref_valid_ = false;
    double frozen_spin_yaw0_deg_ = 0.0;
    bool frozen_spin_yaw0_valid_ = false;
    bool frozen_retreat_picked_ = false;
    bool escape_active_ = false;
};

// 主函数
int main(int argc, char** argv) {
    ros::init(argc, argv, "fly_px4_1_node");
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
            ros::spinOnce();
            ros::WallDuration(0.2).sleep();
        }
        if (ros::ok())
            ROS_INFO("[启动] 仿真时钟就绪 (now=%.3f), 继续启动状态机", ros::Time::now().toSec());
    }

    try {
        Task task(nh);
        ROS_INFO("fly_px4_1 启动 (马路巡逻版: 航点交给 ego 规划, 发现行人转跟踪)");
        ROS_INFO("· 状态流: CLIMB -> TRANSIT -> PATROL <-> TRACK -> LANDING -> DONE");
        ros::spin();
    } catch (const std::exception& error) {
        ROS_FATAL("fly_px4_1 无法启动或运行: %s", error.what());
        return 1;
    }
    return 0;
}
