#include "fly_px4/control_px4.hpp"
#include <stdexcept>

// 50Hz 定时器周期
static constexpr double DT = 0.02;

ControlPx4::ControlPx4(const ros::Publisher& raw_pub, double local_xy_scale)
    : raw_pub_(raw_pub),
      local_xy_scale_(local_xy_scale),
      // 位置锁死滤波: 截止 1.0Hz (fly_a9 用 0.5Hz 带跳变保护, 这里不带跳变保护故调高减滞后)
      pos_lpf_x_(1.0, DT),
      pos_lpf_y_(1.0, DT),
      pos_lpf_z_(1.0, DT),
      // 速度指令本身已光滑(3D PID 内部有加速度限幅), 截止高些降滞后
      vel_lpf_x_(2.5, DT),
      vel_lpf_y_(2.5, DT),
      vel_lpf_z_(2.5, DT) {
    if (!std::isfinite(local_xy_scale_) || local_xy_scale_ <= 0.0)
        throw std::invalid_argument("local_xy_scale must be finite and positive");
}

void ControlPx4::set_altitude_reference(double z, double vz, double az) {
    if (!std::isfinite(z) || !std::isfinite(vz) || !std::isfinite(az))
        throw std::invalid_argument("altitude reference must be finite");
    altitude_reference_z_ = z;
    altitude_reference_vz_ = vz;
    altitude_reference_az_ = az;
    altitude_reference_active_ = true;
}

void ControlPx4::clear_altitude_reference() {
    altitude_reference_active_ = false;
}

void ControlPx4::select_xy_filter(XyFilterMode mode) {
    if (mode == xy_filter_mode_) return;
    if (mode == XyFilterMode::POSITION) {
        pos_lpf_x_.reset();
        pos_lpf_y_.reset();
    } else if (mode == XyFilterMode::VELOCITY) {
        vel_lpf_x_.reset();
        vel_lpf_y_.reset();
        vel_lpf_z_.reset();
    }
    xy_filter_mode_ = mode;
}

double ControlPx4::altitude_position(double z) {
    return altitude_reference_active_ ? altitude_reference_z_ : pos_lpf_z_.apply(z);
}

void ControlPx4::publish_local_target(mavros_msgs::PositionTarget target) {
    if (altitude_reference_active_) {
        // All flight paths share this one reference. Preserve horizontal and
        // yaw fields/masks, including PID, trajectory and fallback commands.
        target.header.stamp = ros::Time::now();
        target.position.z = altitude_reference_z_;
        target.velocity.z = altitude_reference_vz_;
        target.acceleration_or_force.z = altitude_reference_az_;
        target.type_mask &= ~(mavros_msgs::PositionTarget::IGNORE_PZ |
                              mavros_msgs::PositionTarget::IGNORE_VZ |
                              mavros_msgs::PositionTarget::IGNORE_AFZ);
    }
    // Internal positions are physical local metres. Convert once at the
    // MAVROS boundary, including derivatives; Z and yaw keep their units.
    target.position.x /= local_xy_scale_;
    target.position.y /= local_xy_scale_;
    target.velocity.x /= local_xy_scale_;
    target.velocity.y /= local_xy_scale_;
    target.acceleration_or_force.x /= local_xy_scale_;
    target.acceleration_or_force.y /= local_xy_scale_;
    raw_pub_.publish(target);
}

double ControlPx4::yaw_deg_to_rad(double yaw_deg) {
    yaw_deg = std::fmod(yaw_deg, 360.0);
    if (yaw_deg > 180.0)      yaw_deg -= 360.0;
    else if (yaw_deg <= -180.0) yaw_deg += 360.0;
    return yaw_deg * M_PI / 180.0;
}

// 位置 + 速度前馈 + 航向(起飞用: 锁起飞点 XY + 目标高度, 速度前馈=0)
void ControlPx4::fly_posvel_yaw(double x, double y, double z,
                                double vx, double vy, double vz, double yaw_deg,
                                double yaw_rate) {
    select_xy_filter(XyFilterMode::POSITION);
    mavros_msgs::PositionTarget target;
    // position/velocity/yaw 全部保留；有限 yaw_rate 启用角速度前馈。
    target.type_mask = mavros_msgs::PositionTarget::IGNORE_AFX |
                       mavros_msgs::PositionTarget::IGNORE_AFY |
                       mavros_msgs::PositionTarget::IGNORE_AFZ;
    if (std::isfinite(yaw_rate)) target.yaw_rate = yaw_rate;
    else target.type_mask |= mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
    target.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    target.position.x = pos_lpf_x_.apply(x);
    target.position.y = pos_lpf_y_.apply(y);
    target.position.z = altitude_position(z);
    target.velocity.x = vx;
    target.velocity.y = vy;
    target.velocity.z = vz;
    target.yaw = yaw_deg_to_rad(yaw_deg);
    publish_local_target(target);
}

// XY 速度环 + Z 位置锁高 + 航向(3D PID 打点用: XY 走速度环逼近, Z 走 PX4 位置环锁高)
void ControlPx4::fly_velxy_posez_yaw(double vx, double vy, double z,
                                     double yaw_deg, double yaw_rate) {
    select_xy_filter(XyFilterMode::VELOCITY);
    mavros_msgs::PositionTarget target;
    // 提供: VX, VY(速度环), PZ(位置锁), YAW, YAW_RATE; 忽略: PX, PY, AF*
    target.type_mask = mavros_msgs::PositionTarget::IGNORE_PX |
                       mavros_msgs::PositionTarget::IGNORE_PY |
                       mavros_msgs::PositionTarget::IGNORE_AFX |
                       mavros_msgs::PositionTarget::IGNORE_AFY |
                       mavros_msgs::PositionTarget::IGNORE_AFZ;
    target.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    target.velocity.x = vel_lpf_x_.apply(vx);
    target.velocity.y = vel_lpf_y_.apply(vy);
    target.velocity.z = 0.0;
    target.position.z = altitude_position(z);
    target.yaw     = yaw_deg_to_rad(yaw_deg);
    target.yaw_rate = yaw_rate;
    publish_local_target(target);
}

// 三轴速度(地面待命心跳: 全零速, 满足 PX4 进 OFFBOARD 前须有 >2Hz setpoint 流的前置条件)
void ControlPx4::fly_velxyz(double vx, double vy, double vz) {
    select_xy_filter(XyFilterMode::VELOCITY);
    mavros_msgs::PositionTarget target;
    target.type_mask = mavros_msgs::PositionTarget::IGNORE_PX |
                       mavros_msgs::PositionTarget::IGNORE_PY |
                       mavros_msgs::PositionTarget::IGNORE_PZ |
                       mavros_msgs::PositionTarget::IGNORE_AFX |
                       mavros_msgs::PositionTarget::IGNORE_AFY |
                       mavros_msgs::PositionTarget::IGNORE_AFZ |
                       mavros_msgs::PositionTarget::IGNORE_YAW |
                       mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
    target.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    target.velocity.x = vel_lpf_x_.apply(vx);
    target.velocity.y = vel_lpf_y_.apply(vy);
    target.velocity.z = vel_lpf_z_.apply(vz);
    publish_local_target(target);
}

// XY 零速 + Z 定高(悬停)
void ControlPx4::fly_hover(double z) {
    select_xy_filter(XyFilterMode::NONE);
    mavros_msgs::PositionTarget target;
    target.type_mask = mavros_msgs::PositionTarget::IGNORE_PX |
                       mavros_msgs::PositionTarget::IGNORE_PY |
                       mavros_msgs::PositionTarget::IGNORE_VZ |
                       mavros_msgs::PositionTarget::IGNORE_AFX |
                       mavros_msgs::PositionTarget::IGNORE_AFY |
                       mavros_msgs::PositionTarget::IGNORE_AFZ |
                       mavros_msgs::PositionTarget::IGNORE_YAW |
                       mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
    target.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    target.velocity.x = 0.0;
    target.velocity.y = 0.0;
    target.position.z = altitude_position(z);
    publish_local_target(target);
}

// 二维轨迹: 保留水平 PVA，Z 始终走同一个位置参考，不接收轨迹垂直前馈。
void ControlPx4::fly_pva_xy_posez_yaw(double x, double y, double z,
                                     double vx, double vy, double ax, double ay,
                                     double yaw_deg, double yaw_rate) {
    select_xy_filter(XyFilterMode::NONE);
    mavros_msgs::PositionTarget target;
    target.header.stamp = ros::Time::now();
    target.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    target.type_mask = mavros_msgs::PositionTarget::IGNORE_VZ |
                       mavros_msgs::PositionTarget::IGNORE_AFZ;
    if (std::isfinite(yaw_rate)) target.yaw_rate = yaw_rate;
    else target.type_mask |= mavros_msgs::PositionTarget::IGNORE_YAW_RATE;

    target.position.x = x;
    target.position.y = y;
    target.position.z = altitude_position(z);
    target.velocity.x = vx;
    target.velocity.y = vy;
    target.acceleration_or_force.x = ax;
    target.acceleration_or_force.y = ay;
    target.yaw = yaw_deg_to_rad(yaw_deg);
    publish_local_target(target);
}

// 位置+速度+加速度+航向(EXPLORE 用: 转发 RACER PositionCommand, 对应 ego_to_mavros 逻辑).
// RACER B 样条输出已光滑, 不走 LPF 直接下发; min_z>0 时对 Z 钳位下限.
void ControlPx4::fly_pva_yaw(double x, double y, double z,
                             double vx, double vy, double vz,
                             double ax, double ay, double az,
                             double yaw_deg, double min_z, double yaw_rate) {
    select_xy_filter(XyFilterMode::NONE);
    mavros_msgs::PositionTarget target;
    target.header.stamp = ros::Time::now();
    target.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    // P/V/AF/YAW 全部保留；有限 yaw_rate 启用角速度前馈。
    target.type_mask = 0;
    if (std::isfinite(yaw_rate)) target.yaw_rate = yaw_rate;
    else target.type_mask |= mavros_msgs::PositionTarget::IGNORE_YAW_RATE;

    target.position.x = x;
    target.position.y = y;
    target.position.z = (min_z > 0.0 && z < min_z) ? min_z : z;

    target.velocity.x = vx;
    target.velocity.y = vy;
    target.velocity.z = vz;

    target.acceleration_or_force.x = ax;
    target.acceleration_or_force.y = ay;
    target.acceleration_or_force.z = az;

    target.yaw = yaw_deg_to_rad(yaw_deg);
    publish_local_target(target);
}
