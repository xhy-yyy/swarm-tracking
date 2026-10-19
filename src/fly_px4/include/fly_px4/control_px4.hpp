#pragma once

#include <ros/ros.h>
#include <mavros_msgs/PositionTarget.h>
#include <cmath>
#include <limits>

// 一阶低通，首次输入直接初始化。
class FirstOrderLPF {
public:
    // cutoff_hz: 截止频率 Hz; dt: 采样周期 s
    FirstOrderLPF(double cutoff_hz, double dt)
        : alpha_(dt / (dt + 1.0 / (2.0 * M_PI * cutoff_hz))),
          initialized_(false) {}

    double apply(double x) {
        if (!initialized_) {        // 首拍直接钉到目标值, 不做 0->target 的缓慢爬升
            val_ = x;
            initialized_ = true;
            return val_;
        }
        val_ += alpha_ * (x - val_);
        return val_;
    }

    void reset() { initialized_ = false; }

private:
    double alpha_ = 0.0;
    double val_   = 0.0;
    bool initialized_ = false;
};

// 封装 PositionTarget 指令，不负责飞行状态切换。
class ControlPx4 {
public:
    explicit ControlPx4(const ros::Publisher& raw_pub, double local_xy_scale = 1.0);

    // 各模式共享连续 Z 参考；RACER 默认关闭此功能。
    void set_altitude_reference(double z, double vz, double az);
    void clear_altitude_reference();

    // 位置 + 速度前馈 + 航向(定点转向): 提供 PX/PY/PZ/VX/VY/VZ/YAW, 忽略 AF*。
    // yaw_rate 为有限值时提供偏航角速度前馈(rad/s)，默认忽略 YAW_RATE。
    void fly_posvel_yaw(double x, double y, double z,
                        double vx, double vy, double vz, double yaw_deg,
                        double yaw_rate = std::numeric_limits<double>::quiet_NaN());

    // XY 速度环 + Z 位置锁高 + 航向: 提供 VX/VY/PZ/YAW/YAW_RATE, 忽略 PX/PY/AF*
    void fly_velxy_posez_yaw(double vx, double vy, double z,
                             double yaw_deg, double yaw_rate);

    // 三轴速度(不锁位置, 不干预 yaw): 提供 VX/VY/VZ, 忽略 P*/AF*/YAW/YAW_RATE
    void fly_velxyz(double vx, double vy, double vz);

    // XY 零速 + Z 定高(悬停, 不干预 yaw): 提供 VX/VY/PZ, 忽略 PX/PY/VZ/AF*/YAW/YAW_RATE
    void fly_hover(double z);

    // XY 使用原始 PVA，Z 使用共享参考；可选 yaw_rate 单位 rad/s。
    void fly_pva_xy_posez_yaw(double x, double y, double z,
                             double vx, double vy, double ax, double ay,
                             double yaw_deg,
                             double yaw_rate = std::numeric_limits<double>::quiet_NaN());

    // RACER 三维 PVA，Z 按 min_z 钳位；可选 yaw_rate 单位 rad/s。
    void fly_pva_yaw(double x, double y, double z,
                     double vx, double vy, double vz,
                     double ax, double ay, double az,
                     double yaw_deg, double min_z = 0.0,
                     double yaw_rate = std::numeric_limits<double>::quiet_NaN());

private:
    // 模式切换重置 XY 滤波，Z 参考保持连续。
    enum class XyFilterMode { NONE, POSITION, VELOCITY };
    void select_xy_filter(XyFilterMode mode);

    // 角度归一化到 (-180, 180] 并转弧度; 返回 type_mask 中是否需要忽略 yaw(本层不忽略, 留接口对称)
    static double yaw_deg_to_rad(double yaw_deg);
    double altitude_position(double z);
    void publish_local_target(mavros_msgs::PositionTarget target);

    ros::Publisher raw_pub_;
    double local_xy_scale_;
    bool altitude_reference_active_ = false;
    double altitude_reference_z_ = 0.0;
    double altitude_reference_vz_ = 0.0;
    double altitude_reference_az_ = 0.0;

    // 指令低通滤波(50Hz 采样): 位置截止低些抗噪, 速度截止高些抗滞后
    FirstOrderLPF pos_lpf_x_;
    FirstOrderLPF pos_lpf_y_;
    FirstOrderLPF pos_lpf_z_;
    FirstOrderLPF vel_lpf_x_;
    FirstOrderLPF vel_lpf_y_;
    FirstOrderLPF vel_lpf_z_;
    XyFilterMode xy_filter_mode_ = XyFilterMode::NONE;
};
