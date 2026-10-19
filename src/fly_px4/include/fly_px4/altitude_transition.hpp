#pragma once

#include <algorithm>
#include <cmath>

namespace fly_px4 {

struct AltitudeTransitionConfig {
    double max_vz = 0.5;
    double max_az = 1.0;
    double xy_speed_tolerance = 0.2;
    double z_speed_tolerance = 0.1;
    double z_tolerance = 0.15;
    double stable_time = 0.3;
};

struct AltitudeReference {
    double x = 0.0, y = 0.0, z = 0.0, vz = 0.0, az = 0.0;
    bool complete = false;
};

// Horizontal braking, a fixed-XY quintic vertical reference, and measured
// settling. Time alone cannot authorize the horizontal controller.
class AltitudeTransition {
public:
    enum class Phase { BRAKE, RAMP, SETTLE, DONE };

    void configure(const AltitudeTransitionConfig& config) { config_ = config; }

    void begin(double now, double x, double y, double z, double target_z) {
        x_ = x; y_ = y; start_z_ = z; target_z_ = target_z;
        phase_ = Phase::BRAKE;
        stable_since_ = -1.0;
        last_time_ = now;
        restart_ = false;
    }

    void suspend() { restart_ = true; }
    bool complete() const { return phase_ == Phase::DONE && !restart_; }
    double targetZ() const { return target_z_; }
    Phase phase() const { return phase_; }

    AltitudeReference step(double now, double x, double y, double z,
                           double vx, double vy, double vz, bool feedback_valid) {
        if (restart_ || now < last_time_) begin(now, x, y, z, target_z_);
        last_time_ = now;
        if (!feedback_valid || !std::isfinite(vx) || !std::isfinite(vy) ||
                !std::isfinite(vz)) {
            // Feedback loss discards both the running clock and any settling
            // evidence. Resume from the measured position on recovery.
            restart_ = true;
            return reference(z, 0.0, 0.0);
        }
        const bool still = std::hypot(vx, vy) <= config_.xy_speed_tolerance &&
                           std::fabs(vz) <= config_.z_speed_tolerance;
        if (phase_ == Phase::BRAKE) {
            if (stable(now, still)) {
                // Do not pull back to the position before inertial braking.
                x_ = x; y_ = y; start_z_ = z;
                ramp_start_ = now;
                const double distance = std::fabs(target_z_ - start_z_);
                duration_ = std::max(0.25, std::max(
                    1.875 * distance / config_.max_vz,
                    std::sqrt(5.773503 * distance / config_.max_az)));
                phase_ = Phase::RAMP;
                stable_since_ = -1.0;
            }
            return reference(start_z_, 0.0, 0.0);
        }
        if (phase_ == Phase::RAMP) {
            const double u = std::max(0.0, std::min(1.0, (now - ramp_start_) / duration_));
            const double u2 = u*u, u3 = u2*u, u4 = u3*u, u5 = u4*u;
            const double distance = target_z_ - start_z_;
            if (u < 1.0) return reference(
                start_z_ + distance * (10*u3 - 15*u4 + 6*u5),
                distance * (30*u2 - 60*u3 + 30*u4) / duration_,
                distance * (60*u - 180*u2 + 120*u3) / (duration_*duration_));
            phase_ = Phase::SETTLE;
            stable_since_ = -1.0;
        }
        if (phase_ == Phase::SETTLE && stable(now,
                still && std::fabs(z - target_z_) <= config_.z_tolerance))
            phase_ = Phase::DONE;
        return reference(target_z_, 0.0, 0.0);
    }

private:
    bool stable(double now, bool condition) {
        if (!condition) { stable_since_ = -1.0; return false; }
        if (stable_since_ < 0.0) stable_since_ = now;
        return now - stable_since_ >= config_.stable_time;
    }
    AltitudeReference reference(double z, double vz, double az) const {
        AltitudeReference result;
        result.x = x_; result.y = y_; result.z = z;
        result.vz = vz; result.az = az; result.complete = complete();
        return result;
    }
    AltitudeTransitionConfig config_;
    Phase phase_ = Phase::BRAKE;
    double x_ = 0.0, y_ = 0.0, start_z_ = 0.0, target_z_ = 0.0;
    double last_time_ = 0.0, ramp_start_ = 0.0, duration_ = 0.25;
    double stable_since_ = -1.0;
    bool restart_ = true;
};

}  // namespace fly_px4
