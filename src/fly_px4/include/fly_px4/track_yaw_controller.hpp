#pragma once

#include <algorithm>
#include <cmath>
#include <deque>

namespace fly_px4 {

// Angles are world-frame yaw in radians; times are seconds in one clock domain.
struct TrackYawConfig {
    double rate_max = 1.0471975511965976;  // 60 deg/s
    double acc_max = 3.1415926535897932;  // 180 deg/s^2
    double kp = 5.0;                    // Reference tracking, not aircraft feedback.
    double deadband = 0.08726646259971647;  // 5 deg; zero disables the deadband.
    double stationary_tau = 0.12;
    double moving_tau = 0.04;
    double bearing_timeout = 0.3;
};

struct TrackYawCommand {
    double yaw = 0.0;
    double rate = 0.0;
    bool visual_active = false;
};

// 按图像时间更新观测，输出受限且一致的 yaw/yaw_rate 参考。
class TrackYawController {
public:
    void configure(const TrackYawConfig& config) {
        config_ = config;
        const TrackYawConfig defaults;
        config_.rate_max = positive(config_.rate_max, defaults.rate_max);
        config_.acc_max = positive(config_.acc_max, defaults.acc_max);
        config_.kp = positive(config_.kp, defaults.kp);
        config_.deadband = std::isfinite(config_.deadband) && config_.deadband >= 0.0
                           ? std::min(config_.deadband, 3.1415926535897932)
                           : defaults.deadband;
        config_.stationary_tau = positive(config_.stationary_tau, defaults.stationary_tau);
        config_.moving_tau = positive(config_.moving_tau, defaults.moving_tau);
        config_.bearing_timeout = positive(config_.bearing_timeout, defaults.bearing_timeout);
        rate_ = clamp(rate_, -config_.rate_max, config_.rate_max);
    }

    void reset(double yaw_rad, double now_sec) {
        yaw_ = std::isfinite(yaw_rad) ? wrap(yaw_rad) : 0.0;
        rate_ = 0.0;
        last_step_ = std::isfinite(now_sec) ? now_sec : 0.0;
        initialized_ = true;
        clearObservation();
    }

    void observe(double bearing_rad, double stamp_sec) {
        if (!std::isfinite(bearing_rad) || !std::isfinite(stamp_sec)) return;
        // Duplicate/late images must not be counted as motion confirmations.
        if (observed_ && stamp_sec <= received_stamp_) return;
        received_stamp_ = stamp_sec;

        if (!observed_ || stamp_sec - observation_stamp_ > config_.bearing_timeout) {
            const double anchor = initialized_ ? yaw_ : bearing_rad;
            raw_bearing_ = anchor + wrap(bearing_rad - anchor);
            bearing_ = raw_bearing_;
            bearing_rate_ = 0.0;
            observation_stamp_ = stamp_sec;
            observed_ = true;
            moving_ = false;
            pending_jump_ = false;
            pipeline_age_valid_ = false;
            sample_dt_ = 0.05;
            samples_.clear();
            samples_.push_back(Sample(stamp_sec, raw_bearing_));
            return;
        }

        const double dt = stamp_sec - observation_stamp_;
        const double previous_raw = raw_bearing_;
        const double raw = previous_raw + wrap(bearing_rad - previous_raw);
        // 突跳观测需要下一帧确认。
        if (std::fabs(raw - previous_raw) > kJumpAngle
            && std::fabs(raw - previous_raw) > 2.0 * config_.rate_max * dt) {
            if (pending_jump_ && std::fabs(wrap(raw - pending_bearing_)) < kJumpAgreement) {
                raw_bearing_ = raw;
                bearing_ = raw;
                bearing_rate_ = 0.0;
                observation_stamp_ = stamp_sec;
                moving_ = false;
                pending_jump_ = false;
                samples_.clear();
                samples_.push_back(Sample(stamp_sec, raw));
            } else {
                pending_jump_ = true;
                pending_bearing_ = raw;
            }
            return;
        }
        pending_jump_ = false;
        const double raw_rate = (raw - previous_raw) / dt;
        sample_dt_ = clamp(0.7 * sample_dt_ + 0.3 * dt, 0.005, 0.5);

        // On the first credible reverse/stop sample, withdraw the old velocity.
        // Fast gain and a new velocity still require two coherent intervals.
        const bool reverse = moving_ && raw_rate * bearing_rate_ < 0.0
                             && std::fabs(raw - previous_raw) > kReverseAngle
                             && std::fabs(raw_rate) > kMotionRate;
        const bool stopped = moving_ && std::fabs(raw_rate) < kMotionRate * 0.5;
        if (reverse || stopped) {
            bearing_rate_ = 0.0;
            moving_ = false;
            samples_.clear();
            samples_.push_back(Sample(observation_stamp_, previous_raw));
        }

        samples_.push_back(Sample(stamp_sec, raw));
        // Three samples are necessary to confirm two intervals, including at
        // the normal 10 Hz image rate. Adapt the fit horizon to slower cameras.
        const double rate_window = kRateWindow + 2.0 * sample_dt_;
        while (samples_.size() > 3 && stamp_sec - samples_.front().stamp > rate_window)
            samples_.pop_front();

        bool confirmed_motion = false;
        if (samples_.size() >= 3) {
            const Sample& a = samples_[samples_.size() - 3];
            const Sample& b = samples_[samples_.size() - 2];
            const Sample& c = samples_.back();
            const double v1 = (b.bearing - a.bearing) / (b.stamp - a.stamp);
            const double v2 = (c.bearing - b.bearing) / (c.stamp - b.stamp);
            const double net = c.bearing - samples_.front().bearing;
            // A lone displaced box followed by a return cannot turn on the
            // predictive branch. Small stationary box jitter also stays slow.
            confirmed_motion = v1 * v2 > 0.0 && v2 * net > 0.0
                               && std::fabs(v1) > kMotionRate
                               && std::fabs(v2) > kMotionRate
                               && std::fabs(net) > kMotionAngle;
        }

        const double tau = confirmed_motion ? config_.moving_tau : config_.stationary_tau;
        const double gain = -std::expm1(-dt / tau);
        const double predicted = bearing_ + bearing_rate_ * dt;
        bearing_ = predicted + gain * (raw - predicted);
        if (confirmed_motion) {
            const double fit = clamp(fittedRate(), -2.0 * config_.rate_max, 2.0 * config_.rate_max);
            bearing_rate_ += gain * (fit - bearing_rate_);
        } else {
            bearing_rate_ = 0.0;
        }
        moving_ = confirmed_motion;
        raw_bearing_ = raw;
        observation_stamp_ = stamp_sec;
    }

    TrackYawCommand step(double now_sec, double fallback_bearing_rad,
                         double fallback_rate_radps = 0.0,
                         bool fallback_rate_valid = false) {
        if (!initialized_) reset(fallback_bearing_rad, now_sec);
        if (!std::isfinite(now_sec)) return command(false);
        const double dt = now_sec - last_step_;
        if (dt < 0.0) {
            // The caller can reset to measured aircraft yaw on a simulation
            // restart. Internally preserve the last reference and forget old
            // image timestamps rather than integrating a negative interval.
            reset(yaw_, now_sec);
            return command(false);
        }
        const double age = observed_ ? now_sec - observation_stamp_ : config_.bearing_timeout + 1.0;
        const bool visual = observed_ && age >= -kFutureTolerance && age <= config_.bearing_timeout;
        if (observed_ && age < -kFutureTolerance) clearObservation();
        if (visual && (!pipeline_age_valid_ || observation_stamp_ != used_observation_stamp_)) {
            const double delay = std::max(0.0, age);
            pipeline_age_ = pipeline_age_valid_ ? 0.75 * pipeline_age_ + 0.25 * delay : delay;
            pipeline_age_valid_ = true;
            used_observation_stamp_ = observation_stamp_;
        }
        if (dt == 0.0) return command(visual);

        double want = std::isfinite(fallback_bearing_rad)
                      ? yaw_ + wrap(fallback_bearing_rad - yaw_) : yaw_;
        double feedforward = fallback_rate_valid && std::isfinite(fallback_rate_radps)
                             ? fallback_rate_radps : 0.0;
        if (visual) {
            const double image_age = std::max(0.0, age);
            // 仅补偿图像到当前的延迟；失检后衰减角速度，积分与速率参考保持一致。
            const double coast = std::min(config_.bearing_timeout,
                                          std::max(0.06, pipeline_age_ + 1.5 * sample_dt_));
            const double extra_age = std::max(0.0, image_age - coast);
            const double rate_weight = std::exp(-extra_age / kCoastTau);
            const double prediction_time = std::min(image_age, coast)
                                           + kCoastTau * (1.0 - rate_weight);
            const double visual_bearing = bearing_ + bearing_rate_ * prediction_time;
            const double fade_span = config_.bearing_timeout - coast;
            const double fade = fade_span > 0.0 ? clamp(extra_age / fade_span, 0.0, 1.0) : 0.0;
            const double weight = 1.0 - fade * fade * (3.0 - 2.0 * fade);
            want += weight * wrap(visual_bearing - want);
            feedforward += weight * (bearing_rate_ * rate_weight - feedforward);
        }

        // want is at now, while yaw_ still represents the previous step. Put
        // the proportional error at the same epoch before adding its velocity
        // feedforward; otherwise constant motion gains an unintended dt lead.
        const double error = wrap(want - feedforward * dt - yaw_);
        // Compare the current LOS with the held reference, using the shortest
        // angular distance. Suppress both feedback and feedforward inside the
        // deadband; keep the acceleration bound while braking to a hold.
        const bool inside_deadband = config_.deadband > 0.0
                                     && std::fabs(wrap(want - yaw_)) <= config_.deadband;
        const double desired_rate = inside_deadband ? 0.0
            : clamp(feedforward + config_.kp * error, -config_.rate_max, config_.rate_max);
        rate_ += clamp(desired_rate - rate_, -config_.acc_max * dt, config_.acc_max * dt);
        // This discrete derivative is exactly the rate we transmit. Do not
        // clamp yaw independently or send the unclamped LOS feedforward.
        yaw_ += rate_ * dt;
        last_step_ = now_sec;
        if (observed_ && age > config_.bearing_timeout) clearObservation();
        return command(visual);
    }

private:
    struct Sample {
        Sample(double t, double b) : stamp(t), bearing(b) {}
        double stamp;
        double bearing;
    };

    static double wrap(double angle) {
        const double pi = 3.14159265358979323846;
        return std::remainder(angle, 2.0 * pi);
    }
    static double clamp(double value, double lo, double hi) {
        return std::max(lo, std::min(hi, value));
    }
    static double positive(double value, double fallback) {
        return std::isfinite(value) && value > 0.0 ? value : fallback;
    }
    double fittedRate() const {
        const double origin = samples_.back().stamp;
        double mean_t = 0.0, mean_b = 0.0;
        for (const Sample& sample : samples_) {
            mean_t += sample.stamp - origin;
            mean_b += sample.bearing;
        }
        mean_t /= samples_.size();
        mean_b /= samples_.size();
        double numerator = 0.0, denominator = 0.0;
        for (const Sample& sample : samples_) {
            const double t = sample.stamp - origin - mean_t;
            numerator += t * (sample.bearing - mean_b);
            denominator += t * t;
        }
        return denominator > 1e-12 ? numerator / denominator : 0.0;
    }
    void clearObservation() {
        observed_ = false;
        moving_ = false;
        pending_jump_ = false;
        pipeline_age_valid_ = false;
        bearing_rate_ = 0.0;
        samples_.clear();
    }
    TrackYawCommand command(bool visual) const {
        TrackYawCommand result;
        result.yaw = wrap(yaw_);
        result.rate = rate_;
        result.visual_active = visual;
        return result;
    }

    static constexpr double kMotionRate = 0.06;
    static constexpr double kMotionAngle = 0.012;
    static constexpr double kReverseAngle = 0.004;
    static constexpr double kRateWindow = 0.12;
    static constexpr double kCoastTau = 0.06;
    static constexpr double kFutureTolerance = 0.02;
    static constexpr double kJumpAngle = 0.06;
    static constexpr double kJumpAgreement = 0.025;
    TrackYawConfig config_;
    bool initialized_ = false;
    bool observed_ = false;
    bool moving_ = false;
    bool pending_jump_ = false;
    bool pipeline_age_valid_ = false;
    double yaw_ = 0.0;
    double rate_ = 0.0;
    double last_step_ = 0.0;
    double observation_stamp_ = 0.0;
    double received_stamp_ = 0.0;
    double pending_bearing_ = 0.0;
    double pipeline_age_ = 0.0;
    double used_observation_stamp_ = 0.0;
    double raw_bearing_ = 0.0;
    double bearing_ = 0.0;
    double bearing_rate_ = 0.0;
    double sample_dt_ = 0.05;
    std::deque<Sample> samples_;
};

}  // namespace fly_px4
