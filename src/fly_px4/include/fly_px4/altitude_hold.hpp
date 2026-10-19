#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace fly_px4 {

struct AltitudeHoldCommand {
    double z = 0.0;
    double vz = 0.0;
    double az = 0.0;
};

// One vertical reference shared by all horizontal control modes. EGO supplies
// only XY; changing between PID, EGO and hover must not reset this reference.
// Configuration is normally performed once, before reset/flight.
class AltitudeHold {
public:
    void configure(double max_vz, double max_az) {
        if (!finitePositive(max_vz) || !finitePositive(max_az))
            throw std::invalid_argument("altitude limits must be finite and positive");
        max_vz_ = max_vz;
        max_az_ = max_az;
    }

    void reset(double now, double z) {
        validate(now, z);
        current_.z = z;
        current_.vz = current_.az = 0.0;
        target_z_ = z;
        last_time_ = now;
        phase_ = Phase::HOLD;
        initialized_ = true;
    }

    void setTarget(double now, double z) {
        validate(now, z);
        requireInitialized();
        step(now);
        // Repeating a state height at 50 Hz never restarts its transition.
        if (z == target_z_) return;
        target_z_ = z;
        restart(now);
    }

    AltitudeHoldCommand step(double now) {
        if (!std::isfinite(now))
            throw std::invalid_argument("altitude reference time must be finite");
        requireInitialized();
        if (now < last_time_) {
            // A simulator clock reset discards the old future schedule. Anchor
            // at the last emitted position/velocity, brake any motion, then
            // restart toward the same target on the new clock.
            restart(now);
        }
        last_time_ = now;

        if (phase_ == Phase::BRAKE) {
            const double elapsed = std::max(0.0, now - phase_start_);
            if (elapsed < duration_) {
                current_.z = start_z_ + start_vz_ * elapsed
                    + 0.5 * brake_az_ * elapsed * elapsed;
                current_.vz = start_vz_ + brake_az_ * elapsed;
                current_.az = brake_az_;
                return current_;
            }
            // Advance from the physical stopping point, including when one
            // timer call spans both the braking and quintic phases.
            current_.z = start_z_ + 0.5 * start_vz_ * duration_;
            current_.vz = current_.az = 0.0;
            beginRamp(phase_start_ + duration_);
        }

        if (phase_ == Phase::RAMP) {
            const double u = std::max(0.0, (now - phase_start_) / duration_);
            if (u < 1.0) {
                const double distance = target_z_ - start_z_;
                const double u2 = u * u;
                const double u3 = u2 * u;
                const double remaining = 1.0 - u;
                current_.z = start_z_ + distance * u3 * (10.0 + u * (-15.0 + 6.0 * u));
                current_.vz = distance * 30.0 * u2 * remaining * remaining / duration_;
                current_.az = distance * 60.0 * u * remaining * (1.0 - 2.0 * u)
                    / (duration_ * duration_);
                return current_;
            }
            phase_ = Phase::HOLD;
        }

        current_.z = target_z_;
        current_.vz = current_.az = 0.0;
        return current_;
    }

    bool initialized() const { return initialized_; }
    bool transitioning() const { return initialized_ && phase_ != Phase::HOLD; }
    double targetZ() const { return target_z_; }

private:
    enum class Phase { HOLD, BRAKE, RAMP };

    static bool finitePositive(double value) {
        return std::isfinite(value) && value > 0.0;
    }

    static void validate(double now, double z) {
        if (!std::isfinite(now) || !std::isfinite(z))
            throw std::invalid_argument("altitude reference time and height must be finite");
    }

    void requireInitialized() const {
        if (!initialized_)
            throw std::logic_error("reset altitude reference from a valid pose before use");
    }

    void restart(double now) {
        if (current_.vz != 0.0) {
            // A moving reference cannot reverse velocity instantaneously.
            // Constant bounded deceleration preserves Z and VZ; its finite
            // stopping distance is |VZ|^2/(2*max_az), with no XY restriction.
            phase_ = Phase::BRAKE;
            phase_start_ = now;
            start_z_ = current_.z;
            start_vz_ = current_.vz;
            duration_ = std::fabs(start_vz_) / max_az_;
            brake_az_ = std::copysign(max_az_, -start_vz_);
            current_.az = brake_az_;
        } else {
            beginRamp(now);
        }
    }

    void beginRamp(double now) {
        phase_start_ = now;
        start_z_ = current_.z;
        current_.vz = current_.az = 0.0;
        const double distance = std::fabs(target_z_ - start_z_);
        if (distance == 0.0) {
            phase_ = Phase::HOLD;
            return;
        }
        // Quintic smoothstep has max |dz/du|=1.875 and
        // max |d2z/du2|=10/sqrt(3). Its endpoint P/V/A match steady hold.
        duration_ = std::max(0.25, std::max(1.875 * distance / max_vz_,
            std::sqrt(5.773503 * distance / max_az_)));
        phase_ = Phase::RAMP;
    }

    AltitudeHoldCommand current_;
    Phase phase_ = Phase::HOLD;
    bool initialized_ = false;
    double target_z_ = 0.0;
    double max_vz_ = 0.5, max_az_ = 1.0;
    double last_time_ = 0.0, phase_start_ = 0.0, duration_ = 0.25;
    double start_z_ = 0.0, start_vz_ = 0.0, brake_az_ = 0.0;
};

}  // namespace fly_px4
