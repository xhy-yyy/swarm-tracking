#include <fly_px4/altitude_hold.hpp>

#include <cmath>
#include <iostream>
#include <limits>
#include <string>

namespace {
using Hold = fly_px4::AltitudeHold;
using Command = fly_px4::AltitudeHoldCommand;
int checks = 0;
int failures = 0;

void check(bool condition, const std::string& name) {
    ++checks;
    if (!condition) {
        ++failures;
        std::cerr << "FAIL: " << name << '\n';
    }
}

bool near(double a, double b, double tolerance = 1e-9) {
    return std::fabs(a - b) <= tolerance;
}

double duration(double distance, double max_vz, double max_az) {
    return std::max(0.25, std::max(1.875 * std::fabs(distance) / max_vz,
        std::sqrt(5.773503 * std::fabs(distance) / max_az)));
}

void steadyHeight() {
    Hold hold;
    hold.reset(10.0, 4.5);
    check(hold.initialized() && !hold.transitioning(), "reset initializes steady hold");
    bool constant = true;
    for (int i = 0; i <= 1000; ++i) {
        const double now = 10.0 + 0.02 * i;
        hold.setTarget(now, 4.5);
        const auto command = hold.step(now);
        constant = constant && command.z == 4.5 && command.vz == 0.0 && command.az == 0.0;
    }
    check(constant && !hold.transitioning(), "same state target remains exactly constant");
}

void boundedProfile(double from, double to, double max_vz, double max_az,
                    const std::string& name) {
    Hold hold;
    hold.configure(max_vz, max_az);
    hold.reset(10.0, from);
    hold.setTarget(10.0, to);
    auto previous = hold.step(10.0);
    check(near(previous.z, from) && previous.vz == 0.0 && previous.az == 0.0,
          name + " starts at current Z with zero derivatives");
    const double travel_time = duration(to - from, max_vz, max_az);
    const double dt = travel_time / 10000.0;
    const double direction = to > from ? 1.0 : -1.0;
    bool monotone = true, bounded = true, velocity_bounded = true;
    bool acceleration_bounded = true, pva_consistent = true;
    double peak_velocity = 0.0, peak_acceleration = 0.0;
    for (int i = 1; i <= 10000; ++i) {
        const double now = 10.0 + i * dt;
        // Normal state publication occurs at every control tick.
        hold.setTarget(now, to);
        const auto current = hold.step(now);
        monotone = monotone && direction * (current.z - previous.z) >= -1e-10;
        bounded = bounded && current.z >= std::min(from, to) - 1e-9
            && current.z <= std::max(from, to) + 1e-9;
        velocity_bounded = velocity_bounded && std::fabs(current.vz) <= max_vz + 1e-9;
        acceleration_bounded = acceleration_bounded && std::fabs(current.az) <= max_az + 1e-9;
        pva_consistent = pva_consistent
            && std::fabs((current.z - previous.z) / dt - 0.5 * (current.vz + previous.vz)) < 1e-6
            && std::fabs((current.vz - previous.vz) / dt - 0.5 * (current.az + previous.az)) < 1e-6;
        peak_velocity = std::max(peak_velocity, std::fabs(current.vz));
        peak_acceleration = std::max(peak_acceleration, std::fabs(current.az));
        previous = current;
    }
    check(monotone && bounded, name + " monotonically reaches target without reference overshoot");
    check(velocity_bounded && acceleration_bounded, name + " respects configured VZ/AZ limits");
    check(pva_consistent, name + " P/V/A describe the same differentiable motion");
    auto current = hold.step(10.0 + travel_time + 1e-8);
    check(current.z == to && current.vz == 0.0 && current.az == 0.0 && !hold.transitioning(),
          name + " terminates at exact height and zero derivatives");
    current = hold.step(100.0);
    check(current.z == to && current.vz == 0.0 && current.az == 0.0,
          name + " holds endpoint indefinitely");
    std::cout << name << ": peak VZ=" << peak_velocity << ", peak AZ=" << peak_acceleration << '\n';
}

void horizontalModeChanges() {
    Hold alternating_modes, uninterrupted;
    alternating_modes.reset(0.0, 4.5);
    uninterrupted.reset(0.0, 4.5);
    alternating_modes.setTarget(0.0, 2.7);
    uninterrupted.setTarget(0.0, 2.7);
    bool identical = true;
    for (int i = 0; i <= 1000; ++i) {
        const double now = 0.02 * i;
        // PID, EGO and hover consume this one reference; a horizontal mode
        // change supplies no new height/reset operation to the generator.
        if (i % 3 == 0) alternating_modes.setTarget(now, 2.7);
        const auto switched = alternating_modes.step(now);
        const auto continuous = uninterrupted.step(now);
        identical = identical && switched.z == continuous.z
            && switched.vz == continuous.vz && switched.az == continuous.az;
    }
    check(identical, "PID/EGO/hover switching preserves the entire vertical reference");
}

void reverseWhileMoving(double from, double to, const std::string& name) {
    Hold hold;
    hold.reset(0.0, from);
    hold.setTarget(0.0, to);
    const double reverse_time = 0.5 * duration(to - from, 0.5, 1.0);
    const auto before = hold.step(reverse_time);
    const double motion_direction = before.vz > 0.0 ? 1.0 : -1.0;
    check(std::fabs(before.vz) > 0.1, name + " reversal occurs with nonzero vertical speed");
    hold.setTarget(reverse_time, from);
    auto previous = hold.step(reverse_time);
    check(previous.z == before.z && previous.vz == before.vz && std::fabs(previous.az) <= 1.0,
          name + " reversal preserves Z/VZ and bounds AZ");
    const double brake_time = std::fabs(before.vz);
    const double stop_z = before.z + 0.5 * before.vz * brake_time;
    bool brake_consistent = true, brake_bounded = true;
    for (int i = 1; i <= 1000; ++i) {
        const double elapsed = brake_time * i / 1000.0;
        const auto current = hold.step(reverse_time + elapsed);
        const double expected_velocity = before.vz - motion_direction * elapsed;
        const double expected_z = before.z + before.vz * elapsed
            - 0.5 * motion_direction * elapsed * elapsed;
        brake_consistent = brake_consistent && near(current.z, expected_z)
            && near(current.vz, expected_velocity);
        brake_bounded = brake_bounded && motion_direction * (current.z - previous.z) >= -1e-9
            && std::fabs(current.vz) <= std::fabs(before.vz) + 1e-9 && std::fabs(current.az) <= 1.0;
        previous = current;
    }
    check(brake_consistent && brake_bounded, name + " brakes consistently without a velocity step");
    check(near(previous.z, stop_z) && near(previous.vz, 0.0) && near(previous.az, 0.0),
          name + " stops after finite VZ squared over twice AZ distance");
    const double start_time = reverse_time + brake_time;
    const double ramp_time = duration(from - stop_z, 0.5, 1.0);
    bool reversed_monotone = true, bounded = true;
    for (int i = 1; i <= 1000; ++i) {
        const auto current = hold.step(start_time + ramp_time * i / 1000.0);
        reversed_monotone = reversed_monotone && motion_direction * (current.z - previous.z) <= 1e-9;
        bounded = bounded && std::fabs(current.vz) <= 0.5 + 1e-9 && std::fabs(current.az) <= 1.0 + 1e-9;
        previous = current;
    }
    check(reversed_monotone && bounded, name + " climbs/descends smoothly after braking");
    const auto final = hold.step(start_time + ramp_time + 1e-8);
    check(final.z == from && final.vz == 0.0 && final.az == 0.0,
          name + " reaches reversed target at rest");
}

void retargetDuringBraking() {
    Hold hold;
    hold.reset(0.0, 4.5);
    hold.setTarget(0.0, 2.7);
    hold.step(3.0);
    hold.setTarget(3.0, 4.5);
    const auto before = hold.step(3.1);
    hold.setTarget(3.1, 2.7);
    const auto after = hold.step(3.1);
    check(before.z == after.z && before.vz == after.vz && before.az == after.az,
          "retargeting while braking preserves current P/V/A");
    const auto final = hold.step(100.0);
    check(final.z == 2.7 && final.vz == 0.0 && final.az == 0.0,
          "one late timer call correctly spans brake and ramp to latest target");
}

void clockRewind() {
    Hold hold;
    hold.reset(10.0, 4.5);
    hold.setTarget(10.0, 2.7);
    const auto before = hold.step(13.0);
    const auto rewound = hold.step(2.0);
    check(before.z == rewound.z && before.vz == rewound.vz && std::fabs(rewound.az) <= 1.0,
          "clock rewind anchors last emitted P/V and discards future schedule");
    const auto next = hold.step(2.02);
    check(next.z < rewound.z && std::fabs(next.vz) < std::fabs(rewound.vz),
          "clock rewind brakes on the new clock before replanning");
    const auto final = hold.step(100.0);
    check(final.z == 2.7 && final.vz == 0.0 && final.az == 0.0,
          "clock rewind eventually reaches existing target");
    const auto steady = hold.step(0.0);
    check(steady.z == 2.7 && steady.vz == 0.0 && steady.az == 0.0,
          "clock rewind during steady hold keeps constant height");
}

void invalidInputs() {
    Hold hold;
    bool uninitialized_rejected = false, limits_rejected = false, nonfinite_rejected = false;
    try { hold.step(0.0); } catch (const std::logic_error&) { uninitialized_rejected = true; }
    try { hold.configure(0.0, 1.0); } catch (const std::invalid_argument&) { limits_rejected = true; }
    try { hold.reset(0.0, std::numeric_limits<double>::quiet_NaN()); }
    catch (const std::invalid_argument&) { nonfinite_rejected = true; }
    check(uninitialized_rejected && limits_rejected && nonfinite_rejected,
          "invalid limits/time/pose cannot silently produce flight references");
}
}  // namespace

int main() {
    steadyHeight();
    boundedProfile(4.5, 2.7, 0.5, 1.0, "cruise to tracking");
    boundedProfile(2.7, 4.5, 0.5, 1.0, "tracking to cruise");
    boundedProfile(4.5, 4.48, 0.5, 0.2, "short acceleration limited descent");
    boundedProfile(4.5, 4.5001, 0.5, 1.0, "minimum duration climb");
    horizontalModeChanges();
    reverseWhileMoving(4.5, 2.7, "reverse descent");
    reverseWhileMoving(2.7, 4.5, "reverse climb");
    retargetDuringBraking();
    clockRewind();
    invalidInputs();
    std::cout << checks << " checks, " << failures << " failures\n";
    return failures == 0 ? 0 : 1;
}
