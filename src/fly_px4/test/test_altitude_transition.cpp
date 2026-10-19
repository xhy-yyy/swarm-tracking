#include <fly_px4/altitude_transition.hpp>

#include <cmath>
#include <iostream>
#include <limits>
#include <string>

namespace {
using Transition = fly_px4::AltitudeTransition;
using Phase = Transition::Phase;
using Reference = fly_px4::AltitudeReference;

int failures = 0;
int checks = 0;

void check(bool value, const std::string& name) {
    ++checks;
    if (!value) {
        ++failures;
        std::cerr << "FAIL: " << name << '\n';
    }
}

bool near(double a, double b, double tolerance = 1e-9) {
    return std::fabs(a - b) <= tolerance;
}

Reference stationary(Transition& transition, double time, double z,
                     double x = 20.0, double y = -3.0) {
    return transition.step(time, x, y, z, 0.0, 0.0, 0.0, true);
}

void brakeNeedsMeasuredSettling() {
    Transition transition;
    transition.begin(10.0, 20.0, -3.0, 4.5, 2.7);
    auto command = transition.step(10.0, 21.0, -3.0, 4.48, 5.0, 0.0, 0.0, true);
    check(transition.phase() == Phase::BRAKE && !command.complete,
          "cruise speed prevents descent");
    check(near(command.z, 4.5) && near(command.vz, 0.0) && near(command.az, 0.0),
          "braking keeps entry altitude with no vertical feedforward");
    transition.step(10.4, 22.0, -3.0, 4.47, 0.15, 0.15, 0.0, true);
    check(transition.phase() == Phase::BRAKE, "horizontal gate uses vector speed");
    transition.step(11.0, 22.0, -3.0, 4.47, 0.0, 0.0, -0.11, true);
    check(transition.phase() == Phase::BRAKE, "vertical motion also prevents ramp");
    stationary(transition, 11.2, 4.47, 22.0, -3.5);
    stationary(transition, 11.49, 4.47, 22.2, -3.6);
    check(transition.phase() == Phase::BRAKE, "less than 0.3 seconds still is insufficient");
    transition.step(11.495, 22.2, -3.6, 4.47, 0.0, 0.21, 0.0, true);
    stationary(transition, 11.5, 4.46, 22.3, -3.7);
    stationary(transition, 11.79, 4.46, 22.4, -3.8);
    check(transition.phase() == Phase::BRAKE, "movement clears prior braking evidence");
    command = stationary(transition, 11.801, 4.45, 22.5, -4.0);
    check(transition.phase() == Phase::RAMP, "continuous measured stillness starts ramp");
    check(near(command.x, 22.5) && near(command.y, -4.0) && near(command.z, 4.45),
          "ramp anchors after braking at actual position and altitude");
    check(near(command.vz, 0.0) && near(command.az, 0.0), "ramp starts with zero derivatives");
}

void boundedProfile(double start_z, double target_z, double max_vz,
                    double max_az, const std::string& name) {
    fly_px4::AltitudeTransitionConfig config;
    config.max_vz = max_vz;
    config.max_az = max_az;
    Transition transition;
    transition.configure(config);
    transition.begin(0.0, 20.0, -3.0, start_z, target_z);
    stationary(transition, 0.0, start_z);
    Reference previous = stationary(transition, 0.301, start_z);
    check(transition.phase() == Phase::RAMP, name + " begins after braking");
    bool finite = true, position_bounded = true, xy_locked = true;
    bool velocity_bounded = true, acceleration_bounded = true;
    bool position_velocity_consistent = true, velocity_acceleration_consistent = true;
    bool monotone = true, reached_settle = false;
    double peak_velocity = 0.0, peak_acceleration = 0.0;
    const double direction = target_z >= start_z ? 1.0 : -1.0;
    const double dt = 0.002;
    Reference current;
    for (int i = 1; i <= 15000; ++i) {
        // Actual horizontal drift must not make the ramp follow a moving hold point.
        current = stationary(transition, 0.301 + i * dt, start_z,
                             20.0 + 0.1 * std::sin(i), -3.0 + 0.1 * std::cos(i));
        finite = finite && std::isfinite(current.z) && std::isfinite(current.vz)
                         && std::isfinite(current.az);
        position_bounded = position_bounded &&
            current.z >= std::min(start_z, target_z) - 1e-9 &&
            current.z <= std::max(start_z, target_z) + 1e-9;
        xy_locked = xy_locked && near(current.x, 20.0) && near(current.y, -3.0);
        velocity_bounded = velocity_bounded && std::fabs(current.vz) <= max_vz + 1e-8;
        acceleration_bounded = acceleration_bounded && std::fabs(current.az) <= max_az + 1e-8;
        monotone = monotone && direction * (current.z - previous.z) >= -1e-9;
        position_velocity_consistent = position_velocity_consistent &&
            std::fabs((current.z - previous.z) / dt -
                      (current.vz + previous.vz) * 0.5) < 1e-4;
        velocity_acceleration_consistent = velocity_acceleration_consistent &&
            std::fabs((current.vz - previous.vz) / dt -
                      (current.az + previous.az) * 0.5) < 2e-4;
        peak_velocity = std::max(peak_velocity, std::fabs(current.vz));
        peak_acceleration = std::max(peak_acceleration, std::fabs(current.az));
        previous = current;
        if (transition.phase() == Phase::SETTLE) { reached_settle = true; break; }
    }
    check(finite, name + " finite reference");
    check(position_bounded && monotone, name + " no altitude overshoot or reversal");
    check(xy_locked, name + " fixed XY throughout ramp");
    check(velocity_bounded, name + " vertical velocity bounded");
    check(acceleration_bounded, name + " vertical acceleration bounded");
    check(position_velocity_consistent && velocity_acceleration_consistent,
          name + " position velocity acceleration describe the same motion");
    check(reached_settle && near(current.z, target_z) && near(current.vz, 0.0)
          && near(current.az, 0.0), name + " terminal reference reaches target at rest");
    if (std::fabs(start_z - target_z) > config.z_tolerance)
        check(!current.complete, name + " elapsed reference does not authorize actual handoff");
    std::cout << name << ": peak vz=" << peak_velocity
              << ", peak az=" << peak_acceleration << '\n';
}

void actualAltitudeAndVelocityGate() {
    Transition transition;
    transition.begin(0.0, 20.0, -3.0, 4.5, 2.7);
    stationary(transition, 0.0, 4.5);
    stationary(transition, 0.301, 4.5);
    auto command = stationary(transition, 20.0, 3.7);
    check(transition.phase() == Phase::SETTLE && !command.complete,
          "finished reference waits for measured altitude");
    command = stationary(transition, 20.5, 3.7);
    check(!command.complete, "long wait cannot substitute for altitude feedback");
    command = transition.step(21.0, 20.0, -3.0, 2.7, 0.0, 0.0, -0.11, true);
    check(!command.complete, "passing through target with vertical speed cannot hand off");
    stationary(transition, 22.0, 2.7);
    stationary(transition, 22.29, 2.7);
    command = transition.step(22.295, 20.0, -3.0, 2.7, 0.21, 0.0, 0.0, true);
    check(!command.complete, "horizontal motion resets terminal settling evidence");
    stationary(transition, 23.0, 2.7);
    command = stationary(transition, 23.301, 2.7);
    check(command.complete && transition.complete() && transition.phase() == Phase::DONE,
          "actual height and all speeds stable authorizes handoff");
    command = stationary(transition, 24.0, 2.7);
    check(command.complete && near(command.z, 2.7) && near(command.vz, 0.0)
          && near(command.az, 0.0), "DONE keeps terminal reference at rest");
    transition.suspend();
    check(!transition.complete(), "suspend immediately revokes DONE authorization");
    command = stationary(transition, 25.0, 3.1, 25.0, -5.0);
    check(transition.phase() == Phase::BRAKE && !command.complete && near(command.z, 3.1),
          "resume after DONE suspension restarts from actual altitude");
}

void interruptedFeedbackAndClock() {
    Transition transition;
    transition.begin(10.0, 20.0, -3.0, 4.5, 2.7);
    stationary(transition, 10.0, 4.5);
    stationary(transition, 10.301, 4.5);
    stationary(transition, 11.0, 4.4);
    auto command = transition.step(11.1, 20.5, -3.2, 4.3, 0.0, 0.0, 0.0, false);
    check(!command.complete && near(command.vz, 0.0), "feedback loss removes movement authorization");
    command = stationary(transition, 50.0, 4.0, 21.0, -4.0);
    check(transition.phase() == Phase::BRAKE && near(command.x, 21.0)
          && near(command.y, -4.0) && near(command.z, 4.0) && !command.complete,
          "feedback recovery discards elapsed ramp and restarts at actual position");
    stationary(transition, 50.29, 4.0, 21.0, -4.0);
    check(transition.phase() == Phase::BRAKE, "recovered feedback needs fresh braking evidence");
    stationary(transition, 50.301, 4.0, 21.0, -4.0);
    check(transition.phase() == Phase::RAMP, "recovered stillness restarts ramp");
    command = stationary(transition, 5.0, 3.9, 22.0, -5.0);
    check(transition.phase() == Phase::BRAKE && near(command.x, 22.0)
          && near(command.y, -5.0) && near(command.z, 3.9) && !command.complete,
          "clock rewind restarts at measured position");
    command = transition.step(5.1, 22.0, -5.0, 3.9,
                             std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0, true);
    check(!command.complete && near(command.vz, 0.0), "nonfinite speed cannot accumulate evidence");
    command = stationary(transition, 7.0, 3.8, 23.0, -6.0);
    check(transition.phase() == Phase::BRAKE && near(command.z, 3.8),
          "nonfinite speed recovery starts a new brake phase");
    transition.suspend();
    command = stationary(transition, 8.0, 3.6, 24.0, -7.0);
    check(transition.phase() == Phase::BRAKE && near(command.x, 24.0)
          && near(command.y, -7.0) && near(command.z, 3.6),
          "explicit mode interruption resumes at actual position");
}

void reverseMidTransition() {
    Transition transition;
    transition.begin(0.0, 20.0, -3.0, 4.5, 2.7);
    stationary(transition, 0.0, 4.5);
    stationary(transition, 0.301, 4.5);
    auto command = stationary(transition, 2.0, 4.0);
    check(command.vz < 0.0, "descent was in progress before task reversal");
    transition.begin(2.1, 21.0, -4.0, 3.9, 4.5);
    command = transition.step(2.1, 21.0, -4.0, 3.9, 0.0, 0.0, -0.3, true);
    check(transition.phase() == Phase::BRAKE && near(command.z, 3.9)
          && near(command.vz, 0.0) && near(transition.targetZ(), 4.5),
          "reverse request brakes at actual altitude before climbing");
    stationary(transition, 3.0, 3.85, 21.1, -4.1);
    command = stationary(transition, 3.301, 3.84, 21.2, -4.2);
    check(transition.phase() == Phase::RAMP && near(command.z, 3.84)
          && near(command.x, 21.2) && near(command.y, -4.2),
          "reverse ramp anchors to actual stopped position");
    command = stationary(transition, 4.0, 3.9);
    check(command.vz > 0.0 && command.z >= 3.84 && command.z <= 4.5,
          "reverse climb follows new target without old descent feedforward");
}
}  // namespace

int main() {
    brakeNeedsMeasuredSettling();
    boundedProfile(4.5, 2.7, 0.5, 1.0, "descent velocity limited");
    boundedProfile(2.7, 4.5, 0.5, 1.0, "ascent velocity limited");
    boundedProfile(2.7, 3.5, 3.0, 0.15, "ascent acceleration limited");
    boundedProfile(4.5, 3.7, 3.0, 0.15, "descent acceleration limited");
    boundedProfile(4.5, 4.499, 0.5, 1.0, "short descent");
    boundedProfile(2.7, 2.7, 0.5, 1.0, "same altitude");
    actualAltitudeAndVelocityGate();
    interruptedFeedbackAndClock();
    reverseMidTransition();
    if (failures) {
        std::cerr << failures << " of " << checks << " altitude transition checks failed\n";
        return 1;
    }
    std::cout << checks << " altitude transition checks passed\n";
    return 0;
}
