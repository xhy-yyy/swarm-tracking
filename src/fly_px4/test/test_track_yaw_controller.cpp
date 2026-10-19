#include <fly_px4/track_yaw_controller.hpp>

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <string>

namespace {
const double kPi = 3.14159265358979323846;
int failures = 0;

double wrap(double angle) { return std::remainder(angle, 2.0 * kPi); }

// Keep the original precision regressions for the supported zero-deadband mode.
fly_px4::TrackYawConfig continuousConfig() {
    fly_px4::TrackYawConfig config;
    config.deadband = 0.0;
    return config;
}

void check(bool condition, const std::string& name) {
    if (!condition) {
        ++failures;
        std::cerr << "FAIL: " << name << '\n';
    }
}

void bounded(const fly_px4::TrackYawCommand& before,
             const fly_px4::TrackYawCommand& after, double dt,
             const fly_px4::TrackYawConfig& config, const std::string& name) {
    check(std::isfinite(after.yaw) && std::isfinite(after.rate), name + " finite");
    check(std::fabs(after.rate) <= config.rate_max + 1e-10, name + " rate limit");
    check(std::fabs(after.rate - before.rate) <= config.acc_max * dt + 1e-10,
          name + " acceleration limit");
    check(std::fabs(wrap(after.yaw - before.yaw) / dt - after.rate) < 1e-9,
          name + " yaw/rate consistency");
}

void stationaryNoise() {
    auto config = continuousConfig();
    fly_px4::TrackYawController controller;
    controller.configure(config);
    controller.reset(0.0, 0.0);
    fly_px4::TrackYawCommand previous;
    double square = 0.0, max_rate = 0.0;
    int count = 0;
    for (int i = 1; i <= 1000; ++i) {
        const double t = i * 0.02;
        const double noise = 0.005 * std::sin(i * 2.399963229728653);
        controller.observe(noise, t);
        const auto command = controller.step(t, 0.0);
        bounded(previous, command, 0.02, config, "stationary");
        if (i > 100) {
            square += command.yaw * command.yaw;
            max_rate = std::max(max_rate, std::fabs(command.rate));
            ++count;
        }
        previous = command;
    }
    check(std::sqrt(square / count) < 0.001, "stationary jitter attenuated");
    check(max_rate < 0.01, "stationary velocity feedforward stays quiet");

    // One bad box must not create a velocity prediction lasting until timeout.
    controller.observe(0.2, 20.02);
    auto command = controller.step(20.02, 0.0);
    bounded(previous, command, 0.02, config, "single outlier");
    double peak = std::fabs(command.yaw);
    for (int i = 2; i <= 30; ++i) {
        const double t = 20.0 + i * 0.02;
        controller.observe(0.0, t);
        previous = command;
        command = controller.step(t, 0.0);
        bounded(previous, command, 0.02, config, "outlier recovery");
        peak = std::max(peak, std::fabs(command.yaw));
    }
    check(peak < 0.018, "single outlier does not enable fast motion tracking");
    check(std::fabs(command.yaw) < 0.006, "single outlier recovers");
    std::cout << "stationary reference: jitter RMS " << std::sqrt(square / count) * 180.0 / kPi
              << " deg, peak rate " << max_rate * 180.0 / kPi
              << " deg/s, single-box outlier peak " << peak * 180.0 / kPi << " deg\n";
}

void movingAndReversing() {
    auto config = continuousConfig();
    fly_px4::TrackYawController controller;
    controller.configure(config);
    controller.reset(0.0, 0.0);
    fly_px4::TrackYawCommand previous;
    double peak_error = 0.0, reversal_time = -1.0;
    const double turn_at = 2.0;
    const double velocity = 0.4;  // Approximately 2 m/s at 5 m.
    for (int i = 1; i <= 250; ++i) {
        const double t = i * 0.02;
        const double bearing = velocity * (t <= turn_at ? t : 2.0 * turn_at - t);
        // 25 Hz images with 60 ms pipeline latency; controller runs at 50 Hz.
        if (i % 2 == 0 && t >= 0.08) {
            const double image_time = t - 0.06;
            const double image_bearing = velocity * (image_time <= turn_at ? image_time : 2.0 * turn_at - image_time);
            controller.observe(wrap(image_bearing), image_time);
        }
        const auto command = controller.step(t, bearing);
        bounded(previous, command, 0.02, config, "moving/reversing");
        if (t > 1.0 && t < 1.9)
            check(std::fabs(wrap(command.yaw - bearing)) < 0.025, "moving no fixed lead/lag");
        if (t > turn_at && t < turn_at + 0.6) {
            peak_error = std::max(peak_error, std::fabs(wrap(command.yaw - bearing)));
            if (reversal_time < 0.0 && command.rate < 0.0) reversal_time = t - turn_at;
        }
        if (t > turn_at + 0.8)
            check(std::fabs(wrap(command.yaw - bearing)) < 0.025, "reverse converges to current bearing");
        previous = command;
    }
    check(reversal_time > 0.0 && reversal_time < 0.32, "reverse withdraws old motion promptly");
    check(peak_error < 0.15, "reverse error bounded under finite acceleration");
}

void wrapBoundaryAndVariableDt() {
    auto config = continuousConfig();
    fly_px4::TrackYawController controller;
    controller.configure(config);
    const double initial = 179.0 * kPi / 180.0;
    controller.reset(initial, 10.0);
    fly_px4::TrackYawCommand previous;
    previous.yaw = initial;
    double t = 10.0;
    double total_turn = 0.0;
    const double intervals[] = {0.007, 0.031, 0.013, 0.049, 0.02};
    for (int i = 0; i < 160; ++i) {
        const double dt = intervals[i % 5];
        t += dt;
        controller.observe(-179.0 * kPi / 180.0, t);
        const auto command = controller.step(t, -179.0 * kPi / 180.0);
        bounded(previous, command, dt, config, "wrap/variable dt");
        total_turn += wrap(command.yaw - previous.yaw);
        previous = command;
    }
    check(std::fabs(total_turn - 2.0 * kPi / 180.0) < 0.001, "pi boundary takes short path");
    check(std::fabs(previous.rate) < 0.001, "static wrap target settles");

    const auto duplicate = controller.step(t, 1.0);
    check(duplicate.yaw == previous.yaw && duplicate.rate == previous.rate,
          "duplicate control timestamp holds command");
    const auto rewind = controller.step(1.0, 1.0);
    check(rewind.yaw == previous.yaw && rewind.rate == 0.0 && !rewind.visual_active,
          "clock rewind clears observation and velocity");
    const auto after_rewind = controller.step(1.02, 1.0);
    bounded(rewind, after_rewind, 0.02, config, "after clock rewind");
}

void delayedTenHzCamera() {
    const double delays[] = {0.1, 0.14};
    for (double delay : delays) {
        auto config = continuousConfig();
        config.bearing_timeout = 0.35;
        fly_px4::TrackYawController controller;
        controller.configure(config);
        controller.reset(0.0, 0.0);
        fly_px4::TrackYawCommand previous;
        const double speed = 0.4;
        const double turn_at = 2.0;
        const double fallback_bias = 10.0 * kPi / 180.0;
        double peak_error = 0.0, reversal_time = -1.0;
        double steady_error = 0.0, settled_error = 0.0;
        for (int i = 1; i <= 350; ++i) {
            const double t = i * 0.02;
            const double bearing = speed * (t <= turn_at ? t : 2.0 * turn_at - t);
            if (i % 5 == 0 && t >= delay) {
                const double image_time = t - delay;
                const double image_bearing = speed * (image_time <= turn_at ? image_time : 2.0 * turn_at - image_time);
                controller.observe(wrap(image_bearing), image_time);
            }
            // A ten-degree error deliberately prevents fallback from masking
            // a dead observer or a premature fade caused by pipeline latency.
            const auto command = controller.step(t, wrap(bearing + fallback_bias));
            bounded(previous, command, 0.02, config, "delayed 10 Hz camera");
            if (t > 1.0 && t < 1.9) {
                steady_error = std::max(steady_error, std::fabs(wrap(command.yaw - bearing)));
                check(command.visual_active, "normal delayed 10 Hz images stay active");
                check(std::fabs(wrap(command.yaw - bearing)) < 0.035,
                      "delayed 10 Hz visual direction dominates biased fallback");
            }
            if (t > turn_at && t < turn_at + 0.8) {
                peak_error = std::max(peak_error, std::fabs(wrap(command.yaw - bearing)));
                if (reversal_time < 0.0 && command.rate < 0.0) reversal_time = t - turn_at;
            }
            if (t > turn_at + 1.0) {
                settled_error = std::max(settled_error, std::fabs(wrap(command.yaw - bearing)));
                check(std::fabs(wrap(command.yaw - bearing)) < 0.035,
                      "delayed 10 Hz reversal follows camera instead of biased fusion");
            }
            previous = command;
        }
        check(reversal_time > 0.0 && reversal_time < 0.55,
              "delayed 10 Hz reverse has bounded response time");
        check(peak_error < 0.25, "delayed 10 Hz reverse respects physical lag");
        std::cout << "10 Hz camera, delay " << delay << " s: reference steady error "
                  << steady_error * 180.0 / kPi << " deg, reverse response "
                  << reversal_time << " s, reverse peak error "
                  << peak_error * 180.0 / kPi << " deg, settled error "
                  << settled_error * 180.0 / kPi << " deg\n";
    }
}

void staleAndLateImages() {
    auto config = continuousConfig();
    config.bearing_timeout = 0.35;
    fly_px4::TrackYawController controller;
    controller.configure(config);
    controller.reset(0.0, 0.0);
    fly_px4::TrackYawCommand previous;
    for (int i = 1; i <= 75; ++i) {
        const double t = i * 0.02;
        controller.observe(0.4 * t, t);
        const auto command = controller.step(t, 0.4 * t);
        bounded(previous, command, 0.02, config, "before stale");
        previous = command;
    }
    const double last_bearing = 0.6;
    for (int i = 1; i <= 100; ++i) {
        const double t = 1.5 + i * 0.02;
        // Old packets cannot keep visual tracking alive or corrupt its rate.
        if (i < 12) controller.observe(-1.0, 1.4);
        const auto command = controller.step(t, last_bearing);
        bounded(previous, command, 0.02, config, "stale transition");
        if (t > 1.86) check(!command.visual_active, "stale visual source expires");
        previous = command;
    }
    check(std::fabs(wrap(previous.yaw - last_bearing)) < 0.001,
          "stale source does not keep extrapolating old velocity");
    check(std::fabs(previous.rate) < 0.002, "stale fallback settles smoothly");

    controller.observe(-2.0, 9.0);  // A future packet is not an active observation.
    const auto future = controller.step(3.52, last_bearing);
    bounded(previous, future, 0.02, config, "future image");
    check(!future.visual_active, "future visual timestamp rejected by step");
    controller.observe(last_bearing, 3.5);
    const auto recovered = controller.step(3.54, last_bearing);
    bounded(future, recovered, 0.02, config, "after future image");
    check(recovered.visual_active, "valid frame recovers after future timestamp");
}

void fallbackFeedforward() {
    auto config = continuousConfig();
    fly_px4::TrackYawController controller;
    controller.configure(config);
    controller.reset(0.0, 0.0);
    fly_px4::TrackYawCommand previous;
    for (int i = 1; i <= 150; ++i) {
        const double t = i * 0.02;
        const auto command = controller.step(t, 0.4 * t, 0.4, true);
        bounded(previous, command, 0.02, config, "fallback feedforward");
        previous = command;
    }
    check(std::fabs(previous.yaw - 1.2) < 0.015, "valid fallback rate removes steady lag");
    check(std::fabs(previous.rate - 0.4) < 0.003, "fallback rate tracks angular motion");
}

void attitudePlantFeedforwardComparison() {
    // Illustrative attitude/rate plant, not flight measurements.
    // Both cases use the same bounded yaw reference; one also uses yaw-rate feedforward.
    // Integrate at 1ms while holding the reference between controller ticks.
    const double attitude_kp = 2.8;
    const double rate_tau = 0.08;
    const double controller_dt = 0.02;
    const double integration_dt = 0.001;
    fly_px4::TrackYawController controller;
    controller.configure(continuousConfig());
    controller.reset(0.0, 0.0);
    double previous_reference = 0.0;
    double yaw_without_ff = 0.0, rate_without_ff = 0.0;
    double yaw_with_ff = 0.0, rate_with_ff = 0.0;
    double error_without_ff = 0.0, error_with_ff = 0.0;
    int count = 0;
    for (int i = 1; i <= 300; ++i) {
        const double t = i * controller_dt;
        const auto command = controller.step(t, 0.4 * t, 0.4, true);
        for (int j = 0; j < 20; ++j) {
            const double reference = previous_reference + command.rate * (j * integration_dt);
            const double goal_without_ff = attitude_kp * wrap(reference - yaw_without_ff);
            const double goal_with_ff = attitude_kp * wrap(reference - yaw_with_ff) + command.rate;
            rate_without_ff += (goal_without_ff - rate_without_ff) * integration_dt / rate_tau;
            rate_with_ff += (goal_with_ff - rate_with_ff) * integration_dt / rate_tau;
            yaw_without_ff += rate_without_ff * integration_dt;
            yaw_with_ff += rate_with_ff * integration_dt;
        }
        if (t > 3.0) {
            error_without_ff += std::fabs(wrap(command.yaw - yaw_without_ff));
            error_with_ff += std::fabs(wrap(command.yaw - yaw_with_ff));
            ++count;
        }
        previous_reference = command.yaw;
    }
    error_without_ff /= count;
    error_with_ff /= count;
    check(error_without_ff > 0.12 && error_without_ff < 0.17,
          "attitude-only model has expected moving reference lag");
    check(error_with_ff < 0.006 && error_with_ff < 0.1 * error_without_ff,
          "consistent yaw-rate feedforward reduces model heading lag");
    std::cout << "yaw plant model (K=2.8, rate tau=0.08 s): mean reference error, yaw only "
              << error_without_ff * 180.0 / kPi << " deg, yaw+rate "
              << error_with_ff * 180.0 / kPi << " deg\n";
}
void fiveDegreeDeadband() {
    fly_px4::TrackYawConfig config;
    const double deg = kPi / 180.0;
    check(std::fabs(config.deadband - 5.0 * deg) < 1e-12, "default deadband is five degrees");
    for (double sign : {-1.0, 1.0}) {
        fly_px4::TrackYawController controller;
        controller.configure(config);
        controller.reset(0.0, 0.0);
        auto command = controller.step(0.02, sign * 4.999 * deg, sign * 0.4, true);
        check(command.yaw == 0.0 && command.rate == 0.0,
              "coordinate bearing inside deadband suppresses feedforward");
        controller.observe(sign * 4.999 * deg, 0.04);
        command = controller.step(0.04, sign * 30.0 * deg);
        check(command.visual_active && command.yaw == 0.0 && command.rate == 0.0,
              "visual bearing inside deadband overrides an outside coordinate bearing");

        controller.reset(0.0, 0.0);
        fly_px4::TrackYawCommand previous;
        bool turned = false;
        for (int i = 1; i <= 150; ++i) {
            const double t = i * 0.02;
            controller.observe(sign * 5.001 * deg, t);
            command = controller.step(t, sign * 5.001 * deg);
            bounded(previous, command, 0.02, config, "deadband entry");
            turned = turned || sign * command.rate > 0.0;
            previous = command;
        }
        check(turned && sign * command.yaw > 0.0,
              "visual error just outside five degrees starts correcting in either direction");
        check(std::fabs(wrap(sign * 5.001 * deg - command.yaw)) <= config.deadband
              && command.rate == 0.0, "static target settles and holds inside deadband");

        controller.reset(0.0, 0.0);
        previous = fly_px4::TrackYawCommand();
        for (int i = 1; i <= 25; ++i) {
            command = controller.step(i * 0.02, sign * 90.0 * deg);
            bounded(previous, command, 0.02, config, "before deadband braking");
            previous = command;
        }
        const double initial_rate = std::fabs(command.rate);
        for (int i = 26; i <= 55; ++i) {
            command = controller.step(i * 0.02, wrap(previous.yaw + sign * 4.0 * deg),
                                      sign * 0.4, true);
            bounded(previous, command, 0.02, config, "deadband braking");
            check(std::fabs(command.rate) <= std::fabs(previous.rate) + 1e-10,
                  "entering deadband decelerates even with nonzero feedforward");
            previous = command;
        }
        check(initial_rate > 0.5 && command.rate == 0.0, "deadband braking reaches a hold");
    }

    fly_px4::TrackYawController controller;
    controller.configure(config);
    controller.reset(179.0 * deg, 0.0);
    controller.observe(-179.0 * deg, 0.02);
    const auto wrapped = controller.step(0.02, -179.0 * deg);
    check(std::fabs(wrap(wrapped.yaw - 179.0 * deg)) < 1e-12 && wrapped.rate == 0.0,
          "deadband uses shortest distance across the pi boundary");

    config.deadband = std::numeric_limits<double>::quiet_NaN();
    controller.configure(config);
    controller.reset(0.0, 0.0);
    const auto invalid = controller.step(0.02, 4.0 * deg);
    check(invalid.yaw == 0.0 && invalid.rate == 0.0, "invalid deadband uses five degree default");
    config.deadband = 0.0;
    controller.configure(config);
    const auto disabled = controller.step(0.04, 1.0 * deg);
    check(disabled.yaw > 0.0 && disabled.rate > 0.0, "zero deadband enables small corrections");
}
}  // namespace

int main() {
    stationaryNoise();
    movingAndReversing();
    delayedTenHzCamera();
    wrapBoundaryAndVariableDt();
    staleAndLateImages();
    fallbackFeedforward();
    attitudePlantFeedforwardComparison();
    fiveDegreeDeadband();
    if (failures) {
        std::cerr << failures << " yaw controller checks failed\n";
        return EXIT_FAILURE;
    }
    std::cout << "track_yaw_controller: all behavior checks passed\n";
    return EXIT_SUCCESS;
}
