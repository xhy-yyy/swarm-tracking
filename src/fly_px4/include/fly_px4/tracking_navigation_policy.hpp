#pragma once

#include <array>
#include <algorithm>
#include <cmath>
#include <limits>

namespace fly_px4 {

enum class TrackingNavigation { STOP, FRESH, SAVED_REACQUIRE };

// The fixed road network includes x/y=-45 and the actor motion window extends
// to x=-50, y=+-60. Preserve the existing east limit inside EGO's +-180 m map.
struct TrackingMapBounds {
    double x_min = -50.0;
    double x_max = 160.0;
    double y_min = -60.0;
    double y_max = 60.0;
    bool contains(double x, double y) const {
        return std::isfinite(x) && std::isfinite(y) &&
            x >= x_min && x <= x_max && y >= y_min && y <= y_max;
    }
};

// Ages retain their source timestamps. A fresh communication lease does not
// make an old observation fresh; only an explicit REACQUIRE may use its anchor.
inline TrackingNavigation trackingNavigation(int mode, double lease_age,
        double observation_age, double state_age, double observation_timeout,
        double state_timeout, double reacquire_timeout) {
    if (mode < 1 || !std::isfinite(lease_age) || lease_age < 0.0 || lease_age > 1.0 ||
        !std::isfinite(observation_age) || observation_age < 0.0 ||
        !std::isfinite(state_age) || state_age < 0.0)
        return TrackingNavigation::STOP;
    if (observation_age <= observation_timeout && state_age <= state_timeout)
        return TrackingNavigation::FRESH;
    if (mode == 3 && observation_age <= reacquire_timeout && state_age <= reacquire_timeout)
        return TrackingNavigation::SAVED_REACQUIRE;
    return TrackingNavigation::STOP;
}

struct StandoffCandidate {
    double x = 0.0;
    double y = 0.0;
    double peer_clearance = std::numeric_limits<double>::infinity();
    bool inside_bounds = false;
};

// Keep a safe selected direction. Returning to the preferred direction needs
// extra clearance, while losing the hard clearance forces an immediate change.
template <std::size_t N>
int chooseStandoffCandidate(const std::array<StandoffCandidate, N>& candidates,
        int previous, double hard_clearance, double return_margin) {
    const auto safe = [&](int index, double margin) {
        return index >= 0 && index < static_cast<int>(N) &&
            candidates[index].inside_bounds &&
            std::isfinite(candidates[index].x) && std::isfinite(candidates[index].y) &&
            candidates[index].peer_clearance >= hard_clearance + margin;
    };
    if (safe(previous, 0.0)) {
        if (previous != 0 && safe(0, std::max(0.0, return_margin))) return 0;
        return previous;
    }
    for (std::size_t i = 0; i < N; ++i)
        if (safe(static_cast<int>(i), 0.0)) return static_cast<int>(i);
    return -1;
}

// 远距引导点使用规划范围内的固定距离，减少单目距离噪声造成的目标跳变。
inline bool farApproachGoal(double wx, double wy, double sx, double sy,
                            double goal_dist, double& gx, double& gy) {
    gx = sx;
    gy = sy;
    const double dx = sx - wx, dy = sy - wy;
    const double d = std::hypot(dx, dy);
    if (!(goal_dist > 0.0) || !std::isfinite(d) || d <= goal_dist) return false;
    gx = wx + dx * goal_dist / d;
    gy = wy + dy * goal_dist / d;
    return true;
}

// A task's first goal starts its planning wait. Repeated goals never move that
// origin, and old cached command timestamps cannot count as a new response.
class TrackingPlannerWait {
public:
    void reset() { started_ = false; first_goal_ = last_command_ = 0.0; }
    void goalSent(double now) {
        if (!started_ && std::isfinite(now)) {
            started_ = true;
            first_goal_ = last_command_ = now;
        }
    }
    void commandReceived(double stamp) {
        if (started_ && std::isfinite(stamp) && stamp >= first_goal_)
            last_command_ = std::max(last_command_, stamp);
    }
    bool shouldReject(double now, bool has_command, double grace) const {
        return started_ && !has_command && std::isfinite(now) &&
            now >= last_command_ && now - last_command_ >= grace;
    }
    bool waitingForResponse(double now, bool has_command, double grace) const {
        return started_ && !has_command && std::isfinite(now) &&
            now >= last_command_ && now - last_command_ < grace;
    }
private:
    bool started_ = false;
    double first_goal_ = 0.0;
    double last_command_ = 0.0;
};

}  // namespace fly_px4
