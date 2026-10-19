#include <fly_px4/tracking_navigation_policy.hpp>

#include <cmath>
#include <iostream>
#include <limits>
#include <string>

namespace {
int failures = 0;
void check(bool value, const std::string& name) {
    if (!value) { ++failures; std::cerr << "FAIL: " << name << '\n'; }
}

std::array<fly_px4::StandoffCandidate, 12> circle(double peer_x) {
    const double offsets[] = {0, 30, -30, 60, -60, 90, -90, 120, -120, 150, -150, 180};
    std::array<fly_px4::StandoffCandidate, 12> candidates;
    for (std::size_t i = 0; i < candidates.size(); ++i) {
        auto& c = candidates[i];
        const double angle = offsets[i] * std::acos(-1.0) / 180.0;
        c.x = 5.5 * std::cos(angle);
        c.y = 5.5 * std::sin(angle);
        c.inside_bounds = true;
        c.peer_clearance = std::hypot(c.x - peer_x, c.y);
    }
    return candidates;
}

void peerCrossing() {
    auto candidates = circle(8.99);
    int selected = fly_px4::chooseStandoffCandidate(candidates, -1, 3.5, 0.5);
    check(selected == 1, "blocked preferred direction selects safe alternate");
    const double old_x = candidates[selected].x, old_y = candidates[selected].y;
    for (int i = 0; i < 100; ++i) {
        candidates = circle(i % 2 ? 8.99 : 9.01);
        selected = fly_px4::chooseStandoffCandidate(candidates, selected, 3.5, 0.5);
        check(selected == 1, "2 cm peer jitter does not alternate standing point");
        check(std::hypot(candidates[selected].x - old_x, candidates[selected].y - old_y) < 1e-10,
              "safe standing point has no 2.847 m jump");
        check(candidates[selected].peer_clearance >= 3.5, "selected point keeps hard peer clearance");
    }
    candidates = circle(9.51);
    selected = fly_px4::chooseStandoffCandidate(candidates, selected, 3.5, 0.5);
    check(selected == 0, "preferred direction returns after extra clearance");
    candidates = circle(8.99);
    selected = fly_px4::chooseStandoffCandidate(candidates, selected, 3.5, 0.5);
    check(selected == 1, "unsafe selected direction changes immediately");
    candidates[1].inside_bounds = false;
    selected = fly_px4::chooseStandoffCandidate(candidates, selected, 3.5, 0.5);
    check(selected == 2, "map boundary rejects previously selected direction");
    candidates[2].peer_clearance = 3.49;
    selected = fly_px4::chooseStandoffCandidate(candidates, selected, 3.5, 0.5);
    check(selected > 2, "hard clearance takes priority over holding candidate");
    for (auto& c : candidates) c.peer_clearance = 0.0;
    check(fly_px4::chooseStandoffCandidate(candidates, selected, 3.5, 0.5) == -1,
          "all blocked candidates yield failure");
}

void navigationAges() {
    using Navigation = fly_px4::TrackingNavigation;
    const auto decision = [](int mode, double lease, double observation, double state) {
        return fly_px4::trackingNavigation(mode, lease, observation, state, 1.0, 1.0, 25.0);
    };
    check(decision(1, 0.1, 0.5, 0.5) == Navigation::FRESH, "fresh approach unchanged");
    check(decision(2, 0.1, 0.5, 0.5) == Navigation::FRESH, "fresh locked tracking unchanged");
    check(decision(3, 0.1, 0.5, 0.5) == Navigation::FRESH, "new observations end saved-only navigation");
    check(decision(3, 0.1, 16.7, 15.8) == Navigation::SAVED_REACQUIRE,
          "explicit queued REACQUIRE may navigate to old saved anchor");
    check(decision(3, 0.1, 25.0, 24.1) == Navigation::SAVED_REACQUIRE,
          "saved anchor allowed at configured age boundary");
    check(decision(3, 0.1, 25.01, 24.11) == Navigation::STOP,
          "continued fresh leases cannot extend the observation deadline");
    check(decision(3, 0.1, 24.0, 25.01) == Navigation::STOP, "old state independently expires");
    check(decision(1, 0.1, 16.7, 15.8) == Navigation::STOP, "APPROACH cannot use stale observation");
    check(decision(2, 0.1, 16.7, 15.8) == Navigation::STOP, "LOCKED cannot use stale observation");
    check(decision(3, 1.01, 16.7, 15.8) == Navigation::STOP, "expired command lease stops REACQUIRE");
    check(decision(0, 0.1, 0.5, 0.5) == Navigation::STOP, "OFF always stops navigation");
    check(decision(3, 0.1, std::numeric_limits<double>::quiet_NaN(), 1.0) == Navigation::STOP,
          "invalid observation age stops navigation");
    check(decision(3, -0.1, 16.7, 15.8) == Navigation::STOP, "negative lease rejected");
    check(decision(3, 0.1, -1.0, 1.0) == Navigation::STOP, "negative observation age rejected");
}

void realSearchBounds() {
    fly_px4::TrackingMapBounds bounds;
    check(bounds.contains(-43.5, -43.5), "existing southwest patrol waypoint stays inside tracking area");
    check(bounds.contains(-50.0, -60.0), "official western/southern actor boundary included");
    check(!bounds.contains(-50.01, 0.0), "true western outside point rejected");
    check(!bounds.contains(0.0, -60.01), "true southern outside point rejected");
    check(!bounds.contains(160.01, 0.0), "existing eastern hard boundary retained");
    check(!bounds.contains(0.0, 60.01), "northern hard boundary retained");
    auto candidates = circle(1000.0);
    for (auto& c : candidates) {
        c.x -= 33.0;
        c.y -= 46.7;
        c.inside_bounds = bounds.contains(c.x, c.y);
    }
    const int selected = fly_px4::chooseStandoffCandidate(candidates, -1, 3.5, 0.5);
    check(selected >= 0, "logged blue (-33,-46.7) has a legal standing point");
    if (selected >= 0)
        check(bounds.contains(candidates[selected].x, candidates[selected].y), "blue selected point passes real bounds");
    for (auto& c : candidates) {
        c.y -= 30.0;
        c.inside_bounds = bounds.contains(c.x, c.y);
    }
    check(fly_px4::chooseStandoffCandidate(candidates, selected, 3.5, 0.5) == -1,
          "all standing points truly outside still reject");
}

void plannerStartup() {
    fly_px4::TrackingPlannerWait wait;
    check(!wait.shouldReject(1000.0, false, 3.0), "no goal does not create planning failure");
    wait.goalSent(100.0);
    wait.commandReceived(10.0);  // Ancient patrol command cache.
    check(!wait.shouldReject(100.02, false, 3.0), "first tracking goal tolerates ancient/no command cache");
    check(wait.waitingForResponse(100.02, false, 3.0), "startup grace suppresses inherited fallback");
    for (int i = 1; i <= 10; ++i) wait.goalSent(100.0 + i * 0.3);
    check(!wait.shouldReject(102.99, false, 3.0), "first planner response gets full grace");
    check(wait.shouldReject(103.0, false, 3.0), "repeated goals cannot extend startup grace indefinitely");
    check(!wait.waitingForResponse(103.0, false, 3.0), "startup fallback protection expires");
    wait.commandReceived(103.1);
    check(!wait.shouldReject(110.0, true, 3.0), "healthy command stream does not reject");
    check(!wait.shouldReject(106.09, false, 3.0), "new response moves loss deadline");
    check(wait.shouldReject(106.1, false, 3.0), "command loss eventually rejects");
    wait.reset();
    wait.goalSent(200.0);
    check(!wait.shouldReject(200.02, false, 3.0), "next assignment has independent planning grace");
    check(!wait.shouldReject(199.0, false, 3.0), "clock rewind cannot create immediate reject");
}

void farApproach() {
    double gx = 0.0, gy = 0.0;
    check(fly_px4::farApproachGoal(0.0, 0.0, 50.0, 0.0, 22.0, gx, gy) &&
          std::hypot(gx - 22.0, gy) < 1e-9, "far standing point becomes goal 22 m along line of sight");

    // base3 drone 5 / red2, sim 1949.20-1949.40: monocular range flips the
    // standing point 15 m along the line of sight within 0.1 s.
    double ax, ay, bx, by;
    fly_px4::farApproachGoal(65.4, -3.4, 103.0, -11.8, 22.0, ax, ay);
    fly_px4::farApproachGoal(65.4, -3.4, 117.6, -14.9, 22.0, bx, by);
    check(std::hypot(ax - bx, ay - by) < 0.1,
          "logged 15 m range jump moves EGO goal < 0.1 m (no forced replan)");
    check(std::hypot(ax - bx, ay - by) < 2.5 && std::hypot(103.0 - 117.6, -11.8 + 14.9) > 2.5,
          "jump that forced a replan before stays under standoff_force_dist");

    // A real bearing change (standing point switches side) must still move it.
    const double side = 8.6 * std::acos(-1.0) / 180.0;
    fly_px4::farApproachGoal(0.0, 0.0, 50.0, 0.0, 22.0, ax, ay);
    fly_px4::farApproachGoal(0.0, 0.0, 50.0 * std::cos(side), 50.0 * std::sin(side), 22.0, bx, by);
    check(std::hypot(ax - bx, ay - by) > 2.5, "real side switch still triggers immediate republish");

    check(!fly_px4::farApproachGoal(0.0, 0.0, 15.0, 5.0, 22.0, gx, gy) && gx == 15.0 && gy == 5.0,
          "standing point within goal distance passes through unchanged");
    check(!fly_px4::farApproachGoal(0.0, 0.0, 22.0, 0.0, 22.0, gx, gy) && gx == 22.0,
          "boundary distance passes through");
    fly_px4::farApproachGoal(0.0, 0.0, 22.001, 0.0, 22.0, gx, gy);
    check(std::hypot(gx - 22.001, gy) < 2e-3, "goal is continuous across the boundary");
    check(!fly_px4::farApproachGoal(0.0, 0.0, 50.0, 0.0, 0.0, gx, gy) && gx == 50.0,
          "zero distance disables the far goal");
    check(!fly_px4::farApproachGoal(3.0, 4.0, 3.0, 4.0, 22.0, gx, gy) && gx == 3.0 && gy == 4.0,
          "coincident drone and standing point stays finite");
}
}

int main() {
    peerCrossing();
    navigationAges();
    realSearchBounds();
    plannerStartup();
    farApproach();
    if (failures) return 1;
    std::cout << "tracking navigation policy: peer crossing, saved-anchor bounds, planner startup "
                 "and far approach goal passed\n";
    return 0;
}
