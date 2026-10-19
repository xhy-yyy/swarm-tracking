#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace fly_px4 {

struct ReturnScan {
    double angle_min = 0.0;
    double angle_increment = 0.0;
    double range_min = 0.0;
    double range_max = 0.0;
    std::vector<double> ranges;
};

namespace return_scan_detail {

inline double pi() { return 3.14159265358979323846; }
inline double twoPi() { return 2.0 * pi(); }

inline double wrap(double angle) {
    return std::remainder(angle, twoPi());
}

// Ray cells extend half an increment to either side. A near-full scan can
// still contain a small gap; the requested cone must not include that gap.
inline bool coneCovered(const ReturnScan& scan, double bearing,
                        double half_angle) {
    const double step = std::fabs(scan.angle_increment);
    const double start = wrap(scan.angle_min);
    const double end = start +
        static_cast<double>(scan.ranges.size() - 1) * scan.angle_increment;
    const double low = std::min(start, end) - 0.5 * step;
    const double high = std::max(start, end) + 0.5 * step;
    if (high - low >= twoPi()) return true;
    const double center = 0.5 * (low + high);
    const double lifted_bearing = center + wrap(bearing - center);
    return lifted_bearing - half_angle >= low &&
           lifted_bearing + half_angle <= high;
}

}  // namespace return_scan_detail

// Validation concerns scan geometry and declared limits. Individual invalid
// returns only reject candidates whose observation cone contains those rays.
inline bool validReturnScan(const ReturnScan& scan) {
    if (!std::isfinite(scan.angle_min) ||
        !std::isfinite(scan.angle_increment) ||
        !std::isfinite(scan.range_min) || !std::isfinite(scan.range_max) ||
        scan.range_min < 0.0 || scan.range_max <= scan.range_min ||
        scan.ranges.size() < 32) return false;
    const double step = std::fabs(scan.angle_increment);
    if (step <= 1e-12 || step > 0.05) return false;
    const double span = static_cast<double>(scan.ranges.size() - 1) * step;
    // Accept the common layouts with or without a repeated endpoint, and
    // at most one additional missing sample. Cone coverage is checked below.
    return std::isfinite(span) &&
           span >= return_scan_detail::twoPi() - 2.0 * step - 1e-6 &&
           span <= return_scan_detail::twoPi() + step + 1e-6;
}

// This checks a disc in the laser's horizontal plane. It does not observe
// overhead obstacles and cannot certify an upward flight corridor.
inline bool returnScanDiscClear(const ReturnScan& scan, double bearing_rad,
                                double distance, double radius,
                                double extra_clearance = 0.2) {
    if (!validReturnScan(scan) || !std::isfinite(bearing_rad) ||
        !std::isfinite(distance) || !std::isfinite(radius) ||
        !std::isfinite(extra_clearance) || radius <= 0.0 ||
        distance <= radius || extra_clearance < 0.0 ||
        distance - radius < scan.range_min) return false;
    const double bearing = return_scan_detail::wrap(bearing_rad);
    const double half_angle = std::asin(radius / distance);
    if (!return_scan_detail::coneCovered(scan, bearing, half_angle)) return false;

    const double start = return_scan_detail::wrap(scan.angle_min);
    std::size_t checked_rays = 0;
    for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
        const double angle = start + static_cast<double>(i) * scan.angle_increment;
        const double delta = return_scan_detail::wrap(angle - bearing);
        if (std::fabs(delta) > half_angle) continue;

        const double perpendicular = distance * std::sin(delta);
        const double discriminant = radius * radius - perpendicular * perpendicular;
        const double far_intersection = distance * std::cos(delta) +
            std::sqrt(std::max(0.0, discriminant));
        const double required_range = far_intersection + extra_clearance;
        if (!std::isfinite(required_range) || required_range > scan.range_max)
            return false;

        const double range = scan.ranges[i];
        double observed_free_range = 0.0;
        if (std::isinf(range) && range > 0.0) {
            // No return only proves the declared finite sensor range.
            observed_free_range = scan.range_max;
        } else {
            if (!std::isfinite(range) || range < scan.range_min) return false;
            observed_free_range = std::min(range, scan.range_max);
        }
        if (observed_free_range < required_range) return false;
        ++checked_rays;
    }
    return checked_rays >= 3;
}

}  // namespace fly_px4
