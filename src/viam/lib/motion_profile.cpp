#include "viam/lib/motion_profile.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace ethercat::servo {

namespace {

std::int32_t clamp_to_i32(double v) noexcept {
    constexpr double kMax = static_cast<double>(std::numeric_limits<std::int32_t>::max());
    constexpr double kMin = static_cast<double>(std::numeric_limits<std::int32_t>::min());
    if (!(v > kMin)) {  // also catches NaN
        return std::numeric_limits<std::int32_t>::min();
    }
    if (v >= kMax) {
        return std::numeric_limits<std::int32_t>::max();
    }
    return static_cast<std::int32_t>(std::llround(v));
}

}  // namespace

std::int32_t revs_to_counts(double revs, double counts_per_rev, double gear_ratio) noexcept {
    return clamp_to_i32(revs * counts_per_rev * gear_ratio);
}

double counts_to_revs(std::int32_t counts, double counts_per_rev, double gear_ratio) noexcept {
    const double denom = counts_per_rev * gear_ratio;
    return denom != 0.0 ? static_cast<double>(counts) / denom : 0.0;
}

std::int32_t rpm_to_device_velocity(double rpm, double counts_per_rev, double gear_ratio) noexcept {
    // rpm -> rev/s (/60) -> counts/s (* counts_per_rev * gear_ratio).
    return clamp_to_i32(rpm / 60.0 * counts_per_rev * gear_ratio);
}

double device_velocity_to_rpm(std::int32_t dev, double counts_per_rev, double gear_ratio) noexcept {
    const double denom = counts_per_rev * gear_ratio;
    return denom != 0.0 ? static_cast<double>(dev) / denom * 60.0 : 0.0;
}

double clamp_rpm(double rpm, double max_rpm) noexcept {
    const double bound = max_rpm > 0.0 ? max_rpm : 0.0;
    return std::clamp(rpm, -bound, bound);
}

}  // namespace ethercat::servo
