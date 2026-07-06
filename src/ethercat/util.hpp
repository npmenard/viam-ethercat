#pragma once

// Small shared utilities (cold paths only -- error messages, last_error(); never
// the RT loop).
//
// libstdc++ 13 / the GCC-13 toolchain floor.

#include <algorithm>
#include <cmath>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <format>
#include <limits>
#include <string>

namespace ethercat {

template <std::unsigned_integral T>
std::string hex(T v) {
    return std::format("0x{:0{}X}", v, sizeof(T) * 2);
}

// --- unit conversions -------------
//
// Position: counts = revs * counts_per_rev * gear_ratio (the encoder counts the
// MOTOR shaft; gear_ratio is motor-revs per output-rev). Velocity: the drive's
// velocity unit (0x60FF/0x606C) is counts/s. Results are rounded to the nearest
// count and saturated to the int32 range the CiA402 objects use.

inline std::int32_t clamp_to_i32(double v) noexcept {
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

inline std::int32_t revs_to_counts(double revs, double counts_per_rev, double gear_ratio) noexcept {
    return clamp_to_i32(revs * counts_per_rev * gear_ratio);
}

inline double counts_to_revs(std::int32_t counts, double counts_per_rev, double gear_ratio) noexcept {
    const double denom = counts_per_rev * gear_ratio;
    return denom != 0.0 ? static_cast<double>(counts) / denom : 0.0;
}

inline std::int32_t rpm_to_device_velocity(double rpm, double counts_per_rev, double gear_ratio) noexcept {
    // rpm -> rev/s (/60) -> counts/s (* counts_per_rev * gear_ratio).
    return clamp_to_i32(rpm / 60.0 * counts_per_rev * gear_ratio);
}

inline double device_velocity_to_rpm(std::int32_t dev, double counts_per_rev, double gear_ratio) noexcept {
    const double denom = counts_per_rev * gear_ratio;
    return denom != 0.0 ? static_cast<double>(dev) / denom * 60.0 : 0.0;
}

}  // namespace ethercat
