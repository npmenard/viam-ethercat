#pragma once

// Pure unit-conversion math for the servo driver: revs <-> device counts,
// rpm <-> device velocity, rpm clamping, and amps -> torque per-mille. No SDK,
// no hardware, no state -- all free functions, fully unit-testable.
//
// A6 note (kept as config data, never hardcoded here): the A6 expresses
// torque/current limits in per-mille of RATED torque (0..4000), so a current
// limit in amps is converted via the motor's rated current (a per-drive datum).

#include <cstdint>

namespace ethercat::servo {

// Position. counts = revs * counts_per_rev * gear_ratio (the encoder counts the
// MOTOR shaft; gear_ratio is motor-revs per output-rev). Rounded to the nearest
// count and saturated to the int32 range the drive's 0x607A uses.
std::int32_t revs_to_counts(double revs, double counts_per_rev, double gear_ratio) noexcept;
double counts_to_revs(std::int32_t counts, double counts_per_rev, double gear_ratio) noexcept;

// Velocity. The drive's velocity unit (0x60FF) is counts/s here:
//   dev = rpm/60 * counts_per_rev * gear_ratio.
// Rounded + int32-saturated.
std::int32_t rpm_to_device_velocity(double rpm, double counts_per_rev, double gear_ratio) noexcept;
double device_velocity_to_rpm(std::int32_t dev, double counts_per_rev, double gear_ratio) noexcept;

// Clamp an rpm to [-max_rpm, max_rpm] (sign-preserving). `max_rpm` must be >= 0;
// a negative bound is treated as 0 (fully clamped).
double clamp_rpm(double rpm, double max_rpm) noexcept;

}  // namespace ethercat::servo
