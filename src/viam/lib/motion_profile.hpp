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
// MOTOR shaft; gear_ratio is motor-revs per output-rev). Result is rounded to
// the nearest count and clamped to the int32 range the drive's 0x607A uses.
std::int32_t revs_to_counts(double revs, double counts_per_rev, double gear_ratio) noexcept;
double counts_to_revs(double counts, double counts_per_rev, double gear_ratio) noexcept;

// Velocity. device_units = rpm * velocity_scale, where velocity_scale is the
// drive's velocity unit (a per-drive config datum). Rounded + int32-clamped.
std::int32_t rpm_to_device_velocity(double rpm, double velocity_scale) noexcept;
double device_velocity_to_rpm(double device_velocity, double velocity_scale) noexcept;

// Clamp an rpm to [-max_rpm, max_rpm]. `max_rpm` must be >= 0; a negative bound
// is treated as 0 (fully clamped).
double clamp_rpm(double rpm, double max_rpm) noexcept;

// A6 torque/current limit: per-mille of rated torque (0..4000), derived from a
// current limit (amps) via the motor's rated current. permille =
// amps/rated_current_amps * 1000, clamped to [0, 4000]. Returns 0 if
// rated_current_amps <= 0 (no valid conversion).
std::uint16_t amps_to_torque_permille(double amps, double rated_current_amps) noexcept;

}  // namespace ethercat::servo
