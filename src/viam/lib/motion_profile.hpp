#pragma once

// Motion-limit helpers for the servo driver. The pure unit conversions
// (revs <-> counts, rpm <-> device velocity) live in ethercat/util.hpp.
//
// CiA402 note (kept as config data, never hardcoded here): drives express
// torque/current limits in per-mille of RATED torque (0..4000), so a current
// limit in amps is converted via the motor's rated current (a per-drive datum).

#include "ethercat/util.hpp"  // conversions, re-exported into ethercat::servo by the using-declarations below

namespace ethercat::servo {

// The conversions are library utilities; driver code uses them unqualified.
using ethercat::counts_to_revs;
using ethercat::device_velocity_to_rpm;
using ethercat::revs_to_counts;
using ethercat::rpm_to_device_velocity;

// Clamp an rpm to [-max_rpm, max_rpm] (sign-preserving). `max_rpm` must be >= 0;
// a negative bound is treated as 0 (fully clamped).
double clamp_rpm(double rpm, double max_rpm) noexcept;

}  // namespace ethercat::servo
