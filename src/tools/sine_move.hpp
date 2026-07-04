#pragma once

// Soft-started, relative-to-start position sine for the a6_validate CSP bench move. Pure and
// header-only so the trajectory math is unit-testable offline (the safety-critical bits for first
// energized motion: CSP-safe init and soft-start).
//
//   target(t) = pos_enable + amplitude * ramp(t) * sin(2*pi * t / period)
//   ramp(t)   = min(1, t / period)   -- linear 0->1 over the first period, then 1
//
// Two properties that make this safe to stream into CSP (Cyclic Synchronous Position,
// where the master commands an absolute target every cycle and the drive expects no
// step on enable):
//   * CSP-SAFE INIT: at t=0, sin(0)=0 -> target == pos_enable, so streaming starts at
//     the exact position captured when OperationEnabled was reached (zero jump).
//   * SOFT-START: the pure sine has its PEAK velocity at t=0 (cos(0)=1); the linear
//     amplitude envelope ramps that in over the first period, so velocity starts at 0
//     and grows gently -- no velocity step on enable. Position stays continuous
//     throughout (the envelope only scales amplitude, never shifts phase).

#include <cmath>
#include <cstdint>
#include <numbers>

namespace ethercat::tools {

// Commanded CSP target (encoder counts) at `t_s` seconds since enable. `amplitude` is
// in counts, `period_s` in seconds. period_s <= 0 degenerates to holding pos_enable.
inline std::int32_t sine_target_counts(std::int32_t pos_enable, double amplitude, double period_s, double t_s) noexcept {
    if (period_s <= 0.0) {
        return pos_enable;
    }
    const double ramp = (t_s < period_s) ? (t_s / period_s) : 1.0;  // min(1, t/period)
    const double phase = (2.0 * std::numbers::pi * t_s) / period_s;
    const double offset = amplitude * ramp * std::sin(phase);
    return pos_enable + static_cast<std::int32_t>(offset);
}

// The CSP target to STREAM this cycle, covering the whole enable handshake -- the fix
// for the first-energization enable-jump. In CSP the drive latches its initial setpoint
// from the 0x607A in the frame that TRIGGERS OperationEnabled (the cw=0x0F enable-ladder
// frame), NOT from the first frame we observe post-OE. So 0x607A must already equal the
// live actual position on that frame, or the drive slews from `actual` toward whatever
// stale value 0x607A held (e.g. 0) -- an uncontrolled first move from the absolute encoder
// position.
//
//   * Before OperationEnabled (climbing 06->07->0F): command the live ACTUAL position, so
//     every ladder frame -- including the OE-triggering one -- carries target==actual ->
//     genuine zero jump on enable.
//   * At/after OperationEnabled: command the soft-started sine relative to pos_enable
//     (which the caller captures == actual at the OE cycle, so t=0 -> target==pos_enable
//     continues seamlessly from the actual the ladder was streaming).
inline std::int32_t csp_target_counts(
    bool operation_enabled, std::int32_t pos_actual, std::int32_t pos_enable, double amplitude, double period_s, double t_s) noexcept {
    if (!operation_enabled) {
        return pos_actual;  // stream actual through the ladder -> zero jump on the OE-triggering frame
    }
    return sine_target_counts(pos_enable, amplitude, period_s, t_s);
}

}  // namespace ethercat::tools
