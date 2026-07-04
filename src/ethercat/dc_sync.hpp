#pragma once

// DC SYNC0 phase-lock controller (the SOEM ec_sync idiom). Shared by the bench tool, the
// Master's warmup, and the RT loop so there is one implementation. Pure and header-only, so it
// is unit-testable offline against a synthetic DC ramp.

#include <cstdint>

namespace ethercat {

// PI controller that phase-aligns the cyclic wakeup to the DC SYNC0 pulse: returns
// the ns to ADD to the next absolute clock_nanosleep deadline so the DC-time phase
// `(dc_time - shift_ns) mod cycle_ns` converges to 0 (i.e. the master's send lands
// at a fixed phase relative to SYNC0). `integral` is the caller-owned accumulator
// (RT-thread-local; no sharing). Returns 0 when there is no DC clock (dc_time == 0).
//
// The `delta/100` proportional and `integral/20` integral gains are SOEM's ec_sync gains, kept
// unchanged so on-hardware phase-lock matches the known-good reference. This adds the +/- clamp
// and integral anti-windup below. realtime::DcPacer owns the absolute deadline and sleep around
// this call.
//
// The correction is clamped to +/- max_correction_ns: a dc_time glitch or wrap must not spike
// one deadline into a huge sleep or a busy-spin. The integral is clamped too (anti-windup) so a
// long transient cannot accumulate unbounded.
inline long dc_phase_correction(std::int64_t dc_time,
                                std::int64_t cycle_ns,
                                std::int64_t& integral,
                                std::int64_t shift_ns = 0,
                                long max_correction_ns = 50'000) noexcept {
    if (dc_time == 0 || cycle_ns == 0) {
        return 0;  // no DC clock -> nothing to lock to
    }
    // Normalize to [0, cycle) first (defensive: shift_ns > dc_time at startup would
    // otherwise leave a negative un-wrapped remainder), then wrap to (-cycle/2, cycle/2].
    std::int64_t delta = (((dc_time - shift_ns) % cycle_ns) + cycle_ns) % cycle_ns;
    if (delta > cycle_ns / 2) {
        delta -= cycle_ns;  // shortest signed distance to the target phase
    }
    integral += (delta > 0) ? 1 : (delta < 0 ? -1 : 0);
    // Anti-windup: bound the integral to the correction range (in its own units).
    const std::int64_t integral_limit = max_correction_ns * 20;
    if (integral > integral_limit) {
        integral = integral_limit;
    } else if (integral < -integral_limit) {
        integral = -integral_limit;
    }
    long corr = static_cast<long>(-(delta / 100) - (integral / 20));  // P + I (SOEM gains)
    if (corr > max_correction_ns) {
        corr = max_correction_ns;
    } else if (corr < -max_correction_ns) {
        corr = -max_correction_ns;
    }
    return corr;
}

}  // namespace ethercat
