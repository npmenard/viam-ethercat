// Offline tests for the DC SYNC0 phase-lock PI controller. Pure integer math, so
// the convergence is deterministic + testable with no hardware: model a master
// wakeup drifting against a synthetic DC clock and check the PI pulls it into lock.

#include <cstdint>

#include "ethercat/dc_sync.hpp"
#include "test_harness.hpp"

using ethercat::dc_phase_correction;
using ethercat::dc_phase_locked;

TEST("dc_phase_correction: no DC clock (dc_time==0) -> zero correction") {
    std::int64_t integral = 0;
    CHECK_EQ(dc_phase_correction(0, 1'000'000, integral), 0L);
    CHECK_EQ(integral, std::int64_t{0});
    CHECK(!dc_phase_locked(0, 1'000'000));
}

TEST("dc_phase_correction: a large phase error is CLAMPED to +/- max") {
    std::int64_t integral = 0;
    const long c = dc_phase_correction(400'000, 1'000'000, integral, 0, 50'000);
    CHECK(c <= 50'000L);
    CHECK(c >= -50'000L);
}

TEST("dc_phase_correction: converges a 685us phase offset into the lock band") {
    // Reproduces the bench starting condition (dcPhase ~685us at OP). Each cycle the
    // observed DC phase shifts by our (corrected) deviation from the nominal period.
    constexpr std::int64_t cycle = 1'000'000;  // 1 ms
    std::int64_t integral = 0;
    std::int64_t phase = 685'000;  // initial offset
    bool locked = false;
    int lock_cycle = 0;
    for (int i = 0; i < 5000; ++i) {
        const long corr = dc_phase_correction(phase, cycle, integral);
        phase += corr;  // our wakeup moves by the correction relative to the DC clock
        if (dc_phase_locked(phase, cycle)) {
            locked = true;
            lock_cycle = i;
            break;
        }
    }
    CHECK(locked);          // the PI must pull the phase into the +/-50us band...
    CHECK(lock_cycle < 500);  // ...within a few hundred cycles (the warmup budget)
}

TEST("dc_phase_correction: a mid-cycle shift target locks OFF the SYNC0 edge") {
    // The bench fix: target cycle/2 (not 0) so jitter never crosses the pulse.
    constexpr std::int64_t cycle = 1'000'000;
    constexpr std::int64_t shift = cycle / 2;  // 500us mid-cycle target
    std::int64_t integral = 0;
    std::int64_t phase = 1'000;  // just off the SYNC0 edge (worst case for a mid-cycle target; 0 is the no-DC sentinel)
    bool locked = false;
    for (int i = 0; i < 5000; ++i) {
        const long corr = dc_phase_correction(phase, cycle, integral, shift);
        phase = ((phase + corr) % cycle + cycle) % cycle;
        if (dc_phase_locked(phase, cycle, shift)) {
            locked = true;
            break;
        }
    }
    CHECK(locked);
    // The locked phase must sit near cycle/2 (off both 0 and cycle edges).
    CHECK(phase > 400'000);
    CHECK(phase < 600'000);
}

TEST("dc_phase_locked: in-band vs out-of-band") {
    constexpr std::int64_t cycle = 1'000'000;
    CHECK(dc_phase_locked(10'000, cycle, 0, 50'000));        // small +phase -> locked
    CHECK(dc_phase_locked(cycle - 10'000, cycle, 0, 50'000));  // small -phase (wraps) -> locked
    CHECK(!dc_phase_locked(300'000, cycle, 0, 50'000));      // far -> not locked
}

TEST_MAIN()
