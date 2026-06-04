// Offline tests for the a6_validate CSP sine trajectory (#24) -- the safety-critical
// math for the first ENERGIZED motion: CSP-safe init (no position jump on enable) and
// soft-start (no velocity step on enable). Pure integer/float math, deterministic, no
// hardware. The HW bench is the behavioral backstop; this proves the trajectory shape.

#include <cmath>
#include <cstdint>

#include "test_harness.hpp"
#include "tools/sine_move.hpp"

using ethercat::tools::csp_target_counts;
using ethercat::tools::sine_target_counts;

TEST("sine_move: CSP-safe init -- t=0 returns pos_enable exactly (sin(0)=0, zero jump)") {
    // The whole point for CSP: streaming starts at the captured enable position, no step.
    CHECK_EQ(sine_target_counts(50000, 20000.0, 4.0, 0.0), 50000);
    CHECK_EQ(sine_target_counts(-1234, 99999.0, 2.0, 0.0), -1234);  // any origin/amplitude/period
}

TEST("sine_move: full period returns to the start position (sin(2pi)=0)") {
    CHECK_EQ(sine_target_counts(50000, 20000.0, 4.0, 4.0), 50000);  // ramp=1, sin(2pi)~0
}

TEST("sine_move: quarter period inside the ramp -> amplitude*ramp (soft-start scaling)") {
    // t=P/4: ramp=0.25, sin(pi/2)=1 -> offset = 20000*0.25 = 5000 (NOT the full 20000).
    CHECK_EQ(sine_target_counts(0, 20000.0, 4.0, 1.0), 5000);
}

TEST("sine_move: full amplitude only AFTER the ramp completes (first period)") {
    // t=1.25*P (ramp clamped to 1): sin(2.5pi)=1 -> +full amplitude.
    CHECK_EQ(sine_target_counts(0, 20000.0, 4.0, 5.0), 20000);
    // The first in-ramp sine peak (t=P/4) is scaled DOWN -> proves the envelope grows in.
    CHECK(sine_target_counts(0, 20000.0, 4.0, 1.0) < 20000);
}

TEST("sine_move: soft-start -- the first millisecond's step is ~0, far below the un-ramped peak") {
    // 1 ms after enable (1 kHz). Un-ramped sine velocity peaks at t=0 (A*2pi/P ~= 31416
    // counts/s -> ~31 counts/ms). The linear ramp makes the actual first-ms offset
    // O(dt^2) ~= 0, so there is no velocity STEP on enable -- the gentle start the bench needs.
    CHECK(std::abs(sine_target_counts(0, 20000.0, 4.0, 0.001)) <= 1);
}

TEST("sine_move: period <= 0 degenerates to holding pos_enable (no divide-by-zero)") {
    CHECK_EQ(sine_target_counts(777, 20000.0, 0.0, 3.3), 777);
}

// --- csp_target_counts: the enable-handshake streaming rule (first-energization safety) ---

TEST("csp_target: the OE-TRIGGERING frame (operation_enabled=false) commands live ACTUAL, not 0") {
    // THE first-energization gate. The drive latches its initial CSP setpoint from the 0x607A
    // on the cw=0x0F frame that TRIGGERS OperationEnabled -- and on THAT frame operation_enabled
    // is still FALSE (the drive hasn't processed the 0x0F yet). a6_validate's enable-ladder
    // branch calls this with operation_enabled=false for exactly that frame, so it MUST return
    // the live actual (not 0, not pos_enable, not the sine) -> the enabling frame carries
    // target==actual -> zero jump. pos_enable / amplitude / t are all IGNORED until OE.
    const std::int32_t actual = 87654;  // some nonzero absolute-encoder position
    CHECK_EQ(csp_target_counts(false, actual, /*pos_enable*/ 0, 20000.0, 4.0, 0.0), actual);
    CHECK_EQ(csp_target_counts(false, actual, /*pos_enable*/ 999, 20000.0, 4.0, 2.5), actual);
    // The bug this guards: without streaming actual, the enabling frame's 0x607A is a stale 0,
    // so the drive slews from `actual` toward 0 for >=1 cycle on first energization.
    CHECK(csp_target_counts(false, actual, 0, 20000.0, 4.0, 0.0) != 0);
}

TEST("csp_target: OE-triggering frame -> actual, and the first post-OE frame continues from it") {
    // Model the handshake seam: the last ladder frame streams `actual`; the caller captures
    // pos_enable = actual at the OE cycle; the first sine frame is t=0 -> pos_enable. So the
    // command is CONTINUOUS across enable (no step), which is the whole point.
    const std::int32_t actual = -50000;
    const std::int32_t ladder_cmd = csp_target_counts(false, actual, 0, 20000.0, 4.0, 0.0);
    const std::int32_t first_oe_cmd = csp_target_counts(true, actual, /*pos_enable=actual*/ actual, 20000.0, 4.0, 0.0);
    CHECK_EQ(ladder_cmd, actual);
    CHECK_EQ(first_oe_cmd, actual);      // sin(0)=0 -> pos_enable == actual
    CHECK_EQ(ladder_cmd, first_oe_cmd);  // zero jump across the OE seam
}

TEST("csp_target: after OE (operation_enabled=true) -> the soft-started sine") {
    // Post-enable it must match sine_target_counts exactly (CSP-safe init + soft-start).
    CHECK_EQ(csp_target_counts(true, /*actual ignored*/ 12345, 50000, 20000.0, 4.0, 1.0), sine_target_counts(50000, 20000.0, 4.0, 1.0));
    CHECK_EQ(csp_target_counts(true, 0, 0, 20000.0, 4.0, 5.0), 20000);  // full amplitude after ramp
}

TEST_MAIN()
