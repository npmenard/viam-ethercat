// Offline tests for ethercat::realtime (#31 P3a). The DcPacer phase-lock math is
// deterministic + testable via its sleep-free step(dc_time, now) core (the clock is
// injected), so we exercise it against synthetic dc_time / now ramps with no real
// sleeping and no hardware. setup()/lock_current() are env-dependent (SCHED_FIFO
// needs CAP_SYS_NICE) -- we only assert the no-throw / returns-a-bool path offline,
// isolating any SCHED_FIFO effect to a short-lived thread.

#include <sys/mman.h>  // munlockall (test cleanup)

#include <chrono>
#include <cstdint>
#include <memory>
#include <thread>

#include "ethercat/dc_sync.hpp"
#include "ethercat/realtime.hpp"
#include "test_harness.hpp"

namespace realtime = ethercat::realtime;
using ethercat::dc_phase_correction;

namespace {
constexpr std::uint64_t kPeriod = 1'000'000;  // 1 ms
constexpr std::int64_t kShift = static_cast<std::int64_t>(kPeriod) / 2;
}  // namespace

// monotonic_ns is the clock the pacer sleeps against; must be monotone non-decreasing.
TEST("realtime::monotonic_ns is non-decreasing") {
    const std::uint64_t a = realtime::monotonic_ns();
    const std::uint64_t b = realtime::monotonic_ns();
    CHECK(b >= a);
}

// With no DC clock (dc_time==0 -> correction 0) and `now` held behind the deadline,
// step() advances the deadline by EXACTLY one period each call -- pure periodic pacing.
TEST("DcPacer::step advances by exactly one period when un-corrected + on time") {
    realtime::DcPacer pacer(kPeriod, kShift);
    pacer.reset(5 * kPeriod);  // deadline base well ahead of now=0
    CHECK_EQ(pacer.step(0, /*now=*/0), 6 * kPeriod);
    CHECK_EQ(pacer.step(0, /*now=*/0), 7 * kPeriod);
    CHECK_EQ(pacer.step(0, /*now=*/0), 8 * kPeriod);
}

// step() applies dc_phase_correction with its OWN threaded integral: drive a phase
// ramp and check each deadline delta == period + the correction a parallel reference
// dc_phase_correction (same integral sequence) produces -- i.e. the pacer threads the
// PI exactly, it doesn't reinvent or drop the integral.
TEST("DcPacer::step threads dc_phase_correction (correction + integral) exactly") {
    realtime::DcPacer pacer(kPeriod, kShift);
    pacer.reset(0);
    std::int64_t ref_integral = 0;
    std::uint64_t prev = 0;
    for (int i = 0; i < 64; ++i) {
        const std::int64_t dc = 685'000 + i * 137;  // an arbitrary moving phase
        const long ref_corr = dc_phase_correction(dc, static_cast<std::int64_t>(kPeriod), ref_integral, kShift);
        const std::uint64_t dl = pacer.step(dc, /*now=*/0);  // now=0 -> deadlines grow past it, no catch-up
        const auto expected = prev + static_cast<std::uint64_t>(static_cast<long>(kPeriod) + ref_corr);
        CHECK_EQ(dl, expected);
        prev = dl;
    }
    CHECK_EQ(pacer.integral(), ref_integral);  // integral threaded identically
}

// A multi-period overrun catches up by adding WHOLE periods (phase-preserving) rather
// than rebasing to `now`: the new deadline stays on the original grid and lands just
// past now (within one period).
TEST("DcPacer::step overrun catch-up is phase-preserving (whole periods, never rebased)") {
    realtime::DcPacer pacer(kPeriod, kShift);
    constexpr std::uint64_t base = 1000;
    pacer.reset(base);
    const std::uint64_t now = base + 3 * kPeriod + 1234;  // ~3 periods behind
    const std::uint64_t dl = pacer.step(/*dc_time=*/0, now);
    CHECK(dl > now);                    // sleep is always into the future
    CHECK((dl - base) % kPeriod == 0);  // still on the base grid (phase preserved)
    CHECK(dl - now <= kPeriod);         // realigned to the NEXT grid point past now
}

// A correction does NOT defeat the phase grid for the on-time case: with now behind,
// the single-step deadline is exactly base+period+corr (no spurious catch-up).
TEST("DcPacer::step applies one correction without catch-up when on time") {
    realtime::DcPacer pacer(kPeriod, kShift);
    pacer.reset(10 * kPeriod);
    std::int64_t ref_integral = 0;
    const long corr = dc_phase_correction(700'000, static_cast<std::int64_t>(kPeriod), ref_integral, kShift);
    const std::uint64_t dl = pacer.step(700'000, /*now=*/0);
    CHECK_EQ(dl, 10 * kPeriod + static_cast<std::uint64_t>(static_cast<long>(kPeriod) + corr));
}

// Underflow guard: at a rate where the period is BELOW the +/-50us correction clamp
// (here 20us << 50us), a negative correction could drive `period + corr` <= 0. The
// deadline must NEVER move backwards -- assert it strictly increases across a phase
// ramp that drives corrections negative.
TEST("DcPacer::step never moves the deadline backwards (high-rate underflow guard)") {
    constexpr std::uint64_t fast_period = 20'000;  // 50 kHz, < the 50us correction clamp
    realtime::DcPacer pacer(fast_period, static_cast<std::int64_t>(fast_period) / 2);
    pacer.reset(0);
    std::uint64_t prev = 0;
    for (int i = 0; i < 400; ++i) {
        // A phase ramp well ahead of the mid-cycle target -> negative corrections.
        const std::int64_t dc = (static_cast<std::int64_t>(i) * 4096) % static_cast<std::int64_t>(fast_period);
        const std::uint64_t dl = pacer.step(dc, /*now=*/0);
        CHECK(dl > prev);  // strictly forward every step, guard holds even when period+corr<=0
        prev = dl;
    }
}

// The PRODUCTION pace() re-reads the clock each catch-up iteration; exercise that exact
// multi-read path by injecting an ADVANCING clock into advance_to_deadline (the same core
// pace() runs with monotonic_ns). As the clock climbs past further grid points mid-loop,
// the deadline keeps skipping WHOLE periods until it leads the latest reading -- a single
// FIXED snapshot would have stopped earlier, so reaching 5'001'000 proves the re-read
// tracks a moving clock (and stays phase-preserved). Closes the step()-fixed-now vs
// pace()-re-read coverage nuance without changing the production form.
TEST("DcPacer::advance_to_deadline re-read catch-up tracks an ADVANCING clock") {
    realtime::DcPacer pacer(kPeriod, kShift);
    constexpr std::uint64_t base = 1000;
    pacer.reset(base);
    // advance(0) -> next_ = base + period = 1'001'000. Readings climb ~1 period each, so the
    // re-read keeps finding next_ behind: skips to 2'001'000, 3'001'000, 4'001'000, 5'001'000,
    // then 4'500'000 < 5'001'000 stops. A FIXED now=1'500'000 would have stopped at 2'001'000.
    const std::uint64_t clk_seq[] = {1'500'000, 2'500'000, 3'500'000, 4'500'000, 4'500'000, 4'500'000};
    std::size_t i = 0;
    auto now = [&]() noexcept {
        const std::uint64_t v = clk_seq[i];
        if (i + 1 < (sizeof(clk_seq) / sizeof(clk_seq[0]))) {
            ++i;
        }
        return v;
    };
    const std::uint64_t dl = pacer.advance_to_deadline(/*dc_time=*/0, now);
    CHECK_EQ(dl, std::uint64_t{5'001'000});  // re-read tracked the advancing clock (4 skips)
    CHECK((dl - base) % kPeriod == 0);       // phase preserved on the base grid
}

// setup()/lock_current() must be callable + noexcept offline. SCHED_FIFO needs
// CAP_SYS_NICE (returns false without it; may succeed under sudo) -- we don't assert
// the value, only that it doesn't throw/crash. setup() runs in a short-lived thread so
// any SCHED_FIFO effect dies with it; munlockall() drops any page locks afterward.
TEST("realtime::setup + lock_current are noexcept + callable offline") {
    realtime::lock_current();  // benign mlockall(MCL_CURRENT)
    bool sched_ok = false;
    std::thread t([&] { sched_ok = realtime::setup(/*priority=*/10, /*prefault_bytes=*/4096); });
    t.join();
    (void)sched_ok;        // env-dependent: false without CAP_SYS_NICE, true under sudo
    (void)::munlockall();  // cleanup: undo any locks setup()/lock_current() took
    CHECK(true);           // reaching here = no throw/crash
}

// (#40 item 1) The delegating ctor defaults the phase target to period/2 -- identical
// math to the explicit two-arg form.
TEST("DcPacer(period) defaults the shift to period/2 (delegating ctor)") {
    realtime::DcPacer defaulted(kPeriod);
    realtime::DcPacer explicit_shift(kPeriod, kShift);
    defaulted.reset(0);
    explicit_shift.reset(0);
    for (int i = 0; i < 32; ++i) {
        const std::int64_t dc = 700'000 + i * 311;
        CHECK_EQ(defaulted.step(dc, 0), explicit_shift.step(dc, 0));  // identical corrections
    }
    CHECK_EQ(defaulted.integral(), explicit_shift.integral());
}

TEST_MAIN()
