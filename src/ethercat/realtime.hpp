#pragma once

// ethercat::realtime -- shared real-time loop helpers.
//
// One implementation of the RT-thread plumbing: lock-memory + SCHED_FIFO setup and the DC
// SYNC0 phase-locked cyclic pacer.
//
// Layering: these live above Master (DcPacer drives the phase-locked cadence around
// Master::bringup_step / dc_time), keeping Master itself pure bus policy. The DC math is
// dc_sync.hpp's dc_phase_correction (the SOEM ec_sync PI).
//
// RT-safety: setup() and lock_current() are non-RT prelude, called once before the loop.
// DcPacer::pace() and step() are noexcept and allocation-free, safe on the cyclic path.

#include <algorithm>
#include <cassert>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <ctime>

#include "ethercat/dc_sync.hpp"

namespace ethercat {

namespace realtime {

inline constexpr std::size_t kDefaultPrefaultBytes = std::size_t{512} * 1024;  // 512 KiB of stack
inline constexpr int kDefaultRtPriority = 80;
inline constexpr std::uint64_t kNsPerSec = 1'000'000'000ULL;

// CLOCK_MONOTONIC now, in nanoseconds. The clock the absolute-deadline pacer sleeps
// against (clock_nanosleep TIMER_ABSTIME), so they must be the same clock.
inline std::uint64_t monotonic_ns() noexcept {
    timespec ts{};
    (void)clock_gettime(CLOCK_MONOTONIC, &ts);
    return (static_cast<std::uint64_t>(ts.tv_sec) * kNsPerSec) + static_cast<std::uint64_t>(ts.tv_nsec);
}

// Enter the real-time regime for the calling thread (call once in the RT thread prelude,
// before the cyclic loop):
//   * mlockall(MCL_CURRENT|MCL_FUTURE) + mallopt(M_TRIM_THRESHOLD=-1, M_MMAP_MAX=0) to keep
//     every page resident so the loop never takes a page or heap fault;
//   * pre-fault `prefault_bytes` of this thread's stack so the loop's first deep call chain
//     does not fault a fresh stack page (the touched pages stay mapped and are locked by
//     MCL_FUTURE);
//   * SCHED_FIFO at `priority` via pthread_setschedparam(pthread_self(), ...), thread-level.
// mlockall/mallopt/pre-fault are best-effort (a missing CAP_IPC_LOCK only costs jitter). The
// return reflects only the SCHED_FIFO result -- the bit a require_realtime caller throws on --
// so the caller decides throw vs warn. Needs CAP_SYS_NICE (SCHED_FIFO) and CAP_IPC_LOCK
// (mlockall); returns false (no throw) when it cannot get SCHED_FIFO so a best-effort run can
// continue.
bool setup(int priority = kDefaultRtPriority, std::size_t prefault_bytes = kDefaultPrefaultBytes) noexcept;

// mlockall(MCL_CURRENT) only: the pre-spawn lock done in configure() so the RT jthread's later
// allocations do not EAGAIN. Best-effort.
void lock_current() noexcept;

// Preflight probe: can this process obtain SCHED_FIFO at `priority`? Tries
// pthread_setschedparam on a short-lived scratch thread, so unlike setup() it has NO process-wide
// side effects (no mlockall, no mallopt) and never leaves the calling thread realtime. Use it to
// fail fast with a clear error BEFORE opening the bus on hosts without realtime scheduling
// (needs CAP_SYS_NICE or an adequate RLIMIT_RTPRIO -- most stock desktops/servers have neither).
bool sched_fifo_available(int priority) noexcept;

// DC SYNC0 phase-locked cyclic pacer. Owns the absolute clock_nanosleep deadline (`next`) and
// the PI integral accumulator; one pace() per cyclic iteration sleeps to the next
// phase-corrected deadline. Single-thread (the RT loop); no atomics.
//
// pace(dc_time) is step(dc_time, monotonic_ns()) plus the sleep to the deadline. step() is the
// deterministic, sleep-free core, so the phase-lock math and overrun catch-up are unit-testable
// against a synthetic dc_time ramp.
class DcPacer {
   public:
    // period_ns: the cyclic period; shift_ns: the SYNC0 phase target (typically period/2). The
    // first deadline is armed at construction (now + period); call reset() to re-arm against a
    // chosen base. Precondition: period_ns > 0 (asserted). The period must also exceed
    // dc_phase_correction's max_correction (+/-50us): when period > that clamp, `period + corr`
    // is always > 0, so every cycle advances by exactly period + corr. At sub-50us periods the
    // advance() underflow clamp keeps the deadline monotonic but the steady-state advance is no
    // longer exact.
    DcPacer(std::uint64_t period_ns, std::int64_t shift_ns) noexcept
        : period_ns_(period_ns), shift_ns_(shift_ns), next_(monotonic_ns() + period_ns) {
        assert(period_ns_ > 0 && "DcPacer: period_ns must be positive");
    }
    // Default DC phase target: wake mid-cycle between SYNC0 edges (period/2). A delegating ctor
    // rather than a default argument, since a default arg cannot reference a preceding parameter.
    explicit DcPacer(std::uint64_t period_ns) noexcept : DcPacer(period_ns, static_cast<std::int64_t>(period_ns) / 2) {}

    // Re-arm the absolute deadline to `first_deadline_ns` and zero the integral.
    // (Production: align to a known epoch; tests: set a deterministic base.)
    void reset(std::uint64_t first_deadline_ns) noexcept {
        next_ = first_deadline_ns;
        integral_ = 0;
    }

    // Advance the deadline by one phase-corrected period (no catch-up, no sleep). The correction
    // comes from dc_sync.hpp's dc_phase_correction (the wrap, +/-clamp, and anti-windup live
    // there). `dc_time_ns == 0` (no DC clock) gives correction 0 and a pure periodic advance.
    // Underflow guard: corr is bounded by +/-max_correction_ns (50us); at 1 kHz (period 1ms)
    // `period + corr` is always positive, so this is a no-op there. At very high rates (period
    // below the clamp, above ~10 kHz) a negative corr could drive delta <= 0, so clamp to 1 to
    // keep the absolute deadline from moving backwards.
    void advance(std::int64_t dc_time_ns) noexcept {
        const long corr = dc_phase_correction(dc_time_ns, static_cast<std::int64_t>(period_ns_), integral_, shift_ns_);
        const long delta = std::max<long>(static_cast<long>(period_ns_) + corr, 1);
        next_ += static_cast<std::uint64_t>(delta);
    }

    // Advance one phase-corrected period, then skip whole missed periods while the deadline is
    // behind the clock, calling now() each iteration (never rebasing) so a transient overrun
    // realigns to the SYNC0 grid instead of firing an off-phase LRW burst. Templated on the
    // clock: pace() injects monotonic_ns so it re-reads the clock per iteration, and a test can
    // inject an advancing clock to exercise that multi-read path. noexcept and alloc-free (the
    // callable inlines; not std::function). No sleep; returns the new absolute deadline. Mutates
    // the integral and deadline.
    template <class NowFn>
    std::uint64_t advance_to_deadline(std::int64_t dc_time_ns, NowFn now, std::uint32_t* skipped = nullptr) noexcept {
        advance(dc_time_ns);
        std::uint32_t n = 0;  // whole periods skipped: how many cycles late the thread woke
        for (std::uint64_t t = now(); next_ <= t; t = now()) {
            next_ += period_ns_;
            ++n;
        }
        if (skipped != nullptr) {
            *skipped = n;
        }
        return next_;
    }

    // Deterministic, sleep-free unit form: advance_to_deadline with a fixed injected clock (the
    // catch-up sees one constant `now_ns`). Used by the unit tests for the
    // phase/correction/integral math; the advancing-clock re-read path that pace() runs is
    // covered separately via advance_to_deadline.
    std::uint64_t step(std::int64_t dc_time_ns, std::uint64_t now_ns) noexcept {
        return advance_to_deadline(dc_time_ns, [now_ns]() noexcept { return now_ns; });
    }

    // One cyclic iteration: advance and skip-catch-up, re-reading monotonic_ns() each iteration,
    // then sleep (CLOCK_MONOTONIC, TIMER_ABSTIME) to the absolute deadline. Returns the number
    // of whole periods that had to be skipped to catch up, i.e. how many cycles late the thread
    // woke (0 on a healthy cycle). A non-zero value means the RT thread was starved (host
    // contention, priority inversion, page fault) and process data gapped that long; the caller
    // surfaces it.
    std::uint32_t pace(std::int64_t dc_time_ns) noexcept {
        std::uint32_t skipped = 0;
        (void)advance_to_deadline(dc_time_ns, []() noexcept { return monotonic_ns(); }, &skipped);
        timespec ts{};
        ts.tv_sec = static_cast<std::time_t>(next_ / kNsPerSec);
        ts.tv_nsec = static_cast<long>(next_ % kNsPerSec);
        (void)clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, nullptr);
        return skipped;
    }

    std::uint64_t deadline() const noexcept {
        return next_;
    }
    std::int64_t integral() const noexcept {
        return integral_;
    }

   private:
    std::uint64_t period_ns_;
    std::int64_t shift_ns_;
    std::uint64_t next_;
    std::int64_t integral_ = 0;
};

}  // namespace realtime
}  // namespace ethercat
