#pragma once

// ethercat::realtime -- shared real-time loop helpers (#31 P3a).
//
// Extracts the RT-thread plumbing that the bench tool (a6_validate) and the
// production ServoController each hand-rolled today, so there is ONE
// implementation of: lock-memory + SCHED_FIFO setup, the DC SYNC0 phase-locked
// cyclic pacer, and the bring-up-to-OPERATIONAL pump. ADDITIVE in P3a -- neither
// consumer is migrated yet (P3b/P3c do that, behavior-preserving + HW-gated).
//
// Layering: these live ABOVE Master (DcPacer + run_to_operational drive
// Master::bringup_step / dc_time), keeping Master itself pure bus-policy. The DC
// math is dc_sync.hpp's dc_phase_correction (the SOEM ec_sync PI), unchanged.
//
// RT-safety: setup()/lock_current() are non-RT prelude (called ONCE before the
// loop). DcPacer::pace() and step() are noexcept and allocation-free -- safe on
// the cyclic path. run_to_operational() is the bring-up pump (pre-steady-state).

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <ctime>
#include <functional>

#include "ethercat/dc_sync.hpp"

namespace ethercat {

// Forward declarations (run_to_operational is defined in realtime.cpp, so the
// header doesn't pull in master.hpp). BringupStatus has a fixed underlying type,
// so it can be forward-declared.
class Master;
enum class BringupStatus : std::uint8_t;

namespace realtime {

inline constexpr std::size_t kDefaultPrefaultBytes = 512U * 1024U;  // 512 KiB of stack
inline constexpr int kDefaultRtPriority = 80;
inline constexpr std::uint64_t kNsPerSec = 1'000'000'000ULL;

// CLOCK_MONOTONIC now, in nanoseconds. The clock the abs-deadline pacer sleeps
// against (clock_nanosleep TIMER_ABSTIME), so they MUST be the same clock.
inline std::uint64_t monotonic_ns() noexcept {
    timespec ts{};
    (void)clock_gettime(CLOCK_MONOTONIC, &ts);
    return (static_cast<std::uint64_t>(ts.tv_sec) * kNsPerSec) + static_cast<std::uint64_t>(ts.tv_nsec);
}

// Enter the real-time regime for the CALLING thread (call ONCE in the RT thread
// prelude, before the cyclic loop):
//   * mlockall(MCL_CURRENT|MCL_FUTURE) + mallopt(M_TRIM_THRESHOLD=-1, M_MMAP_MAX=0)
//     -- keep every page resident so the loop never takes a page/heap fault;
//   * pre-fault `prefault_bytes` of THIS thread's stack -- so the loop's first
//     deep call chain doesn't fault a fresh stack page (the one genuinely new
//     bit vs the hand-rolled versions; the touched pages stay mapped + are locked
//     by MCL_FUTURE);
//   * SCHED_FIFO at `priority` via pthread_setschedparam(pthread_self(), ...)
//     -- thread-level (the unified form; a6_validate's process-level
//     sched_setscheduler folds into this).
// mlockall/mallopt/pre-fault are BEST-EFFORT (a missing CAP_IPC_LOCK only costs
// jitter). The RETURN reflects ONLY the SCHED_FIFO result -- the load-bearing
// bit a require_realtime caller throws on -- so the caller decides throw-vs-warn.
// Needs CAP_SYS_NICE (SCHED_FIFO) and CAP_IPC_LOCK (mlockall); returns false
// (no throw) when it can't get SCHED_FIFO so a best-effort run can continue.
bool setup(int priority = kDefaultRtPriority, std::size_t prefault_bytes = kDefaultPrefaultBytes) noexcept;

// mlockall(MCL_CURRENT) only -- the pre-spawn lock the ServoController does in
// configure() so the RT jthread's later allocations don't EAGAIN. Best-effort.
void lock_current() noexcept;

// DC SYNC0 phase-locked cyclic pacer. Owns the absolute clock_nanosleep deadline
// (`next`) + the PI integral accumulator; one pace() per cyclic iteration sleeps
// to the next phase-corrected deadline. Single-thread (the RT loop); no atomics.
//
// pace(dc_time) == step(dc_time, monotonic_ns()) + sleep-to-deadline. step() is
// the deterministic, sleep-free core (now injected) so the phase-lock math +
// overrun catch-up are unit-testable against a synthetic dc_time ramp.
class DcPacer {
   public:
    // period_ns: the cyclic period; shift_ns: the SYNC0 phase target (dc_sync.hpp,
    // typically period/2). The first deadline is armed at construction (now +
    // period); call reset() to re-arm against a chosen base.
    DcPacer(std::uint64_t period_ns, std::int64_t shift_ns) noexcept
        : period_ns_(period_ns), shift_ns_(shift_ns), next_(monotonic_ns() + period_ns) {}

    // Re-arm the absolute deadline to `first_deadline_ns` and zero the integral.
    // (Production: align to a known epoch; tests: set a deterministic base.)
    void reset(std::uint64_t first_deadline_ns) noexcept {
        next_ = first_deadline_ns;
        integral_ = 0;
    }

    // Deterministic, sleep-FREE core: advance the deadline by one phase-corrected
    // period, then phase-preservingly catch up past `now_ns` (add WHOLE periods --
    // NEVER rebase) so a transient overrun realigns to the SYNC0 grid instead of
    // firing an off-phase burst. Mutates the integral + deadline; returns the new
    // absolute deadline. `dc_time_ns == 0` (no DC clock) yields a pure periodic
    // advance (correction 0).
    std::uint64_t step(std::int64_t dc_time_ns, std::uint64_t now_ns) noexcept {
        const long corr = dc_phase_correction(dc_time_ns, static_cast<std::int64_t>(period_ns_), integral_, shift_ns_);
        next_ += static_cast<std::uint64_t>(static_cast<long>(period_ns_) + corr);
        while (next_ <= now_ns) {
            next_ += period_ns_;
        }
        return next_;
    }

    // One cyclic iteration: compute the next phase-corrected deadline from the
    // current DC time + clock, then sleep (CLOCK_MONOTONIC, TIMER_ABSTIME) to it.
    void pace(std::int64_t dc_time_ns) noexcept {
        const std::uint64_t deadline = step(dc_time_ns, monotonic_ns());
        timespec ts{};
        ts.tv_sec = static_cast<std::time_t>(deadline / kNsPerSec);
        ts.tv_nsec = static_cast<long>(deadline % kNsPerSec);
        (void)clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, nullptr);
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

// Pump Master::bringup_step() to a terminal state, owning the phase-locked cadence
// via `pacer`. Per iteration: query `sync_faulted()` (the caller's read of the
// drive's Er74.1 "no SYNC0" from the prior step's feedback -- keeps Master free of
// CiA402 semantics), step the bring-up, then pace() the cycle. Returns when the
// bring-up reaches Operational or Aborted, or BringupStatus::Aborted if `timeout`
// elapses first (a bounded give-up -- repeated Er74 OP-entry wedges the A6).
//
// For the SIMPLE consumers (#21, a6_validate). The ServoController KEEPS its inline
// prelude (it publishes the drive-fault tier on abort, which this thin pump does not).
BringupStatus run_to_operational(Master& master,
                                 DcPacer& pacer,
                                 const std::function<bool()>& sync_faulted,
                                 std::chrono::milliseconds timeout);

}  // namespace realtime
}  // namespace ethercat
