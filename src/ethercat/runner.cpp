#include "ethercat/runner.hpp"

#include <algorithm>
#include <cassert>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>
#include <utility>

#include "ethercat/errors.hpp"

namespace ethercat {

const char* to_string(StopReason r) noexcept {
    switch (r) {
        case StopReason::None:
            return "None";
        case StopReason::Requested:
            return "Requested";
        case StopReason::BusFault:
            return "BusFault";
        case StopReason::BringupAborted:
            return "BringupAborted";
        case StopReason::RtSetupFailed:
            return "RtSetupFailed";
        case StopReason::Wedged:
            return "Wedged";
    }
    return "Unknown";
}

const char* to_string(RunnerPhase p) noexcept {
    switch (p) {
        case RunnerPhase::Idle:
            return "Idle";
        case RunnerPhase::BringingUp:
            return "BringingUp";
        case RunnerPhase::Running:
            return "Running";
        case RunnerPhase::Stopping:
            return "Stopping";
        case RunnerPhase::Stopped:
            return "Stopped";
    }
    return "Unknown";
}

// --- CycleContext ----------------------------------------------------------

void CycleContext::request_stop() noexcept {
    // Legal from inside step()/hooks (the RT error channel) and harmless if the control
    // cached the ctx, since it only latches an atomic. Routed through core_ (the RT-shared
    // state the thread lives in), reachable for the whole RT lifetime.
    core_->latch_reason(StopReason::Requested);
    core_->stop_flag_.store(true, std::memory_order_release);
}

WkcStats CycleContext::wkc() const noexcept {
    check_live();
    return core_->master_.wkc_stats();
}

bool CycleContext::fault() const noexcept {
    check_live();
    return core_->master_.fault();
}

void CycleContext::check_live() const noexcept {
    // Debug-only: a control that touches the ctx outside its dispatch window gets a loud,
    // immediate failure in debug builds. In release this compiles to nothing, and per the
    // owned-data design (inputs_/outputs_ are by-value, not spans into live buffers) the
    // worst case there is safe-stale: a valid object with last-cycle data, never a dangling
    // or in-flight read.
    assert(live_ && "CycleContext used outside its dispatch window (#47 valid on the RT thread, during dispatch only)");
}

// --- RtCore (the RT-thread-shared cyclic state) ---------------

void RtCore::request_stop() noexcept {
    latch_reason(StopReason::Requested);
    stop_flag_.store(true, std::memory_order_release);
}

void RtCore::latch_reason(StopReason r) noexcept {
    StopReason expected = StopReason::None;
    (void)reason_.compare_exchange_strong(expected, r, std::memory_order_acq_rel);  // first cause wins
}

// --- Runner ----------------------------------------------------------------

Runner::Runner(Master& master, RunnerConfig cfg) noexcept : master_(master) {
    if (cfg.teardown_cycles == 0) {
        cfg.teardown_cycles = 1;  // documented floor
    }
    // The RT state is heap-held from construction so its address is stable for the thread to
    // capture and so a wedge can leak it intact. OOM here terminates (noexcept); a Runner you
    // cannot allocate is unrecoverable regardless.
    rt_core_ = std::make_unique<RtCore>(master, cfg);
}

Runner::~Runner() {
    stop();
}

void Runner::attach(std::uint16_t slave_id, SlaveControl& control) {
    if (started_.load(std::memory_order_acquire)) {
        throw Error("Runner::attach: controls must be attached before start()");
    }
    if (slave_id == 0 || slave_id > master_.slave_count()) {
        throw Error("Runner::attach: slave " + std::to_string(slave_id) + " out of range (bus has " +
                    std::to_string(master_.slave_count()) + ")");
    }
    for (const RtCore::Attached& a : rt_core_->controls_) {
        if (a.slave_id == slave_id) {
            throw Error("Runner::attach: slave " + std::to_string(slave_id) + " already has a control attached");
        }
    }
    rt_core_->controls_.emplace_back(slave_id, &control, rt_core_.get());  // ctx built in place (non-movable, #47)
}

void Runner::start() {
    if (started_.load(std::memory_order_acquire)) {
        throw Error("Runner::start: already started (one start() per Runner; restart = a fresh Runner)");
    }
    if (rt_core_->controls_.empty()) {
        throw Error("Runner::start: no controls attached");
    }
    // Non-RT hooks first, the only throwing phase. A throw here aborts start() cleanly:
    // nothing locked, no thread, no rt_active bracket, master untouched. on_configured gets
    // the restricted ConfigContext, never a raw Master&.
    for (RtCore::Attached& a : rt_core_->controls_) {
        ConfigContext cfg{master_, a.slave_id};
        a.control->on_configured(cfg);
    }
    realtime::lock_current();
    started_.store(true, std::memory_order_release);
    rt_core_->phase_.store(RunnerPhase::BringingUp, std::memory_order_relaxed);
    // The thread captures the RtCore*, not `this`, so it never reaches a Runner member, and
    // a leaked RtCore (on a wedge) carries everything the thread needs.
    rt_core_->thread_ = std::jthread([core = rt_core_.get()](const std::stop_token& st) { core->rt_body(st); });
}

void Runner::request_stop() noexcept {
    rt_core_->request_stop();
}

void Runner::stop() noexcept {
    if (!started_.load(std::memory_order_acquire)) {
        return;  // never started (or a failed start): nothing to tear down
    }
    RtCore& core = *rt_core_;
    // Only ~Runner and run() reach here, both on the owner thread, so this never runs on the
    // RT thread.
    core.request_stop();  // latch Requested (first-cause) + set the flag

    // Bounded wait: wait for the RT loop to finish its stopping window and mark Stopped, up
    // to a derived or configured ceiling. A non-wedged teardown reaches Stopped well inside
    // it; a wedged step() never does.
    std::chrono::nanoseconds bound = core.cfg_.stop_join_timeout;
    if (bound <= std::chrono::nanoseconds::zero()) {
        const std::uint64_t period_ns = 1'000'000'000ULL / master_.loop_rate_hz();
        const auto derived = std::chrono::nanoseconds((static_cast<std::uint64_t>(core.cfg_.teardown_cycles) + 20U) * period_ns * 4U);
        bound = std::max<std::chrono::nanoseconds>(std::chrono::milliseconds(250), derived);
    }
    const auto deadline = std::chrono::steady_clock::now() + bound;
    while (core.phase_.load(std::memory_order_acquire) != RunnerPhase::Stopped) {
        if (std::chrono::steady_clock::now() >= deadline) {
            // Wedged, so fail-stop. The RT thread is stuck inside control->step() and will
            // never return; it holds the ctx (Runner-internal, leakable) and the
            // externally-owned control (a reference from attach(), not ours to leak). If this
            // returned, the orderly dtor chain would free state the abandoned thread is still
            // using, a use-after-free that could corrupt the host process heap. A thread
            // parked in foreign code cannot be reclaimed, and the consumer's control cannot be
            // kept alive past its owner, so abort the process instead of tearing down. The
            // drive is already safe (process data gapped upstream of the wedge, so the SM
            // watchdog de-energizes it in about 50 ms); the supervisor restarts the module.
            (void)std::fprintf(stderr,
                               "[ethercat] Runner::stop: RT loop WEDGED -- step() did not return within the bounded "
                               "teardown ceiling. The RT thread is parked inside a control's step() and cannot be "
                               "reclaimed; continuing teardown would use-after-free the control it still holds. "
                               "FAIL-STOP: aborting the module process. The drive de-energizes via the SM watchdog "
                               "(PD gapped upstream of the wedge); the supervisor will restart the module.\n");
            (void)std::fflush(stderr);
            std::abort();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    // Clean exit: the loop reached Stopped, so the join returns immediately.
    if (core.thread_.joinable()) {
        core.thread_.join();
    }
    master_.close();  // the INIT teardown (idempotent at the backend)
}

void Runner::run() {
    start();
    // Block until the RT loop ends (any cause), then run the join/close teardown.
    while (rt_core_->phase_.load(std::memory_order_acquire) != RunnerPhase::Stopped) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    stop();
}

template <class Fn>
void RtCore::dispatch(Attached& a, std::uint64_t cycle, std::int64_t dc, bool stopping, Fn&& fn) noexcept {
    CycleContext& ctx = a.ctx;
    const std::span<const std::byte> in = master_.input_image(a.slave_id);
    const std::span<std::byte> out = master_.outputs(a.slave_id);

    // Copy-in: refresh the ctx's owned input from this cycle's latched feedback. Applies to
    // every ctx-touching hook, including sync_faulted() during bring-up, which loads
    // FaultCode and must see the refreshed input.
    ctx.input_size_ = in.size();
    std::memcpy(ctx.inputs_.data(), in.data(), in.size());
    // Seed the owned output from the live command image so a field the hook does not store
    // carries its current wire value over. This makes a read-only or partial-write hook
    // behave as if it wrote directly through to the live image.
    ctx.output_size_ = out.size();
    std::memcpy(ctx.outputs_.data(), out.data(), out.size());

    ctx.cycle_ = cycle;
    ctx.dc_time_ = dc;
    ctx.stopping_ = stopping;
    ctx.live_ = true;  // the dispatch window: ctx is legal only in here
    fn(ctx);
    ctx.live_ = false;

    // Copy-out: the owned output goes to the wire and ships with the next process(). A store
    // made through an escaped (stale) handle after this point lands in the owned buffer only
    // and never reaches here, so it never reaches the wire.
    std::memcpy(out.data(), ctx.outputs_.data(), out.size());
}

void RtCore::rt_body(const std::stop_token& st) noexcept {
    (void)st;
    // realtime setup (mlockall/mallopt/prefault/SCHED_FIFO). If SCHED setup fails and
    // require_realtime is set, latch an abort and skip bring-up.
    if (!realtime::setup(cfg_.rt_priority) && cfg_.require_realtime) {
        latch_reason(StopReason::RtSetupFailed);
        for (Attached& a : controls_) {
            a.control->on_stop(StopReason::RtSetupFailed);
        }
        phase_.store(RunnerPhase::Stopped, std::memory_order_release);
        return;
    }

    const bool dc = master_.dc_enabled();
    const std::uint64_t period_ns = 1'000'000'000ULL / master_.loop_rate_hz();
    // The single pacer: a Runner local, untouchable by consumers, carried gapless across
    // bring-up, steady, and the stopping window. The regime is selected by the input:
    // pace(dc ? dc_time : 0).
    realtime::DcPacer pacer(period_ns);  // mid-cycle phase target (the default)

    // --- BRING-UP: pace plus bringup_step (OR-ing the controls' sync_faulted), bounded by
    // bringup_timeout. Exactly one OP request per start(): bringup_step owns the single
    // request, and every abort path below exits without re-entering bring-up. A restart is
    // the consumer's call via a fresh Runner.
    const std::uint64_t give_up_at = realtime::monotonic_ns() + static_cast<std::uint64_t>(cfg_.bringup_timeout.count()) * 1'000'000ULL;
    bool operational = false;
    while (!stop_flag_.load(std::memory_order_acquire)) {
        bool any_sync_fault = false;
        bool all_present = true;                                    // every control's drive feedback must look alive to confirm OP
        const std::int64_t bring_dct = dc ? master_.dc_time() : 0;  // ctx contract: 0 when DC off
        for (Attached& a : controls_) {
            dispatch(a, 0, bring_dct, false, [&](CycleContext& ctx) {
                const CycleContext& cctx = static_cast<const CycleContext&>(ctx);
                any_sync_fault = any_sync_fault || a.control->sync_faulted(cctx);
                all_present = all_present && a.control->drive_present(cctx);
            });
        }
        const BringupStatus bs = master_.bringup_step(any_sync_fault, all_present);
        pacer.pace(dc ? master_.dc_time() : 0);
        if (bs == BringupStatus::Operational) {
            operational = true;
            break;
        }
        if (bs == BringupStatus::Aborted || realtime::monotonic_ns() >= give_up_at) {
            latch_reason(StopReason::BringupAborted);
            break;
        }
    }
    if (!operational) {
        // Aborted bring-up or a stop request before OP. No stopping window: the drive was
        // never enabled, so there is no consumer disable policy to give time to; stop() runs
        // the close-to-INIT teardown.
        if (reason_.load(std::memory_order_relaxed) == StopReason::None) {
            latch_reason(StopReason::Requested);  // stop request during bring-up
        }
        const StopReason r = reason_.load(std::memory_order_relaxed);
        for (Attached& a : controls_) {
            a.control->on_stop(r);
        }
        phase_.store(RunnerPhase::Stopped, std::memory_order_release);
        return;
    }

    // --- OPERATIONAL: hooks first (before any step()), then steady.
    phase_.store(RunnerPhase::Running, std::memory_order_release);
    const std::int64_t op_dct = dc ? master_.dc_time() : 0;
    for (Attached& a : controls_) {
        dispatch(a, 0, op_dct, false, [&](CycleContext& ctx) { a.control->on_operational(ctx); });
    }

    // --- STEADY plus the stopping window, one loop: process(), latch, step (in attach/slave
    // order), pace. A stop cause (Master's WKC fault latch, request_stop, or stop()) triggers
    // on_stop(reason) once, then teardown_cycles more cycles run with ctx.stopping() == true
    // so the control's disable policy ships with process data still flowing. A fault during
    // the window does not cut it short: in the partial-fault case the disable may still reach
    // the drive; in the dead-bus case the cost is at most a window of no-reply frames.
    std::uint64_t cycle = 0;
    bool stopping = false;
    std::uint32_t window_left = 0;
    // RT-overrun instrument: pace() returns how many whole periods it had to skip to catch up
    // (0 is healthy).
    constexpr std::uint64_t kRtOverrunReportNs = 5'000'000;  // >=5ms starvation is abnormal at any sane rate
    bool rt_overrun_logged = false;
    std::uint32_t rt_overrun_worst = 0;
    for (;;) {
        master_.process();
        // ONE selection feeds BOTH the ctx contract (dc_time_ns()==0 when DC is off)
        // and the pacing input (pace(0) = pure period) -- the §4 regime switch.
        const std::int64_t dct = dc ? master_.dc_time() : 0;
        if (!stopping) {
            if (master_.fault()) {
                latch_reason(StopReason::BusFault);  // steady health is the WKC latch (no AL polling here)
                // Emit a one-shot line naming the fault. master_.last_error() is lock-free (it
                // reads the fault atomics), and this fires exactly once, since the next cycle
                // takes the stopping path and skips this block.
                (void)std::fprintf(stderr, "[ethercat] BUS FAULT -- %s. Entering teardown.\n", master_.last_error().c_str());
                (void)std::fflush(stderr);
                stop_flag_.store(true, std::memory_order_release);
            }
            if (stop_flag_.load(std::memory_order_acquire)) {
                stopping = true;
                window_left = cfg_.teardown_cycles;
                phase_.store(RunnerPhase::Stopping, std::memory_order_release);
                const StopReason r = reason_.load(std::memory_order_relaxed);
                for (Attached& a : controls_) {
                    a.control->on_stop(r);
                }
            }
            // No SDO servicing here: do_command SDOs run on the caller's non-RT thread (the
            // SOEM v2 port is thread-safe), so the RT loop never touches the mailbox.
        }
        for (Attached& a : controls_) {
            dispatch(a, cycle, dct, stopping, [&](CycleContext& ctx) { a.control->step(ctx); });
        }
        const std::uint32_t skipped = pacer.pace(dct);
        if (skipped > rt_overrun_worst) {
            rt_overrun_worst = skipped;
        }
        if (static_cast<std::uint64_t>(skipped) * period_ns >= kRtOverrunReportNs && !rt_overrun_logged) {
            rt_overrun_logged = true;  // one-shot -- a fault/teardown typically follows within cycles
            (void)std::fprintf(stderr,
                               "[ethercat] RT cycle overrun %.1fms (%u cycles) at cycle %llu -- the SCHED_FIFO RT thread was "
                               "starved (host contention / page fault / priority inversion).\n",
                               static_cast<double>(static_cast<std::uint64_t>(skipped) * period_ns) / 1e6,
                               skipped,
                               static_cast<unsigned long long>(cycle));
            (void)std::fflush(stderr);
        }
        ++cycle;
        if (stopping) {
            // Event-driven early-out: a control doing a controlled ramp-stop signals
            // teardown_complete() once it is de-energized at rest (ramped velocity to 0, then
            // disabled), so the teardown ends as soon as it is safe. teardown_cycles remains
            // the hard cap so a never-completing control (e.g. a dead bus that can't reach
            // SwitchOnDisabled) still exits bounded.
            bool all_torn_down = true;
            for (const Attached& a : controls_) {
                all_torn_down = all_torn_down && a.control->teardown_complete();
            }
            // The entering cycle is the window's first stopping cycle: up to teardown_cycles
            // step() dispatches see ctx.stopping() == true (floor 1).
            --window_left;
            if (all_torn_down || window_left == 0) {
                break;
            }
        }
    }
    phase_.store(RunnerPhase::Stopped, std::memory_order_release);
}

}  // namespace ethercat
