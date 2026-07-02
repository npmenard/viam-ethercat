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
    // Legal from inside step()/hooks (the RT error channel) AND harmless if the
    // control cached the ctx -- it only latches an atomic. Routed through core_ (the
    // RT-shared state the thread itself lives in), reachable for the whole RT lifetime.
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
    // Debug-only (#47 TODO-1): a control that touches the ctx outside its dispatch
    // window gets a loud, immediate failure in debug builds. In release this compiles
    // to nothing -- and per the owned-data design (inputs_/outputs_ are by-value, not
    // spans into live buffers) the worst case there is SAFE-STALE: a valid object with
    // last-cycle data, never a dangling/in-flight read. There is no release-mode
    // counter (the old contract_violations was dropped: owned data makes the escape
    // harmless rather than merely counted).
    assert(live_ && "CycleContext used outside its dispatch window (#47 TODO-1: valid on the RT thread, during dispatch only)");
}

// --- RtCore (the RT-thread-shared cyclic state; #TODO-3 / #52) ---------------

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
    // The RT state is heap-held from construction so its address is STABLE for the
    // thread to capture and so a wedge can LEAK it intact (#52). OOM here terminates
    // (noexcept) -- a Runner you cannot allocate is unrecoverable regardless.
    rt_core_ = std::make_unique<RtCore>(master, cfg);
}

Runner::~Runner() {
    stop();
}

void Runner::attach(std::uint16_t slave_id, SlaveControl& control) {
    if (started_.load(std::memory_order_acquire)) {
        throw ConfigError("Runner::attach: controls must be attached before start()");
    }
    if (slave_id == 0 || slave_id > master_.slave_count()) {
        throw ConfigError("Runner::attach: slave " + std::to_string(slave_id) + " out of range (bus has " +
                          std::to_string(master_.slave_count()) + ")");
    }
    for (const RtCore::Attached& a : rt_core_->controls_) {
        if (a.slave_id == slave_id) {
            throw ConfigError("Runner::attach: slave " + std::to_string(slave_id) + " already has a control attached");
        }
    }
    rt_core_->controls_.emplace_back(slave_id, &control, rt_core_.get());  // ctx built in place (non-movable, #47 TODO-1)
}

void Runner::start() {
    if (started_.load(std::memory_order_acquire)) {
        throw ConfigError("Runner::start: already started (one start() per Runner; restart = a fresh Runner)");
    }
    if (rt_core_->controls_.empty()) {
        throw ConfigError("Runner::start: no controls attached");
    }
    // NON-RT hooks first -- the ONLY throwing phase. A throw here aborts start()
    // cleanly: nothing locked, no thread, no rt_active bracket, master untouched.
    // on_configured gets the RESTRICTED ConfigContext (§3a, TODO-10), never a raw Master&.
    for (RtCore::Attached& a : rt_core_->controls_) {
        ConfigContext cfg{master_, a.slave_id};
        a.control->on_configured(cfg);
    }
    realtime::lock_current();
    started_.store(true, std::memory_order_release);
    rt_core_->phase_.store(RunnerPhase::BringingUp, std::memory_order_relaxed);
    // #39 bracket: declared active EXACTLY across the RT thread's lifetime; cleared
    // after the join in stop() (the one audited site post-#47).
    master_.set_rt_active(true);
    // The thread captures the RtCore* (NOT `this`) -- so it never reaches a Runner
    // member, and a leaked RtCore (#52 wedge) carries everything the thread needs.
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
    // PRIVATE (#TODO-3): only ~Runner + run() reach here, both owner-thread -- so this
    // never runs on the RT thread and the old self-join guard is gone by construction.
    core.request_stop();  // latch Requested (first-cause) + set the flag

    // BOUNDED wait (#TODO-3 H1): wait for the RT loop to finish its stopping window and
    // mark Stopped, up to a derived/configured ceiling. A non-wedged teardown reaches
    // Stopped well inside it; a WEDGED step() never does.
    std::chrono::nanoseconds bound = core.cfg_.stop_join_timeout;
    if (bound <= std::chrono::nanoseconds::zero()) {
        const std::uint64_t period_ns = 1'000'000'000ULL / master_.loop_rate_hz();
        const auto derived = std::chrono::nanoseconds((static_cast<std::uint64_t>(core.cfg_.teardown_cycles) + 20U) * period_ns * 4U);
        bound = std::max<std::chrono::nanoseconds>(std::chrono::milliseconds(250), derived);
    }
    const auto deadline = std::chrono::steady_clock::now() + bound;
    while (core.phase_.load(std::memory_order_acquire) != RunnerPhase::Stopped) {
        if (std::chrono::steady_clock::now() >= deadline) {
            // WEDGED -> FAIL-STOP (#52). The RT thread is stuck INSIDE control->step() and
            // will never return; it holds the ctx (Runner-internal, leakable) AND the
            // EXTERNALLY-owned control (a reference from attach() -- NOT ours to leak). If
            // we returned, the orderly dtor chain (~ServoController -> ~Runner -> destroy
            // the control) would free state the abandoned thread is actively using ->
            // use-after-free, possibly heap-corrupting the host process. There is no safe
            // way to reclaim a thread parked in foreign code, and we cannot keep the
            // consumer's control alive past its owner. So we do NOT tear down: abort the
            // process. The drive is already SAFE (PD gapped upstream of the wedge -> the SM
            // watchdog de-energizes ~50ms); the supervisor (viam-server) restarts the
            // module. Deterministic fail-stop beats undefined behavior; this is a
            // genuinely unrecoverable RT fault. (H1 LIVENESS is still met -- the operator
            // is not stuck: the process restarts -- and no rogue thread corrupts a
            // torn-down heap.)
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
    master_.set_rt_active(false);  // joined -- single port owner again
    master_.close();               // the proven INIT teardown (idempotent at the backend)
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

    // COPY-IN (#47 TODO-1): refresh the ctx's OWNED input from this cycle's latched
    // feedback. Applies to EVERY ctx-touching hook -- incl. sync_faulted() during
    // bring-up, which loads FaultCode and must see the refreshed input.
    ctx.input_size_ = in.size();
    std::memcpy(ctx.inputs_.data(), in.data(), in.size());
    // SEED the OWNED output from the live command image, so a field the hook does NOT
    // store carries its current wire value over (exactly the old direct-span semantic;
    // a read-only hook then copies back a no-op). Owned buffers also persist across
    // cycles, but seeding makes a read-only or partial-write hook behavior-identical to
    // the pre-TODO-1 span that wrote through to the live image.
    ctx.output_size_ = out.size();
    std::memcpy(ctx.outputs_.data(), out.data(), out.size());

    ctx.cycle_ = cycle;
    ctx.dc_time_ = dc;
    ctx.stopping_ = stopping;
    ctx.live_ = true;  // the dispatch window (#47 TODO-1): ctx is legal ONLY in here
    fn(ctx);
    ctx.live_ = false;

    // COPY-OUT: the owned output goes to the wire; it ships with the NEXT process().
    // A store made through an escaped (stale) handle after this point lands in the
    // owned buffer only and never reaches here -> never reaches the wire.
    std::memcpy(out.data(), ctx.outputs_.data(), out.size());
}

void RtCore::rt_body(const std::stop_token& st) noexcept {
    (void)st;
    // realtime setup (mlockall/mallopt/prefault/SCHED_FIFO). require_realtime
    // semantics preserved: SCHED failure + require -> latched abort, no bring-up.
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
    // THE one pacer (#47 §4): a Runner local -- structurally untouchable by
    // consumers; carried gapless across bring-up -> steady -> the stopping window.
    // Strategy selection is the INPUT: pace(dc ? dc_time : 0).
    realtime::DcPacer pacer(period_ns);  // mid-cycle phase target (the #40 default)

    // --- BRING-UP (the run_to_operational internals, inlined so this pacer is THE
    // pacer): pace + bringup_step(OR over the controls' sync_faulted), bounded by
    // bringup_timeout. EXACTLY ONE OP request per start() -- bringup_step owns the
    // single request; every abort path below exits WITHOUT re-entering bring-up
    // (the no-hammer invariant; restart = the consumer's call via a fresh Runner).
    const std::uint64_t give_up_at = realtime::monotonic_ns() + static_cast<std::uint64_t>(cfg_.bringup_timeout.count()) * 1'000'000ULL;
    bool operational = false;
    while (!stop_flag_.load(std::memory_order_acquire)) {
        bool any_sync_fault = false;
        const std::int64_t bring_dct = dc ? master_.dc_time() : 0;  // ctx contract: 0 when DC off
        for (Attached& a : controls_) {
            dispatch(a, 0, bring_dct, false, [&](CycleContext& ctx) {
                any_sync_fault = any_sync_fault || a.control->sync_faulted(static_cast<const CycleContext&>(ctx));
            });
        }
        const BringupStatus bs = master_.bringup_step(any_sync_fault);
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
        // Aborted bring-up or a stop request before OP. No stopping window: the drive
        // was never enabled, so there is no consumer disable-policy to give time to
        // (§5b BringingUp rows); stop() runs the close-to-INIT teardown.
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

    // --- STEADY + the stopping window, one loop (#47 §5): process() -> latch ->
    // step (attach/slave order) -> pace. Stop causes (Master's WKC fault latch |
    // request_stop | stop()) trigger on_stop(reason) ONCE, then `teardown_cycles`
    // MORE cycles run with ctx.stopping()==true so the control's disable policy
    // ships with PD still flowing. A fault DURING the window does NOT cut it short
    // (settled at the gate): in the partial-fault case the disable may still reach
    // the drive; in the dead-bus case the cost is <=window of no-reply frames.
    std::uint64_t cycle = 0;
    bool stopping = false;
    std::uint32_t window_left = 0;
    for (;;) {
        master_.process();
        // ONE selection feeds BOTH the ctx contract (dc_time_ns()==0 when DC is off)
        // and the pacing input (pace(0) = pure period) -- the §4 regime switch.
        const std::int64_t dct = dc ? master_.dc_time() : 0;
        if (!stopping) {
            if (master_.fault()) {
                latch_reason(StopReason::BusFault);  // steady health = the WKC latch (no AL polling here)
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
            // #22 steady-state SDO: the Runner is the single port owner, so it is the ONLY
            // place a marshaled SDO transfer may run. Service AT MOST ONE pending request per
            // steady cycle, right after process()/publish. No-op (one relaxed atomic load)
            // when none is pending. NOT during the stopping window -- the controlled-stop ramp
            // has its own tight de-energize deadline that a mailbox round-trip must not eat.
            master_.service_sdo();
        }
        for (Attached& a : controls_) {
            dispatch(a, cycle, dct, stopping, [&](CycleContext& ctx) { a.control->step(ctx); });
        }
        pacer.pace(dct);
        ++cycle;
        if (stopping) {
            // EVENT-DRIVEN early-out (#47-P3b R1): a control doing a CONTROLLED ramp-stop signals
            // teardown_complete() once it is de-energized AT REST (ramped vel->0 THEN disabled), so
            // the teardown ends as soon as it is SAFE -- the common already-stopped case doesn't pay
            // the full generous window, and a moving stop still ramps fully (complete stays false
            // until rest). `teardown_cycles` remains the hard CAP so a never-completing control
            // (e.g. a dead bus that can't reach SwitchOnDisabled) still exits bounded.
            bool all_torn_down = true;
            for (const Attached& a : controls_) {
                all_torn_down = all_torn_down && a.control->teardown_complete();
            }
            // The entering cycle is the window's FIRST stopping cycle: exactly up to
            // `teardown_cycles` step() dispatches see ctx.stopping()==true (floor 1).
            --window_left;
            if (all_torn_down || window_left == 0) {
                break;
            }
        }
    }
    phase_.store(RunnerPhase::Stopped, std::memory_order_release);
}

}  // namespace ethercat
