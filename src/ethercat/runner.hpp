#pragma once

// ethercat::Runner + SlaveControl + CycleContext (#47) -- the inversion.
//
// Before #47, consumers owned the RT thread, pacing, bring-up, and teardown and
// called library primitives. After it, THE LIBRARY owns everything from start()
// to close: realtime setup, the (one) DcPacer, the bring-up pump, the steady
// process/step cycle, the stopping window, and master.close(). The consumer
// derives SlaveControl and implements per-cycle POLICY in step(). DC vs free-run
// is a config fact the Runner reads -- consumer step() is identical either way.
//
// LAYERING: Master stays pure bus policy (config/remap/bringup_step/process --
// caller-paced, thread-free, exactly the #31 boundary). The Runner is the
// orchestration layer composing them, and the ONE audited home of the #39
// set_rt_active bracket, the spawn/join, and the teardown sequence.
//
// THE ENFORCEMENT WIN: the once-per-cycle/fixed-point step() cadence is enforced
// BY CONSTRUCTION -- the Runner owns the only call site (contrast #38's
// update()-contract-on-the-caller). Misbehaving steps degrade FAIL-SAFE: a
// blocking/slow step() becomes a pacer overrun -> DcPacer's phase-preserving
// catch-up + WKC/cycle visibility -- observable and recoverable (skipped
// cycles, counters move), never silent corruption, never an off-phase burst.

#include <atomic>
#include <chrono>
#include <cstdint>
#include <thread>
#include <vector>

#include "ethercat/master.hpp"
#include "ethercat/realtime.hpp"

namespace ethercat {

// Why the Runner stopped (latched, first cause wins; readable non-RT via status()).
enum class StopReason : std::uint8_t {
    None,            // not stopped
    Requested,       // ctx.request_stop() / Runner::request_stop() / Runner::stop()
    BusFault,        // Master's consecutive-WKC fault latch fired in steady state
    BringupAborted,  // bring-up gave up (bounded -- the no-hammer invariant; no auto-retry)
    RtSetupFailed,   // SCHED_FIFO unavailable && RunnerConfig::require_realtime
};
const char* to_string(StopReason r) noexcept;

enum class RunnerPhase : std::uint8_t { Idle, BringingUp, Running, Stopping, Stopped };
const char* to_string(RunnerPhase p) noexcept;

// Non-RT snapshot of the Runner's state (atomics; phase/reason individually relaxed).
struct RunnerStatus {
    RunnerPhase phase = RunnerPhase::Idle;
    StopReason reason = StopReason::None;
};

struct RunnerConfig {
    int rt_priority = realtime::kDefaultRtPriority;
    // Throw-from-start() semantics preserved from ServoController: when true and
    // SCHED_FIFO is unavailable, the run aborts (StopReason::RtSetupFailed) instead
    // of running best-effort.
    bool require_realtime = false;
    // Bring-up give-up (wall time). Sits ABOVE Master's own op_await window by default
    // so the Master's verdict (not the pump's) decides; the pump bound is the backstop.
    std::chrono::milliseconds bringup_timeout{120'000};
    // The stopping WINDOW (#47 §5): after on_stop(), this many MORE steady cycles run
    // with ctx.stopping()==true so the control's POLICY can disable its drive with PD
    // still flowing (e.g. CiA402 cw->0x00). Floor 1. Safety does NOT depend on it: a
    // window-ignoring control still ends at master.close()'s proven INIT teardown.
    std::uint32_t teardown_cycles = 100;  // ~100 ms @ 1 kHz
};

class Runner;

// The per-cycle, per-slave RT surface handed to SlaveControl -- the #30 RT FORM ONLY.
// Rpdo/Tpdo (the throwing copy forms) are deliberately NOT here: they resolve
// per-call and throw, which the step() contract bans. Non-RT consumers keep the
// copy forms (read_rpdo / their own atomics) OUTSIDE step(), as before.
// No Master&, no SDO, no map access, no raw image pointers -- load/store at
// pre-resolved FieldLocation handles is the whole hot-path surface.
class CycleContext {
   public:
    // Read a typed field from THIS cycle's latched input image (the feedback the
    // cycle's process() just exchanged; stable for the whole step()).
    template <PdoScalar T>
    T load(FieldLocation loc) const noexcept {
        check_live();
        return load_le<T>(inputs_, loc);
    }
    // Stage a typed field into the output image. SHIPS WITH CYCLE N+1's process()
    // -- the same 1-cycle command latency P2b verified on hardware. Step authors:
    // a store this cycle is on the wire NEXT cycle.
    template <PdoScalar T>
    void store(FieldLocation loc, T v) noexcept {
        check_live();
        store_le<T>(outputs_, loc, v);
    }
    // Steady-cycle counter: 0 at the first step() after Operational.
    std::uint64_t cycle() const noexcept {
        check_live();
        return cycle_;
    }
    // DC system time of this cycle's exchange; 0 when DC is disabled (the pacing
    // regimes differ ONLY here -- consumer step() code is identical).
    std::int64_t dc_time_ns() const noexcept {
        check_live();
        return dc_time_;
    }
    // True during the stopping window (§5): run the drive-disable policy now.
    bool stopping() const noexcept {
        check_live();
        return stopping_;
    }
    // The RT-side error/stop channel: latches StopReason::Requested and enters the
    // stopping window. NO exceptions cross the RT boundary -- this is the way out.
    void request_stop() noexcept;
    // The #40 WKC health counters (fields individually relaxed, +/-1 skew by design).
    WkcStats wkc() const noexcept;

   private:
    friend class Runner;
    explicit CycleContext(Runner* runner) noexcept : runner_(runner) {}
    // Contract check (#47 §3b, "runtime-checked"): ctx is valid ONLY during its own
    // dispatch window (the Runner marks live around each hook/step call). A control
    // that caches the ctx pointer and uses it outside its cycle is a contract
    // violation: counted ALWAYS (Runner::contract_violations) + asserted in debug.
    // (Our release builds define NDEBUG, so the counter is the testable signal.)
    void check_live() const noexcept;

    Runner* runner_;
    std::span<const std::byte> inputs_;  // refreshed each cycle (the latched feedback)
    std::span<std::byte> outputs_;       // the live command image (ships next cycle)
    std::uint64_t cycle_ = 0;
    std::int64_t dc_time_ = 0;
    bool stopping_ = false;
    bool live_ = false;  // set by the Runner around dispatch only
};

// The consumer interface: per-slave POLICY. Derive it; the library runs everything.
class SlaveControl {
   public:
    SlaveControl() = default;
    SlaveControl(const SlaveControl&) = default;
    SlaveControl& operator=(const SlaveControl&) = default;
    SlaveControl(SlaveControl&&) = default;
    SlaveControl& operator=(SlaveControl&&) = default;
    virtual ~SlaveControl() = default;

    // NON-RT, pre-spawn, the ONLY hook that may THROW: resolve typed fields
    // (resolve_rx/resolve_tx<F> -- the width assert fires here, at configure time),
    // run one-time SDOs (e.g. a 0x605A regime readback via master.sdo_read -- the
    // single-port-owner phase), validate config. A throw aborts start() cleanly:
    // nothing spawned, no bracket set, master untouched.
    virtual void on_configured(Master& master, std::uint16_t slave_id) {
        (void)master;
        (void)slave_id;
    }
    // RT, once, the first cycle AT Operational -- before the first step().
    virtual void on_operational(CycleContext& ctx) noexcept {
        (void)ctx;
    }
    // RT, once, on entering the stopping path (any cause; the reason says which).
    // The stopping WINDOW follows (steady cycles with ctx.stopping()==true) -- put
    // the drive-disable policy in step()-during-stopping, not here.
    virtual void on_stop(StopReason reason) noexcept {
        (void)reason;
    }
    // RT, every steady cycle, at the fixed point: inputs latched -> step() ->
    // outputs ship with the next exchange. Bounded, no alloc, no blocking, noexcept.
    // Never called before Operational. (Cadence is enforced by construction --
    // the Runner owns the only call site.)
    virtual void step(CycleContext& ctx) noexcept = 0;
    // RT, every BRING-UP cycle: "is the drive reporting a sync fault?" -- the
    // no-sync-fault-class gate signal for bringup_step. DEFAULT false; the
    // device-aware control implements it (e.g. a mapped error-code field equals
    // its drive's no-sync code). This hook is what keeps Master AND Runner free
    // of vendor AND CiA402 knowledge (the layering audit's rule, preserved).
    virtual bool sync_faulted(const CycleContext& ctx) const noexcept {
        (void)ctx;
        return false;
    }
};

// The orchestration layer (#47 §1): owns the RT thread + everything around it.
// Master must be open()+init()+configure()d (the consumer's throwing config phase)
// before start(). One Runner per Master; not copyable/movable.
class Runner {
   public:
    Runner(Master& master, RunnerConfig cfg) noexcept;
    Runner(const Runner&) = delete;
    Runner& operator=(const Runner&) = delete;
    Runner(Runner&&) = delete;
    Runner& operator=(Runner&&) = delete;
    ~Runner();  // stop()

    // Attach a control to a slave (1-based). PRE-start only. Throws ConfigError on
    // attach-after-start, an unknown slave id, or a duplicate attach for the slave.
    void attach(std::uint16_t slave_id, SlaveControl& control);

    // Non-RT hooks (on_configured, may throw -> nothing spawned) -> lock_current ->
    // set_rt_active(true) -> spawn the RT thread (realtime::setup -> bring-up ->
    // steady -> stopping window, per §5). Throws ConfigError on no-controls/restart.
    void start();
    // Graceful stop: request (latches Requested if nothing latched yet) -> the RT
    // loop runs its stopping window and exits -> join -> set_rt_active(false) ->
    // master.close() (the proven INIT teardown). Idempotent; safe when never started.
    void stop() noexcept;
    // Convenience (the a6_validate / #21 shape): start() + block until the RT loop
    // ends (poll status()), then the stop() teardown. SIGINT integration = the
    // consumer's handler calling request_stop().
    void run();

    // Non-RT stop request (e.g. from a signal-handler-adjacent thread): latches
    // Requested; the RT loop enters its stopping window on the next cycle.
    void request_stop() noexcept;

    RunnerStatus status() const noexcept {
        return RunnerStatus{phase_.load(std::memory_order_relaxed), reason_.load(std::memory_order_relaxed)};
    }

    // #47 §3b contract-violation counter (ctx-use-outside-cycle, RT-thread re-entry
    // into stop()): counted ALWAYS (our release builds define NDEBUG, so the debug
    // asserts alone would be invisible there); asserted in debug builds too. Test 10
    // proves the checks non-vacuous against a deliberately-misbehaving control.
    std::uint32_t contract_violations() const noexcept {
        return violations_.load(std::memory_order_relaxed);
    }

   private:
    friend class CycleContext;

    struct Attached {
        std::uint16_t slave_id;
        SlaveControl* control;
        CycleContext ctx;
    };

    void rt_body(const std::stop_token& st) noexcept;
    void latch_reason(StopReason r) noexcept;  // first cause wins (CAS from None)
    void note_violation() noexcept;
    // Refresh a ctx for this cycle + dispatch one hook/step with the live window set.
    template <class Fn>
    void dispatch(Attached& a, std::uint64_t cycle, std::int64_t dc, bool stopping, Fn&& fn) noexcept;

    Master& master_;
    RunnerConfig cfg_;
    std::vector<Attached> controls_;  // attach/slave order = step order
    std::atomic<RunnerPhase> phase_{RunnerPhase::Idle};
    std::atomic<StopReason> reason_{StopReason::None};
    std::atomic<bool> stop_flag_{false};
    std::atomic<std::uint32_t> violations_{0};
    std::atomic<bool> started_{false};
    std::thread::id rt_tid_{};  // set at spawn; the stop()-from-RT re-entrancy check
    std::jthread rt_;           // last member
};

}  // namespace ethercat
