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

#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <span>
#include <thread>

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
    Wedged,          // teardown (#TODO-3 H1): a wedged step() did not reach Stopped within the
                     // bounded join -> the thread was DETACHED so stop() returns (liveness). The
                     // drive is still SAFE (PD gapped upstream of the wedge -> SM watchdog fires);
                     // the MODULE is wedged. The HW watchdog is the de-energize backstop, never
                     // the in-process join. Surfaced via status() + a loud log.
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
    // The BOUNDED-JOIN ceiling (#TODO-3 H1): the most wall time stop()/~Runner waits for
    // the RT loop to finish its stopping window and exit before declaring it WEDGED and
    // DETACHING it (so teardown returns -- liveness -- instead of hanging on a wedged
    // step()). 0 = derive: max(250ms, (teardown_cycles + 20 slack) x period x 4) -- the
    // x4 covers a dead bus where each receive blocks ~EC_TIMEOUTRET. Must exceed the
    // genuine window wall time, or a healthy slow teardown is misread as wedged.
    std::chrono::milliseconds stop_join_timeout{0};
};

class Runner;
// The heap-held, RT-thread-shared cyclic state (#TODO-3 / #52). The Runner owns it by
// unique_ptr; the RT thread lives INSIDE it (capturing the RtCore*, not the Runner), so
// the clean teardown's join is a clean barrier and a wedge fail-stops rather than tearing
// down under the parked thread. See the class def below.
class RtCore;
// White-box test access to the PRIVATE teardown (#TODO-3): consumers stop by dropping
// the Runner, but the H1 bounded-join / Wedged + H4 concurrency tests must drive stop()
// directly. Defined only in runner_test; never in production. (The API stays private:
// a consumer cannot name stop(); only this declared peer can.)
struct RunnerTestPeer;

// The per-cycle, per-slave RT surface handed to SlaveControl -- the #30 RT FORM ONLY.
// The per-call-resolving, throwing forms (resolve_rx/resolve_tx) are deliberately NOT
// here: they resolve per-call and throw, which the step() contract bans. Non-RT
// consumers keep those (or their own atomics) OUTSIDE step(), as before.
// No Master&, no SDO, no map access, no raw image pointers -- load/store at
// pre-resolved FieldLocation handles is the whole hot-path surface.
//
// OWNED DATA (#47 §3b, TODO-1): the input/output images are held BY VALUE (fixed
// arrays @ kMaxPdoBytes), NOT spans into the Runner's live process buffers.
// dispatch() copies the slave's input image IN before each hook and the owned
// output buffer OUT after. The win: a control that stashes the ctx and touches it
// after step() returns reads a VALID object with STALE data -- never a dangling or
// in-flight read (at 1 kHz a wild read into repointed buffers could command
// dangerous motion). C++ can't forbid the escape (no borrow checker), so we make
// the escape HARMLESS instead. A store through a stale handle lands in the
// already-copied-back owned buffer and never reaches the wire.
//
// CONTRACT: valid on the RT thread, during the dispatch window ONLY. Owned data
// fixes the realistic single-RT-thread escape; it does NOT make the ctx safe to
// SHARE ACROSS THREADS -- a non-RT thread reading the ctx while the RT thread
// copies the next cycle in is still a data race (per-access locking is the only
// fix; out of scope). The debug assert (check_live) steers developers away from
// ALL out-of-window use, the cross-thread case included.
//
// SCOPING CAVEAT (do not over-read "owned data = always safe"): "safe-stale" holds
// only WITHIN THE RtCore'S LIFETIME. The ctx lives in the RtCore's controls_ deque
// (heap-owned by the Runner), so a handle that OUTLIVES the Runner+RtCore (used after a
// clean teardown destroys them) is a use-after-free, not safe-stale. Deterministic
// owner-side teardown/ownership is TODO-3's territory; within a live Runner, escape is
// harmless. (A wedge never destroys the RtCore -- it fail-stops, #52.)
class CycleContext {
   public:
    // Read a typed field from THIS cycle's latched input image (the feedback the
    // cycle's process() just exchanged; stable for the whole step()). Reads the
    // OWNED input copy that dispatch() refreshed before the hook.
    template <PdoScalar T>
    T load(FieldLocation loc) const noexcept {
        check_live();
        // View only this slave's valid Tx prefix, so bounds behave exactly as the
        // old image-sized span did (a loc past the real image is still out-of-range).
        return load_le<T>(std::span<const std::byte>(inputs_).first(input_size_), loc);
    }
    // Stage a typed field into the OWNED output buffer; dispatch() copies it to the
    // wire AFTER the hook, so it SHIPS WITH CYCLE N+1's process() -- the same
    // 1-cycle command latency P2b verified on hardware. Step authors: a store this
    // cycle is on the wire NEXT cycle.
    template <PdoScalar T>
    void store(FieldLocation loc, T v) noexcept {
        check_live();
        store_le<T>(std::span<std::byte>(outputs_).first(output_size_), loc, v);
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
    // The Master's LATCHED bus-fault state (consecutive-WKC-error latch). Symmetric with
    // wkc() -- a consumer step() that drives its own per-cycle fault policy off the bus
    // tier (e.g. the servo module's two-tier fault) reads it HERE rather than touching the
    // Master directly (the Runner is the sole Master toucher, #47-P3 §2). Same value the
    // Runner itself uses to latch StopReason::BusFault.
    bool fault() const noexcept;

    // Deleted copy AND move (#47 TODO-1): the ctx is a long-lived Runner-owned
    // member, handed out by reference per dispatch. It was IMPLICITLY copyable
    // (only the ctor is private, so the copy ctor was implicitly public --
    // `auto saved = ctx;` compiled). Deleting both makes a stash a compile error,
    // not a silent value-copy that would alias the owned buffers in confusing ways.
    CycleContext(const CycleContext&) = delete;
    CycleContext& operator=(const CycleContext&) = delete;
    CycleContext(CycleContext&&) = delete;
    CycleContext& operator=(CycleContext&&) = delete;

   private:
    friend class RtCore;
    explicit CycleContext(RtCore* core) noexcept : core_(core) {}
    // Contract check (#47 §3b, TODO-1): the ctx is valid ONLY during its own
    // dispatch window (the Runner sets live_ around each hook/step call). A control
    // that caches the ctx and touches it outside its window trips this in DEBUG --
    // a loud, immediate failure pointing at the misuse. In release it compiles to
    // nothing, and per the owned-data design the worst case there is safe-stale
    // (a valid object with last-cycle data), never corruption -- so there is no
    // release-mode counter (the old contract_violations counter is gone: the user
    // found it contrived, and owned data makes the escape harmless rather than
    // merely counted).
    void check_live() const noexcept;

    RtCore* core_;  // the RT-shared state (request_stop/wkc go through it; #52)
    // Owned images (NOT spans into live buffers). dispatch() copies the slave input
    // in before the hook and copies this output out after -- so an escaped ctx reads
    // safe-stale data and an escaped store never reaches the wire.
    std::array<std::byte, kMaxPdoBytes> inputs_{};   // refreshed each cycle (latched feedback)
    std::array<std::byte, kMaxPdoBytes> outputs_{};  // staged command (copied to wire post-hook)
    std::size_t input_size_ = 0;                     // valid prefix of inputs_  (this slave's Tx image)
    std::size_t output_size_ = 0;                    // valid prefix of outputs_ (this slave's Rx image)
    std::uint64_t cycle_ = 0;
    std::int64_t dc_time_ = 0;
    bool stopping_ = false;
    bool live_ = false;  // set by the Runner around dispatch only
};

// The configure-time surface handed to on_configured (#47 §3a, TODO-10): the RESTRICTED,
// pre-spawn analog of CycleContext. Exposes ONLY the legitimate configure-time operations
// -- typed field resolution (the width assert fires here) + one-time SDOs -- bound to this
// control's slave. It NEVER exposes process()/cyclic PD or a raw Master&.
//
// WHY (the H1 structural close, DA's seam): a raw Master& would let a control stash it and
// call process() from a wedged step(), keeping PD flowing so the SM watchdog never fires ->
// the cycling-wedge (energized-forever) becomes REACHABLE. With no process() reachable from
// ANY hook (ctx or this), "a wedged step() is watchdog-safe" is STRUCTURAL, not contractual.
// The SDO seam does not re-open the door: a stashed ConfigContext used to sdo_write from
// step() hits the #39 rt_active guard -> THROWS through the noexcept step -> terminate
// (loud, fails CLOSED) -- never a silent watchdog-defeat. Stashing it gains a control nothing.
class ConfigContext {
   public:
    // Resolve a typed Rx/Tx field to a pre-resolved FieldLocation handle (the width assert
    // vs the mapped bit_length fires HERE, at configure). Same as Master::resolve_rx/tx<F>,
    // bound to this slave -- the control never names the slave id or touches the Master.
    template <class F>
    FieldLocation resolve_rx() const {
        return master_.resolve_rx<F>(slave_id_);
    }
    template <class F>
    FieldLocation resolve_tx() const {
        return master_.resolve_tx<F>(slave_id_);
    }
    // OPTIONAL resolve: UNMAPPED FieldLocation (mapped()==false) when the object is absent from
    // the map, instead of throwing (a generic consumer maps some fields only in some modes). A
    // mapped-but-wrong-width object still throws. Guard the per-cycle load/store on mapped().
    template <class F>
    FieldLocation resolve_rx_optional() const {
        return master_.resolve_rx_optional<F>(slave_id_);
    }
    template <class F>
    FieldLocation resolve_tx_optional() const {
        return master_.resolve_tx_optional<F>(slave_id_);
    }
    // One-time SDOs while the consumer is the single port owner (pre-RT). e.g. a regime
    // readback or a vendor fault-reset. Post-spawn (a stashed handle), Master's rt_active
    // guard makes these THROW.
    void sdo_write(std::uint16_t index, std::uint8_t sub, std::span<const std::byte> data) {
        master_.sdo_write(slave_id_, index, sub, data);
    }
    std::size_t sdo_read(std::uint16_t index, std::uint8_t sub, std::span<std::byte> out) {
        return master_.sdo_read(slave_id_, index, sub, out);
    }
    std::uint16_t slave_id() const noexcept {
        return slave_id_;
    }

   private:
    friend class Runner;
    ConfigContext(Master& master, std::uint16_t slave_id) noexcept : master_(master), slave_id_(slave_id) {}
    Master& master_;
    std::uint16_t slave_id_;
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
    // (cfg.resolve_rx/resolve_tx<F> -- the width assert fires here, at configure time),
    // run one-time SDOs (e.g. a 0x605A regime readback via cfg.sdo_read -- the
    // single-port-owner phase), validate config. A throw aborts start() cleanly:
    // nothing spawned, no bracket set, master untouched. Receives the RESTRICTED
    // ConfigContext (§3a), NOT a raw Master& -- so no process() is reachable from any hook.
    virtual void on_configured(ConfigContext& cfg) {
        (void)cfg;
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
    // RT, every BRING-UP cycle: "does the drive's feedback look PLAUSIBLE / alive?" -- an ADDITIONAL
    // OP-confirm gate for bringup_step, beyond the working counter. WIRE-PROVEN (#71/#25): a DC-only
    // A6 requested into OP under free-run does NOT visibly refuse -- it sits at SAFE-OP (AL 0x0027)
    // yet contributes a FULL working counter while ZOMBIE-PDOing (statusword stuck 0x0, dead inputs).
    // WKC-alone then wrongly declares OP and the enable ladder spins forever on a dead drive. DEFAULT
    // true (WKC-only, the old behavior); the device-aware control implements it (e.g. statusword != 0).
    // A control that returns false through the whole OP-await window makes bring-up GIVE UP
    // (BringupAborted -> the AL-status diagnostic names AL 0x0027) instead of enabling a dead drive.
    virtual bool drive_present(const CycleContext& ctx) const noexcept {
        (void)ctx;
        return true;
    }
    // RT, every STOPPING cycle (after step()): "is this control safely torn down -- de-energized
    // AT REST -- so the Runner may end the teardown window EARLY?" The window (teardown_cycles) is
    // the CAP; this is the event-driven early-out. DEFAULT false -> run the full window (unchanged
    // for a control that doesn't opt in). A control doing a CONTROLLED ramp-stop (ramp vel->0 THEN
    // de-energize) returns true once it reaches rest+disabled, so the common already-stopped case
    // doesn't pay the full generous window AND a moving stop still ramps fully before close().
    virtual bool teardown_complete() const noexcept {
        return false;
    }
};

// The RT-thread-shared cyclic state (#TODO-3 / #52). EVERYTHING the RT thread touches that
// the Runner OWNS lives HERE, on the heap, owned by the Runner via unique_ptr -- and the
// thread itself lives here too (thread_). The RT loop captures the RtCore* (NOT the Runner),
// so it never reaches a Runner member; on a CLEAN stop the Runner joins thread_ (the barrier)
// then destroys this normally -- no member is touched after the join. (On a WEDGE the Runner
// does NOT tear down at all: it FAIL-STOPS via std::abort(), #52 -- the RT thread is parked
// inside the EXTERNALLY-owned control's step(), which RtCore cannot leak, so the only
// UAF-free option is to not free anything. See Runner::stop().) Not copyable/movable.
class RtCore {
   public:
    RtCore(Master& master, RunnerConfig cfg) noexcept : master_(master), cfg_(cfg) {}
    RtCore(const RtCore&) = delete;
    RtCore& operator=(const RtCore&) = delete;
    RtCore(RtCore&&) = delete;
    RtCore& operator=(RtCore&&) = delete;

    void request_stop() noexcept;              // latch Requested + set the flag (RT or owner)
    void latch_reason(StopReason r) noexcept;  // first cause wins (CAS from None)
    void rt_body(const std::stop_token& st) noexcept;

    // Holds a control + its owned-data CycleContext. The ctx is non-copyable AND
    // non-movable (#47 TODO-1), so Attached is too -- hence controls_ is a std::deque
    // (node-based: stable addresses) populated by in-place emplace_back. The ctor builds
    // the ctx in place from the RtCore* (CycleContext's private ctor; Attached, a member
    // of RtCore which is its friend, may call it).
    struct Attached {
        Attached(std::uint16_t id, SlaveControl* c, RtCore* core) noexcept : slave_id(id), control(c), ctx(core) {}
        std::uint16_t slave_id;
        SlaveControl* control;
        CycleContext ctx;
    };

    // Refresh a ctx for this cycle (copy the slave input IN), dispatch one hook/step with
    // the live window set, then copy the ctx output OUT to the wire (#47 TODO-1).
    template <class Fn>
    void dispatch(Attached& a, std::uint64_t cycle, std::int64_t dc, bool stopping, Fn&& fn) noexcept;

    friend class CycleContext;  // request_stop()/wkc() reach master_/stop_flag_ through core_
    friend class Runner;        // the owner drives attach/start/stop on this

    Master& master_;
    RunnerConfig cfg_;
    std::deque<Attached> controls_;  // attach/slave order = step order
    std::atomic<RunnerPhase> phase_{RunnerPhase::Idle};
    std::atomic<StopReason> reason_{StopReason::None};
    std::atomic<bool> stop_flag_{false};
    std::jthread thread_;  // spawned by Runner::start(); joined on clean stop; a wedge fail-stops the process
};

// The orchestration layer (#47 §1): owns the RT state (RtCore) + the teardown around it.
// Master must be open()+init()+configure()d (the consumer's throwing config phase) before
// start(). One Runner per Master; not copyable/movable.
class Runner {
   public:
    Runner(Master& master, RunnerConfig cfg) noexcept;
    Runner(const Runner&) = delete;
    Runner& operator=(const Runner&) = delete;
    Runner(Runner&&) = delete;
    Runner& operator=(Runner&&) = delete;
    // RAII teardown (#TODO-3): the destructor IS the teardown -- it runs the graceful stop
    // (request -> bounded-join the stopping window -> set_rt_active(false) -> master.close()),
    // so de-energize-on-destruction is STRUCTURAL on every non-wedged path. The OWNER stops
    // deterministically by DROPPING the Runner. There is NO public stop(); the only two stop
    // paths are ctx.request_stop() (RT) and destruction. A wedged step() can't hang it -- the
    // wait is bounded (H1: on timeout the process FAIL-STOPS via std::abort(), #52 -- a wedged
    // RT thread parked in foreign step() is unrecoverable, so we abort rather than tear down
    // state it still holds; the supervisor restarts the module).
    ~Runner();

    // Attach a control to a slave (1-based). PRE-start only. Throws Error on
    // attach-after-start, an unknown slave id, or a duplicate attach for the slave.
    // LIFETIME CONTRACT (#TODO-3): the control is held by reference and the RT thread
    // calls control->step() until teardown JOINS that thread (in ~Runner). So the control
    // MUST OUTLIVE the Runner -- declare/own it BEFORE the Runner (the dtor joins first,
    // the control dies after). A control destroyed while the thread still runs is a UAF.
    // (ServoController owns control + Runner as members in that order; a6_validate declares
    // the control before the Runner.)
    void attach(std::uint16_t slave_id, SlaveControl& control);

    // Non-RT hooks (on_configured(ConfigContext&), may throw -> nothing spawned) ->
    // lock_current -> set_rt_active(true) -> spawn the RT thread INSIDE the RtCore
    // (realtime::setup -> bring-up -> steady -> stopping window, per §5). Throws
    // Error on no-controls/restart.
    void start();
    // Convenience (the a6_validate / #21 shape): start() + block until the RT loop ends
    // (poll status()), then the teardown. SIGINT integration = the consumer's handler
    // calling request_stop(). (For a custom poll loop, call start() then DROP the Runner to
    // stop -- the dtor teardown -- since stop() is not public.)
    void run();

    // Non-RT stop REQUEST (a signal-handler-adjacent thread, or the SDK's Stoppable::stop()
    // on the gRPC thread): latches Requested + sets the flag; the RT loop enters its stopping
    // window next cycle. Does NOT join -- the join is the dtor's job. Pairs with
    // status()-polling for command-and-confirm.
    void request_stop() noexcept;

    // RELAXED diagnostic snapshot, NOT a happens-before edge: polling status().phase ==
    // Stopped does NOT synchronize-with the RT thread's writes. To read control-published
    // state, use the CONTROL's own atomics (or, in a test, the dtor JOIN as the edge) --
    // never this poll. (StopReason::Wedged is never OBSERVED here -- a wedge fail-stops the
    // process inside stop(), #52, so there is no surviving Runner to report it.)
    RunnerStatus status() const noexcept {
        return RunnerStatus{rt_core_->phase_.load(std::memory_order_relaxed), rt_core_->reason_.load(std::memory_order_relaxed)};
    }

   private:
    friend struct RunnerTestPeer;  // #TODO-3: white-box access to private stop() in tests only

    // The teardown, PRIVATE (#TODO-3): called only by ~Runner and run() (both non-RT,
    // owner-thread). request -> BOUNDED-wait the stopping window -> on success join + clear
    // the #39 bracket + master.close(); on timeout (a wedged step()) FAIL-STOP via
    // std::abort() after a loud log (#52: the parked thread holds the externally-owned
    // control we can't leak, so tearing down would UAF -- abort instead; the drive is
    // SM-watchdog-safe and the supervisor restarts the module). Private => unreachable from
    // the RT thread => the old self-join guard (+ its rt_tid_ atomic, #49) are DELETED.
    void stop() noexcept;

    Master& master_;                   // owner-side close()/set_rt_active(false) on the clean path
    std::unique_ptr<RtCore> rt_core_;  // the RT-shared state + thread (heap: stable address; never released)
    std::atomic<bool> started_{false};
};

}  // namespace ethercat
