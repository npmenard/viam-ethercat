#pragma once

// ethercat::Runner + SlaveControl + CycleContext.
//
// The library owns everything from start() to close: realtime setup, the single
// DcPacer, the bring-up pump, the steady process/step cycle, the stopping window,
// and master.close(). The consumer derives SlaveControl and implements per-cycle
// policy in step(). DC vs free-run is a config fact the Runner reads; consumer
// step() code is identical either way.
//
// Layering: Master stays pure bus policy (config/remap/bringup_step/process,
// caller-paced and thread-free). The Runner is the orchestration layer composing
// them, and the sole home of the set_rt_active bracket, the spawn/join, and the
// teardown sequence.
//
// The once-per-cycle step() cadence is enforced by construction: the Runner owns
// the only call site. A blocking or slow step() becomes a pacer overrun, which
// DcPacer absorbs with phase-preserving catch-up while the WKC and cycle counters
// keep moving, so an overrun is observable and recoverable rather than silent
// corruption or an off-phase burst.

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
    BringupAborted,  // bring-up gave up (bounded; no auto-retry)
    RtSetupFailed,   // SCHED_FIFO unavailable && RunnerConfig::require_realtime
    Wedged,          // a step() did not reach Stopped within the bounded join. stop() then
                     // fail-stops the process with std::abort() rather than tearing down state
                     // the parked thread still holds. The drive stays safe: process data gaps
                     // upstream of the wedge, so the slave's SM watchdog de-energizes it.
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
    // When true and SCHED_FIFO is unavailable, the run aborts (StopReason::RtSetupFailed)
    // instead of running best-effort.
    bool require_realtime = false;
    // Bring-up give-up (wall time). Sits above Master's own op_await window by default so
    // the Master's verdict decides and this bound is only the backstop.
    std::chrono::milliseconds bringup_timeout{120'000};
    // The stopping window: after on_stop(), this many more steady cycles run with
    // ctx.stopping() == true so the control's policy can disable its drive with process
    // data still flowing (e.g. CiA402 controlword -> 0x00). Floor 1. Safety does not depend
    // on it; a window-ignoring control still ends at master.close()'s INIT teardown.
    std::uint32_t teardown_cycles = 100;  // ~100 ms @ 1 kHz
    // The most wall time stop()/~Runner waits for the RT loop to finish its stopping window
    // and exit before declaring it wedged and fail-stopping the process. 0 means derive:
    // max(250ms, (teardown_cycles + 20 slack) x period x 4); the x4 covers a dead bus where
    // each receive blocks about EC_TIMEOUTRET. Must exceed the genuine window wall time, or a
    // healthy slow teardown is misread as wedged.
    std::chrono::milliseconds stop_join_timeout{0};
};

class Runner;
// The heap-held, RT-thread-shared cyclic state. The Runner owns it by unique_ptr; the RT
// thread lives inside it and captures the RtCore*, not the Runner, so the clean teardown's
// join is a clean barrier and a wedge fail-stops rather than tearing down under the parked
// thread. See the class definition below.
class RtCore;
// White-box test access to the private teardown: consumers stop by dropping the Runner, but
// the bounded-join, Wedged, and concurrency tests must drive stop() directly. Defined only
// in the test build. stop() stays private; only this declared peer can name it.
struct RunnerTestPeer;

// The per-cycle, per-slave RT surface handed to SlaveControl. Only the RT form of field
// access lives here: the per-call-resolving, throwing forms (resolve_rx/resolve_tx) are
// not, because they resolve per-call and throw, which the step() contract bans. Non-RT
// consumers keep those (or their own atomics) outside step(). No Master&, no SDO, no map
// access, no raw image pointers: load/store at pre-resolved FieldLocation handles is the
// whole hot-path surface.
//
// The input and output images are held by value (fixed arrays of kMaxPdoBytes), not spans
// into the Runner's live process buffers. dispatch() copies the slave's input image in
// before each hook and copies the owned output buffer out after. So a control that stashes
// the ctx and touches it after step() returns reads a valid object with stale data, never a
// dangling or in-flight read (at 1 kHz a wild read into repointed buffers could command
// dangerous motion). C++ cannot forbid the escape, so the design makes it harmless instead:
// a store through a stale handle lands in the already-copied-back owned buffer and never
// reaches the wire.
//
// The ctx is valid on the RT thread, during its dispatch window only. Owned data fixes the
// single-RT-thread escape; it does not make the ctx safe to share across threads -- a non-RT
// thread reading the ctx while the RT thread copies the next cycle in is still a data race
// (per-access locking would be the fix, and is out of scope). The debug assert (check_live)
// steers developers away from all out-of-window use, including the cross-thread case.
//
// The safe-stale property holds only within the RtCore's lifetime. The ctx lives in the
// RtCore's controls_ deque (heap-owned by the Runner), so a handle that outlives the Runner
// and RtCore (used after a clean teardown destroys them) is a use-after-free, not
// safe-stale. Within a live Runner the escape is harmless; a wedge never destroys the
// RtCore, it fail-stops.
class CycleContext {
   public:
    // Read a typed field from this cycle's latched input image (the feedback the cycle's
    // process() just exchanged, stable for the whole step()). Reads the owned input copy
    // that dispatch() refreshed before the hook.
    template <PdoScalar T>
    T load(FieldLocation loc) const noexcept {
        check_live();
        // View only this slave's valid Tx prefix, so a loc past the real image is out of range.
        return load_le<T>(std::span<const std::byte>(inputs_).first(input_size_), loc);
    }
    // Stage a typed field into the owned output buffer; dispatch() copies it to the wire
    // after the hook, so it ships with the next cycle's process(). A store made this cycle
    // is on the wire the next cycle (one cycle of command latency).
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
    // DC system time of this cycle's exchange; 0 when DC is disabled. The pacing regimes
    // differ only here; consumer step() code is identical.
    std::int64_t dc_time_ns() const noexcept {
        check_live();
        return dc_time_;
    }
    // True during the stopping window: run the drive-disable policy now.
    bool stopping() const noexcept {
        check_live();
        return stopping_;
    }
    // The RT-side error/stop channel: latches StopReason::Requested and enters the stopping
    // window. No exceptions cross the RT boundary; this is the way out.
    void request_stop() noexcept;
    // The WKC health counters (fields individually relaxed, +/-1 skew by design).
    WkcStats wkc() const noexcept;
    // The Master's latched bus-fault state (the consecutive-WKC-error latch). A consumer
    // step() that drives its own per-cycle fault policy off the bus tier reads it here rather
    // than touching the Master directly, since the Runner is the sole Master toucher. Same
    // value the Runner uses to latch StopReason::BusFault.
    bool fault() const noexcept;

    // Copy and move are deleted: the ctx is a long-lived Runner-owned member handed out by
    // reference per dispatch. Deleting them makes `auto saved = ctx;` a compile error rather
    // than a silent value-copy that would alias the owned buffers.
    CycleContext(const CycleContext&) = delete;
    CycleContext& operator=(const CycleContext&) = delete;
    CycleContext(CycleContext&&) = delete;
    CycleContext& operator=(CycleContext&&) = delete;

   private:
    friend class RtCore;
    explicit CycleContext(RtCore* core) noexcept : core_(core) {}
    // The ctx is valid only during its own dispatch window (the Runner sets live_ around each
    // hook/step call). A control that caches the ctx and touches it outside its window trips
    // this assert in debug builds. In release it compiles to nothing, and per the owned-data
    // design the worst case there is safe-stale (a valid object with last-cycle data), never
    // corruption.
    void check_live() const noexcept;

    RtCore* core_;  // the RT-shared state (request_stop/wkc go through it)
    // Owned images, not spans into live buffers. dispatch() copies the slave input in before
    // the hook and copies this output out after, so an escaped ctx reads safe-stale data and
    // an escaped store never reaches the wire.
    std::array<std::byte, kMaxPdoBytes> inputs_{};   // refreshed each cycle (latched feedback)
    std::array<std::byte, kMaxPdoBytes> outputs_{};  // staged command (copied to wire post-hook)
    std::size_t input_size_ = 0;                     // valid prefix of inputs_  (this slave's Tx image)
    std::size_t output_size_ = 0;                    // valid prefix of outputs_ (this slave's Rx image)
    std::uint64_t cycle_ = 0;
    std::int64_t dc_time_ = 0;
    bool stopping_ = false;
    bool live_ = false;  // set by the Runner around dispatch only
};

// The configure-time surface handed to on_configured: the restricted, pre-spawn analog of
// CycleContext. Exposes only the configure-time operations -- typed field resolution (the
// width assert fires here) and one-time SDOs -- bound to this control's slave. It never
// exposes process()/cyclic PD or a raw Master&.
//
// A raw Master& would let a control stash it and call process() from a wedged step(),
// keeping process data flowing so the SM watchdog never fires and the energized-forever
// wedge becomes reachable. With no process() reachable from any hook, a wedged step() is
// watchdog-safe by construction. The SDO seam does not re-open this: a stashed ConfigContext
// used to sdo_write from step() hits the rt_active guard and throws through the noexcept
// step, terminating loudly rather than silently defeating the watchdog.
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
    // Optional resolve: returns nullopt when the object is absent from the map instead of
    // throwing, since a consumer maps some fields only in some modes. A mapped-but-wrong-width
    // object still throws. Store the std::optional and guard the per-cycle load/store on it
    // (deref with *loc -- never .value(), which can throw, on the RT path).
    template <class F>
    std::optional<FieldLocation> try_resolve_rx() const {
        return master_.try_resolve_rx<F>(slave_id_);
    }
    template <class F>
    std::optional<FieldLocation> try_resolve_tx() const {
        return master_.try_resolve_tx<F>(slave_id_);
    }
    // One-time SDOs while the consumer is the single port owner (pre-RT), e.g. a regime
    // readback or a vendor fault-reset. Post-spawn, via a stashed handle, Master's rt_active
    // guard makes these throw.
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

    // Non-RT, pre-spawn, and the only hook that may throw: resolve typed fields
    // (cfg.resolve_rx/resolve_tx<F>, where the width assert fires), run one-time SDOs
    // (e.g. a 0x605A regime readback via cfg.sdo_read, during the single-port-owner phase),
    // and validate config. A throw aborts start() cleanly: nothing spawned, no bracket set,
    // master untouched. Receives the restricted ConfigContext, not a raw Master&, so no
    // process() is reachable from any hook.
    virtual void on_configured(ConfigContext& cfg) {
        (void)cfg;
    }
    // RT, once, the first cycle AT Operational -- before the first step().
    virtual void on_operational(CycleContext& ctx) noexcept {
        (void)ctx;
    }
    // RT, once, on entering the stopping path (any cause; the reason says which). The
    // stopping window follows (steady cycles with ctx.stopping() == true); put the
    // drive-disable policy in step() during stopping, not here.
    virtual void on_stop(StopReason reason) noexcept {
        (void)reason;
    }
    // RT, every steady cycle: inputs latched, then step(), then outputs ship with the next
    // exchange. Bounded, no allocation, no blocking, noexcept. Never called before
    // Operational. The Runner owns the only call site, so the cadence is enforced by construction.
    virtual void step(CycleContext& ctx) noexcept = 0;
    // RT, every bring-up cycle: does the drive report a sync fault? A gate signal for
    // bringup_step. Defaults to false; a device-aware control implements it (e.g. a mapped
    // error-code field equals its drive's no-sync code). This hook keeps Master and Runner
    // free of vendor and CiA402 knowledge.
    virtual bool sync_faulted(const CycleContext& ctx) const noexcept {
        (void)ctx;
        return false;
    }
    // RT, every bring-up cycle: does the drive's feedback look alive? An OP-confirm gate for
    // bringup_step, beyond the working counter -- a workaround for drives that do not visibly
    // refuse a bad OP request: refused into OP (e.g. a DC-only drive asked to free-run) they
    // sit at SAFE-OP (AL 0x0027) yet contribute a full working counter while their statusword
    // stays 0x0 and their inputs read dead, so a WKC-only check wrongly declares OP and the
    // enable ladder spins forever. Defaults to true (WKC only); a
    // device-aware control implements it (e.g. statusword != 0). A control that returns false
    // through the whole OP-await window makes bring-up give up (BringupAborted, and the
    // AL-status diagnostic names AL 0x0027) instead of enabling a dead drive.
    virtual bool drive_present(const CycleContext& ctx) const noexcept {
        (void)ctx;
        return true;
    }
    // RT, every stopping cycle (after step()): is this control safely torn down -- de-energized
    // at rest -- so the Runner may end the teardown window early? teardown_cycles is the cap;
    // this is the event-driven early-out. Defaults to false, running the full window. A control
    // doing a controlled ramp-stop (ramp velocity to 0, then de-energize) returns true once it
    // reaches rest and is disabled, so an already-stopped control does not pay the full window
    // while a moving stop still ramps fully before close().
    virtual bool teardown_complete() const noexcept {
        return false;
    }
};

// The RT-thread-shared cyclic state. Everything the RT thread touches that the Runner owns
// lives here, on the heap, owned via unique_ptr, and the thread itself lives here too
// (thread_). The RT loop captures the RtCore*, not the Runner, so it never reaches a Runner
// member; on a clean stop the Runner joins thread_ (the barrier) then destroys this normally,
// touching no member after the join. On a wedge the Runner does not tear down at all: it
// fail-stops via std::abort(), because the RT thread is parked inside the externally-owned
// control's step(), which RtCore cannot leak, so not freeing anything is the only
// use-after-free-free option. See Runner::stop(). Not copyable or movable.
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

    // Holds a control and its owned-data CycleContext. The ctx is non-copyable and
    // non-movable, so Attached is too, which is why controls_ is a std::deque (node-based,
    // stable addresses) populated by in-place emplace_back. The ctor builds the ctx in place
    // from the RtCore* via CycleContext's private ctor, which Attached, a friend of RtCore,
    // may call.
    struct Attached {
        Attached(std::uint16_t id, SlaveControl* c, RtCore* core) noexcept : slave_id(id), control(c), ctx(core) {}
        std::uint16_t slave_id;
        SlaveControl* control;
        CycleContext ctx;
    };

    // Refresh a ctx for this cycle (copy the slave input in), dispatch one hook/step with the
    // live window set, then copy the ctx output out to the wire.
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

// The orchestration layer: owns the RT state (RtCore) and the teardown around it. Master
// must be open()+init()+configure()d before start(). One Runner per Master; not copyable or
// movable.
class Runner {
   public:
    Runner(Master& master, RunnerConfig cfg) noexcept;
    Runner(const Runner&) = delete;
    Runner& operator=(const Runner&) = delete;
    Runner(Runner&&) = delete;
    Runner& operator=(Runner&&) = delete;
    // The destructor is the teardown: it runs the graceful stop (request, bounded-join the
    // stopping window, then master.close()), so de-energize-on-destruction happens on every
    // non-wedged path. The owner stops deterministically by dropping the Runner. There is no
    // public stop(); the only two stop paths are ctx.request_stop() (RT) and destruction. A
    // wedged step() cannot hang it: the wait is bounded, and on timeout the process fail-stops
    // via std::abort() rather than tearing down state a thread parked in foreign step() still
    // holds. The supervisor restarts the module.
    ~Runner();

    // Attach a control to a slave (1-based). Pre-start only. Throws Error on
    // attach-after-start, an unknown slave id, or a duplicate attach for the slave. The
    // control is held by reference and the RT thread calls control->step() until teardown
    // joins that thread in ~Runner, so the control must outlive the Runner: declare or own it
    // before the Runner. A control destroyed while the thread still runs is a use-after-free.
    void attach(std::uint16_t slave_id, SlaveControl& control);

    // Runs the non-RT hooks (on_configured(ConfigContext&), which may throw, in which case
    // nothing spawns), locks memory, then spawns the RT thread inside the RtCore
    // (realtime::setup, bring-up, steady, stopping window). Throws Error if no controls are
    // attached or on restart.
    void start();
    // Convenience: start(), block until the RT loop ends (polling status()), then tear down.
    // SIGINT integration is the consumer's handler calling request_stop(). For a custom poll
    // loop, call start() then drop the Runner to stop, since stop() is not public.
    void run();

    // Non-RT stop request (a signal-handler-adjacent thread, or the SDK's Stoppable::stop()
    // on the gRPC thread): latches Requested and sets the flag; the RT loop enters its
    // stopping window next cycle. Does not join -- that is the dtor's job. Pairs with
    // status()-polling for command-and-confirm.
    void request_stop() noexcept;

    // A relaxed diagnostic snapshot, not a happens-before edge: polling status().phase ==
    // Stopped does not synchronize with the RT thread's writes. To read control-published
    // state, use the control's own atomics (or, in a test, the dtor join as the edge), never
    // this poll. StopReason::Wedged is never observed here, because a wedge fail-stops the
    // process inside stop(), leaving no surviving Runner to report it.
    RunnerStatus status() const noexcept {
        return RunnerStatus{rt_core_->phase_.load(std::memory_order_relaxed), rt_core_->reason_.load(std::memory_order_relaxed)};
    }

   private:
    friend struct RunnerTestPeer;  // white-box access to private stop() in tests only

    // The teardown, private: called only by ~Runner and run(), both on the non-RT owner
    // thread. Requests the stop, bounded-waits the stopping window, and on success joins and
    // calls master.close(); on timeout (a wedged step()) it fail-stops via std::abort() after
    // a loud log, because the parked thread holds the externally-owned control and tearing
    // down would use-after-free. The drive is SM-watchdog-safe and the supervisor restarts
    // the module. Being private, it is unreachable from the RT thread.
    void stop() noexcept;

    Master& master_;                   // owner-side close() on the clean path
    std::unique_ptr<RtCore> rt_core_;  // the RT-shared state + thread (heap: stable address; never released)
    std::atomic<bool> started_{false};
};

}  // namespace ethercat
