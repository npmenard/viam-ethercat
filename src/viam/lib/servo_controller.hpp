#pragma once

//
// ServoController owns the real-time loop for ONE servo. SDK-free (src/viam/lib)
// so it is unit-testable on a SimBackend with no Viam SDK and no hardware. It
// owns: the EtherCAT Master, the CommandQueue, the RT thread, the resolved field
// offsets, the lifecycle FSM, and the published ControllerState atomics. The
// library left four behaviors to the driver; this is where they live:
//   (a) fault-reset RISING-EDGE re-arm (Cia402Fsm::step returns the level),
//   (b) last-good + staleness -> fail-safe is_powered()/is_moving(),
//   (c) move-complete predicate |target-actual|<=tol && |vel|<=vthresh (NEVER
//       statusword bit10), and
//   (d) the std::variant lifecycle FSM.
//
// CONCURRENCY CONTRACT (load-bearing):
//  * The RT thread is the ONLY toucher of master_ / the queue's pop during
//    operation. reconfigure() JOINS the RT thread before master_.reset(), so the
//    reset never races the loop.
//  * EVERY non-RT accessor + the go_to/go_for completion wait reads ONLY
//    ControllerState atomics + stopping_ -- NEVER master_ -- so a concurrent
//    is_powered()/parked-go_to can't deref a pointer reconfigure() is resetting.
//  * commands_ is held BY VALUE and never reset until the dtor (after join), so a
//    late non-RT push() can't hit a destroyed queue (DA #5).
//  * api_mutex_ (shared for the API, exclusive for start/stop/reconfigure)
//    serializes lifecycle-vs-lifecycle and lifecycle-vs-push. go_to RELEASES it
//    before parking (its wait uses only atomics), so a multi-second move doesn't
//    block reconfigure.

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <span>
#include <string>
#include <thread>
#include <variant>

#include "ethercat/backend.hpp"
#include "ethercat/cia402.hpp"
#include "ethercat/cia402_policy.hpp"
#include "ethercat/master.hpp"
#include "ethercat/pdo_cache.hpp"
#include "ethercat/runner.hpp"
#include "viam/lib/servo_config.hpp"

namespace ethercat::servo {

// RT-published state read by the non-RT motor API. Every field is atomic; the RT
// loop is the SOLE writer. The non-RT side derives all of is_powered/is_moving/
// position/completion from THESE -- never from master_.
struct ControllerState {
    std::atomic<std::int32_t> position_counts{0};  // 0x6064 actual, before zero-offset
    std::atomic<std::int32_t> velocity{0};         // device velocity units
    std::atomic<bool> powered{false};              // OperationEnabled this cycle
    std::atomic<bool> moving{false};               // !move-complete
    std::atomic<bool> faulted{false};              // DRIVE/BUS only: master_->fault() || status.fault() (move-errors are Tier-2, NOT here)
    std::atomic<std::int32_t> fault_wkc{0};        // WKC at a live bus fault (payload; published BEFORE wkc_faulted release)
    std::atomic<std::int32_t> expected_wkc{0};     // constant after start(); for last_error() (lock-free, master_-free)
    // Per-tier fault liveness (spec #16): last_error() composes EVERY active tier so
    // a both-true Er74 (drive 0x603F + bus WKC->0) reports root cause AND symptom,
    // never masking one. Each (flag, payload) pair: payload relaxed-stored BEFORE the
    // flag release-stored (RT, sole writer); cross-tier skew is benign (quasi-static).
    std::atomic<bool> wkc_faulted{false};                // BUS tier = master_->fault(); pairs with fault_wkc
    std::atomic<bool> drive_faulted{false};              // DRIVE tier = status.fault() (bit3); pairs with drive_fault_code
    std::atomic<std::uint16_t> drive_fault_code{0};      // 0x603F live-read every faulted cycle; relaxed before drive_faulted release
    std::atomic<std::uint64_t> loop_cycle{0};            // heartbeat counter
    std::atomic<std::uint64_t> last_cycle_time_ns{0};    // CLOCK_MONOTONIC ns at last iteration (watchdog; 0 = never published)
    std::atomic<std::int32_t> zero_offset_counts{0};     // SetZero software offset
    std::atomic<std::uint32_t> active_generation{0};     // gen RT adopted from the applied SetTarget (post-coalescing)
    std::atomic<std::uint32_t> completed_generation{0};  // gen that reached target; RT stores-release then notify (no lock)
    std::atomic<std::uint32_t> failed_generation{0};     // gen that stalled/timed out; wakes its waiter to throw
};

// #47-P3a: ServoController IS the module SlaveControl -- it already owns all the RT
// policy + state, so it implements the Runner's hooks directly. The library Runner owns
// the RT thread / pacing / bring-up / teardown; this wrapper owns the Master (persists the
// resource lifetime) and BORROWS it to a one-shot Runner (rt_runner_, last member ->
// destroyed first -> ~Runner joins before Master/state tear down).
class ServoController : public SlaveControl {
   public:
    using BackendFactory = std::function<std::unique_ptr<EcatBackend>()>;

    // Production: backend factory = a SoemBackend maker (so reconfigure() can
    // build a fresh backend). Validates config (no I/O); throws ConfigError.
    explicit ServoController(ServoConfig config);
    // Test/DI: inject a backend factory (SimBackend maker in offline tests).
    ServoController(ServoConfig config, BackendFactory backend_factory);

    ServoController(const ServoController&) = delete;
    ServoController& operator=(const ServoController&) = delete;
    ServoController(ServoController&&) = delete;
    ServoController& operator=(ServoController&&) = delete;
    ~ServoController();  // stop()

    // NON-RT lifecycle (exclusive api_mutex_). start(): build+init+configure the
    // Master to SAFE-OP (may throw InitError), resolve field offsets ONCE, spawn the
    // RT thread; the promise/future handshake makes start() throw if the RT thread
    // can't get SCHED_FIFO and require_realtime.
    //
    // CONTRACT (#20): start() SUCCESS means "the RT thread is launched + scheduled",
    // NOT "operational / powered". Because the DC bring-up must be GAPLESS, OP is
    // reached INSIDE the RT loop (configure() stops at SAFE-OP -- it cannot reach OP
    // without gapping the process-data handoff -> Er74), so start() CANNOT block until
    // OP. It still THROWS synchronously on the one thing it can guarantee up front --
    // the RT loop being able to RUN (setup_realtime() + require_realtime), via the
    // started-promise. The DC bring-up OUTCOME is observed ASYNCHRONOUSLY:
    //   - reached OP   -> is_operational() / is_powered() become true;
    //   - aborted      -> Er74.1 (no SYNC0) in the gate -> the RT loop surfaces the
    //                     drive tier + RtError::NotOperational via last_error() and
    //                     EXITS without auto-retry (repeated Er74 OP-entry wedges the
    //                     A6); recovery is an explicit reconfigure()/restart.
    // A command issued before OP+enabled degrades gracefully: await_move() waits/
    // times-out (the FSM never reaches the completion generation) rather than acting
    // on a non-operational drive.
    void start();
    // Teardown: stopping_=true + notify_all (wake parked waiters) -> request_stop
    // + join (RETURNS before any reset/destroy).
    void stop() noexcept;
    void reconfigure(ServoConfig config);  // stop() -> rebuild master_ from the factory -> start()

    // --- non-RT motor API (shared api_mutex_; go_to/go_for release before park) ---
    void set_rpm(double rpm);                 // PV only (rejects in PP)
    void go_for(double rpm, double revs);     // PP: relative move; PV: timed run
    void go_to(double rpm, double position);  // PP only (rejects in PV)
    void halt() noexcept;
    // Set the software zero so the CURRENT actual reads `offset_revs` (default 0).
    // offset_revs != 0 reads config_ -> takes the shared lock; offset 0 is the
    // common stop/zero case.
    void set_zero(double offset_revs = 0.0) noexcept;
    // Operator recovery / power control (surfaced via the module's do_command).
    void request_fault_reset() noexcept;  // edge the CiA402 fault-reset + clear the controller-error latch
    void enable() noexcept;               // re-enable from Disabled
    void disable() noexcept;              // disable voltage (coast)

    // --- non-RT accessors (master_-FREE: ControllerState atomics + stopping_) ---
    double position_revs() const noexcept;
    bool is_moving() const noexcept;   // moving && rt_alive() && !stopping_
    bool is_powered() const noexcept;  // powered && rt_alive() && !stopping_
    bool is_disconnected() const noexcept;
    std::string last_error() const;
    // Last published device velocity (0x606C from the wire when mapped, else the
    // instantaneous estimate). RAW device units -- the module layer converts to
    // Viam units (the 0x606C scaling is a drive-unit question, applied there, not
    // here). master_-free + lock-free, symmetric with position_revs().
    std::int32_t velocity_counts() const noexcept;
    // RT loop heartbeat counter (cycles since start). Lock-free atomic read, master_-free.
    // For tests that need a CYCLE-based bound (e.g. #18: assert a give-up happens within
    // N loop-cycles, robust to the async loop's wall-clock rate under TSan/SCHED_OTHER).
    std::uint64_t loop_cycle() const noexcept;
    // #39: the underlying Master, for SINGLE-PORT-OWNER SDO use ONLY (an operator/tool
    // doing ad-hoc CoE pre-start()/post-stop() -- and the DA-required bracket test).
    // NOT part of the master_-free accessor contract: callers MUST NOT touch it
    // concurrently with start()/stop()/reconfigure() (those reset it under the exclusive
    // lock); during a running RT phase Master's own rt_active guard makes SDO throw.
    // nullptr before the first start(). Steady-state SDO is #22's queue, not this.
    Master* master_for_sdo() noexcept {
        return master_.get();
    }

   private:
    // --- lifecycle FSM (std::variant; each state's step() in the .cpp) ---
    struct Init {};
    struct Enabling {};
    struct Operational {};
    struct Resetting {};  // holds the fault-reset intent across the drive's clear-reflect latency (spec #18)
    struct Faulted {};
    struct Disabled {};
    using Lifecycle = std::variant<Init, Enabling, Operational, Resetting, Faulted, Disabled>;

    // The PP new-setpoint handshake now lives in the shared Cia402Policy (#47-P3b): the wrapper
    // delegates the Operational healthy-path to policy_.step() and reads its handshake-idle /
    // handshake-timeout signals (completion gate / abort). No wrapper-side handshake sub-FSM.

    // CONTROLLER-tier fault reasons (the CTRL tier only -- spec #16 moved the BUS
    // WkcFault out to state_.wkc_faulted). The RT thread only STORES the enum (no
    // string alloc, no mutex on the hot path); last_error() composes the text non-RT.
    enum class RtError : std::uint8_t {
        None,
        HandshakeTimeout,
        MoveStalled,
        NotOperational,
        FaultResetFailed,
        MotorStopped,   // #47-P3b R3: an in-flight move CANCELLED by stop()/halt() -> waiter throws
        MotorDisabled,  // #47-P3b R3: an in-flight move CANCELLED by disable() (operator de-energize) -> waiter throws
        ModeMismatch,   // #47-P3c/#57: 0x6061 != commanded mode at SwitchedOn -> REFUSE to energize (fail-closed, #45)
    };

    // The RT thread body (loop while !st.stop_requested()). `started` is fulfilled
    // after a clean prelude (or set to an InitError exception on RT-sched failure
    // && require_realtime) -> bounded start() handshake. The promise lives in the
    // thread (reconfigure-safe). On exit: leave outputs safe (Halt/disable) + a
    // final process(), then return so join() completes.
    // --- SlaveControl hooks (RT; the Runner owns the thread/pacing/bring-up/teardown) ---
    // on_configured: no-op (field-resolve + vendor reset stay in start(), pre-Runner-start,
    // single-port-owner -- behavior-identical to today). sync_faulted: the 0x603F==sync_fault_code
    // bring-up gate. step: drain + snapshot + step_lifecycle + publish (the old steady body).
    // on_stop: map the Runner's StopReason to the two-tier fault (bring-up abort / bus / clean).
    void on_configured(ConfigContext& cfg) override;
    bool sync_faulted(const CycleContext& ctx) const noexcept override;
    void on_operational(CycleContext& ctx) noexcept override;
    void step(CycleContext& ctx) noexcept override;
    void on_stop(StopReason reason) noexcept override;
    // #47-P3b R1: event-driven teardown early-out -- true once the LIFECYCLE-stop has de-energized
    // the drive AT REST (statusword SwitchOnDisabled during the stopping window), so the Runner
    // ends the (generous, decel>0) teardown window as soon as the controlled ramp completes rather
    // than always spinning the full cap. RT-only read (same thread as step()).
    bool teardown_complete() const noexcept override {
        return stop_at_rest_;
    }

    // Reset per-run state + construct/attach/start the one-shot Runner; on a start-time
    // failure → Degraded-but-alive (§8), never rethrows past here. Shared by start()/reconfigure().
    void spawn_runner();
    void reset_run_state();                              // zero the per-run atomics + RT-only working state
    void resolve_fields();                               // cache controlword/status/target/actual/velocity FieldLocations
    bool rxpdo_has(std::uint16_t index) const noexcept;  // is `index` mapped in the RxPDO? (optional-field probe)
    bool txpdo_has(std::uint16_t index) const noexcept;  // is `index` mapped in the TxPDO? (optional feedback probe)
    std::string fault_gloss(std::uint16_t code) const;   // 0x603F code -> config label (empty if unknown); cold path
    bool rt_alive() const noexcept;                      // !watchdog_expired() && !state_.faulted  (master_-FREE)
    bool watchdog_expired() const noexcept;              // (now - last_cycle_time_ns) > watchdog_ns

    ServoConfig config_;
    BackendFactory backend_factory_;

    // Resolved once at start(); indexed by the RT loop without a map find.
    // INVARIANT (learned from the 0x6060 + 0x6081 gaps): every RxPDO field the drive
    // CONSUMES must be written by the RT loop each cycle (or SDO-set at configure) --
    // controlword + target position/velocity + profile velocity here. A mapped-but-
    // unwritten command field makes the drive use its default (silent wrong behavior
    // on hardware). The SimBackend consumes each field's WIRE value (not a config
    // shortcut) so a missing write FAILS an offline test, not just the bench.
    FieldLocation f_ctrlword_;
    FieldLocation f_statusword_;
    FieldLocation f_target_;
    FieldLocation f_actual_;
    FieldLocation f_velocity_;
    FieldLocation f_profile_velocity_;  // 0x6081 PP move speed; !mapped() if not in the map (optional)
    // #47-P3c/#57 enable-ladder mode fields (OPTIONAL). The module's own enable FSM (not the policy)
    // climbs to OE, so it must itself (a) SEED 0x6060 = commanded mode through the ladder when 0x6060 is
    // RxPDO-mapped (else a PDO-following drive enables in mode 0 -- the #56 shape, bench maps only), and
    // (b) enforce the #45 mode-echo GATE when 0x6061 is TxPDO-mapped (refuse OE if 0x6061 != commanded --
    // the A6 production map DOES map 0x6061). Both !mapped() => the respective step is inert.
    FieldLocation f_mode_wr_;    // 0x6060 mode-of-operation (RxPDO write); !mapped() => SDO-set only (prod)
    FieldLocation f_mode_disp_;  // 0x6061 mode-display (TxPDO read); !mapped() => no mode-echo gate
    // #47-P3c/#57 enable-time mode-echo gate state (RT-only). STICKY once resolved so the drive doesn't
    // oscillate RTSO<->SwitchedOn: Pending until the drive is SwitchedOn with 0x6061 mapped, then Passed
    // (0x6061 == commanded -> allow OE) or Failed (mismatch -> latch RtError::ModeMismatch + de-energize).
    // Reset to Pending on Init->Enabling so a fresh bring-up / fault-recovery re-checks.
    enum class ModeGate : std::uint8_t { Pending, Passed, Failed };
    ModeGate mode_gate_ = ModeGate::Pending;
    // TxPDO feedback fields (spec #16). Both OPTIONAL (!mapped() => not in the map):
    FieldLocation f_fault_code_;       // 0x603F U16 drive error code (last_error gloss)
    FieldLocation f_velocity_actual_;  // 0x606C S32 velocity-actual (wire velocity; else estimate)

    // #47-P3b: the GENERIC CiA402 motion policy (shared with a6_validate's A6Control). The
    // module's Operational healthy-path (enable-hold + PP handshake + PV stream + Halt) delegates
    // HERE; the wrapper keeps the two-tier fault + #18 fault-reset machine + completion-generations
    // + stall watchdog (rev-6 signed boundary). Parameterized by a DeviceProfile mapped from
    // ServoConfig (module flags: bit8 Halt, no PV pos-mirror, 4-phase handshake + ack timeout).
    static DeviceProfile make_module_profile(const ServoConfig& c) noexcept;
    Cia402Policy policy_;
    // 0x6085 readback from policy_.configure (0 = quick-stop not configured). WRITTEN once by the
    // RT thread in on_configured (pre-steady), READ by the non-RT velocity guard -> atomic.
    std::atomic<std::uint32_t> qs_decel_echoed_{0};
    // Effective PV/PP velocity ceiling (counts/s) DERIVED from the teardown window (#47-P3b R1):
    // max the echoed 0x6085 decel can ramp to 0 within (teardown_window - margin). 0 = no guard
    // (quick-stop not configured). WRITTEN once in on_configured (RT), READ non-RT by set_rpm/go_to.
    std::atomic<std::int64_t> vel_budget_cps_{0};
    // RT teardown window in cycles (the Runner's stopping-window CAP): sized so a decel>0 Quick-Stop
    // ramp completes before close(); 2 for the opt-out coast. The VEL budget derives from this same
    // value -> window and budget are one source of truth.
    std::uint32_t teardown_window_cycles() const noexcept;
    // Clamp a commanded velocity (counts/s) to the stoppable-within-teardown-window budget (#47-P3b
    // R1). Applied to the PV setpoint (0x60FF) AND the PP move speed (0x6081). Inert when unconfigured.
    std::int32_t clamp_to_stop_budget(std::int32_t vel_cps) const noexcept;

    Cia402Fsm fsm_;
    ControllerState state_;
    std::atomic<RtError> rt_error_{RtError::None};

    // Non-RT-readable teardown signal (separate from the jthread stop_token):
    // drives the go_to/go_for wait predicate + the accessors' fail-safe.
    std::atomic<bool> stopping_{false};
    std::atomic<std::uint32_t> next_generation_{0};  // non-RT: assigns unique move ids
    // #47-P3b R3 SINGLE-IN-FLIGHT slot: the generation of the ACTIVE blocking move (go_to/go_for),
    // or 0 = FREE. A new blocking move CLAIMS it via a SINGLE CAS that reclaims a slot whose gen is
    // already TERMINAL (completed/failed) -- no check-then-claim TOCTOU between two gRPC callers. The
    // waiter does NOT release it (reclaim-if-terminal on the next claim). Non-RT (API-thread) owned.
    std::atomic<std::uint32_t> motion_slot_{0};
    std::atomic<std::uint64_t> watchdog_ns_{0};      // RT-liveness window (set at start; config-free reads)
    // #54 P3a §8 Degraded-but-alive: set when start()/bring-up fails (RT-spawn / on_configured
    // refusal / drive AL-reject) -- motion APIs throw "{degraded_reason_}", accessors fail-safe,
    // the process NEVER crashes; reconfigure()/start() clear it on a clean retry. degraded_
    // (lock-free) gates the accessors; degraded_reason_ is read/written under api_mutex_.
    std::atomic<bool> degraded_{false};
    std::string degraded_reason_;  // set (under api_mutex_) on a SYNCHRONOUS start failure; async (RT) failures use last_error()

    mutable std::shared_mutex api_mutex_;  // API=shared, lifecycle(start/stop/reconfigure)=exclusive

    std::mutex completion_mutex_;  // go_to/go_for waiter side only; RT NEVER locks it
    std::condition_variable completion_cv_;

    // --- RT-ONLY working state (single-thread; plain members, no atomics/locks).
    // Touched exclusively by run_rt_loop() / the lifecycle step()s. ---
    std::uint16_t last_cw_ = 0;                 // for the fault-reset rising-edge re-arm
    std::uint32_t reset_cycles_remaining_ = 0;  // Resetting-window countdown, RT-only (spec #18)
    std::uint32_t clear_streak_ = 0;            // consecutive dev!=Fault cycles in Resetting (type-c debounce; RT-only, #18)
    std::int32_t target_counts_ = 0;            // latched PP target
    std::uint32_t profile_vel_ = 0;             // latched PP profile velocity
    std::int32_t pv_velocity_ = 0;              // latched PV target velocity
    std::int32_t last_progress_actual_ = 0;     // move no-progress watchdog
    std::uint32_t stall_cycles_ = 0;
    std::int32_t prev_actual_ = 0;  // previous-cycle actual (instantaneous velocity estimate)
    bool first_cycle_ = true;       // skip the velocity estimate on the first cycle
    bool halted_ = false;           // STICKY Stop: Halt stays asserted until a new motion command
    // #47-P3b M6 (PV->PP hold-switch): a PV motion-hold that holds zero VELOCITY (bit8) drifts under
    // load -- the drive has no position loop in PV. When the map is switch-capable (0x6060 + 0x607A both
    // RxPDO-mapped), a Halt of a PV move instead switches the drive to PP-at-current-counts (the generic
    // mode-switch, then a PP setpoint = the position latched AT the halt) so the drive's position loop
    // LOCKS the shaft. pv_hold_token_ (high-bit base, never collides with real move gens which start at
    // 1) kicks the policy's PP handshake for the hold WITHOUT touching active_generation (the halt already
    // failed the in-flight move -- the hold is not a completable move). On mode_switch_failed the hold
    // reverts to the interim PV-at-0 bit8 hold (accept small drift, never de-energize -- spec §A R1).
    bool pv_hold_capable_ = false;  // set at resolve: PV mode AND 0x6060 AND 0x607A both mapped
    bool pv_hold_as_pp_ = false;    // STICKY: currently holding a halted PV motor via PP-at-counts
    std::uint32_t pv_hold_token_ = 0x80000000u;  // policy token that kicks the PP hold handshake (never a real gen)
    bool stop_at_rest_ = false;     // RT-only (#47-P3b R1): drive reached SwitchOnDisabled during the stopping window -> teardown early-out
    // Controller-error tier: one-shot latches (HandshakeTimeout/MoveStalled) set by
    // the FSM, cleared ONLY by an explicit fault_reset. The bus WkcFault tier is
    // LIVE (recomputed from master_->fault() each cycle) and is NOT stored here, so a
    // persistent bus fault correctly reappears after a fault_reset.
    RtError latched_ctrl_error_ = RtError::None;
    mutable std::uint16_t last_sync_code_ = 0;  // #54: 0x603F read in (const) sync_faulted (bring-up), consumed by on_stop(BringupAborted)

    // FSM helpers (RT-only). Defined in the .cpp. ctx replaces the old direct master_
    // output writes / master_->fault() reads (the Runner is the sole Master toucher).
    std::uint16_t step_lifecycle(CycleContext& ctx, Status status, const CommandBatch& batch, std::int32_t actual) noexcept;
    std::uint16_t fault_reset_with_rearm(Status status) noexcept;
    // Reads THIS cycle's owned input snapshot via ctx (0x603F, statusword, etc. all from
    // the same latched image the Runner copied in -- structurally consistent, as before).
    void publish_state(CycleContext& ctx, Status status, std::int32_t actual, std::int32_t velocity) noexcept;
    // Abort the in-flight move: set BOTH tiers -- latched_ctrl_error_ (+rt_error_
    // for last_error) AND failed_generation+notify (to wake the go_to waiter
    // PROMPTLY). The invariant: every FSM path that fails the active move calls this.
    void abort_active_move(RtError reason) noexcept;
    // Park on generation `g`'s completion (bounded wait_for, lost-wakeup-immune)
    // then classify the wake and THROW on stop/abort/fault/timeout. Shared by
    // go_to (absolute) and go_for (relative) so both get identical semantics.
    void await_move(std::uint32_t generation, std::chrono::milliseconds timeout);
    // #47-P3b R3 single-in-flight slot (non-RT / API thread). gen_terminal: has this move reached a
    // terminal (completed|failed) state? try_claim_motion_slot: CAS the slot to `gen`, reclaiming it
    // only if FREE or holding a TERMINAL gen -> false if a LIVE blocking move owns it (M7b, no TOCTOU).
    bool gen_terminal(std::uint32_t gen) const noexcept;
    bool try_claim_motion_slot(std::uint32_t gen) noexcept;
    bool motion_slot_busy() const noexcept;  // a LIVE (non-terminal) blocking move holds the slot
    // Submit a PV velocity setpoint (rpm -> guarded device counts) WITHOUT the slot check -- for
    // set_rpm (post its own check) and go_for(PV) (which owns the slot for its whole timed run).
    void push_velocity(double rpm) noexcept;

    // commands_ BY VALUE -> never reset until dtor (no stop-time push-vs-destroy
    // UAF). master_ unique_ptr -> rebuilt by reconfigure() AFTER join (RT thread
    // is its only cyclic user). rt_thread_ LAST -> destroyed/joined first.
    CommandQueue commands_;
    std::unique_ptr<Master> master_;  // WRAPPER-OWNED: persists the resource lifetime; the Runner only BORROWS it
    Lifecycle lifecycle_{Init{}};

    // #54 P3a: the one-shot library Runner BORROWS master_ + holds *this as its control. LAST
    // member -> destroyed FIRST -> ~Runner bounded-joins the RT thread + master.close()->INIT
    // BEFORE master_/commands_/state tear down (the control MUST outlive the Runner).
    std::unique_ptr<Runner> rt_runner_;  // MUST be last member
};

}  // namespace ethercat::servo
