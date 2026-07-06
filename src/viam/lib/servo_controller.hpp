#pragma once

//
// ServoController owns the real-time loop for one servo. It is SDK-free (src/viam/lib), separate
// from the Viam module glue. It owns the EtherCAT Master,
// the CommandQueue, the RT thread, the resolved field offsets, the lifecycle FSM, and the
// published ControllerState atomics. The behaviors the library leaves to the driver live here:
//   (a) fault-reset rising-edge re-arm (Cia402Fsm::step returns the level),
//   (b) staleness -> fail-safe is_powered()/is_moving(),
//   (c) is-moving/reached: the generic base trusts statusword bit 10; a device subclass
//       overrides reached_target() for drives that tie bit 10 permanently high,
//   (d) the runtime mode-switch orchestration (run_mode_switch), and
//   (e) the std::variant lifecycle FSM.
//
// Concurrency contract (load-bearing):
//  * The RT thread is the only toucher of master_ and the queue's pop during operation.
//    reconfigure() joins the RT thread before master_.reset(), so the reset never races the loop.
//  * Every non-RT accessor and the go_to/go_for completion wait reads only ControllerState
//    atomics and stopping_, never master_, so a concurrent is_powered() or parked go_to cannot
//    dereference a pointer reconfigure() is resetting.
//  * commands_ is held by value and never reset until the dtor (after join), so a late non-RT
//    push() cannot hit a destroyed queue.
//  * api_mutex_ (shared for the API, exclusive for start/stop/reconfigure) serializes
//    lifecycle-vs-lifecycle and lifecycle-vs-push. go_to releases it before parking (its wait
//    uses only atomics), so a multi-second move does not block reconfigure.

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <optional>
#include <shared_mutex>
#include <span>
#include <string>
#include <thread>
#include <variant>
#include <vector>

#include "ethercat/backend.hpp"
#include "ethercat/cia402.hpp"
#include "ethercat/cia402_sequencer.hpp"
#include "ethercat/master.hpp"
#include "ethercat/pdo_cache.hpp"
#include "ethercat/runner.hpp"
#include "viam/lib/servo_config.hpp"

namespace ethercat::servo {

// The user-facing "this host has no realtime scheduling" explanation: what failed (SCHED_FIFO at
// `priority`), why the module needs it, and how to fix it (viam-server realtime permission /
// PREEMPT_RT kernel / require_realtime=false opt-out for development). Thrown by
// start()/reconfigure() on a non-realtime host and composed into last_error() by the RT-thread
// backstop, so the same clear text reaches the Viam UI on every path.
std::string rt_unavailable_message(int priority);

// RT-published state read by the non-RT motor API. Every field is atomic; the RT loop is the
// sole writer. The non-RT side derives is_powered/is_moving/position/completion from these,
// never from master_.
struct ControllerState {
    std::atomic<std::int32_t> position_counts{0};  // 0x6064 actual, before zero-offset
    std::atomic<std::int32_t> velocity{0};         // device velocity units
    std::atomic<bool> powered{false};              // OperationEnabled this cycle
    std::atomic<bool> moving{false};               // !move-complete
    std::atomic<bool> faulted{false};  // drive/bus only: master_->fault() || status.fault() (move errors are the controller tier, not here)
    std::atomic<std::int32_t> fault_wkc{0};     // WKC at a live bus fault (payload; published before the wkc_faulted release)
    std::atomic<std::int32_t> expected_wkc{0};  // constant after start(); for last_error() (lock-free, master_-free)
    // Per-tier fault liveness: last_error() composes every active tier, so a both-true sync-loss
    // (drive 0x603F plus bus WKC -> 0) reports root cause and symptom without masking either. In
    // each (flag, payload) pair the payload is relaxed-stored before the flag is release-stored
    // (RT, sole writer); cross-tier skew is benign.
    std::atomic<bool> wkc_faulted{false};                // bus tier = master_->fault(); pairs with fault_wkc
    std::atomic<bool> drive_faulted{false};              // drive tier = status.fault() (bit 3); pairs with drive_fault_code
    std::atomic<std::uint16_t> drive_fault_code{0};      // 0x603F live-read every faulted cycle; relaxed before the drive_faulted release
    std::atomic<std::uint16_t> bringup_al_code{0};       // ESC AL status code at a bring-up give-up (e.g. 0x0027 free-run); 0 = none
    std::atomic<std::uint64_t> loop_cycle{0};            // heartbeat counter
    std::atomic<std::int32_t> zero_offset_counts{0};     // SetZero software offset
    std::atomic<std::uint32_t> active_generation{0};     // gen RT adopted from the applied SetTarget (post-coalescing)
    std::atomic<std::uint32_t> completed_generation{0};  // gen that reached target; RT stores-release then notify (no lock)
    std::atomic<std::uint32_t> failed_generation{0};     // gen that failed (handshake timeout / cancelled); wakes its waiter to throw
};

// ServoController is the module's SlaveControl: it owns the RT policy and state, so it
// implements the Runner's hooks directly. The library Runner owns the RT thread, pacing,
// bring-up, and teardown; this wrapper owns the Master (persisting the resource lifetime) and
// lends it to a one-shot Runner (rt_runner_ is the last member, so it is destroyed first and
// ~Runner joins before the Master and state tear down).
class ServoController : public SlaveControl {
   public:
    // Validates config (no I/O); throws Error. start()/reconfigure() build the
    // SoemBackend-backed Master.
    explicit ServoController(ServoConfig config);

    ServoController(const ServoController&) = delete;
    ServoController& operator=(const ServoController&) = delete;
    ServoController(ServoController&&) = delete;
    ServoController& operator=(ServoController&&) = delete;
    ~ServoController();  // stop()

    // Non-RT lifecycle (exclusive api_mutex_). start() first preflights the realtime requirement
    // (require_realtime: can this process obtain SCHED_FIFO?) and throws rt_unavailable_message()
    // BEFORE any bus I/O -- on a non-realtime host the user sees the actionable config error at
    // load, not a drive error later. It then builds, inits, and configures the Master to SAFE-OP
    // (may throw Error), resolves field offsets once, and spawns the RT thread.
    //
    // start() succeeding means the RT thread is launched and scheduled, not that the drive is
    // operational or powered. The DC bring-up must be gapless, so OP is reached inside the RT
    // loop (configure() stops at SAFE-OP; it cannot reach OP without gapping the process-data
    // handoff, which would sync-fault the drive), and start() cannot block until OP. It throws synchronously
    // only on what it can guarantee up front: that the RT loop can run (realtime setup plus
    // require_realtime), via the started-promise. The DC bring-up outcome is observed asynchronously:
    //   - reached OP: is_operational() / is_powered() become true;
    //   - aborted: the no-sync fault held in the gate, so the RT loop surfaces the drive tier and
    //              RtError::NotOperational via last_error() and exits without auto-retry (repeated
    //              failed OP entries wedge some drives); recovery is an explicit reconfigure() or
    //              restart.
    // A command issued before OP and enabled degrades gracefully: await_move() blocks until the RT
    // loop exits or the drive faults, rather than acting on a non-operational drive.
    void start();
    // Teardown: set stopping_ = true and bump_wake to wake parked waiters, then request_stop and
    // join. Returns before any reset or destroy.
    void stop() noexcept;
    void reconfigure(ServoConfig config);  // stop() -> rebuild master_ from the factory -> start()

    // --- non-RT motor API (shared api_mutex_; go_to/go_for release before park) ---
    // After a bus loss (NIC down, cable pulled -- StopReason::BusFault), the motion verbs first
    // attempt an inline full rebuild (maybe_recover_bus): they throw a clear error while the bus
    // is still gone and succeed again once it is back, so a client retry loop IS the reconnect
    // policy. No background reconnect thread.
    void set_rpm(double rpm);                 // PV only (rejects in PP)
    void go_for(double rpm, double revs);     // PP: relative move; PV: timed run
    void go_to(double rpm, double position);  // PP only (rejects in PV)
    void halt() noexcept;
    // Set the software zero so the current actual reads `offset_revs` (default 0). A non-zero
    // offset reads config_ and takes the shared lock; offset 0 is the common stop/zero case.
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
    // Last published device velocity (0x606C from the wire when mapped, else the instantaneous
    // estimate). Raw device units; the module layer converts to Viam units. master_-free and
    // lock-free, symmetric with position_revs().
    std::int32_t velocity_counts() const noexcept;
    // RT loop heartbeat counter (cycles since start). Lock-free atomic read, master_-free. For
    // tests that need a cycle-based bound (e.g. assert a give-up happens within N loop cycles,
    // robust to the async loop's wall-clock rate under TSan/SCHED_OTHER).
    std::uint64_t loop_cycle() const noexcept;
    // The underlying Master, for single-port-owner SDO use only (an operator or tool doing ad-hoc
    // CoE before start() or after stop()). Not part of the master_-free accessor contract:
    // callers must not touch it concurrently with start()/stop()/reconfigure() (those reset it
    // under the exclusive lock), and during a running RT phase Master's own rt_active guard makes
    // SDO throw. nullptr before the first start(). Steady-state SDO uses sdo_read below, not this.
    Master* master_for_sdo() noexcept {
        return master_.get();
    }

    // Read a CoE object on this servo's slave while the RT loop runs. Generic raw-bytes surface;
    // unit conversion is the module layer's job (do_command). Takes the shared api_mutex_ so it
    // cannot race reconfigure() resetting master_; the wait is bounded by `timeout`, so a
    // mid-flight reconfigure blocks only that long (unlike a multi-second move, which releases the
    // lock). Throws Error if not started, SdoError on a CoE abort or timeout. Returns bytes read.
    std::size_t sdo_read(std::uint16_t index,
                         std::uint8_t sub,
                         std::span<std::byte> out,
                         std::chrono::milliseconds timeout = std::chrono::milliseconds(200));
    // Motor rated current (amps), for the module's per-mille to amps conversion. Read under the
    // shared lock (reconfigure() rewrites config_ under the exclusive lock).
    double rated_current_amps() const noexcept;

   protected:
    // --- Device seams. The base is the generic CiA402 servo driver; a device subclass
    // overrides these to add its vendor specifics. All three are consulted only
    // outside the constructor (start()/reconfigure()/RT loop), so a subclass override dispatches
    // normally (no virtual-during-construction trap). ---
    //
    // Vendor fault-reset SDO, run once pre-RT-spawn (single port owner). nullopt (base) means no
    // vendor reset: the standard CiA402 controlword bit-7 in-loop path is the only reset.
    // Workaround seam for drives whose faults clear only through a proprietary SDO, not bit 7.
    virtual std::optional<ethercat::SdoWrite> vendor_fault_reset_sdo() const {
        return std::nullopt;
    }
    // The drive's "SYNC0 not yet established" 0x603F code, fed to the DC bring-up gate and the
    // on_stop diagnostic. nullopt (base) means no sync-fault detection (the gate signal is always
    // false).
    virtual std::optional<std::uint16_t> sync_fault_code() const noexcept {
        return std::nullopt;
    }
    // 0x603F code to human label for last_error() (cold path). Empty (base) means bare hex, so the
    // line is never wrong, just less descriptive; a subclass glosses its vendor codes.
    virtual std::string fault_description(std::uint16_t code) const;
    // Move-complete seam (the driver owns is-moving/reached, not the sequencer). Given this cycle's
    // "actual is within tolerance of target", "the shaft is position-stable", and the statusword,
    // decide whether a PP move has reached. The generic base trusts the drive's statusword bit 10
    // (target-reached); a device subclass overrides to `pos_near_target && pos_stable` for drives
    // that tie bit 10 permanently high. RT-only (publish_state); position_stable() has already advanced
    // its ring this cycle, so an override composes the two booleans and must not call
    // position_stable() again.
    virtual bool reached_target(bool pos_near_target, bool pos_stable, Status status) noexcept {
        (void)pos_near_target;
        (void)pos_stable;
        return status.target_reached();
    }
    // Noise-robust "shaft is still" heuristic: stable once the last N cycles' position range is
    // within position_tolerance_counts. Protected so a subclass reach override can compose it. Advances
    // a ring, so call exactly once per cycle (publish_state does).
    bool position_stable(std::int32_t actual) noexcept;

   private:
    // --- lifecycle FSM (std::variant; each state's step() in the .cpp) ---
    struct Init {};
    struct Enabling {};
    struct Operational {};
    struct Resetting {};  // holds the fault-reset intent across the drive's clear-reflect latency
    struct Faulted {};
    struct Disabled {};
    using Lifecycle = std::variant<Init, Enabling, Operational, Resetting, Faulted, Disabled>;

    // The PP new-set-point handshake lives in the shared Cia402Sequencer: the wrapper delegates the
    // Operational healthy-path to sequencer_.step() and reads its handshake-idle and
    // handshake-timeout signals (completion gate / abort).

    // Controller-tier fault reasons (this tier only; the bus WkcFault lives in
    // state_.wkc_faulted). The RT thread only stores the enum (no string alloc, no mutex on the
    // hot path); last_error() composes the text non-RT.
    enum class RtError : std::uint8_t {
        None,
        HandshakeTimeout,
        NotOperational,
        FaultResetFailed,
        MotorStopped,   // an in-flight move cancelled by stop()/halt(); the waiter throws
        MotorDisabled,  // an in-flight move cancelled by disable() (operator de-energize); the waiter throws
        ModeMismatch,   // 0x6061 != commanded mode at SwitchedOn; refuse to energize
        RtSetupFailed,  // SCHED_FIFO denied on the RT thread (backstop; start() preflights and throws first)
    };

    // --- SlaveControl hooks (RT; the Runner owns the thread, pacing, bring-up, and teardown) ---
    // on_configured is a no-op (field-resolve and vendor reset stay in start(), pre-Runner-start,
    // single port owner). sync_faulted is the 0x603F == sync_fault_code bring-up gate. step drains,
    // snapshots, steps the lifecycle, and publishes. on_stop maps the Runner's StopReason to the
    // two-tier fault (bring-up abort / bus / clean).
    void on_configured(ConfigContext& cfg) override;
    bool sync_faulted(const CycleContext& ctx) const noexcept override;
    bool drive_present(const CycleContext& ctx) const noexcept override;  // statusword != 0 (live PDO)
    void on_operational(CycleContext& ctx) noexcept override;
    void step(CycleContext& ctx) noexcept override;
    void on_stop(StopReason reason) noexcept override;
    // Event-driven teardown early-out: true once the lifecycle-stop has de-energized the drive at
    // rest (statusword SwitchOnDisabled during the stopping window), so the Runner ends the
    // teardown window as soon as the controlled ramp completes rather than spinning the full cap.
    // RT-only read (same thread as step()).
    bool teardown_complete() const noexcept override {
        return stop_at_rest_;
    }

    // Reach OP with bounded retry (clear errors and rebuild the master between attempts); Degraded
    // after kMaxBringupAttempts. Shared by start()/reconfigure() (master_ built to SAFE-OP).
    void bring_up();
    // One bring-up attempt: start the RT pump and bounded-poll the async outcome. true = reached OP.
    bool attempt_bringup();
    // Reset per-run state and construct, attach, and start the one-shot Runner; on a start-time
    // failure go Degraded-but-alive, never rethrowing past here. Shared by start()/reconfigure().
    void spawn_runner();
    void reset_run_state();  // zero the per-run atomics + RT-only working state
    void resolve_fields();   // cache controlword/status/target/actual/velocity FieldLocations
    // Run the device fault-reset seam (vendor_fault_reset_sdo()) once pre-RT-spawn, while this
    // thread is still the single port owner (after Master::configure(), before the RT thread
    // spawns). Best-effort: a failed clear is logged, not fatal. nullopt seam means no-op.
    void run_vendor_fault_reset();
    // Inline bus recovery, called by the motion verbs BEFORE their shared lock: no-op unless
    // bus_lost_; otherwise (exclusive lock) tear down the dead run and attempt one full rebuild,
    // throwing a clear "will retry on the next motion call" Error while the bus stays gone.
    void maybe_recover_bus();
    bool rt_alive() const noexcept;  // !rt_exited_ && !faulted (event-based, no clock; master_-free)
    void bump_wake() noexcept;       // wake every parked await_move waiter (C++20 atomic notify)

    ServoConfig config_;

    // Resolved once at start(); indexed by the RT loop without a map find. Required fields are
    // plain FieldLocation (throwing resolve at start); optional fields are
    // std::optional<FieldLocation> (nullopt = not in the map; guard the per-cycle load/store).
    // The sequencer owns writing the PP/PV command fields (target/velocity/profile velocity).
    FieldLocation f_ctrlword_;
    FieldLocation f_statusword_;
    FieldLocation f_actual_;
    // Enable-ladder mode fields (optional). The module's own enable FSM (not the sequencer) climbs to
    // OperationEnabled, so it must itself (a) seed 0x6060 = commanded mode through the ladder when
    // 0x6060 is RxPDO-mapped (else a PDO-following drive enables in mode 0), and (b) enforce the
    // mode-echo gate when 0x6061 is TxPDO-mapped (refuse OperationEnabled if 0x6061 != commanded;
    // the fixed superset map does map 0x6061). Both nullopt means the respective step is inert.
    std::optional<FieldLocation> f_mode_wr_;    // 0x6060 mode-of-operation (RxPDO write); nullopt means SDO-set only
    std::optional<FieldLocation> f_mode_disp_;  // 0x6061 mode-display (TxPDO read); nullopt means no mode-echo gate
    // Enable-time mode-echo gate state (RT-only). Sticky once resolved so the drive does not
    // oscillate ReadyToSwitchOn <-> SwitchedOn: Pending until the drive is SwitchedOn with 0x6061
    // mapped, then Passed (0x6061 == commanded, allow OperationEnabled) or Failed (mismatch, latch
    // RtError::ModeMismatch and de-energize). Reset to Pending on Init->Enabling so a fresh
    // bring-up or fault-recovery re-checks.
    enum class ModeGate : std::uint8_t { Pending, Passed, Failed };
    ModeGate mode_gate_ = ModeGate::Pending;
    // TxPDO feedback fields, both optional (nullopt means not in the map):
    std::optional<FieldLocation> f_fault_code_;       // 0x603F U16 drive error code (last_error gloss)
    std::optional<FieldLocation> f_velocity_actual_;  // 0x606C S32 velocity-actual (wire velocity; else estimate)

    // The generic CiA402 motion sequencer (shared with the bench validation tool). The module's
    // Operational healthy-path (enable-hold, PP handshake, PV stream, Halt) delegates here; the
    // wrapper keeps the two-tier fault, fault-reset machine, completion generations, the driver
    // mode-switch, and is-moving/reached. Constructed with just the quick-stop decel value.
    Cia402Sequencer sequencer_;
    // 0x6085 readback from sequencer_.configure (0 = quick-stop not configured). Written once by the
    // RT thread in on_configured (pre-steady), read by the non-RT velocity guard, so atomic.
    std::atomic<std::uint32_t> qs_decel_echoed_{0};
    // Effective PV/PP velocity ceiling (counts/s) derived from the teardown window: the max the
    // echoed 0x6085 decel can ramp to 0 within (teardown_window - margin). 0 = no guard (quick-stop
    // not configured). Written once in on_configured (RT), read non-RT by set_rpm/go_to.
    std::atomic<std::int64_t> vel_budget_cps_{0};
    // RT teardown window in cycles (the Runner's stopping-window cap): sized so a decel>0
    // quick-stop ramp completes before close(); 2 for the opt-out coast. The velocity budget derives
    // from this same value, so window and budget share one source of truth.
    std::uint32_t teardown_window_cycles() const noexcept;
    // Clamp a commanded velocity (counts/s) to the stoppable-within-teardown-window budget. Applied
    // to the PV setpoint (0x60FF) and the PP move speed (0x6081). Inert when unconfigured.
    std::int32_t clamp_to_stop_budget(std::int32_t vel_cps) const noexcept;

    Cia402Fsm fsm_;
    ControllerState state_;
    std::atomic<RtError> rt_error_{RtError::None};

    // Non-RT-readable teardown signal (separate from the jthread stop_token): drives the
    // go_to/go_for wait predicate and the accessors' fail-safe.
    std::atomic<bool> stopping_{false};
    std::atomic<std::uint32_t> next_generation_{0};  // non-RT: assigns unique move ids
    // Single-in-flight slot: the generation of the active blocking move (go_to/go_for), or 0 for
    // free. A new blocking move claims it via a single CAS that reclaims a slot whose gen is already
    // terminal (completed/failed), so there is no check-then-claim race between two gRPC callers. The
    // waiter does not release it (the next claim reclaims it if terminal). Non-RT (API thread) owned.
    std::atomic<std::uint32_t> motion_slot_{0};
    // Event-based RT aliveness (no clock watchdog): rt_exited_ is set by the RT loop's on_stop() on
    // any exit; wake_seq_ is bumped and C++20-notified by every terminal transition, fault onset,
    // stop, and on_stop, and await_move blocks on it (no timeout, no condvar). Both lock-free.
    std::atomic<bool> rt_exited_{false};
    std::atomic<std::uint64_t> wake_seq_{0};
    // Bus-loss marker (RT-latched by on_stop(StopReason::BusFault), lock-free). While set, the
    // motion verbs route through maybe_recover_bus() -- teardown of the dead run + one inline
    // rebuild attempt per call. Cleared before a rebuild spawns (so a NEW bus fault during or
    // right after the rebuild re-latches it and is never lost) and re-set on a failed attempt.
    std::atomic<bool> bus_lost_{false};
    // Degraded-but-alive: set when start()/bring-up fails (RT-spawn, on_configured refusal, or
    // drive AL-reject) -- motion APIs throw degraded_reason_, accessors fail-safe, and the process
    // never crashes; reconfigure()/start() clear it on a clean retry. degraded_ (lock-free) gates
    // the accessors; degraded_reason_ is read and written under api_mutex_.
    std::atomic<bool> degraded_{false};
    std::string degraded_reason_;  // set (under api_mutex_) on a synchronous start failure; async RT failures use last_error()

    mutable std::shared_mutex api_mutex_;  // API=shared, lifecycle(start/stop/reconfigure)=exclusive

    // --- RT-only working state (single-thread; plain members, no atomics or locks). Touched
    // exclusively by the RT loop and the lifecycle step()s. ---
    std::uint16_t last_cw_ = 0;                 // for the fault-reset rising-edge re-arm
    std::uint32_t reset_cycles_remaining_ = 0;  // Resetting-window countdown, RT-only
    std::uint32_t clear_streak_ = 0;            // consecutive dev!=Fault cycles in Resetting (RT-only)
    std::int32_t target_counts_ = 0;            // latched PP target
    std::uint32_t profile_vel_ = 0;             // latched PP profile velocity
    std::int32_t pv_velocity_ = 0;              // latched PV target velocity
    std::int32_t prev_actual_ = 0;              // previous-cycle actual (instantaneous velocity estimate)
    bool first_cycle_ = true;                   // skip the velocity estimate on the first cycle
    // Noise-robust "stopped": a ring of the last N actual positions; stable when the window's range
    // (max-min) <= position_tolerance_counts. Sized in resolve_fields (~20ms at the loop rate). RT-only.
    std::vector<std::int32_t> pos_hist_;
    std::size_t pos_hist_idx_ = 0;
    std::uint32_t pos_hist_filled_ = 0;  // entries written so far (window not "full" until == pos_hist_.size())
    // (position_stable() is declared protected above so a subclass reach override can compose it.)
    // The Cia402 mode to command this cycle: the fixed config mode (PP/PV) or, for a switchable
    // config, the current switch_intent_. RT-only (reads switch_intent_).
    Cia402Mode commanded_cia402_mode() const noexcept;
    bool commanded_is_pp() const noexcept {
        return commanded_cia402_mode() == Cia402Mode::ProfilePosition;
    }
    bool halted_ = false;  // sticky Stop: Halt stays asserted until a new motion command
    // A PV motion-hold that holds zero velocity (bit 8) drifts under load, because the drive has
    // no position loop in PV. When the map is switch-capable (0x6060 and 0x607A both RxPDO-mapped),
    // a Halt of a PV move instead switches the drive to PP-at-current-counts (the driver mode-switch
    // below, then a PP setpoint at the position latched at the halt) so the drive's position loop
    // locks the shaft. pending_new_setpoint_ arms the sequencer's PP handshake for the hold without
    // touching active_generation (the halt already failed the in-flight move). On a switch give-up
    // the hold reverts to the interim PV-at-0 bit-8 hold (accept small drift, never de-energize).
    // switch_intent_ is the current motion intent (RT-only), always meaningful. The command batch
    // sets it (go_to/go_for -> PP, set_rpm -> PV). Default PP so the drive enables in PP.
    // commanded_cia402_mode() maps it to the Cia402Mode the sequencer commands this cycle.
    ControlMode switch_intent_ = ControlMode::ProfilePosition;
    bool pv_hold_capable_ = false;  // set at resolve: 0x6060 and 0x607A both mapped
    bool pv_hold_as_pp_ = false;    // sticky: currently holding a halted PV motor via PP-at-counts
    // The driver signals a new set-point explicitly. Set true when a new PP target is adopted (or a
    // PP hold begins); consumed on the next Operational sequencer step (a switch defers consumption --
    // run_mode_switch returns before the consume, keeping this pending).
    bool pending_new_setpoint_ = false;
    // Driver-owned runtime mode-switch: when the drive's confirmed 0x6061 mode differs from the
    // intent's Cia402 mode, the wrapper holds energized, brings the motor to rest (Stopping), commands
    // the target mode via the sequencer and awaits the 0x6061 echo (Settling), then runs the target mode's
    // motion body. Fail-safe give-up (never a throw). RT-only.
    enum class SwitchPhase : std::uint8_t { None, Stopping, Settling };
    SwitchPhase switch_phase_ = SwitchPhase::None;
    std::uint32_t switch_cycles_ = 0;  // stop-first / settle window counter
    bool at_rest_ = false;             // last publish_state's "stopped" verdict; run_mode_switch's stop-first gate reads it (1-cycle stale)
    bool stop_at_rest_ = false;        // RT-only: drive reached SwitchOnDisabled during the stopping window -> teardown early-out
    // Controller-error tier: one-shot latches (e.g. HandshakeTimeout) set by the FSM, cleared only
    // by an explicit fault_reset. The bus WkcFault tier is live (recomputed from master_->fault()
    // each cycle) and is not stored here, so a persistent bus fault correctly reappears after a
    // fault_reset.
    RtError latched_ctrl_error_ = RtError::None;
    mutable std::uint16_t last_sync_code_ = 0;  // 0x603F read in (const) sync_faulted (bring-up), consumed by on_stop(BringupAborted)

    // FSM helpers (RT-only). Defined in the .cpp. ctx is used for output writes and fault reads,
    // since the Runner is the sole Master toucher.
    std::uint16_t step_lifecycle(CycleContext& ctx, Status status, const CommandBatch& batch, std::int32_t actual) noexcept;
    // Driver-owned runtime mode-switch step (Stopping/Settling); returns the cw the sequencer wrote.
    std::uint16_t run_mode_switch(CycleContext& ctx, std::int32_t actual, Cia402Mode want) noexcept;
    // Safe give-up for a switch that couldn't confirm (motor won't stop / echo never arrives). No throw.
    void mode_switch_give_up(std::int8_t confirmed) noexcept;
    std::uint16_t fault_reset_with_rearm(Status status) noexcept;
    // Reads this cycle's owned input snapshot via ctx (0x603F, statusword, etc. all from the same
    // latched image the Runner copied in).
    void publish_state(CycleContext& ctx, Status status, std::int32_t actual, std::int32_t velocity) noexcept;
    // Abort the in-flight move: set both tiers -- latched_ctrl_error_ (and rt_error_ for
    // last_error) and failed_generation plus notify (to wake the go_to waiter promptly). Every FSM
    // path that fails the active move calls this.
    void abort_active_move(RtError reason) noexcept;
    // Park on generation `g`'s completion (no timeout: a C++20 atomic wait on wake_seq_,
    // lost-wakeup-immune via the seq re-check), then classify the wake and throw on stop/abort/fault.
    // Shared by go_to (absolute) and go_for (relative) so both get identical semantics.
    void await_move(std::uint32_t generation);
    // Single-in-flight slot (non-RT / API thread). gen_terminal: has this move reached a terminal
    // (completed or failed) state? try_claim_motion_slot: CAS the slot to `gen`, reclaiming it only
    // if free or holding a terminal gen, and returning false if a live blocking move owns it.
    bool gen_terminal(std::uint32_t gen) const noexcept;
    bool try_claim_motion_slot(std::uint32_t gen) noexcept;
    bool motion_slot_busy() const noexcept;  // a live (non-terminal) blocking move holds the slot
    // Submit a PV velocity setpoint (rpm to guarded device counts) without the slot check, for
    // set_rpm (after its own check) and go_for(PV) (which owns the slot for its whole timed run).
    void push_velocity(double rpm) noexcept;

    // commands_ is held by value, so it is never reset until the dtor (no stop-time
    // push-vs-destroy use-after-free). master_ is a unique_ptr rebuilt by reconfigure() after the
    // join (the RT thread is its only cyclic user). The Runner is the last member, destroyed and
    // joined first.
    CommandQueue commands_;
    std::unique_ptr<Master> master_;  // wrapper-owned: persists the resource lifetime; the Runner only borrows it
    Lifecycle lifecycle_{Init{}};

    // The one-shot library Runner borrows master_ and holds *this as its control. Last member, so
    // destroyed first: ~Runner bounded-joins the RT thread and runs master.close() to INIT before
    // master_/commands_/state tear down (the control must outlive the Runner).
    std::unique_ptr<Runner> rt_runner_;  // must be the last member
};

}  // namespace ethercat::servo
