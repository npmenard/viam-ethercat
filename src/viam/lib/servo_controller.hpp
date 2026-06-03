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
#include "ethercat/master.hpp"
#include "ethercat/pdo_cache.hpp"
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

class ServoController {
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
    // Master (may throw InitError), resolve field offsets ONCE, spawn the RT
    // thread; the promise/future handshake makes start() throw if the RT thread
    // can't get SCHED_FIFO and require_realtime.
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

   private:
    // --- lifecycle FSM (std::variant; each state's step() in the .cpp) ---
    struct Init {};
    struct Enabling {};
    struct Operational {};
    struct Resetting {};  // holds the fault-reset intent across the drive's clear-reflect latency (spec #18)
    struct Faulted {};
    struct Disabled {};
    using Lifecycle = std::variant<Init, Enabling, Operational, Resetting, Faulted, Disabled>;

    // PP new-set-point handshake sub-FSM (cycle-stepped, with a timeout).
    enum class Handshake : std::uint8_t { Idle, WriteTarget, AwaitAck, ClearBit4, AwaitAckClear };

    // CONTROLLER-tier fault reasons (the CTRL tier only -- spec #16 moved the BUS
    // WkcFault out to state_.wkc_faulted). The RT thread only STORES the enum (no
    // string alloc, no mutex on the hot path); last_error() composes the text non-RT.
    enum class RtError : std::uint8_t { None, HandshakeTimeout, MoveStalled, NotOperational, FaultResetFailed };

    // The RT thread body (loop while !st.stop_requested()). `started` is fulfilled
    // after a clean prelude (or set to an InitError exception on RT-sched failure
    // && require_realtime) -> bounded start() handshake. The promise lives in the
    // thread (reconfigure-safe). On exit: leave outputs safe (Halt/disable) + a
    // final process(), then return so join() completes.
    void run_rt_loop(const std::stop_token& st, std::promise<void> started) noexcept;
    bool setup_realtime() const noexcept;                // mlockall + mallopt + SCHED_FIFO; false on RT-sched failure
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
    FieldLocation f_profile_velocity_;  // 0x6081 PP move speed; byte_width==0 if not mapped (optional)
    // TxPDO feedback fields (spec #16). Both OPTIONAL (byte_width==0 => unmapped):
    FieldLocation f_fault_code_;       // 0x603F U16 drive error code (last_error gloss)
    FieldLocation f_velocity_actual_;  // 0x606C S32 velocity-actual (wire velocity; else estimate)

    Cia402Fsm fsm_;
    ControllerState state_;
    std::atomic<RtError> rt_error_{RtError::None};

    // Non-RT-readable teardown signal (separate from the jthread stop_token):
    // drives the go_to/go_for wait predicate + the accessors' fail-safe.
    std::atomic<bool> stopping_{false};
    std::atomic<std::uint32_t> next_generation_{0};  // non-RT: assigns unique move ids
    std::atomic<std::uint64_t> watchdog_ns_{0};      // RT-liveness window (set at start; config-free reads)

    mutable std::shared_mutex api_mutex_;  // API=shared, lifecycle(start/stop/reconfigure)=exclusive

    std::mutex completion_mutex_;  // go_to/go_for waiter side only; RT NEVER locks it
    std::condition_variable completion_cv_;

    // --- RT-ONLY working state (single-thread; plain members, no atomics/locks).
    // Touched exclusively by run_rt_loop() / the lifecycle step()s. ---
    std::uint16_t last_cw_ = 0;              // for the fault-reset rising-edge re-arm
    Handshake handshake_ = Handshake::Idle;  // PP set-point handshake sub-state
    std::uint32_t handshake_cycles_remaining_ = 0;
    std::uint32_t reset_cycles_remaining_ = 0;  // Resetting-window countdown, RT-only (spec #18)
    std::int32_t target_counts_ = 0;            // latched PP target
    std::uint32_t profile_vel_ = 0;             // latched PP profile velocity
    std::int32_t pv_velocity_ = 0;              // latched PV target velocity
    std::int32_t last_progress_actual_ = 0;     // move no-progress watchdog
    std::uint32_t stall_cycles_ = 0;
    std::int32_t prev_actual_ = 0;  // previous-cycle actual (instantaneous velocity estimate)
    bool first_cycle_ = true;       // skip the velocity estimate on the first cycle
    bool halted_ = false;           // STICKY Stop: Halt stays asserted until a new motion command
    // Controller-error tier: one-shot latches (HandshakeTimeout/MoveStalled) set by
    // the FSM, cleared ONLY by an explicit fault_reset. The bus WkcFault tier is
    // LIVE (recomputed from master_->fault() each cycle) and is NOT stored here, so a
    // persistent bus fault correctly reappears after a fault_reset.
    RtError latched_ctrl_error_ = RtError::None;

    // FSM helpers (RT-only). Defined in the .cpp.
    std::uint16_t step_lifecycle(Status status, const CommandBatch& batch, std::int32_t actual) noexcept;
    std::uint16_t step_handshake(std::uint16_t base_cw, Status status) noexcept;
    std::uint16_t fault_reset_with_rearm(Status status) noexcept;
    // `in` is THIS cycle's single input-image snapshot (spec #16 §10): 0x603F is
    // sliced from the SAME span as statusword/actual, so the code matches the fault
    // state it is reported with -- structurally, not by timing luck.
    void publish_state(Status status, std::int32_t actual, std::int32_t velocity, std::span<const std::byte> in) noexcept;
    // Abort the in-flight move: set BOTH tiers -- latched_ctrl_error_ (+rt_error_
    // for last_error) AND failed_generation+notify (to wake the go_to waiter
    // PROMPTLY). The invariant: every FSM path that fails the active move calls this.
    void abort_active_move(RtError reason) noexcept;
    // Park on generation `g`'s completion (bounded wait_for, lost-wakeup-immune)
    // then classify the wake and THROW on stop/abort/fault/timeout. Shared by
    // go_to (absolute) and go_for (relative) so both get identical semantics.
    void await_move(std::uint32_t generation, std::chrono::milliseconds timeout);

    // commands_ BY VALUE -> never reset until dtor (no stop-time push-vs-destroy
    // UAF). master_ unique_ptr -> rebuilt by reconfigure() AFTER join (RT thread
    // is its only cyclic user). rt_thread_ LAST -> destroyed/joined first.
    CommandQueue commands_;
    std::unique_ptr<Master> master_;
    Lifecycle lifecycle_{Init{}};

    std::jthread rt_thread_;  // MUST be last member
};

}  // namespace ethercat::servo
