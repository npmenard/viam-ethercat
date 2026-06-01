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
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <shared_mutex>
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
    std::atomic<std::int32_t> position_counts{0};        // 0x6064 actual, before zero-offset
    std::atomic<std::int32_t> velocity{0};               // device velocity units
    std::atomic<bool> powered{false};                    // OperationEnabled this cycle
    std::atomic<bool> moving{false};                     // !move-complete
    std::atomic<bool> faulted{false};                    // master_->fault() || status.fault() || rt_error_!=None (RT-side master_ deref)
    std::atomic<std::int32_t> fault_wkc{0};              // WKC at a live bus fault (payload; published by rt_error_ release)
    std::atomic<std::int32_t> expected_wkc{0};           // constant after start(); for last_error() (lock-free, master_-free)
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
    void set_zero() noexcept;

    // --- non-RT accessors (master_-FREE: ControllerState atomics + stopping_) ---
    double position_revs() const noexcept;
    bool is_moving() const noexcept;   // moving && rt_alive() && !stopping_
    bool is_powered() const noexcept;  // powered && rt_alive() && !stopping_
    bool is_disconnected() const noexcept;
    std::string last_error() const;

   private:
    // --- lifecycle FSM (std::variant; each state's step() in the .cpp) ---
    struct Init {};
    struct Enabling {};
    struct Operational {};
    struct Faulted {};
    struct Disabled {};
    using Lifecycle = std::variant<Init, Enabling, Operational, Faulted, Disabled>;

    // PP new-set-point handshake sub-FSM (cycle-stepped, with a timeout).
    enum class Handshake : std::uint8_t { Idle, WriteTarget, AwaitAck, ClearBit4, AwaitAckClear };

    // RT-loop fault reasons. The RT thread only STORES the enum (no string alloc,
    // no mutex on the hot path); last_error() composes the human text non-RT.
    enum class RtError : std::uint8_t { None, WkcFault, HandshakeTimeout, MoveStalled, NotOperational };

    // The RT thread body (loop while !st.stop_requested()). `started` is fulfilled
    // after a clean prelude (or set to an InitError exception on RT-sched failure
    // && require_realtime) -> bounded start() handshake. The promise lives in the
    // thread (reconfigure-safe). On exit: leave outputs safe (Halt/disable) + a
    // final process(), then return so join() completes.
    void run_rt_loop(const std::stop_token& st, std::promise<void> started) noexcept;
    bool setup_realtime() const noexcept;    // mlockall + mallopt + SCHED_FIFO; false on RT-sched failure
    void resolve_fields();                   // cache controlword/status/target/actual/velocity FieldLocations
    bool rt_alive() const noexcept;          // !watchdog_expired() && !state_.faulted  (master_-FREE)
    bool watchdog_expired() const noexcept;  // (now - last_cycle_time_ns) > watchdog_ns

    ServoConfig config_;
    BackendFactory backend_factory_;

    // Resolved once at start(); indexed by the RT loop without a map find.
    FieldLocation f_ctrlword_;
    FieldLocation f_statusword_;
    FieldLocation f_target_;
    FieldLocation f_actual_;
    FieldLocation f_velocity_;

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
    std::int32_t target_counts_ = 0;         // latched PP target
    std::uint32_t profile_vel_ = 0;          // latched PP profile velocity
    std::int32_t pv_velocity_ = 0;           // latched PV target velocity
    std::int32_t last_progress_actual_ = 0;  // move no-progress watchdog
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
    void publish_state(Status status, std::int32_t actual, std::int32_t velocity) noexcept;

    // commands_ BY VALUE -> never reset until dtor (no stop-time push-vs-destroy
    // UAF). master_ unique_ptr -> rebuilt by reconfigure() AFTER join (RT thread
    // is its only cyclic user). rt_thread_ LAST -> destroyed/joined first.
    CommandQueue commands_;
    std::unique_ptr<Master> master_;
    Lifecycle lifecycle_{Init{}};

    std::jthread rt_thread_;  // MUST be last member
};

}  // namespace ethercat::servo
