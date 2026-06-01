#pragma once

// ============================================================================
// DRAFT skeleton for review (architect: FSM/loop structure + fail-safe;
// devils-advocate: spsc-single-popper teardown contract + boundary atomics).
// The RT-loop body + lifecycle FSM step()s live in the .cpp and are held for
// review before finalizing. Not yet wired into CMake.
// ============================================================================
//
// ServoController owns the real-time loop for ONE servo. SDK-free (src/viam/lib)
// so it is unit-testable on a SimBackend with no Viam SDK and no hardware. It
// owns: the EtherCAT Master, the CommandQueue, the RT thread, the resolved field
// offsets, the lifecycle FSM, and the published ControllerState atomics. The
// library left four behaviors to the driver; this is where they live:
//   (a) fault-reset RISING-EDGE re-arm (Cia402Fsm::step returns the level),
//   (b) last-good + staleness -> fail-safe is_powered()/is_moving(),
//   (c) move-complete predicate |target-actual|<=tol && |vel|<=vthresh (never
//       statusword bit10), and
//   (d) the std::variant lifecycle FSM (Init/Enabling/Operational/Faulted/
//       Disabled).

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <future>
#include <memory>
#include <mutex>
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
// loop is the sole writer.
struct ControllerState {
    std::atomic<std::int32_t> position_counts{0};  // 0x6064 actual, before zero-offset
    std::atomic<std::int32_t> velocity{0};         // device velocity units
    std::atomic<bool> powered{false};              // OperationEnabled this cycle
    std::atomic<bool> moving{false};               // !move-complete
    std::atomic<bool> faulted{false};
    std::atomic<std::uint64_t> loop_cycle{0};            // heartbeat: non-RT detects a dead loop
    std::atomic<std::uint64_t> last_cycle_time_ns{0};    // CLOCK_MONOTONIC ns at last iteration (rt_alive watchdog)
    std::atomic<std::int32_t> zero_offset_counts{0};     // SetZero software offset
    std::atomic<std::uint32_t> active_generation{0};     // gen the RT loop is executing (adopted from the applied SetTarget)
    std::atomic<std::uint32_t> completed_generation{0};  // gen that reached target; RT stores-release then notifies (no lock)
};

class ServoController {
   public:
    // Production: builds a SoemBackend from config.ifname. Validates config (no
    // I/O). Throws ConfigError on a bad config.
    explicit ServoController(ServoConfig config);
    // Test/DI: inject a backend (SimBackend in offline tests). Same validation.
    ServoController(ServoConfig config, std::unique_ptr<EcatBackend> backend);

    ServoController(const ServoController&) = delete;
    ServoController& operator=(const ServoController&) = delete;
    ServoController(ServoController&&) = delete;
    ServoController& operator=(ServoController&&) = delete;
    ~ServoController();  // == stop()

    // NON-RT lifecycle. start(): master.init()+configure() (may throw InitError),
    // resolve the field offsets ONCE, then spawn the RT thread (which may signal
    // back an RT-scheduling failure -> InitError if require_realtime).
    void start();
    // Teardown invariant (DA contract): set stop -> JOIN the RT thread (wait for
    // it to RETURN) -> THEN destroy/clear the CommandQueue + Master. A signalled-
    // but-unjoined RT thread is still pop'ping the spsc queue.
    void stop() noexcept;
    void reconfigure(ServoConfig config);  // stop() (join returns) -> swap -> start()

    // --- non-RT motor API (Phase 6's ServoMotor calls these) ---
    void set_rpm(double rpm);                 // PV only (rejects in PP)
    void go_for(double rpm, double revs);     // PP: relative move; PV: timed run
    void go_to(double rpm, double position);  // PP only (rejects in PV)
    void halt() noexcept;
    void set_zero() noexcept;

    double position_revs() const noexcept;
    bool is_moving() const noexcept;   // moving && rt_alive() (fail-safe)
    bool is_powered() const noexcept;  // powered && rt_alive() (fail-safe)
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

    // RT-loop fault reasons. The RT thread only ever STORES an enum (no string
    // alloc, no mutex on the hot path); last_error() composes the human text
    // non-RT.
    enum class RtError : std::uint8_t { None, WkcFault, HandshakeTimeout, MoveStalled, NotOperational };

    // The RT thread body (loop runs while !stop_). `started` is fulfilled after a
    // clean prelude (or set to an InitError exception if setup_realtime() fails &&
    // require_realtime), so start() gets a bounded handshake. Reconfigure-safe:
    // the promise lives in the thread, not as a member. On exit it leaves outputs
    // safe (Halt/disable) + a final process(), and the loop returns so join()
    // completes.
    void run_rt_loop(std::promise<void> started) noexcept;
    bool setup_realtime() noexcept;            // mlockall + mallopt + SCHED_FIFO; false on RT-sched failure
    void resolve_fields();                     // cache controlword/status/target/actual/velocity FieldLocations
    bool rt_alive() const noexcept;            // (now - last_cycle_time_ns) < watchdog && !master.fault() && snapshot.is_live()
    void set_last_error(std::string message);  // NON-RT only (start/reconfigure)

    ServoConfig config_;

    // Resolved once at start(); indexed by the RT loop without a map find.
    FieldLocation f_ctrlword_;
    FieldLocation f_statusword_;
    FieldLocation f_target_;
    FieldLocation f_actual_;
    FieldLocation f_velocity_;

    Cia402Fsm fsm_;
    ControllerState state_;
    std::atomic<RtError> rt_error_{RtError::None};

    // Single stop signal for BOTH the RT loop (`while (!stop_)`) and the API
    // waiters (in their wait_for predicate). stop() sets it, notify_all()s the
    // completion CV, then joins.
    std::atomic<bool> stop_{false};
    std::atomic<std::uint32_t> next_generation_{0};  // non-RT: assigns unique move ids

    mutable std::mutex error_mutex_;  // guards last_error_ (NON-RT only)
    std::string last_error_;

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

    // commands_ is held BY VALUE and is NEVER destroyed/reset until the
    // ServoController dtor (which runs after stop()/join) -- so there is no
    // stop()-time destroy-vs-push UAF (DA #5): a late non-RT push() always
    // targets a live queue, and after join() nothing pops. reconfigure() reuses
    // it (drains stale, single-threaded after join); its capacity is fixed at
    // construction. Declared before rt_thread_ so it outlives the RT thread.
    // master_ is a unique_ptr because reconfigure() rebuilds it AFTER join (the
    // RT thread is its only cyclic user, so a post-join reset is race-free).
    CommandQueue commands_;
    std::unique_ptr<Master> master_;
    Lifecycle lifecycle_{Init{}};

    // Plain std::thread (DA's call): the loop checks stop_; the dtor calls stop()
    // (idempotent: guarded join) before any member is destroyed. Declared LAST ->
    // destroyed FIRST -> already joined when commands_/master_ are torn down.
    std::thread rt_thread_;  // MUST be last member
};

}  // namespace ethercat::servo
