#include "viam/lib/servo_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <utility>

#include <malloc.h>
#include <pthread.h>
#include <sched.h>
#include <sys/mman.h>

#include "ethercat/errors.hpp"
#include "ethercat/pdo_buffer.hpp"
#include "ethercat/soem_backend.hpp"
#include "viam/lib/motion_profile.hpp"

namespace ethercat::servo {

namespace {

constexpr std::uint16_t kCtrlword = 0x6040;
constexpr std::uint16_t kStatusword = 0x6041;
constexpr std::uint16_t kTargetPos = 0x607A;
constexpr std::uint16_t kActualPos = 0x6064;
constexpr std::uint16_t kTargetVel = 0x60FF;
constexpr std::uint16_t kProfileVel = 0x6081;  // PP move speed (carries the GoTo/GoFor rpm); optional in the map
constexpr std::uint16_t kFaultCode = 0x603F;   // drive error code (TxPDO, optional feedback)
constexpr std::uint16_t kVelActual = 0x606C;   // velocity actual value (TxPDO, optional feedback)
constexpr std::uint64_t kNsPerSec = 1'000'000'000ULL;

// 4-digit uppercase hex of a U16 (e.g. 0x8700 -> "8700"); for last_error()'s
// "drive fault 0x...." line. Cold path; no iostream/locale.
std::string to_hex16(std::uint16_t v) {
    static constexpr char kDigits[] = "0123456789ABCDEF";
    std::string s(4, '0');
    for (int i = 0; i < 4; ++i) {
        s[static_cast<std::size_t>(3 - i)] = kDigits[(v >> (4U * static_cast<unsigned>(i))) & 0xFU];
    }
    return s;
}

std::uint64_t monotonic_ns() noexcept {
    timespec ts{};
    (void)clock_gettime(CLOCK_MONOTONIC, &ts);
    return (static_cast<std::uint64_t>(ts.tv_sec) * kNsPerSec) + static_cast<std::uint64_t>(ts.tv_nsec);
}

Cia402Mode to_cia402_mode(ControlMode mode) noexcept {
    return mode == ControlMode::ProfileVelocity ? Cia402Mode::ProfileVelocity : Cia402Mode::ProfilePosition;
}

ServoConfig validated(ServoConfig config) {
    config.validate();
    return config;
}

MasterConfig build_master_config(const ServoConfig& c) {
    SlaveConfig slave;
    slave.slave_id = c.slave_id;
    slave.rxpdo = c.rxpdo;
    slave.txpdo = c.txpdo;
    slave.default_mode = to_cia402_mode(c.mode);

    MasterConfig mc;
    mc.ifname = c.ifname;
    mc.target_loop_rate_hz = c.target_loop_rate_hz;
    mc.slaves = {slave};
    mc.max_consecutive_wkc_errors = static_cast<std::uint32_t>(c.max_consecutive_wkc_errors);
    mc.use_distributed_clocks = c.use_distributed_clocks;
    // DC bring-up (bench-tuned defaults; only used when DC is on): phase-locking
    // warmup before OP + a post-OP settle grace while the phase finishes locking.
    constexpr std::uint32_t kDefaultDcLockCycles = 2000;
    constexpr std::uint32_t kDefaultDcSettleCycles = 1000;
    mc.dc_lock_cycles = c.use_distributed_clocks ? kDefaultDcLockCycles : 0;
    mc.dc_settle_cycles = c.use_distributed_clocks ? kDefaultDcSettleCycles : 0;
    return mc;
}

}  // namespace

ServoController::ServoController(ServoConfig config)
    : ServoController(std::move(config), [] { return std::unique_ptr<EcatBackend>(std::make_unique<SoemBackend>()); }) {}

ServoController::ServoController(ServoConfig config, BackendFactory backend_factory)
    : config_(validated(std::move(config))), backend_factory_(std::move(backend_factory)), commands_(config_.command_queue_capacity) {
    if (!backend_factory_) {
        throw ConfigError("ServoController: null backend factory");
    }
}

ServoController::~ServoController() {
    stop();
}

void ServoController::start() {
    const std::unique_lock<std::shared_mutex> lk(api_mutex_);

    // NOTE: memory for the DC PLL warmup is locked inside Master::configure() (right
    // before the warmup, MCL_CURRENT only) so it doesn't MCL_FUTURE-lock this thread
    // before the RT jthread spawns (which would EAGAIN the thread stack). The RT
    // thread's setup_realtime() does the full MCL_CURRENT|MCL_FUTURE for the loop.
    master_ = std::make_unique<Master>(build_master_config(config_), backend_factory_());
    master_->init();
    master_->configure();
    resolve_fields();

    // Reset per-run state.
    stopping_.store(false, std::memory_order_release);
    rt_error_.store(RtError::None, std::memory_order_relaxed);
    state_.faulted.store(false, std::memory_order_relaxed);
    state_.loop_cycle.store(0, std::memory_order_relaxed);
    state_.last_cycle_time_ns.store(0, std::memory_order_relaxed);
    state_.active_generation.store(0, std::memory_order_relaxed);
    state_.completed_generation.store(0, std::memory_order_relaxed);
    state_.failed_generation.store(0, std::memory_order_relaxed);
    next_generation_.store(0, std::memory_order_relaxed);
    state_.expected_wkc.store(master_->expected_wkc(), std::memory_order_relaxed);  // constant; read lock-free by last_error()
    lifecycle_ = Init{};
    last_cw_ = 0;
    handshake_ = Handshake::Idle;
    prev_actual_ = 0;
    first_cycle_ = true;
    halted_ = false;
    latched_ctrl_error_ = RtError::None;
    last_progress_actual_ = 0;
    stall_cycles_ = 0;

    const std::uint64_t period_ns = kNsPerSec / config_.target_loop_rate_hz;
    const std::uint64_t stall_ns = config_.stall_threshold_cycles * period_ns;
    watchdog_ns_.store(std::max<std::uint64_t>(stall_ns, 20'000'000ULL), std::memory_order_release);

    std::promise<void> started;
    std::future<void> ready = started.get_future();
    rt_thread_ =
        std::jthread([this](const std::stop_token& st, std::promise<void> p) { run_rt_loop(st, std::move(p)); }, std::move(started));

    // Bounded handshake: rethrows InitError if the RT thread couldn't get RT
    // scheduling (and require_realtime), or fails loudly if it never signals.
    if (ready.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {
        stopping_.store(true, std::memory_order_release);
        rt_thread_.request_stop();
        if (rt_thread_.joinable()) {
            rt_thread_.join();
        }
        throw InitError("ServoController: RT thread failed to start within 2s");
    }
    ready.get();  // rethrows the InitError set by setup_realtime() failure
}

void ServoController::stop() noexcept {
    const std::unique_lock<std::shared_mutex> lk(api_mutex_);
    stopping_.store(true, std::memory_order_release);
    completion_cv_.notify_all();  // wake any parked go_to/go_for waiters
    rt_thread_.request_stop();
    if (rt_thread_.joinable()) {
        rt_thread_.join();
    }
}

void ServoController::reconfigure(ServoConfig config) {
    ServoConfig next = validated(std::move(config));
    const std::unique_lock<std::shared_mutex> lk(api_mutex_);
    // Stop + join (RT thread gone) before touching master_.
    stopping_.store(true, std::memory_order_release);
    completion_cv_.notify_all();
    rt_thread_.request_stop();
    if (rt_thread_.joinable()) {
        rt_thread_.join();
    }
    master_.reset();  // safe: RT thread (its only cyclic user) is joined
    config_ = std::move(next);

    // Restart with the new config (same body as start(), lock already held).
    master_ = std::make_unique<Master>(build_master_config(config_), backend_factory_());
    master_->init();
    master_->configure();
    resolve_fields();
    stopping_.store(false, std::memory_order_release);
    rt_error_.store(RtError::None, std::memory_order_relaxed);
    state_.faulted.store(false, std::memory_order_relaxed);
    state_.loop_cycle.store(0, std::memory_order_relaxed);
    state_.last_cycle_time_ns.store(0, std::memory_order_relaxed);
    state_.active_generation.store(0, std::memory_order_relaxed);
    state_.completed_generation.store(0, std::memory_order_relaxed);
    state_.failed_generation.store(0, std::memory_order_relaxed);
    next_generation_.store(0, std::memory_order_relaxed);
    state_.expected_wkc.store(master_->expected_wkc(), std::memory_order_relaxed);  // constant; read lock-free by last_error()
    lifecycle_ = Init{};
    last_cw_ = 0;
    handshake_ = Handshake::Idle;
    prev_actual_ = 0;
    first_cycle_ = true;
    halted_ = false;
    latched_ctrl_error_ = RtError::None;
    last_progress_actual_ = 0;
    stall_cycles_ = 0;
    const std::uint64_t period_ns = kNsPerSec / config_.target_loop_rate_hz;
    watchdog_ns_.store(std::max<std::uint64_t>(config_.stall_threshold_cycles * period_ns, 20'000'000ULL), std::memory_order_release);
    std::promise<void> started;
    std::future<void> ready = started.get_future();
    rt_thread_ =
        std::jthread([this](const std::stop_token& st, std::promise<void> p) { run_rt_loop(st, std::move(p)); }, std::move(started));
    if (ready.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {
        stopping_.store(true, std::memory_order_release);
        rt_thread_.request_stop();
        if (rt_thread_.joinable()) {
            rt_thread_.join();
        }
        throw InitError("ServoController: RT thread failed to restart within 2s");
    }
    ready.get();
}

void ServoController::resolve_fields() {
    const std::uint16_t s = config_.slave_id;
    f_ctrlword_ = master_->rx_field(s, kCtrlword, 0);
    f_statusword_ = master_->tx_field(s, kStatusword, 0);
    f_actual_ = master_->tx_field(s, kActualPos, 0);
    if (config_.mode == ControlMode::ProfilePosition) {
        f_target_ = master_->rx_field(s, kTargetPos, 0);
        // Profile velocity (0x6081) is OPTIONAL in the map. If the drive maps it
        // (the A6 does), the RT loop must write the commanded speed there every
        // cycle -- else the move runs at the drive's default speed (rpm ignored).
        f_profile_velocity_ = rxpdo_has(kProfileVel) ? master_->rx_field(s, kProfileVel, 0) : FieldLocation{};
    } else {
        f_velocity_ = master_->rx_field(s, kTargetVel, 0);
    }
    // OPTIONAL TxPDO feedback (spec #16) -- both modes. byte_width==0 => unmapped, so
    // the RT loop falls back (velocity estimate) / omits the tier (fault code).
    f_fault_code_ = txpdo_has(kFaultCode) ? master_->tx_field(s, kFaultCode, 0) : FieldLocation{};
    f_velocity_actual_ = txpdo_has(kVelActual) ? master_->tx_field(s, kVelActual, 0) : FieldLocation{};
}

bool ServoController::rxpdo_has(std::uint16_t index) const noexcept {
    for (const auto& [pdo, entries] : config_.rxpdo.entries) {
        for (const PdoEntry& e : entries) {
            if (e.index == index) {
                return true;
            }
        }
    }
    return false;
}

bool ServoController::txpdo_has(std::uint16_t index) const noexcept {
    for (const auto& [pdo, entries] : config_.txpdo.entries) {
        for (const PdoEntry& e : entries) {
            if (e.index == index) {
                return true;
            }
        }
    }
    return false;
}

bool ServoController::setup_realtime() const noexcept {
    // These are process-global, called ONCE in the RT thread prelude -- the
    // concurrency-mt-unsafe lints are about global locale/env state, N/A here.
    // NOLINTBEGIN(concurrency-mt-unsafe)
    (void)mlockall(MCL_CURRENT | MCL_FUTURE);
    (void)mallopt(M_TRIM_THRESHOLD, -1);
    (void)mallopt(M_MMAP_MAX, 0);
    // NOLINTEND(concurrency-mt-unsafe)

    sched_param param{};
    param.sched_priority = config_.rt_priority;
    return pthread_setschedparam(pthread_self(), SCHED_FIFO, &param) == 0;
}

bool ServoController::watchdog_expired() const noexcept {
    const std::uint64_t last = state_.last_cycle_time_ns.load(std::memory_order_acquire);
    if (last == 0) {
        return true;  // never published yet
    }
    return (monotonic_ns() - last) > watchdog_ns_.load(std::memory_order_acquire);
}

bool ServoController::rt_alive() const noexcept {
    return !watchdog_expired() && !state_.faulted.load(std::memory_order_acquire);
}

std::uint16_t ServoController::fault_reset_with_rearm(Status status) noexcept {
    const std::uint16_t level = fsm_.step(status, Cia402State::OperationEnabled);  // 0x80 while Fault
    // Drive the RISING edge: if bit7 is asserted again while it was already
    // asserted last cycle, drop it for one cycle so the drive sees a fresh edge.
    if ((level & ControlWord::kFaultResetBit) != 0 && (last_cw_ & ControlWord::kFaultResetBit) != 0) {
        return static_cast<std::uint16_t>(level & ~ControlWord::kFaultResetBit);
    }
    return level;
}

void ServoController::abort_active_move(RtError reason) noexcept {
    latched_ctrl_error_ = reason;  // diagnostic tier (last_error)
    // Publish the reason BEFORE failed_generation so last_error() is consistent the
    // instant the waiter observes the abort (publish_state recomputes rt_error_
    // again this cycle -- idempotent for a controller error). NOT for WkcFault.
    rt_error_.store(reason, std::memory_order_release);
    const std::uint32_t g = state_.active_generation.load(std::memory_order_relaxed);
    if (g != 0 && state_.completed_generation.load(std::memory_order_relaxed) != g) {
        state_.failed_generation.store(g, std::memory_order_release);  // abort tier: wakes the go_to waiter
        completion_cv_.notify_all();                                   // RT never LOCKS completion_mutex_
    }
}

std::uint16_t ServoController::step_handshake(std::uint16_t base_cw, Status status) noexcept {
    const std::uint16_t s = config_.slave_id;
    switch (handshake_) {
        case Handshake::Idle:
            return base_cw;
        case Handshake::WriteTarget:
            store_le<std::int32_t>(master_->outputs(s).subspan(f_target_.byte_offset, 4), target_counts_);
            handshake_ = Handshake::AwaitAck;
            handshake_cycles_remaining_ = config_.handshake_timeout_cycles;
            return ControlWord::with_new_setpoint(base_cw, true);  // raise bit4
        case Handshake::AwaitAck:
            if (status.setpoint_acknowledged()) {
                handshake_ = Handshake::ClearBit4;
                return ControlWord::with_new_setpoint(base_cw, true);
            }
            if (handshake_cycles_remaining_ == 0) {
                abort_active_move(RtError::HandshakeTimeout);  // latch + wake the waiter PROMPTLY (correct reason)
                handshake_ = Handshake::Idle;
                return base_cw;  // drop bit4
            }
            --handshake_cycles_remaining_;
            return ControlWord::with_new_setpoint(base_cw, true);
        case Handshake::ClearBit4:
            handshake_ = Handshake::AwaitAckClear;
            handshake_cycles_remaining_ = config_.handshake_timeout_cycles;
            return base_cw;  // drop bit4
        case Handshake::AwaitAckClear:
            if (!status.setpoint_acknowledged()) {
                handshake_ = Handshake::Idle;
            } else if (handshake_cycles_remaining_ == 0) {
                abort_active_move(RtError::HandshakeTimeout);  // latch + wake the waiter PROMPTLY (correct reason)
                handshake_ = Handshake::Idle;
            } else {
                --handshake_cycles_remaining_;
            }
            return base_cw;
    }
    return base_cw;
}

std::uint16_t ServoController::step_lifecycle(Status status, const CommandBatch& batch, std::int32_t actual) noexcept {
    const Cia402State dev = status.decode();
    const bool bus_fault = master_->fault();

    // A new motion command (or enable) clears the sticky Halt.
    if (batch.set_target.has_value() || batch.set_velocity.has_value() || batch.enable) {
        halted_ = false;
    }
    if (batch.halt) {
        halted_ = true;  // STICKY: stays asserted across cycles until a new motion command
    }

    // Adopt a new PP target (generation rides in the command, post-coalescing).
    if (config_.mode == ControlMode::ProfilePosition && batch.set_target.has_value()) {
        const SetTarget& t = *batch.set_target;
        if (t.generation != state_.active_generation.load(std::memory_order_relaxed)) {
            target_counts_ = t.relative ? static_cast<std::int32_t>(actual + t.counts) : t.counts;
            profile_vel_ = t.profile_velocity;
            last_progress_actual_ = actual;
            stall_cycles_ = 0;
            latched_ctrl_error_ = RtError::None;  // a fresh move starts with a clean diagnostic slate
            state_.active_generation.store(t.generation, std::memory_order_release);
            handshake_ = Handshake::WriteTarget;  // restart the PP handshake for the new target
        }
    }
    if (config_.mode == ControlMode::ProfileVelocity && batch.set_velocity.has_value()) {
        pv_velocity_ = batch.set_velocity->velocity;
    }

    if (std::holds_alternative<Init>(lifecycle_)) {
        if (dev == Cia402State::Fault) {
            lifecycle_ = Faulted{};
        } else {
            lifecycle_ = Enabling{};
        }
        return ControlWord::disable_voltage();
    }
    if (std::holds_alternative<Enabling>(lifecycle_)) {
        if (dev == Cia402State::Fault) {
            lifecycle_ = Faulted{};
            return ControlWord::disable_voltage();  // latch the fault; reset is EXPLICIT (Faulted handles it)
        }
        if (dev == Cia402State::OperationEnabled) {
            lifecycle_ = Operational{};
        }
        return fsm_.step(status, Cia402State::OperationEnabled);
    }
    if (std::holds_alternative<Operational>(lifecycle_)) {
        if (dev == Cia402State::Fault || bus_fault) {
            lifecycle_ = Faulted{};
            return ControlWord::disable_voltage();
        }
        if (batch.disable) {
            lifecycle_ = Disabled{};
            return ControlWord::disable_voltage();
        }
        if (batch.quick_stop) {
            return ControlWord::quick_stop();
        }
        std::uint16_t cw = ControlWord::enable_operation();
        if (config_.mode == ControlMode::ProfilePosition) {
            cw = step_handshake(cw, status);
            // Write the commanded move speed to profile velocity (0x6081) every cycle
            // when it's mapped -- else the drive uses its default speed and the rpm
            // passed to go_to/go_for is silently ignored on hardware.
            if (f_profile_velocity_.byte_width != 0) {
                store_le<std::uint32_t>(master_->outputs(config_.slave_id).subspan(f_profile_velocity_.byte_offset, 4), profile_vel_);
            }
        } else if (f_velocity_.byte_width != 0) {
            store_le<std::int32_t>(master_->outputs(config_.slave_id).subspan(f_velocity_.byte_offset, 4), pv_velocity_);
        }
        if (halted_) {
            cw = ControlWord::with_halt(cw, true);  // Stop = Halt (bit8, sticky), NOT QuickStop
        }
        return cw;
    }
    if (std::holds_alternative<Resetting>(lifecycle_)) {
        // Operator override: a disable while resetting wins.
        if (batch.disable) {
            lifecycle_ = Disabled{};
            clear_streak_ = 0;
            return ControlWord::disable_voltage();
        }
        // DEBOUNCE the clear: a refault re-arms the streak, so a type-(c) momentary
        // clear-then-refault never confirms (it is NOT mistaken for success).
        if (dev != Cia402State::Fault) {
            ++clear_streak_;
        } else {
            clear_streak_ = 0;
        }
        // SUCCESS (type-a): the clear HELD for K consecutive cycles -> confirmed recovery.
        if (clear_streak_ >= config_.fault_reset_clear_confirm_cycles) {
            lifecycle_ = Enabling{};
            clear_streak_ = 0;
            return fsm_.step(status, Cia402State::OperationEnabled);  // hand off to the enable ladder
        }
        // Window remaining -> keep working. Decrement EVERY cycle (Fault OR confirming),
        // NOT only on Fault cycles, so a flickering drive's TOTAL dwell stays bounded by
        // the window regardless of flicker period. Present the reset edge ONLY while in
        // Fault; while confirming a clear return a NEUTRAL controlword (don't pulse bit7
        // at an already-clearing drive).
        if (reset_cycles_remaining_ > 0) {
            --reset_cycles_remaining_;
            return (dev == Cia402State::Fault) ? fault_reset_with_rearm(status) : ControlWord::disable_voltage();
        }
        // GIVE-UP (type-b never-cleared AND type-c never-confirmed): window expired
        // without a CONFIRMED clear. Revert to Faulted + diagnostic; do NOT re-enter
        // Resetting (no spin). Cleared by the NEXT operator fault_reset.
        latched_ctrl_error_ = RtError::FaultResetFailed;
        lifecycle_ = Faulted{};
        clear_streak_ = 0;
        return ControlWord::disable_voltage();
    }
    if (std::holds_alternative<Faulted>(lifecycle_)) {
        if (batch.fault_reset) {
            // Enter Resetting and HOLD the reset intent for the recovery window: the
            // command-queue fault_reset is a one-shot (consumed this cycle), so the
            // sub-state -- not the flag -- carries the intent across the drive's
            // clear-reflect latency. (A persistent bus WkcFault still reappears next
            // cycle via the live tier -- it needs reconfigure, not fault_reset.)
            latched_ctrl_error_ = RtError::None;  // clear the prior diagnostic (incl. a prior FaultResetFailed)
            lifecycle_ = Resetting{};
            reset_cycles_remaining_ = config_.fault_reset_window_cycles;
            clear_streak_ = 0;                      // start the type-c debounce fresh
            return fault_reset_with_rearm(status);  // present the first reset edge
        }
        return ControlWord::disable_voltage();
    }
    // Disabled
    if (batch.enable) {
        lifecycle_ = Enabling{};
    }
    return ControlWord::disable_voltage();
}

void ServoController::publish_state(Status status, std::int32_t actual, std::int32_t velocity, std::span<const std::byte> in) noexcept {
    state_.position_counts.store(actual, std::memory_order_relaxed);
    state_.velocity.store(velocity, std::memory_order_relaxed);

    const bool powered = status.operation_enabled();
    state_.powered.store(powered, std::memory_order_relaxed);

    const std::uint32_t g = state_.active_generation.load(std::memory_order_relaxed);
    const bool move_active = g != 0 && state_.completed_generation.load(std::memory_order_relaxed) != g &&
                             state_.failed_generation.load(std::memory_order_relaxed) != g;

    // Move-complete predicate: |target - actual| <= tol && |vel| <= vthresh (NEVER
    // bit10). Only meaningful in PP (move_active implies a go_to generation).
    const bool at_target =
        std::abs(actual - target_counts_) <= config_.position_tolerance_counts && std::abs(velocity) <= config_.velocity_threshold;

    // is_moving: PP = an active positioned move not yet at target; PV = the drive
    // is actually turning (|velocity| above the threshold). target_counts_ is
    // never assigned in PV, so the PP position predicate must NOT drive PV moving.
    const bool moving = (config_.mode == ControlMode::ProfilePosition) ? (powered && move_active && !at_target)
                                                                       : (powered && std::abs(velocity) > config_.velocity_threshold);
    state_.moving.store(moving, std::memory_order_relaxed);

    // PP generation protocol: completion + no-progress watchdog (PP-only via move_active).
    if (powered && move_active && at_target && handshake_ == Handshake::Idle) {
        state_.completed_generation.store(g, std::memory_order_release);  // publish BEFORE notify
        completion_cv_.notify_all();                                      // no completion_mutex_ held
    } else if (move_active) {
        // No-progress watchdog: if the actual isn't advancing toward target for
        // too long, fail the move (wakes its waiter to throw, instead of a silent
        // wait). Window = move_timeout_ms (or 4x the stall threshold).
        if (std::abs(actual - last_progress_actual_) <= config_.position_tolerance_counts) {
            ++stall_cycles_;
        } else {
            stall_cycles_ = 0;
            last_progress_actual_ = actual;
        }
        const std::uint64_t rate = config_.target_loop_rate_hz;
        const std::uint32_t limit = config_.move_timeout_ms != 0 ? static_cast<std::uint32_t>(config_.move_timeout_ms * rate / 1000ULL)
                                                                 : static_cast<std::uint32_t>(config_.stall_threshold_cycles * 4);
        if (stall_cycles_ > limit) {
            abort_active_move(RtError::MoveStalled);  // latch + wake the waiter (same invariant as the handshake timeout)
        }
    }

    // Per-tier fault publish (spec #16; AFTER the watchdog so a stall set this cycle
    // shows now). last_error() COMPOSES every active tier -- never picks one -- so a
    // both-true Er74 (drive 0x603F + bus WKC->0) reports root cause AND symptom. In
    // each tier the payload is relaxed-stored BEFORE the flag is release-stored, so a
    // master_-free reader never sees a true flag with a stale payload (cross-tier skew
    // is benign: fault state is quasi-static once latched).
    const bool bus_fault = master_->fault();
    // BUS tier: payload (the raw last-exchange WKC -- the actual bad value at fault)
    // then flag.
    state_.fault_wkc.store(master_->last_wkc(), std::memory_order_relaxed);
    state_.wkc_faulted.store(bus_fault, std::memory_order_release);
    // DRIVE tier: live-read 0x603F from THIS cycle's snapshot EVERY faulted cycle (not
    // edge-captured) so a code the drive latches a frame or two after it sets bit3 is
    // still picked up ("code pending" collapses to the rare hard-drop race only).
    const std::uint16_t drive_code = f_fault_code_.byte_width != 0 ? load_le<std::uint16_t>(in.subspan(f_fault_code_.byte_offset, 2)) : 0;
    state_.drive_fault_code.store(drive_code, std::memory_order_relaxed);
    state_.drive_faulted.store(status.fault(), std::memory_order_release);
    // CTRL tier: the published mirror of the latch (tracks abort, clears on fault_reset).
    rt_error_.store(latched_ctrl_error_, std::memory_order_release);
    // state_.faulted is the DRIVE/BUS gate ONLY (de-powers the motor + wakes the
    // waiter's fault branch). Controller move-errors (HandshakeTimeout/MoveStalled)
    // deliberately stay OUT: they fail the in-flight move (via failed_generation) but
    // must NOT de-power an otherwise-healthy drive.
    state_.faulted.store(bus_fault || status.fault(), std::memory_order_release);

    state_.last_cycle_time_ns.store(monotonic_ns(), std::memory_order_release);
    state_.loop_cycle.fetch_add(1, std::memory_order_relaxed);
}

void ServoController::run_rt_loop(const std::stop_token& st, std::promise<void> started) noexcept {
    if (!setup_realtime() && config_.require_realtime) {
        started.set_exception(std::make_exception_ptr(
            InitError("real-time scheduling unavailable (need CAP_SYS_NICE/RLIMIT_RTPRIO); set require_realtime=false "
                      "to run best-effort")));
        return;
    }
    started.set_value();

    const std::uint16_t slave = config_.slave_id;
    const std::uint64_t period_ns = kNsPerSec / config_.target_loop_rate_hz;
    std::uint64_t next = monotonic_ns() + period_ns;

    while (!st.stop_requested()) {
        const CommandBatch batch = commands_.drain();

        // ONE input-image snapshot per cycle (spec #16 §10): statusword/bit3, actual,
        // 0x603F, 0x606C all slice from THIS span -- never a second input_image() call
        // -- so the fault code matches the fault state it is reported with structurally.
        const std::span<const std::byte> in = master_->input_image(slave);
        const Status status{load_le<std::uint16_t>(in.subspan(f_statusword_.byte_offset, 2))};
        const std::int32_t actual = load_le<std::int32_t>(in.subspan(f_actual_.byte_offset, 4));
        if (first_cycle_) {
            prev_actual_ = actual;  // avoid a spurious huge velocity on cycle 0
            first_cycle_ = false;
        }
        // Velocity from the wire (0x606C) when mapped, else the instantaneous estimate
        // (actual-delta * loop rate). One branch; identical fallback when unmapped.
        const std::int32_t velocity =
            f_velocity_actual_.byte_width != 0
                ? load_le<std::int32_t>(in.subspan(f_velocity_actual_.byte_offset, 4))
                : static_cast<std::int32_t>(static_cast<std::int64_t>(actual - prev_actual_) * config_.target_loop_rate_hz);
        prev_actual_ = actual;

        const std::uint16_t cw = step_lifecycle(status, batch, actual);
        store_le<std::uint16_t>(master_->outputs(slave).subspan(f_ctrlword_.byte_offset, 2), cw);
        last_cw_ = cw;

        master_->process();
        publish_state(status, actual, velocity, in);

        next += period_ns;
        timespec deadline{};
        deadline.tv_sec = static_cast<std::time_t>(next / kNsPerSec);
        deadline.tv_nsec = static_cast<long>(next % kNsPerSec);
        (void)clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &deadline, nullptr);
    }

    // Leave the drive in a safe state and flush it out.
    if (master_) {
        store_le<std::uint16_t>(master_->outputs(slave).subspan(f_ctrlword_.byte_offset, 2), ControlWord::disable_voltage());
        master_->process();
    }
}

void ServoController::set_rpm(double rpm) {
    const std::shared_lock<std::shared_mutex> lk(api_mutex_);
    if (config_.mode != ControlMode::ProfileVelocity) {
        throw ConfigError("set_rpm requires Profile Velocity (PV) mode; this servo is configured PP -- use go_to/go_for");
    }
    const double clamped = clamp_rpm(rpm, config_.max_motor_speed_rpm);
    const std::int32_t dev = rpm_to_device_velocity(clamped, config_.counts_per_rev, config_.gear_ratio);
    (void)commands_.push(Command{SetVelocity{dev}});
}

void ServoController::await_move(std::uint32_t generation, std::chrono::milliseconds timeout) {
    const std::uint32_t g = generation;
    // Bounded wait_for re-check loop (lost-wakeup-immune; RT never locks the CV).
    // Predicate is master_-FREE (state_ atomics + stopping_ + watchdog).
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    constexpr auto slice = std::chrono::milliseconds(2);
    std::unique_lock<std::mutex> lk(completion_mutex_);
    auto done = [&] {
        return state_.completed_generation.load(std::memory_order_acquire) >= g ||
               state_.active_generation.load(std::memory_order_acquire) > g ||
               state_.failed_generation.load(std::memory_order_acquire) >= g || state_.faulted.load(std::memory_order_acquire) ||
               stopping_.load(std::memory_order_acquire) || watchdog_expired();
    };
    while (!done() && std::chrono::steady_clock::now() < deadline) {
        completion_cv_.wait_for(lk, slice);
    }
    lk.unlock();

    // Classify the wake (order matters: stop/dead first; then this move's own
    // abort -- handshake-timeout or stall, which set failed_generation but NOT
    // faulted, so check it BEFORE the generic drive-fault branch; last_error()
    // carries the precise reason enum; then a real drive/bus fault; then success).
    if (stopping_.load(std::memory_order_acquire) || watchdog_expired()) {
        throw BusError("move: controller stopped / RT loop not alive");
    }
    if (state_.failed_generation.load(std::memory_order_acquire) >= g && state_.completed_generation.load(std::memory_order_acquire) < g) {
        throw BusError("move aborted (" + last_error() + ")");
    }
    if (state_.faulted.load(std::memory_order_acquire)) {
        throw BusError("move: drive faulted during the move");
    }
    if (state_.completed_generation.load(std::memory_order_acquire) >= g || state_.active_generation.load(std::memory_order_acquire) > g) {
        return;  // completed (or superseded by a newer move -- benign)
    }
    throw BusError("move timed out");
}

void ServoController::go_to(double rpm, double position) {
    std::uint32_t g = 0;
    std::chrono::milliseconds move_timeout{0};
    {
        const std::shared_lock<std::shared_mutex> lk(api_mutex_);
        if (config_.mode != ControlMode::ProfilePosition) {
            throw ConfigError("go_to requires Profile Position (PP) mode; this servo is configured PV -- use set_rpm");
        }
        // ABSOLUTE target in the ZEROED frame: add zero_offset_counts to map the
        // user's zeroed position to the raw encoder frame, so go_to(X) lands where
        // position_revs()==X after reset_zero (get_position is zeroed too).
        const std::int32_t counts = static_cast<std::int32_t>(revs_to_counts(position, config_.counts_per_rev, config_.gear_ratio) +
                                                              state_.zero_offset_counts.load(std::memory_order_acquire));
        const double clamped_rpm = clamp_rpm(rpm, config_.max_motor_speed_rpm);
        const std::int32_t prof = rpm_to_device_velocity(clamped_rpm, config_.counts_per_rev, config_.gear_ratio);
        g = next_generation_.fetch_add(1, std::memory_order_relaxed) + 1;
        move_timeout = config_.move_timeout_ms != 0 ? std::chrono::milliseconds(config_.move_timeout_ms) : std::chrono::milliseconds(30000);
        (void)commands_.push(Command{SetTarget{counts, static_cast<std::uint32_t>(std::abs(prof)), false, g}});
    }  // release the shared lock BEFORE parking (so reconfigure isn't blocked for the whole move)

    await_move(g, move_timeout);
}

void ServoController::go_for(double rpm, double revs) {
    if (config_.mode == ControlMode::ProfilePosition) {
        std::uint32_t g = 0;
        std::chrono::milliseconds move_timeout{0};
        {
            const std::shared_lock<std::shared_mutex> lk(api_mutex_);
            // RELATIVE move (frame-agnostic): push SetTarget{relative=true} so the FSM
            // computes target = actual + delta. Do NOT route through go_to -- go_to now
            // adds zero_offset (absolute frame), which would double-shift a relative move.
            const std::int32_t delta = revs_to_counts(revs, config_.counts_per_rev, config_.gear_ratio);
            const double clamped_rpm = clamp_rpm(rpm, config_.max_motor_speed_rpm);
            const std::int32_t prof = rpm_to_device_velocity(clamped_rpm, config_.counts_per_rev, config_.gear_ratio);
            g = next_generation_.fetch_add(1, std::memory_order_relaxed) + 1;
            move_timeout =
                config_.move_timeout_ms != 0 ? std::chrono::milliseconds(config_.move_timeout_ms) : std::chrono::milliseconds(30000);
            (void)commands_.push(Command{SetTarget{delta, static_cast<std::uint32_t>(std::abs(prof)), true, g}});
        }
        await_move(g, move_timeout);
        return;
    }
    // PV: run at rpm for the time to cover `revs`, then halt.
    double duration_s = 0.0;
    {
        const std::shared_lock<std::shared_mutex> lk(api_mutex_);
        const double effective_rpm = clamp_rpm(rpm, config_.max_motor_speed_rpm);
        if (effective_rpm != 0.0) {
            duration_s = std::abs(revs / (effective_rpm / 60.0));
        }
    }
    set_rpm(rpm);
    if (duration_s > 0.0) {
        std::this_thread::sleep_for(std::chrono::duration<double>(duration_s));
    }
    {
        const std::shared_lock<std::shared_mutex> lk(api_mutex_);
        (void)commands_.push(Command{SetVelocity{0}});  // command zero velocity so the run actually stops
    }
    halt();
}

void ServoController::halt() noexcept {
    const std::shared_lock<std::shared_mutex> lk(api_mutex_);
    (void)commands_.push(Command{Halt{}});
}

void ServoController::request_fault_reset() noexcept {
    const std::shared_lock<std::shared_mutex> lk(api_mutex_);
    (void)commands_.push(Command{FaultReset{}});
}

void ServoController::enable() noexcept {
    const std::shared_lock<std::shared_mutex> lk(api_mutex_);
    (void)commands_.push(Command{Enable{}});
}

void ServoController::disable() noexcept {
    const std::shared_lock<std::shared_mutex> lk(api_mutex_);
    (void)commands_.push(Command{Disable{}});
}

void ServoController::set_zero(double offset_revs) noexcept {
    const std::int32_t cur = state_.position_counts.load(std::memory_order_acquire);
    if (offset_revs == 0.0) {
        // Common case: make the current actual read 0. Pure atomics, no lock.
        state_.zero_offset_counts.store(cur, std::memory_order_release);
        return;
    }
    // Make the current actual read offset_revs: zero = current - offset_in_counts.
    // Reads config_ conversion params -> shared lock (vs reconfigure's swap).
    const std::shared_lock<std::shared_mutex> lk(api_mutex_);
    const std::int32_t offset_counts = revs_to_counts(offset_revs, config_.counts_per_rev, config_.gear_ratio);
    state_.zero_offset_counts.store(static_cast<std::int32_t>(cur - offset_counts), std::memory_order_release);
}

double ServoController::position_revs() const noexcept {
    const std::shared_lock<std::shared_mutex> lk(api_mutex_);
    const std::int32_t pos =
        state_.position_counts.load(std::memory_order_acquire) - state_.zero_offset_counts.load(std::memory_order_acquire);
    return counts_to_revs(pos, config_.counts_per_rev, config_.gear_ratio);
}

bool ServoController::is_moving() const noexcept {
    return state_.moving.load(std::memory_order_acquire) && rt_alive() && !stopping_.load(std::memory_order_acquire);
}

bool ServoController::is_powered() const noexcept {
    return state_.powered.load(std::memory_order_acquire) && rt_alive() && !stopping_.load(std::memory_order_acquire);
}

bool ServoController::is_disconnected() const noexcept {
    return stopping_.load(std::memory_order_acquire) || watchdog_expired();
}

std::int32_t ServoController::velocity_counts() const noexcept {
    return state_.velocity.load(std::memory_order_relaxed);
}

std::string ServoController::fault_gloss(std::uint16_t code) const {
    // Config-data lookup (NOT a hardcoded A6 table): 0x603F code -> human label.
    // Unknown code -> empty, so last_error() shows just the bare hex. Cold path.
    for (const auto& [c, label] : config_.fault_code_labels) {
        if (c == code) {
            return label;
        }
    }
    return {};
}

std::string ServoController::last_error() const {
    // Cold-but-LOCK-FREE and master_-FREE (symmetric with is_powered/is_moving): read
    // the three published tier flags (acquire) + their payloads. COMPOSE every active
    // tier -- never pick one -- so a both-true Er74 reports root cause AND symptom.
    // (fault_gloss reads config_, taking the shared lock -- fine, this is non-RT.)
    std::string out;
    const auto append = [&out](const std::string& s) {
        if (!out.empty()) {
            out += "; ";
        }
        out += s;
    };

    // DRIVE (root cause) -- pair read: flag acquire, then code relaxed.
    if (state_.drive_faulted.load(std::memory_order_acquire)) {
        const std::uint16_t code = state_.drive_fault_code.load(std::memory_order_relaxed);
        if (code != 0) {
            const std::string gloss = fault_gloss(code);
            append("drive fault 0x" + to_hex16(code) + (gloss.empty() ? "" : " (" + gloss + ")"));
        } else {
            append("drive fault (code pending)");
        }
    }
    // BUS (symptom + recovery).
    if (state_.wkc_faulted.load(std::memory_order_acquire)) {
        append("EtherCAT working-counter fault: got " + std::to_string(state_.fault_wkc.load(std::memory_order_relaxed)) + ", expected " +
               std::to_string(state_.expected_wkc.load(std::memory_order_relaxed)) + " -- bus re-init required");
    }
    // CTRL (latched controller error).
    switch (rt_error_.load(std::memory_order_acquire)) {
        case RtError::HandshakeTimeout:
            append("Profile-Position set-point acknowledge timed out");
            break;
        case RtError::MoveStalled:
            append("move stalled (no progress)");
            break;
        case RtError::NotOperational:
            append("drive not operational");
            break;
        case RtError::FaultResetFailed:
            append("fault-reset ineffective -- cause persists");
            break;
        case RtError::None:
            break;
    }
    return out;
}

}  // namespace ethercat::servo
