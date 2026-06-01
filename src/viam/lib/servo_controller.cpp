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
constexpr std::uint64_t kNsPerSec = 1'000'000'000ULL;

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
    lifecycle_ = Init{};
    last_cw_ = 0;
    handshake_ = Handshake::Idle;
    prev_actual_ = 0;

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
    lifecycle_ = Init{};
    last_cw_ = 0;
    handshake_ = Handshake::Idle;
    prev_actual_ = 0;
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
    } else {
        f_velocity_ = master_->rx_field(s, kTargetVel, 0);
    }
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
                rt_error_.store(RtError::HandshakeTimeout, std::memory_order_relaxed);
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
                rt_error_.store(RtError::HandshakeTimeout, std::memory_order_relaxed);
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

    // Adopt a new PP target (generation rides in the command, post-coalescing).
    if (config_.mode == ControlMode::ProfilePosition && batch.set_target.has_value()) {
        const SetTarget& t = *batch.set_target;
        if (t.generation != state_.active_generation.load(std::memory_order_relaxed)) {
            target_counts_ = t.relative ? static_cast<std::int32_t>(actual + t.counts) : t.counts;
            profile_vel_ = t.profile_velocity;
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
            return fault_reset_with_rearm(status);
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
        } else if (f_velocity_.byte_width != 0) {
            store_le<std::int32_t>(master_->outputs(config_.slave_id).subspan(f_velocity_.byte_offset, 4), pv_velocity_);
        }
        if (batch.halt) {
            cw = ControlWord::with_halt(cw, true);  // Stop = Halt (bit8), NOT QuickStop
        }
        return cw;
    }
    if (std::holds_alternative<Faulted>(lifecycle_)) {
        if (batch.fault_reset) {
            lifecycle_ = Enabling{};
            return fault_reset_with_rearm(status);
        }
        return ControlWord::disable_voltage();
    }
    // Disabled
    if (batch.enable) {
        lifecycle_ = Enabling{};
    }
    return ControlWord::disable_voltage();
}

void ServoController::publish_state(Status status, std::int32_t actual, std::int32_t velocity) noexcept {
    state_.position_counts.store(actual, std::memory_order_relaxed);
    state_.velocity.store(velocity, std::memory_order_relaxed);

    const bool powered = status.operation_enabled();
    state_.powered.store(powered, std::memory_order_relaxed);

    const bool faulted = master_->fault() || status.fault() || rt_error_.load(std::memory_order_relaxed) != RtError::None;
    state_.faulted.store(faulted, std::memory_order_release);

    // Move-complete: |target - actual| <= tol && |vel| <= vthresh (NEVER bit10).
    const bool at_target =
        std::abs(actual - target_counts_) <= config_.position_tolerance_counts && std::abs(velocity) <= config_.velocity_threshold;
    const bool moving = powered && !at_target;
    state_.moving.store(moving, std::memory_order_relaxed);

    if (powered && at_target && handshake_ == Handshake::Idle) {
        const std::uint32_t g = state_.active_generation.load(std::memory_order_relaxed);
        if (state_.completed_generation.load(std::memory_order_relaxed) != g) {
            state_.completed_generation.store(g, std::memory_order_release);  // publish BEFORE notify
            completion_cv_.notify_all();                                      // no completion_mutex_ held
        }
    }

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

        const std::span<const std::byte> in = master_->input_image(slave);
        const Status status{load_le<std::uint16_t>(in.subspan(f_statusword_.byte_offset, 2))};
        const std::int32_t actual = load_le<std::int32_t>(in.subspan(f_actual_.byte_offset, 4));
        const std::int32_t velocity =
            static_cast<std::int32_t>(static_cast<std::int64_t>(actual - prev_actual_) * config_.target_loop_rate_hz);
        prev_actual_ = actual;

        const std::uint16_t cw = step_lifecycle(status, batch, actual);
        store_le<std::uint16_t>(master_->outputs(slave).subspan(f_ctrlword_.byte_offset, 2), cw);
        last_cw_ = cw;

        master_->process();
        publish_state(status, actual, velocity);

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

void ServoController::go_to(double rpm, double position) {
    std::uint32_t g = 0;
    {
        const std::shared_lock<std::shared_mutex> lk(api_mutex_);
        if (config_.mode != ControlMode::ProfilePosition) {
            throw ConfigError("go_to requires Profile Position (PP) mode; this servo is configured PV -- use set_rpm");
        }
        const std::int32_t counts = revs_to_counts(position, config_.counts_per_rev, config_.gear_ratio);
        const double clamped_rpm = clamp_rpm(rpm, config_.max_motor_speed_rpm);
        const std::int32_t prof = rpm_to_device_velocity(clamped_rpm, config_.counts_per_rev, config_.gear_ratio);
        g = next_generation_.fetch_add(1, std::memory_order_relaxed) + 1;
        (void)commands_.push(Command{SetTarget{counts, static_cast<std::uint32_t>(std::abs(prof)), false, g}});
    }  // release the shared lock BEFORE parking (so reconfigure isn't blocked for the whole move)

    // Bounded wait_for re-check loop (lost-wakeup-immune; RT never locks the CV).
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(30);
    constexpr auto slice = std::chrono::milliseconds(2);
    std::unique_lock<std::mutex> lk(completion_mutex_);
    auto done = [&] {
        return state_.completed_generation.load(std::memory_order_acquire) == g ||
               state_.active_generation.load(std::memory_order_acquire) > g ||
               state_.completed_generation.load(std::memory_order_acquire) > g || state_.faulted.load(std::memory_order_acquire) ||
               stopping_.load(std::memory_order_acquire) || watchdog_expired();
    };
    while (!done() && std::chrono::steady_clock::now() < deadline) {
        completion_cv_.wait_for(lk, slice);
    }
    if (stopping_.load(std::memory_order_acquire) || watchdog_expired()) {
        throw BusError("go_to: controller stopped / RT loop not alive");
    }
    if (state_.faulted.load(std::memory_order_acquire)) {
        throw BusError("go_to: drive faulted during the move");
    }
    // completed==g -> success; active>g||completed>g -> superseded (benign); timeout -> return.
}

void ServoController::go_for(double rpm, double revs) {
    if (config_.mode == ControlMode::ProfilePosition) {
        double rev_pos = 0.0;
        {
            const std::shared_lock<std::shared_mutex> lk(api_mutex_);
            const std::int32_t cur = state_.position_counts.load(std::memory_order_acquire);
            const std::int32_t delta = revs_to_counts(revs, config_.counts_per_rev, config_.gear_ratio);
            rev_pos = counts_to_revs(static_cast<std::int32_t>(cur + delta), config_.counts_per_rev, config_.gear_ratio);
        }  // release before delegating (shared_mutex is not recursive)
        go_to(rpm, rev_pos);
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
    halt();
}

void ServoController::halt() noexcept {
    const std::shared_lock<std::shared_mutex> lk(api_mutex_);
    (void)commands_.push(Command{Halt{}});
}

void ServoController::set_zero() noexcept {
    const std::shared_lock<std::shared_mutex> lk(api_mutex_);
    state_.zero_offset_counts.store(state_.position_counts.load(std::memory_order_acquire), std::memory_order_release);
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

void ServoController::set_last_error(std::string message) {
    const std::lock_guard<std::mutex> lk(error_mutex_);
    last_error_ = std::move(message);
}

std::string ServoController::last_error() const {
    const std::shared_lock<std::shared_mutex> api_lk(api_mutex_);
    std::string out;
    switch (rt_error_.load(std::memory_order_acquire)) {
        case RtError::WkcFault:
            out = "EtherCAT working-counter fault";
            break;
        case RtError::HandshakeTimeout:
            out = "Profile-Position set-point acknowledge timed out";
            break;
        case RtError::MoveStalled:
            out = "move stalled (no progress)";
            break;
        case RtError::NotOperational:
            out = "drive not operational";
            break;
        case RtError::None:
            break;
    }
    if (master_ && master_->fault()) {
        const std::string m = master_->last_error();
        if (!m.empty()) {
            out = out.empty() ? m : (out + "; " + m);
        }
    }
    const std::lock_guard<std::mutex> err_lk(error_mutex_);
    if (!last_error_.empty()) {
        out = out.empty() ? last_error_ : (out + "; " + last_error_);
    }
    return out;
}

}  // namespace ethercat::servo
