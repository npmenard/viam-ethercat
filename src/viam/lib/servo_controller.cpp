#include "viam/lib/servo_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <utility>

#include "ethercat/errors.hpp"
#include "ethercat/log.hpp"
#include "ethercat/pdo_buffer.hpp"
#include "ethercat/realtime.hpp"
#include "ethercat/soem_backend.hpp"
#include "ethercat/util.hpp"
#include "viam/lib/motion_mode.hpp"
#include "viam/lib/motion_profile.hpp"
#include "viam/lib/sdo_codec.hpp"

namespace ethercat::servo {

namespace {

// The drive has to reach Switch On Disabled before its parameter store will answer. One cycle of
// the enable ladder is all it takes; this is a bound, not an expectation.
constexpr int kStoreDisableWaitCycles = 500;  // x2 ms
// Waiting out the parameter store. The drive reports it as running by answering 0; the bench has
// shown a save still unfinished 20 s after the download was answered, so the patience is generous.
constexpr auto kStorePoll = std::chrono::milliseconds(100);
constexpr int kStorePolls = 300;  // 30 s
// Bring-up patience for a recovery attempt. A healthy drive reaches OP in under half a second.
constexpr std::uint32_t kRecoveryOpAwaitMs = 3000;

// A drive's "no-sync" fault code is a device seam (sync_fault_code(), nullopt in this generic
// base), not a constant here, so this core carries no vendor value. The bring-up gate reads it
// through the seam (nullopt means no detection).
// Controlled-stop watchdog headroom: the teardown window and velocity budget reserve this much
// time below the full window so the ramp finishes before close() (a slave's sync/SM watchdog is
// typically a few tens of ms).
constexpr double kStopWindowMarginS = 0.05;
// Bring-up retry: attempt OP up to this many times, clearing drive errors and rebuilding the master
// (INIT bounce) between attempts. Bounded, because a persistent fault must give up rather than hammer
// (repeated failed OP entries wedge some drives' EtherCAT interface until a power cycle).
// kBringupWaitCap is a defensive backstop above the
// Runner's own bring-up bound (120s), so the async-outcome poll never hangs if a signal is missed.
constexpr unsigned kMaxBringupAttempts = 5;
constexpr std::chrono::milliseconds kBringupWaitCap{130'000};

ServoConfig validated(ServoConfig config) {
    config.validate();
    // The PDO map is a fixed driver-defined superset (never config), so set it unconditionally.
    config.set_fixed_pdo_map();
    // reached/is_moving is a noise-robust position-delta check (see publish_state);
    // position_tolerance_counts is the "close enough and stable" band. Default it to
    // counts_per_rev/720 (0.5 deg) when unset (<= 0) so it scales with encoder resolution. An
    // exact-match predicate never completes under encoder noise. velocity_threshold stays an optional
    // override: > 0 is a velocity gate; 0 is the position-delta method (the default).
    if (config.position_tolerance_counts <= 0) {
        const auto half_degree = static_cast<std::int32_t>(std::lround(config.counts_per_rev / 720.0));
        config.position_tolerance_counts = std::max(half_degree, 1);  // floor of 1 for a tiny-count encoder
    }
    return config;
}

MasterConfig build_master_config(const ServoConfig& c) {
    SlaveConfig slave;
    slave.slave_id = c.slave_id;
    slave.rxpdo = c.rxpdo;
    slave.txpdo = c.txpdo;
    // Always switch-capable; seed the SDO default mode to PP. 0x6060 is RxPDO-mapped, so the
    // per-cycle PDO mode from the selected motion mode governs at runtime; this is only the
    // pre-cycling default.
    // No slave.fault_reset: the vendor reset is consumer-side (run_vendor_fault_reset, executed
    // pre-RT-spawn in start()/reconfigure()).
    slave.sync_cycle_granularity_ns = c.sync_cycle_granularity_ns;  // Master validates rate vs granularity up front

    MasterConfig mc;
    mc.ifname = c.ifname;
    mc.target_loop_rate_hz = c.target_loop_rate_hz;
    mc.slaves = {slave};
    mc.max_consecutive_wkc_errors = static_cast<std::uint32_t>(c.max_consecutive_wkc_errors);
    mc.use_distributed_clocks = c.use_distributed_clocks;
    // op_await_timeout_ms is not a config knob; MasterConfig's fixed 30s bring-up give-up patience applies.
    // Post-OP DC settle grace (cycles) while the SYNC0 phase finishes locking: suppress the
    // WKC-fault latch so a residual transient does not trip a spurious Error. The bring-up settle
    // bound uses MasterConfig's own default (dc_op_gate_cycles).
    constexpr std::uint32_t kDefaultDcSettleCycles = 1000;
    mc.dc_settle_cycles = c.use_distributed_clocks ? kDefaultDcSettleCycles : 0;
    return mc;
}

}  // namespace

std::string rt_unavailable_message(int priority) {
    return "realtime scheduling is not available on this machine: the module could not enable "
           "SCHED_FIFO (realtime priority " +
           std::to_string(priority) +
           ") for its EtherCAT cycle thread. EtherCAT servo control needs a steady 1 kHz cycle, so the "
           "host must allow realtime scheduling. To fix: run viam-server as root, or grant it realtime "
           "permission (systemd: LimitRTPRIO=99 in the viam-server unit; or an rtprio entry in "
           "/etc/security/limits.d). A PREEMPT_RT Linux kernel is strongly recommended for reliable "
           "motion. To evaluate WITHOUT realtime (development only -- motion timing is not guaranteed "
           "and the drive may fault), set \"require_realtime\": false in this motor's attributes.";
}

// The device fault-reset seam (vendor_fault_reset_sdo()) run once pre-RT-spawn, while this thread
// is still the single port owner (after Master::configure(), before the RT thread spawns). The
// vendor datum is carried by the device subclass, never config. Best-effort: a failed
// clear is logged, not fatal, and the bring-up gate still guards OP entry.
void ServoController::run_vendor_fault_reset() {
    const std::optional<SdoWrite> reset = vendor_fault_reset_sdo();
    if (!reset.has_value()) {
        return;
    }
    try {
        master_->sdo_write(config_.slave_id, reset->index, reset->subindex, reset->data);
    } catch (const Error& e) {
        ETHERCAT_LOG_WARN("servo",
                          "slave {}: vendor fault-reset SDO {}:{:02X} failed (continuing): {}",
                          config_.slave_id,
                          hex(reset->index),
                          reset->subindex,
                          e.what());
    }
}

ServoController::ServoController(ServoConfig config) : config_(validated(std::move(config))), commands_(config_.command_queue_capacity) {}

ServoController::~ServoController() {
    stop();
}

void ServoController::start() {
    const std::unique_lock<std::shared_mutex> lk(api_mutex_);

    // Realtime preflight, BEFORE any bus I/O: probe whether this process can obtain SCHED_FIFO at
    // the configured priority. Most hosts a Viam module lands on are NOT set up for realtime, and
    // the failure must be a clear, actionable config error at load -- not a misleading "drive not
    // operational" minutes later after the NIC was opened and bring-up ran. The probe runs on a
    // scratch thread (no process-wide side effects); the RT thread's realtime::setup() remains the
    // authoritative backstop (privileges could change between the probe and the spawn).
    log_start_summary("start");
    if (config_.require_realtime && !realtime::sched_fifo_available(config_.rt_priority)) {
        throw Error(rt_unavailable_message(config_.rt_priority));
    }

    // configure() reaches SAFE-OP and does no memory lock (residency is RT-setup's job, not
    // thread-free bus policy). The RT thread then runs the DC bring-up prelude (settle, request OP,
    // await OP) to Operational; realtime::setup() does the full MCL_CURRENT|MCL_FUTURE in-thread,
    // post-spawn. An in-thread, post-spawn MCL_FUTURE never sees this thread's later jthread stack
    // alloc, and nothing cyclic runs before that in-thread lock, so no SYNC0-critical page-fault
    // window opens.
    master_ = std::make_unique<Master>(build_master_config(config_));
    master_->init();
    master_->configure();  // -> SAFE-OP (may throw Error; the SDK retries)
    bring_up();            // resolve, clear errors, and reach OP, with bounded retry
    recovery_ = std::jthread([this](const std::stop_token& stop) { recovery_loop(stop); });
}

// One line naming the effective config a run starts from.
void ServoController::log_start_summary(const char* what) const {
    ETHERCAT_LOG_INFO("servo",
                      "{}: interface '{}' slave {} loop {} Hz DC {} sync granularity {} ns require_realtime {} rt_priority {} "
                      "counts/rev {} gear {} max_rpm {} quick_stop_decel {} log_level {}",
                      what,
                      config_.ifname,
                      config_.slave_id,
                      config_.target_loop_rate_hz,
                      config_.use_distributed_clocks ? "on" : "off",
                      config_.sync_cycle_granularity_ns,
                      config_.require_realtime,
                      config_.rt_priority,
                      config_.counts_per_rev,
                      config_.gear_ratio,
                      config_.max_motor_speed_rpm,
                      config_.quick_stop_decel,
                      log::to_string(log::level()));
}

// Zero the per-run published atomics + RT-only working state (shared by start()/reconfigure()).
void ServoController::reset_run_state() {
    stopping_.store(false, std::memory_order_release);
    rt_error_.store(RtError::None, std::memory_order_relaxed);
    state_.faulted.store(false, std::memory_order_relaxed);
    state_.drive_faulted.store(false, std::memory_order_relaxed);  // clear a prior bring-up drive tier on restart
    state_.bringup_al_code.store(0, std::memory_order_relaxed);    // clear a prior AL-refusal code on restart
    state_.loop_cycle.store(0, std::memory_order_relaxed);
    state_.active_generation.store(0, std::memory_order_relaxed);
    state_.completed_generation.store(0, std::memory_order_relaxed);
    state_.failed_generation.store(0, std::memory_order_relaxed);
    next_generation_.store(0, std::memory_order_relaxed);
    motion_slot_.store(0, std::memory_order_relaxed);                               // free the single-in-flight slot on (re)start
    state_.expected_wkc.store(master_->expected_wkc(), std::memory_order_relaxed);  // constant; read lock-free by last_error()
    lifecycle_ = Init{};
    state_.homing_state.store(static_cast<std::uint8_t>(HomingState::Idle), std::memory_order_relaxed);
    state_.homed.store(false, std::memory_order_relaxed);  // an incremental reference does not survive a restart
    state_.home_counts.store(0, std::memory_order_relaxed);
    last_cw_ = 0;
    if (motion_) {
        motion_->reset();  // the mode's per-run RT state (handshake/latches/generator) for reuse
    }
    prev_actual_ = 0;
    first_cycle_ = true;
    stop_at_rest_ = false;
    at_rest_ = false;
    latched_ctrl_error_ = RtError::None;
    last_sync_code_ = 0;
    rt_exited_.store(false, std::memory_order_release);  // event-based RT aliveness: fresh run, loop is live
}

// The Runner's stopping-window cap (cycles): sized so a quick-stop ramp completes before
// close()->INIT de-energizes (no torque-cut at speed). One source of truth with the velocity budget.
std::uint32_t ServoController::teardown_window_cycles() const noexcept {
    // A mode that owns its own controlled stop sizes the window itself; the quick-stop logic below is the floor.
    const std::uint32_t mode_cycles = motion_ ? motion_->shutdown_cycles() : 0;
    // Opt-out (no drive-side controlled stop): disable-voltage coast is instant, so a 2-cycle window
    // unless the mode needs more.
    if (config_.quick_stop_decel == 0) {
        return std::max<std::uint32_t>(2, mode_cycles);
    }
    // decel>0: size the window to the controlled-stop budget so the quick-stop ramp completes before
    // close()->INIT (no torque-cut). window_cycles = controlled_stop_window_ms x loop_rate. The event
    // gate (teardown_complete) exits earlier once at rest; this is the hard cap.
    const std::uint64_t rate = config_.target_loop_rate_hz;
    const std::uint64_t cyc = (static_cast<std::uint64_t>(config_.controlled_stop_window_ms) * rate + 999ULL) / 1000ULL;
    return std::max<std::uint32_t>(static_cast<std::uint32_t>(std::max<std::uint64_t>(cyc, 2ULL)), mode_cycles);
}

void ServoController::spawn_runner() {
    reset_run_state();
    degraded_.store(false, std::memory_order_release);
    degraded_reason_.clear();
    RunnerConfig rc;
    rc.rt_priority = config_.rt_priority;
    rc.require_realtime = config_.require_realtime;
    // The sync-fault OP-entry gate (bringup_step -> Aborted) decides a failed bring-up, not this wall
    // bound; keep it well above Master's own op-await window (the pump backstop).
    rc.bringup_timeout = std::chrono::milliseconds(120'000);
    // Teardown window: for a controlled quick-stop (quick_stop_decel>0) size it to the controlled-stop
    // window so the ramp reaches rest before master.close()->INIT de-energizes (no torque-cut at
    // speed); the event gate (teardown_complete) exits as soon as the drive is at rest, so an
    // already-stopped case does not pay the full window. Opt-out coast = 2 cycles.
    rc.teardown_cycles = teardown_window_cycles();
    rt_runner_ = std::make_unique<Runner>(*master_, rc);
    try {
        rt_runner_->attach(config_.slave_id, *this);
        rt_runner_->start();  // on_configured (no-op), then spawn the RT thread
    } catch (const Error& e) {
        // Refusal or attach failure at start -> Degraded; drop the un-started Runner.
        degraded_reason_ = e.what();
        degraded_.store(true, std::memory_order_release);
        rt_runner_.reset();  // ~Runner: never started, so no join/close, just frees
    }
}

// Bring the drive to Operational with bounded retry. Drive errors are cleared before each attempt
// (run_vendor_fault_reset). A failed attempt -- bring-up aborted by intermittent enumeration,
// mailbox-not-ready, or a transient sync miss -- is recovered by a full master rebuild (INIT bounce,
// PRE-OP settle, DC re-arm), requesting OP exactly once per attempt rather than an OP re-request
// hammer (which wedges some drives). After kMaxBringupAttempts the drive stays Degraded with the AL/fault
// surfaced by on_stop. The caller holds the exclusive api_mutex_ and has already built and configured
// master_ (SAFE-OP) for the first attempt.
void ServoController::bring_up() {
    for (unsigned attempt = 1;; ++attempt) {
        select_motion_mode();
        resolve_fields();
        run_vendor_fault_reset();  // clear drive errors before bring-up (device seam; single port owner)
        if (attempt_bringup()) {
            return;  // reached OP
        }
        if (!degraded_reason_.empty()) {
            // A configure-time refusal (control mode, quick-stop objects) is not transient: fail start()/reconfigure() with it.
            const std::string why = degraded_reason_;
            rt_runner_.reset();
            throw Error(why);
        }
        if (attempt >= kMaxBringupAttempts) {
            ETHERCAT_LOG_ERROR("servo",
                               "slave {}: bring-up FAILED after {} attempts -- giving up (Degraded): {}",
                               config_.slave_id,
                               attempt,
                               last_error());
            return;  // Degraded: degraded_ and the AL/fault tier are already set by on_stop(BringupAborted)
        }
        ETHERCAT_LOG_WARN("servo", "slave {}: bring-up attempt {} failed -- clearing errors + retrying", config_.slave_id, attempt);
        // Full recovery for the next attempt: drop the Runner (join the exited RT thread, close()->INIT)
        // and master, then rebuild (INIT bounce, PRE-OP settle, DC re-arm). Requests OP once next attempt.
        rt_runner_.reset();
        master_.reset();
        master_ = std::make_unique<Master>(build_master_config(config_));
        master_->init();
        master_->configure();  // -> SAFE-OP
    }
}

// Start the RT bring-up and bounded-poll its async outcome. true = reached OP (Runner phase
// Running); false = bring-up aborted (on_stop set rt_exited_, or the attach-refusal set degraded_).
bool ServoController::attempt_bringup() {
    spawn_runner();  // reset_run_state (clears rt_exited_), then start the RT thread's DC bring-up pump
    if (degraded_.load(std::memory_order_acquire)) {
        return false;  // Runner attach/start refusal: no RT thread, immediate fail
    }
    // The Runner bounds bring-up itself (op-await + bringup_timeout); poll for EITHER outcome. The cap is
    // a defensive backstop above that bound, so this never hangs if a signal is somehow missed.
    const auto deadline = std::chrono::steady_clock::now() + kBringupWaitCap;
    while (std::chrono::steady_clock::now() < deadline) {
        if (rt_runner_ != nullptr && rt_runner_->status().phase == RunnerPhase::Running) {
            return true;  // OP reached -- steady loop running
        }
        if (rt_exited_.load(std::memory_order_acquire) || degraded_.load(std::memory_order_acquire)) {
            return false;  // bring-up aborted
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return rt_runner_ != nullptr && rt_runner_->status().phase == RunnerPhase::Running;
}

void ServoController::stop() noexcept {
    // Before the lock: a retry in flight holds it, and asking it to finish while holding the lock
    // ourselves would deadlock. A healthy bus leaves the thread sleeping, so this returns at once.
    recovery_.request_stop();
    if (recovery_.joinable()) {
        recovery_.join();
    }
    const std::unique_lock<std::shared_mutex> lk(api_mutex_);
    stopping_.store(true, std::memory_order_release);
    bump_wake();  // wake any parked go_to/go_for waiters
    // Drop the Runner: ~Runner runs the bounded teardown (join the RT thread, then master.close()->INIT).
    // A wedged step() fail-stops the process, not an unbounded hang.
    rt_runner_.reset();
}

void ServoController::reconfigure(ServoConfig config) {
    ServoConfig next = validated(std::move(config));
    const std::unique_lock<std::shared_mutex> lk(api_mutex_);
    // Drop the Runner first (its ~Runner joins the RT thread and runs close()->INIT) before touching
    // master_; the RT thread is master_'s only cyclic user, so this is the join barrier.
    stopping_.store(true, std::memory_order_release);
    bump_wake();
    rt_runner_.reset();
    master_.reset();  // safe: Runner (master_'s only cyclic user) is destroyed
    // A reconfigure IS a full rebuild: consume any pending bus-loss marker so it cannot tear
    // down the fresh run on the next motion call. A new fault re-latches it as usual.
    bus_lost_.store(false, std::memory_order_release);
    config_ = std::move(next);

    // Restart with the new config (same body as start(), lock already held).
    log_start_summary("reconfigure");
    if (config_.require_realtime && !realtime::sched_fifo_available(config_.rt_priority)) {
        throw Error(rt_unavailable_message(config_.rt_priority));
    }
    master_ = std::make_unique<Master>(build_master_config(config_));
    master_->init();
    master_->configure();
    bring_up();                   // resolve, clear errors, and reach OP, with bounded retry
    if (!recovery_.joinable()) {  // a reconfigure keeps the retry thread it was started with
        recovery_ = std::jthread([this](const std::stop_token& stop) { recovery_loop(stop); });
    }
}

void ServoController::resolve_fields() {
    // Resolve each RT field once here (non-RT, at start) to a cached offset; the RT loop then
    // reads/writes at the cached byte_offset with no per-cycle resolve, throw, or map-walk. All
    // resolves go through the TYPED API: a missing required object is a loud configure error, a
    // missing optional one is nullopt, and every hit is width-checked against the Field's type.
    const std::uint16_t s = config_.slave_id;
    f_ctrlword_ = master_->resolve_rx<cia402::ControlWord>(s);
    f_statusword_ = master_->resolve_tx<cia402::Statusword>(s);
    f_actual_ = master_->resolve_tx<cia402::PositionActual>(s);
    // One cycle of travel cannot exceed the speed ceiling; x8 leaves room for a drive that overshoots
    // it before its own limits bite, so only a frame re-base clears the bar. Reset the running total:
    // a bring-up re-reads the drive's position, which is where the encoder frame is established.
    const double ceiling_cps =
        std::abs(static_cast<double>(rpm_to_device_velocity(config_.max_motor_speed_rpm, config_.counts_per_rev, config_.gear_ratio)));
    const double per_cycle = ceiling_cps / static_cast<double>(config_.target_loop_rate_hz);  // validate(): 1..1000
    rebase_threshold_ = static_cast<std::int64_t>(per_cycle * 8.0) + 1000;
    frame_rebased_this_session_ = false;
    // The PP/PV command objects (0x607A/0x6081/0x60FF) are the sequencer's to resolve and write.
    // Optional TxPDO feedback, both modes. nullopt means not in the map, so the RT loop falls back
    // (velocity estimate) or omits the tier (fault code).
    f_fault_code_ = master_->try_resolve_tx<cia402::FaultCode>(s);
    f_velocity_actual_ = master_->try_resolve_tx<cia402::VelocityActual>(s);
    // Enable-ladder mode fields: 0x6060 (write, seed the mode through the ladder) is present only in a
    // PDO-mapped-0x6060 map; 0x6061 (read, the mode-echo gate) is present in the fixed superset map. Both
    // optional, so nullopt makes the respective enable-ladder step inert.
    f_mode_wr_ = master_->try_resolve_rx<cia402::ModeOfOperation>(s);
    f_mode_disp_ = master_->try_resolve_tx<cia402::ModeDisplay>(s);
    resolve_device_fields(*master_, s);  // the subclass's own objects (device seam)
    ETHERCAT_LOG_DEBUG("servo",
                       "slave {}: resolved fields: ctrlword@{} statusword@{} actual@{} fault_code@{} velocity@{} mode_wr@{} mode_disp@{}"
                       " mode {}",
                       s,
                       f_ctrlword_.byte_offset,
                       f_statusword_.byte_offset,
                       f_actual_.byte_offset,
                       f_fault_code_ ? static_cast<long>(f_fault_code_->byte_offset) : -1L,
                       f_velocity_actual_ ? static_cast<long>(f_velocity_actual_->byte_offset) : -1L,
                       f_mode_wr_ ? static_cast<long>(f_mode_wr_->byte_offset) : -1L,
                       f_mode_disp_ ? static_cast<long>(f_mode_disp_->byte_offset) : -1L,
                       motion_ ? motion_->name() : "unselected");

    // Size the position-stability window to ~20 ms at the loop rate (>= 3 cycles), reset it.
    // Pre-allocated here (non-RT, pre-spawn) so the RT loop never allocates. Re-sized on each start/reconfigure.
    const std::uint32_t win = std::max<std::uint32_t>(3, static_cast<std::uint32_t>(config_.target_loop_rate_hz) / 50);
    pos_hist_.assign(win, 0);
    pos_hist_idx_ = 0;
    pos_hist_filled_ = 0;

    // A module without a configured quick_stop_decel stops via an uncontrolled disable-voltage
    // coast -- predictable, and safe because it won't quick-stop against an unverified decel. But on
    // a load-holding or vertical axis a coast drifts or drops the load, so surface the opt-out at
    // bring-up (non-RT, once) rather than let an operator find out the hard way. Not a hard
    // requirement, just discoverable.
    if (config_.quick_stop_decel == 0 && motion_->shutdown_cycles() == 0) {  // a mode with its own controlled stop needs none
        ETHERCAT_LOG_WARN("servo",
                          "slave {}: quick_stop_decel not set -> STOP is an uncontrolled disable-voltage coast (safe, but a "
                          "load-holding axis will drift/drop); set quick_stop_decel_rpm_per_s (0x6085) for a controlled ramp-stop",
                          s);
    }
}

bool ServoController::rt_alive() const noexcept {
    // Event-based aliveness (no clocks): the RT loop's on_stop() sets rt_exited_ on any exit (clean
    // stop, bus fault, bring-up abort, RT-setup failure), so a lock-free accessor sees the loop is no
    // longer servicing without touching rt_runner_ (which reconfigure() resets). A drive fault also
    // counts as not-alive for command purposes.
    return !rt_exited_.load(std::memory_order_acquire) && !state_.faulted.load(std::memory_order_acquire);
}

// Wake every parked await_move waiter. Bumped and notified by every terminal transition (move
// completed or failed), a drive-fault onset, stop()/reconfigure(), and the RT loop's on_stop()
// (async exit). C++20 atomic wait/notify, no mutex, RT-safe.
void ServoController::bump_wake() noexcept {
    wake_seq_.fetch_add(1, std::memory_order_release);
    wake_seq_.notify_all();
}

std::uint16_t ServoController::enter_resetting(CycleContext& ctx) noexcept {
    lifecycle_ = Resetting{};
    reset_cycles_ = 0;
    clear_streak_ = 0;
    motion_->reset();  // no command survives a fault: recovery re-enables into a hold, never into the old motion
    const std::uint16_t fc = f_fault_code_ ? ctx.load<std::uint16_t>(*f_fault_code_) : std::uint16_t{0};
    if (config_.estop_fault_code != 0 && fc == config_.estop_fault_code) {
        // RT-context log (one-shot; the RT rule in log.hpp)
        ETHERCAT_LOG_WARN(
            "servo", "slave {}: emergency stop engaged ({}) -- no fault-reset edges until it is released", config_.slave_id, hex(fc));
        return ControlWord::disable_voltage();
    }
    // RT-context log (one-shot; the RT rule in log.hpp)
    ETHERCAT_LOG_WARN("servo",
                      "slave {}: drive fault {} -- presenting a fault-reset edge every {} cycles until it clears",
                      config_.slave_id,
                      hex(fc),
                      config_.fault_reset_window_cycles);
    return ControlWord::fault_reset();  // bit 7 rising edge (the previous controlword had it low)
}

void ServoController::abort_active_move(RtError reason) noexcept {
    latched_ctrl_error_ = reason;  // diagnostic tier (last_error)
    // Publish the reason before failed_generation so last_error() is consistent the instant the
    // waiter observes the abort (publish_state recomputes rt_error_ again this cycle, idempotent
    // for a controller error). Not for WkcFault.
    rt_error_.store(reason, std::memory_order_release);
    const std::uint32_t g = state_.active_generation.load(std::memory_order_relaxed);
    if (g != 0 && state_.completed_generation.load(std::memory_order_relaxed) != g) {
        state_.failed_generation.store(g, std::memory_order_release);  // abort tier: wakes the go_to waiter
        bump_wake();
    }
}

bool ServoController::position_stable(std::int32_t actual) noexcept {
    // Push `actual` into the ring; stable once the window is full and its range (max-min) is within
    // position_tolerance_counts. Encoder jitter at rest stays within tolerance, so it reads stable;
    // real motion widens the range, so it reads not stable. A not-yet-full window is not stable
    // (still settling). O(N), N ~ 20ms of cycles. N (~20ms) and the tolerance (0.5deg) jointly set
    // two floors: (1) noise-immunity -- the drive's position jitter over the window must stay below
    // the tolerance (encoder dither is normally orders of magnitude below it), else at-rest would read
    // "moving"; (2) min-detectable velocity ~ tolerance/(N*cycle) ~ 182/(20ms) ~ 9100 c/s ~ 4 rpm, so
    // a creep slower than that reads "stopped". That is intended for a servo (reached is still guarded
    // by |actual-target| <= tolerance, so a slow move far from target is not false-reached). Widen N
    // or tighten the tolerance if a use-case needs finer slow-creep detection.
    if (pos_hist_.empty()) {
        return false;  // never sized (pre-start): treat as moving, fail-safe
    }
    pos_hist_[pos_hist_idx_] = actual;
    pos_hist_idx_ = (pos_hist_idx_ + 1) % pos_hist_.size();
    if (pos_hist_filled_ < pos_hist_.size()) {
        ++pos_hist_filled_;
        return false;
    }
    std::int32_t lo = pos_hist_[0];
    std::int32_t hi = pos_hist_[0];
    for (const std::int32_t p : pos_hist_) {
        lo = std::min(lo, p);
        hi = std::max(hi, p);
    }
    return (hi - lo) <= config_.position_tolerance_counts;
}

Cia402Mode ServoController::commanded_cia402_mode() const noexcept {
    // The selected motion mode owns the 0x6060 value (a switching mode returns its current intent).
    // Before a mode is selected -- never on the RT path -- default to PP.
    return motion_ ? motion_->commanded_mode() : Cia402Mode::ProfilePosition;
}

MotionFeedback ServoController::feedback(CycleContext& ctx, Status status, std::int32_t actual) const noexcept {
    MotionFeedback fb;
    fb.status = status;
    fb.actual = actual;
    if (f_mode_disp_) {
        fb.mode_echo = ctx.load<cia402::ModeDisplay::type>(*f_mode_disp_);
    }
    fb.at_rest = at_rest_;
    fb.near_target =
        std::abs(static_cast<std::int64_t>(actual) - motion_->target()) <= static_cast<std::int64_t>(config_.position_tolerance_counts);
    fb.target_reached_bit_usable = target_reached_bit_usable();
    return fb;
}

bool ServoController::estop_engaged() const noexcept {
    // Two independent reasons to refuse motion: a device inhibit (seam), or the configured 0x603F
    // emergency-stop code on a faulted drive.
    if (state_.inhibit.load(std::memory_order_acquire) != static_cast<std::uint8_t>(Inhibit::None)) {
        return true;
    }
    return config_.estop_fault_code != 0 && state_.drive_faulted.load(std::memory_order_acquire) &&
           state_.drive_fault_code.load(std::memory_order_relaxed) == config_.estop_fault_code;
}

std::uint16_t ServoController::drive_fault_code() const noexcept {
    return state_.drive_fault_code.load(std::memory_order_relaxed);
}

bool ServoController::drive_warning() const noexcept {
    return state_.drive_warning.load(std::memory_order_relaxed);
}

const ServoConfig::AbsoluteEncoder& ServoController::absolute_encoder() const noexcept {
    return config_.absolute_encoder;
}

bool ServoController::emergency_stopped() const noexcept {
    const std::shared_lock<std::shared_mutex> lk(api_mutex_);
    return estop_engaged();
}

const char* to_string(HomingState state) noexcept {
    switch (state) {
        case HomingState::Idle:
            return "idle";
        case HomingState::Running:
            return "running";
        case HomingState::Done:
            return "done";
        case HomingState::Failed:
            return "failed";
    }
    return "?";
}

const char* to_string(HomingFail fail) noexcept {
    switch (fail) {
        case HomingFail::None:
            return "";
        case HomingFail::ModeNotAdopted:
            return "the drive did not adopt homing mode (0x6061 never echoed 6)";
        case HomingFail::DriveError:
            return "the drive reported a homing error (statusword bit 13)";
        case HomingFail::Timeout:
            return "the drive did not attain home within the timeout";
        case HomingFail::Cancelled:
            return "cancelled by stop or disable";
        case HomingFail::DriveFault:
            return "the drive faulted during the run";
        case HomingFail::NotOperational:
            return "the drive was not operational when the run was to start";
    }
    return "?";
}

const char* ServoController::motion_mode_name() const noexcept {
    const std::shared_lock<std::shared_mutex> lk(api_mutex_);
    return motion_ ? motion_->name() : "unselected";
}

// Construct the motion mode named by config_.motion_mode. Runs pre-spawn on the non-RT thread; the
// mode's resolve() then selects it on the drive by SDO, so an unsupported mode fails start() with a
// clear error rather than a drive that never moves.
void ServoController::select_motion_mode() {
    // The configured quick-stop rate is counts/s^2; the drive's 0x6085 may not be (device seam).
    const std::uint32_t qs = accel_to_device(config_.quick_stop_decel, accel_units_per_cps2());
    if (config_.motion_mode == MotionModeKind::Profile) {
        motion_ = std::make_unique<ProfilePositionMode>(qs, 2);
        ETHERCAT_LOG_INFO("servo", "slave {}: control_mode profile (PP for go_to/go_for, PV for set_rpm)", config_.slave_id);
        return;
    }
    const double dt = 1.0 / static_cast<double>(config_.target_loop_rate_hz);
    const double accel_rpm_s = config_.max_accel_rpm_per_s > 0.0 ? config_.max_accel_rpm_per_s : config_.max_motor_speed_rpm;
    const double accel_cps2 = static_cast<double>(rpm_to_device_velocity(accel_rpm_s, config_.counts_per_rev, config_.gear_ratio));
    const double vmax_cps =
        static_cast<double>(rpm_to_device_velocity(config_.max_motor_speed_rpm, config_.counts_per_rev, config_.gear_ratio));
    motion_ = std::make_unique<CyclicPositionMode>(dt, accel_cps2, vmax_cps, qs);
    ETHERCAT_LOG_INFO("servo",
                      "slave {}: control_mode csp (master-side trapezoid, accel {} counts/s^2, vmax {} counts/s)",
                      config_.slave_id,
                      accel_cps2,
                      vmax_cps);
}

std::uint16_t ServoController::step_lifecycle(CycleContext& ctx, Status status, const CommandBatch& batch, std::int32_t actual) noexcept {
    const Cia402State dev = status.decode();
    const bool bus_fault = ctx.fault();

    // Command routing to the selected motion mode. A halt cancels any in-flight blocking move and,
    // order-preserving, latches the mode's halt only when it was the latest stop-relevant command in
    // this drain (Stop() then GoTo() coalesced means the caller wants to move, so the halt is superseded).
    if (batch.halt) {
        abort_active_move(RtError::MotorStopped);  // the waiter throws "motor stopped"
        if (batch.halt_supersedes) {
            motion_->halt();
        }
    }
    if (batch.disable) {
        abort_active_move(RtError::MotorDisabled);  // the waiter throws "motor disabled"
    }
    // (abort_active_move is a no-op when no move is live -- gen 0 or already terminal -- so cancelling
    //  an already-completed move does not overwrite its success: first-terminal-wins.)

    // Adopt a new positioning target (generation rides in the command, post-coalescing).
    if (batch.set_target.has_value()) {
        const SetTarget& t = *batch.set_target;
        if (t.generation != state_.active_generation.load(std::memory_order_relaxed)) {
            if (t.relative) {
                motion_->go_for(t.counts, t.profile_velocity, actual);
            } else {
                motion_->go_to(t.counts, t.profile_velocity);
            }
            latched_ctrl_error_ = RtError::None;  // a fresh move starts with a clean diagnostic slate
            state_.active_generation.store(t.generation, std::memory_order_release);
        }
    }
    if (batch.set_velocity.has_value()) {
        motion_->set_velocity(batch.set_velocity->velocity);
    }
    // A homing request that reaches a cycle outside Operational (a fault landed between the caller's
    // check and this cycle) must be failed, not dropped with the batch: its caller is parked on the
    // published state and the run's timeout only counts inside the Homing state.
    if (batch.start_homing.has_value() && !std::holds_alternative<Operational>(lifecycle_)) {
        publish_homing_result(HomingFail::NotOperational);
    }

    // Device inhibit. Preempts every state but Disabled (a deliberate de-energize stays) and
    // outranks the fault path: there is nothing to reset while an input holds the drive down, so
    // fault-reset edges would only hammer it. Shutdown is also step one of the recovery such a stop
    // needs, since it latches in the drive.
    const Inhibit inh = inhibit(ctx);
    state_.inhibit.store(static_cast<std::uint8_t>(inh), std::memory_order_release);
    const bool stopped = std::holds_alternative<SafetyStop>(lifecycle_) || std::holds_alternative<QuickStopping>(lifecycle_);
    if (inh != Inhibit::None && !std::holds_alternative<Disabled>(lifecycle_)) {
        if (!stopped) {
            abort_active_move(RtError::MotorStopped);  // the in-flight move's waiter throws
            motion_->reset();                          // no command survives the stop; release re-enables into a hold
        }
        if (inh == Inhibit::QuickStop) {
            lifecycle_ = QuickStopping{};
            return ControlWord::quick_stop();
        }
        lifecycle_ = SafetyStop{};  // an escalation from QuickStop lands here too, already aborted
        return ControlWord::shutdown();
    }
    if (stopped) {
        // Released. Hand to the enable ladder, which climbs Shutdown -> SwitchOn -> EnableOperation.
        lifecycle_ = Enabling{};
        mode_echo_wait_ = 0;
        return fsm_.step(status, Cia402State::OperationEnabled);
    }

    if (std::holds_alternative<Init>(lifecycle_)) {
        if (dev == Cia402State::Fault) {
            return enter_resetting(ctx);  // a fault latched before this run (previous session, refused SDO)
        }
        lifecycle_ = Enabling{};
        mode_echo_wait_ = 0;
        return ControlWord::disable_voltage();
    }
    if (std::holds_alternative<Enabling>(lifecycle_)) {
        if (dev == Cia402State::Fault) {
            return enter_resetting(ctx);
        }
        // Operator override, as in every other state that can receive one. Without it a disable
        // issued while the ladder is climbing -- which is exactly where a finished homing run leaves
        // it -- is dropped, and the drive energizes against the caller's request.
        if (batch.disable) {
            lifecycle_ = Disabled{};
            motion_->reset();  // enable() re-energizes into a hold
            return ControlWord::disable_voltage();
        }
        if (f_mode_wr_) {
            ctx.store<cia402::ModeOfOperation::type>(*f_mode_wr_, static_cast<std::int8_t>(commanded_cia402_mode()));
        }
        // Energize only once 0x6061 echoes the commanded mode; hold at SwitchedOn meanwhile. A drive
        // that never echoes it is reported once (ModeMismatch) and stays at SwitchedOn, de-energized.
        const auto want = static_cast<std::int8_t>(commanded_cia402_mode());
        const bool echoed = !f_mode_disp_ || ctx.load<cia402::ModeDisplay::type>(*f_mode_disp_) == want;
        if (!echoed && (dev == Cia402State::SwitchedOn || dev == Cia402State::OperationEnabled)) {
            if (++mode_echo_wait_ == kModeEchoCycles) {
                latched_ctrl_error_ = RtError::ModeMismatch;
                // RT-context log (one-shot; the RT rule in log.hpp)
                ETHERCAT_LOG_WARN("servo",
                                  "slave {}: 0x6061 = {} != commanded {} after {} cycles -- not energizing",
                                  config_.slave_id,
                                  static_cast<int>(ctx.load<cia402::ModeDisplay::type>(*f_mode_disp_)),
                                  static_cast<int>(want),
                                  kModeEchoCycles);
            }
            return fsm_.step(status, Cia402State::SwitchedOn);
        }
        if (echoed) {
            mode_echo_wait_ = 0;
            if (latched_ctrl_error_ == RtError::ModeMismatch) {
                latched_ctrl_error_ = RtError::None;
            }
        }
        if (dev == Cia402State::OperationEnabled) {
            lifecycle_ = Operational{};
            // RT-context log (one-shot; the RT rule in log.hpp)
            ETHERCAT_LOG_INFO("servo", "slave {}: drive reached OperationEnabled (energized) at cycle {}", config_.slave_id, ctx.cycle());
        }
        return fsm_.step(status, Cia402State::OperationEnabled);
    }
    if (std::holds_alternative<Operational>(lifecycle_)) {
        if (dev == Cia402State::Fault || bus_fault) {
            // Terminate the in-flight blocking move (its waiter throws with last_error(); the
            // single-in-flight slot frees) and start the standard recovery.
            abort_active_move(RtError::NotOperational);
            return enter_resetting(ctx);
        }
        if (batch.disable) {
            lifecycle_ = Disabled{};
            motion_->reset();  // likewise: enable() re-energizes into a hold
            return ControlWord::disable_voltage();
        }
        if (batch.start_homing.has_value()) {
            lifecycle_ = Homing{};
            homing_cycles_ = 0;
            homing_echo_wait_ = 0;
            homing_timeout_cycles_ = batch.start_homing->timeout_cycles;
            motion_->reset();
            return ControlWord::enable_operation();  // bit 4 low, so the start edge next cycle is real
        }
        if (batch.quick_stop) {
            return ControlWord::quick_stop();
        }
        // The selected motion mode writes its command objects and returns the controlword; the
        // controller keeps completion generations, the fault tiers, and is-moving/reached.
        const std::uint16_t cw = motion_->step(ctx, feedback(ctx, status, actual));
        if (const MotionFault f = motion_->fault(); f != MotionFault::None) {  // one cycle per event
            // RT-context log (one-shot; the RT rule in log.hpp)
            ETHERCAT_LOG_WARN("servo", "slave {}: {} -- the command fails", config_.slave_id, to_string(f));
            abort_active_move(f == MotionFault::ModeNotAdopted ? RtError::ModeMismatch : RtError::HandshakeTimeout);
        }
        return cw;
    }
    if (std::holds_alternative<Homing>(lifecycle_)) {
        if (dev == Cia402State::Fault || bus_fault) {
            finish_homing(HomingFail::DriveFault);
            return enter_resetting(ctx);
        }
        if (batch.disable || batch.halt || batch.quick_stop) {  // the operator's stop wins over the run
            finish_homing(HomingFail::Cancelled);
            if (batch.disable) {
                lifecycle_ = Disabled{};
            }
            return ControlWord::disable_voltage();
        }
        if (f_mode_wr_) {  // a map that carries 0x6060: the PDO governs, so the loop writes mode 6 too
            ctx.store<cia402::ModeOfOperation::type>(*f_mode_wr_, static_cast<std::int8_t>(Cia402Mode::Homing));
        }
        const bool echoed =
            !f_mode_disp_ || ctx.load<cia402::ModeDisplay::type>(*f_mode_disp_) == static_cast<std::int8_t>(Cia402Mode::Homing);
        if (!echoed) {
            if (++homing_echo_wait_ >= kModeEchoCycles) {
                finish_homing(HomingFail::ModeNotAdopted);
            }
            return ControlWord::enable_operation();
        }
        if (status.homing_error()) {
            finish_homing(HomingFail::DriveError);
            return ControlWord::with_halt(ControlWord::enable_operation(), true);
        }
        if (status.homing_attained()) {
            // home() reports this raw count as the axis reference, so the software zero must go with
            // it: an offset left over from a set_zero would put position_revs() in another frame.
            state_.zero_offset_counts.store(0, std::memory_order_relaxed);
            state_.home_counts.store(actual, std::memory_order_relaxed);
            // The re-base for THIS search has already landed by the time the drive flags attained,
            // so the total is current: the origin, expressed in the frame the search started from,
            // is what is reported now less what the frame moved during the search. That frame is
            // the encoder's plus the offset the drive was holding, which the write then cancels.
            state_.home_encoder_rel.store(static_cast<std::int64_t>(actual) - frame_rebase_, std::memory_order_relaxed);
            // Only the first search since bring-up measures against the power-on frame. A later one
            // measures against whatever this search left behind, and no stored offset describes that
            // -- deriving from it is how a good datum gets overwritten with a plausible wrong one.
            state_.home_encoder_rel_valid.store(!frame_rebased_this_session_, std::memory_order_relaxed);
            frame_rebased_this_session_ = true;
            state_.homed.store(true, std::memory_order_release);
            finish_homing(HomingFail::None);
            return ControlWord::enable_operation();
        }
        if (++homing_cycles_ >= homing_timeout_cycles_) {
            finish_homing(HomingFail::Timeout);
            return ControlWord::with_halt(ControlWord::enable_operation(), true);
        }
        // Bit 4 starts the procedure and stays high for its duration (in homing mode it is the
        // "homing operation start" bit, not the profile-position new-set-point).
        return ControlWord::with_new_setpoint(ControlWord::enable_operation(), true);
    }
    if (std::holds_alternative<Resetting>(lifecycle_)) {
        if (batch.disable) {  // operator override
            lifecycle_ = Disabled{};
            return ControlWord::disable_voltage();
        }
        // Standard CiA402 recovery: a fault-reset edge (bit 7) once per window while the drive stays in
        // Fault, bit low in between so the next one is a real edge. Once the drive has left Fault for
        // the confirm streak, hand back to the enable ladder. last_error() shows the fault meanwhile.
        clear_streak_ = dev != Cia402State::Fault ? clear_streak_ + 1 : 0;
        if (clear_streak_ >= config_.fault_reset_clear_confirm_cycles) {
            // RT-context log (one-shot; the RT rule in log.hpp)
            ETHERCAT_LOG_INFO("servo", "slave {}: drive fault cleared after {} cycles -- re-enabling", config_.slave_id, reset_cycles_);
            lifecycle_ = Enabling{};
            mode_echo_wait_ = 0;
            return fsm_.step(status, Cia402State::OperationEnabled);
        }
        ++reset_cycles_;
        if (batch.fault_reset) {  // operator request: an edge now
            reset_cycles_ = 0;
            return ControlWord::fault_reset();
        }
        // No reset edges while the configured emergency-stop code is present: the stop is engaged, and
        // clearing it is the operator's act (release, then fault_reset when the drive latches it).
        const std::uint16_t fc = f_fault_code_ ? ctx.load<std::uint16_t>(*f_fault_code_) : std::uint16_t{0};
        const bool estop = config_.estop_fault_code != 0 && fc == config_.estop_fault_code;
        const bool edge = dev == Cia402State::Fault && !estop && (reset_cycles_ % config_.fault_reset_window_cycles) == 0;
        return edge ? ControlWord::fault_reset() : ControlWord::disable_voltage();
    }
    // Disabled
    if (batch.enable) {
        lifecycle_ = Enabling{};
        mode_echo_wait_ = 0;
    }
    return ControlWord::disable_voltage();
}

void ServoController::publish_state(CycleContext& ctx, Status status, std::int32_t actual, std::int32_t velocity) noexcept {
    state_.position_counts.store(actual, std::memory_order_relaxed);
    state_.velocity.store(velocity, std::memory_order_relaxed);

    const bool powered = status.operation_enabled();
    state_.powered.store(powered, std::memory_order_relaxed);
    state_.switch_on_disabled.store(status.decode() == Cia402State::SwitchOnDisabled, std::memory_order_relaxed);

    const std::uint32_t g = state_.active_generation.load(std::memory_order_relaxed);
    const bool move_active = g != 0 && state_.completed_generation.load(std::memory_order_relaxed) != g &&
                             state_.failed_generation.load(std::memory_order_relaxed) != g;

    // Rest and reach. position_stable() advances its ring, so call it exactly once per cycle.
    const bool pos_stable = position_stable(actual);
    at_rest_ = pos_stable;
    const MotionFeedback fb = feedback(ctx, status, actual);
    const bool at_target = motion_->is_target_reached(fb);
    // Moving: the mode commands motion, a positioning move is still under way, or the shaft is not at
    // rest (a ramp-down after set_rpm(0), which no command reflects).
    const bool moving =
        powered && (std::holds_alternative<Homing>(lifecycle_) || motion_->is_moving() || (move_active && !at_target) || !pos_stable);
    state_.moving.store(moving, std::memory_order_relaxed);
    state_.mode_echo.store(f_mode_disp_ ? ctx.load<cia402::ModeDisplay::type>(*f_mode_disp_) : std::int8_t{0}, std::memory_order_relaxed);

    // PP generation protocol: completion (PP-only via move_active). There is no no-progress watchdog;
    // a stuck move parks in await_move until the client stops it, the drive faults, or the RT loop
    // exits (client-owned cancellation, consistent with the no-timeout wait).
    if (powered && move_active && at_target) {
        state_.completed_generation.store(g, std::memory_order_release);  // publish before the wake
        bump_wake();
    }

    // Per-tier fault publish. last_error() composes every active tier, so a both-true sync-loss (drive
    // 0x603F plus bus WKC -> 0) reports root cause and symptom. In each tier the payload is
    // relaxed-stored before the flag is release-stored, so a master_-free reader never sees a true
    // flag with a stale payload (cross-tier skew is benign: fault state is quasi-static once latched).
    const bool bus_fault = ctx.fault();
    // Bus tier: payload (the raw last-exchange WKC, the actual bad value at fault, ==
    // master_->last_wkc()) then flag.
    state_.fault_wkc.store(ctx.wkc().last, std::memory_order_relaxed);
    state_.wkc_faulted.store(bus_fault, std::memory_order_release);
    // Drive tier: live-read 0x603F from this cycle's owned snapshot every faulted cycle (not
    // edge-captured) so a code the drive latches a frame or two after it sets bit 3 is still picked
    // up ("code pending" collapses to the rare hard-drop race only).
    const std::uint16_t drive_code = f_fault_code_ ? ctx.load<std::uint16_t>(*f_fault_code_) : 0;
    state_.drive_fault_code.store(drive_code, std::memory_order_relaxed);
    state_.drive_warning.store(status.warning(), std::memory_order_relaxed);
    state_.drive_faulted.store(status.fault(), std::memory_order_release);
    // Controller tier: the published mirror of the latch (tracks abort, clears on fault_reset).
    rt_error_.store(latched_ctrl_error_, std::memory_order_release);
    // state_.faulted is the drive/bus gate only (de-powers the motor and wakes the waiter's fault
    // branch). Controller move-errors (HandshakeTimeout) deliberately stay out: they fail the
    // in-flight move (via failed_generation) but must not de-power an otherwise-healthy drive. On a
    // fault onset wake any parked waiter (event-based; no clock watchdog to catch it now).
    const bool now_faulted = bus_fault || status.fault();
    const bool was_faulted = state_.faulted.load(std::memory_order_relaxed);
    state_.faulted.store(now_faulted, std::memory_order_release);
    if (now_faulted && !was_faulted) {
        bump_wake();
    }

    state_.loop_cycle.fetch_add(1, std::memory_order_relaxed);
}

// --- SlaveControl hooks: the RT body split across the Runner's lifecycle. The Runner owns
// realtime setup, the DC bring-up pump, the single DcPacer, pacing, the steady cadence, the
// stopping window, and master.close(). What remains here is driver policy.

void ServoController::on_configured(ConfigContext& cfg) {
    configure_device(cfg);  // vendor setup first: the sequencer's SDOs may depend on it
    // Resolve the sequencer's typed fields and run its quick-stop SDO setup here (non-RT, pre-spawn,
    // single port owner -- the one hook that may throw; a throw aborts start() cleanly and goes
    // Degraded). The module's own f_* offsets and vendor reset still resolve in start()/reconfigure();
    // this adds the sequencer's resolution (same Master, same SAFE-OP phase). needs_quick_stop is gated on
    // a configured 0x6085 (quick_stop_decel > 0); the echoed value backs the velocity-window guard. A
    // mismatched 0x605A or absent 0x6085 throws.
    const std::uint32_t echoed = motion_->resolve(cfg);  // the mode's fields + one-time SDOs (quick-stop setup in PP)
    qs_decel_echoed_.store(echoed, std::memory_order_release);
    // Derive the velocity guard budget from the teardown window (a single source of truth, so the
    // window and the budget cannot disagree): the max velocity the echoed 0x6085 decel can ramp to 0
    // within (teardown_window - margin). 0 makes the guard inert.
    std::int64_t budget = 0;
    if (echoed > 0) {
        const double window_s = static_cast<double>(teardown_window_cycles()) / static_cast<double>(config_.target_loop_rate_hz);
        const double decel_cps2 = static_cast<double>(echoed) / accel_units_per_cps2();
        const double b = decel_cps2 * std::max(0.0, window_s - kStopWindowMarginS);
        budget = static_cast<std::int64_t>(b);
    }
    vel_budget_cps_.store(budget, std::memory_order_release);
}

void ServoController::on_operational(CycleContext& ctx) noexcept {
    // No explicit seed: the std::variant lifecycle climbs Init->Enabling->Operational inside
    // step_lifecycle off the drive state.
    (void)ctx;
}

void ServoController::stage_bringup_outputs(CycleContext& ctx) noexcept {
    // The mode's set-points track the actual from the first process-data frame (CSP: 0x607A = 0x6064).
    // A zero target against a non-zero actual is a following-error fault at OP entry, before the
    // enable ladder runs. Profile mode has nothing to stage: the drive holds its own set-points.
    const Status st{ctx.load<cia402::Statusword::type>(f_statusword_)};
    motion_->track(ctx, feedback(ctx, st, ctx.load<std::int32_t>(f_actual_)));
}

bool ServoController::drive_present(const CycleContext& ctx) const noexcept {
    // OP-confirm gate: a live drive populates a non-zero statusword; a drive that zombie-PDOs (a
    // DC-only drive requested into OP under free-run -- AL 0x0027, dead TxPDO) leaves it 0x0. Gating
    // OP-confirm on this makes bring-up give up (BringupAborted, with the AL-status diagnostic) instead
    // of reaching OP on a full-WKC-but-dead drive and spinning the enable ladder forever.
    return ctx.load<cia402::Statusword::type>(f_statusword_) != 0;
}

bool ServoController::sync_faulted(const CycleContext& ctx) const noexcept {
    // The bring-up gate: drive-sync-faulted = mapped 0x603F == the configured no-sync code; nullopt
    // (none declared) means always false. Stash the read code for on_stop's bring-up-abort diagnostic
    // (sync_faulted and on_stop both run on the RT thread).
    const std::uint16_t code = f_fault_code_ ? ctx.load<std::uint16_t>(*f_fault_code_) : 0;
    last_sync_code_ = code;
    const std::optional<std::uint16_t> no_sync = sync_fault_code();  // device seam (base nullopt)
    return no_sync.has_value() && code == *no_sync;
}

void ServoController::step(CycleContext& ctx) noexcept {
    if (ctx.stopping()) {
        // Lifecycle-stop: the Runner enters its stopping window on any stop cause, including a bus
        // fault (master_.fault()). During stopping the drive de-energizes but keeps publishing, so
        // last_error() composes the bus/drive fault that triggered the stop. The Runner ships the
        // final process and close()->INIT after the window.
        const Status sstatus{ctx.load<std::uint16_t>(f_statusword_)};
        const std::int32_t sactual = ctx.load<std::int32_t>(f_actual_);
        const std::int32_t svel =
            f_velocity_actual_ ? ctx.load<std::int32_t>(*f_velocity_actual_)
                               : static_cast<std::int32_t>(static_cast<std::int64_t>(sactual - prev_actual_) * config_.target_loop_rate_hz);
        prev_actual_ = sactual;
        // The mode owns the controlled stop (PP: the drive's quick-stop when configured, else disable-voltage).
        const std::uint16_t scw = motion_->step_shutdown(ctx, feedback(ctx, sstatus, sactual));
        ctx.store<std::uint16_t>(f_ctrlword_, scw);
        last_cw_ = scw;
        // Event-driven teardown early-out: once the drive is de-energized at rest (SwitchOnDisabled --
        // the 0x605A==2 auto-transition at zero, or the disable-voltage backstop / opt-out coast
        // landing), signal the Runner it may end the teardown window. A moving stop keeps this false
        // until the controlled ramp reaches rest, so close() never cuts torque at speed; the decel>0
        // window is the hard cap.
        stop_at_rest_ = sstatus.switch_on_disabled();
        publish_state(ctx, sstatus, sactual, svel);
        return;
    }

    const CommandBatch batch = commands_.drain();

    // One input snapshot per cycle: statusword/bit 3, actual, 0x603F, 0x606C all read from this
    // cycle's owned image (the Runner copied it in before step()), so the fault code matches the fault
    // state it is reported with.
    const Status status{ctx.load<std::uint16_t>(f_statusword_)};
    const std::int32_t actual = ctx.load<std::int32_t>(f_actual_);
    if (first_cycle_) {
        prev_actual_ = actual;  // avoid a spurious huge velocity on cycle 0
        first_cycle_ = false;
    }
    // Velocity from the wire (0x606C) when mapped, else the instantaneous estimate
    // (actual-delta * loop rate). One branch; identical fallback when unmapped.
    const std::int32_t velocity =
        f_velocity_actual_ ? ctx.load<std::int32_t>(*f_velocity_actual_)
                           : static_cast<std::int32_t>(static_cast<std::int64_t>(actual - prev_actual_) * config_.target_loop_rate_hz);
    prev_actual_ = actual;

    // One-shot on every 0x603F change (fault onset and clear) with the context that explains it.
    if (f_fault_code_) {
        const auto fc = ctx.load<cia402::FaultCode::type>(*f_fault_code_);
        if (fc != last_logged_fault_code_) {
            // RT-context log (one-shot; the RT rule in log.hpp)
            ETHERCAT_LOG_INFO("servo",
                              "slave {}: 0x603F {} -> {} at cycle {} (statusword 0x{:04X}, 0x6061 echo {}, last cw 0x{:04X})",
                              config_.slave_id,
                              hex(last_logged_fault_code_),
                              hex(fc),
                              ctx.cycle(),
                              status.raw,
                              f_mode_disp_ ? static_cast<int>(ctx.load<cia402::ModeDisplay::type>(*f_mode_disp_)) : -1,
                              last_cw_);
            last_logged_fault_code_ = fc;
        }
    }

    // Before the lifecycle: a search's re-base must already be in the total that step reads when the
    // drive flags home attained.
    track_frame_rebase(actual, std::holds_alternative<Homing>(lifecycle_));
    const std::uint16_t cw = step_lifecycle(ctx, status, batch, actual);
    ctx.store<std::uint16_t>(f_ctrlword_, cw);
    last_cw_ = cw;
    if (!std::holds_alternative<Operational>(lifecycle_)) {
        motion_->track(ctx, feedback(ctx, status, actual));  // keep the command objects sane while not enabled
    }

    // process() ships the cw and latches the next input (Runner-owned, around step()), so a store this
    // cycle is on the wire the next cycle. publish reads this cycle's snapshot.
    publish_state(ctx, status, actual, velocity);
}

void ServoController::on_stop(StopReason reason) noexcept {
    // Event-based aliveness: the RT loop is exiting (any reason). Latch it and wake parked waiters so a
    // go_to blocked on an async exit (bus fault or bring-up abort, where the API never set stopping_)
    // returns promptly instead of hanging.
    rt_exited_.store(true, std::memory_order_release);
    bump_wake();
    if (reason == StopReason::BringupAborted) {
        // Bring-up gave up, with two independent possible causes, both surfaced: (1) the DC-sync
        // gate -- 0x603F == the configured no-sync code held, the SYNC0-didn't-take case;
        // (2) the ESC AL status code -- the drive refused an AL transition, e.g. AL 0x0027 "Freerun
        // not supported" when a DC-only drive is requested into OP without SYNC0
        // (use_distributed_clocks=false).
        //
        // Read the AL status code (cached, no port I/O; this runs on the Runner's RT thread, the sole
        // master toucher) and publish it; only attribute the drive (0x603F) tier when the sync code
        // was actually the configured no-sync fault. Prefer the latched last-non-zero AL code from
        // AWAIT: the zombie-PDO free-run drive sits at SAFE-OP+AL-0x0027 but the live code reads 0 at
        // the give-up (reack_op ACKs the error on the timeout cycle), so the live read alone would
        // surface "drive not operational" with no cause. bringup_al_code() holds the real 0x0027 seen
        // mid-AWAIT; fall back to the live read.
        std::uint16_t al = master_ != nullptr ? master_->bringup_al_code() : 0;
        if (al == 0 && master_ != nullptr) {
            al = master_->al_status_code(config_.slave_id);
        }
        const std::string al_msg = Master::describe_al_code(al);  // static: no live master needed
        state_.bringup_al_code.store(al, std::memory_order_relaxed);
        const std::optional<std::uint16_t> no_sync = sync_fault_code();  // device seam (base nullopt)
        const bool sync_fault = no_sync.has_value() && last_sync_code_ != 0 && last_sync_code_ == *no_sync;
        if (sync_fault) {
            state_.drive_fault_code.store(last_sync_code_, std::memory_order_relaxed);
            state_.drive_faulted.store(true, std::memory_order_release);
        }
        rt_error_.store(RtError::NotOperational, std::memory_order_release);
        state_.faulted.store(true, std::memory_order_release);
        degraded_.store(true, std::memory_order_release);  // bring-up failed -> Degraded (APIs throw via last_error())
        ETHERCAT_LOG_ERROR("servo",
                           "slave {}: bring-up FAILED: drive not operational{}{}",
                           config_.slave_id,
                           al != 0 ? " -- drive refused OP: AL " + hex(al) + " (" + al_msg + ")" : std::string(),
                           (al == 0x0027 && !config_.use_distributed_clocks)
                               ? " -- freerun not supported; this drive requires use_distributed_clocks=true"
                               : "");
    } else if (reason == StopReason::RtSetupFailed) {
        // Realtime scheduling unavailable and require_realtime -> Degraded-but-alive. Backstop only:
        // start()/reconfigure() preflight this and throw before any bus I/O; reaching here means the
        // privileges changed between the probe and the RT-thread spawn. RtSetupFailed (not
        // NotOperational) so last_error() names the realtime cause instead of blaming the drive.
        rt_error_.store(RtError::RtSetupFailed, std::memory_order_release);
        degraded_.store(true, std::memory_order_release);
    } else if (reason == StopReason::BusFault) {
        // The bus died mid-run (interface down, cable pulled, slave dropped off): mark the loss
        // RECOVERABLE. The next motion call tears the dead run down and attempts a full rebuild
        // inline (maybe_recover_bus) -- recovery is API-driven, never a background poll.
        //
        // Publish the WKC tier HERE: the steady loop's publish runs before the Runner's fault
        // check, and once the latch fires every remaining cycle is a stopping-window dispatch --
        // so without this store last_error() reads EMPTY for the whole outage (HW-caught by the
        // link-loss test, P3). This is the RT thread, the sole master_ toucher, so the reads are
        // safe (same as the BringupAborted branch above).
        state_.fault_wkc.store(master_ != nullptr ? master_->last_wkc() : -1, std::memory_order_relaxed);
        state_.wkc_faulted.store(true, std::memory_order_release);
        bus_lost_.store(true, std::memory_order_release);
    }
    // Requested: clean teardown; parked waiters are woken by stop()'s notify_all.
}

// API-driven bus recovery: no background reconnect loop by design (a motor that lost its bus must
// not re-energize on its own schedule; it re-energizes when an operator/client asks it to move).
// A motion verb called after a bus loss lands here first: tear down the dead run -- the exited RT
// thread and the Master holding the stale socket -- then attempt ONE full rebuild (open, configure
// to SAFE-OP, bring-up to OP) inline in the caller. While the interface is still down the rebuild
// fails at open() in milliseconds and the call returns a clear error; once the link is back a call
// rebuilds, then executes its motion normally. The client's retry loop is the reconnect policy;
// the controller stays alive and queryable (fail-safe reads, last_error) throughout.
void ServoController::maybe_recover_bus() {
    if (!bus_lost_.load(std::memory_order_acquire)) {
        return;
    }
    const std::unique_lock<std::shared_mutex> lk(api_mutex_);
    if (!bus_lost_.load(std::memory_order_acquire)) {
        return;  // another caller already recovered the bus
    }
    // Full teardown before rebuilding: join the exited RT thread, close the stale socket. Same
    // shape as reconfigure(); the rebuild must start from nothing so open() gets a fresh handle.
    stopping_.store(true, std::memory_order_release);
    bump_wake();
    rt_runner_.reset();
    master_.reset();
    // Clear the marker BEFORE the rebuild spawns: a NEW bus fault during or right after bring-up
    // re-latches it on the RT thread, and clearing afterwards could overwrite that latch.
    bus_lost_.store(false, std::memory_order_release);
    try {
        MasterConfig mc = build_master_config(config_);
        // A fraction of the patience start() gets. There, one attempt is all there is and a human is
        // reading the error; here another attempt follows in two seconds, so waiting out a drive that
        // is still powering up buys nothing and delays the attempt that would have worked.
        mc.op_await_timeout_ms = kRecoveryOpAwaitMs;
        master_ = std::make_unique<Master>(std::move(mc));
        master_->init();
        master_->configure();  // -> SAFE-OP
        bring_up();            // -> OP with bounded retry (gives up by setting degraded_)
    } catch (const Error& e) {
        // Rebuild failed outright (typically: the interface is still down). Stay torn down and
        // recoverable; the next motion call retries.
        rt_runner_.reset();
        master_.reset();
        bus_lost_.store(true, std::memory_order_release);
        throw Error("bus lost and recovery failed (" + std::string(e.what()) + ") -- retrying every 2 s, and on the next motion call");
    }
    if (degraded_.load(std::memory_order_acquire)) {
        // bring_up() exhausted its attempts: the bus enumerates but the drive would not reach OP.
        // Stay torn down and recoverable rather than parked on a half-alive master.
        const std::string cause = last_error();
        rt_runner_.reset();
        master_.reset();
        bus_lost_.store(true, std::memory_order_release);
        throw Error("bus lost and recovery failed (" + cause + ") -- retrying every 2 s, and on the next motion call");
    }
    ETHERCAT_LOG_INFO("servo", "slave {}: bus recovered -- drive back to OPERATIONAL", config_.slave_id);
}

// The same rebuild a motion verb would make, on a timer. A drive whose control power is cycled
// takes the bus with it, and nothing asks a powered-off axis to move -- so without this the axis
// stays down after power comes back, until someone commands a move. While the drive is off the
// attempt fails at enumeration in milliseconds, which is what makes retrying this often cheap.
void ServoController::recovery_loop(const std::stop_token& stop) {
    constexpr auto kTick = std::chrono::milliseconds(100);  // so a stop request is honoured promptly
    constexpr int kTicksPerRetry = 20;                      // retry every 2 s while the bus is gone
    constexpr int kQuietRetries = 30;                       // then repeat the same reason ~once a minute
    int failures = 0;
    while (!stop.stop_requested()) {
        for (int i = 0; i < kTicksPerRetry && !stop.stop_requested(); ++i) {
            std::this_thread::sleep_for(kTick);
        }
        // Only the stop token ends this loop. stopping_ is a per-run flag that a failed rebuild
        // leaves set (it is cleared when a run spawns, which is the thing that did not happen), so
        // testing it here would retire the retry after its first failure -- exactly when the drive
        // is off and the retry is the point.
        if (stop.stop_requested() || !bus_lost_.load(std::memory_order_acquire)) {
            failures = 0;
            continue;
        }
        try {
            maybe_recover_bus();  // logs the recovery itself
            failures = 0;
        } catch (const std::exception& e) {
            // Expected for as long as the drive is off; say so once, then stay quiet about it.
            ++failures;
            if (failures == 1 || failures % kQuietRetries == 0) {
                ETHERCAT_LOG_WARN("servo", "slave {}: bus still down, retrying every 2 s: {}", config_.slave_id, e.what());
            }
        }
    }
}

void ServoController::set_rpm(double rpm) {
    maybe_recover_bus();
    const std::shared_lock<std::shared_mutex> lk(api_mutex_);
    if (degraded_.load(std::memory_order_acquire)) {  // §8 Degraded-but-alive: motion APIs throw, never act
        throw Error("set_rpm unavailable: " + (degraded_reason_.empty() ? last_error() : degraded_reason_));
    }
    if (estop_engaged()) {
        throw Error("set_rpm: emergency stop engaged -- release it before commanding motion");
    }
    // Always switch-capable: set_rpm ensures PV at runtime, never rejected on mode. A PV setpoint
    // yields to a live blocking move (a go_for timed run) and is rejected "operation ongoing"
    // (including set_rpm(0); halt() is the stop verb). PV setpoints are latest-wins among themselves
    // (no slot), so this rejects only under a live blocking move.
    if (motion_slot_busy()) {
        throw Error("set_rpm: a motion operation is already in progress");
    }
    refuse_mode_switch_while_moving("set_rpm", MotionCommand::Velocity);
    push_velocity(rpm);
}

// Private: convert rpm to guarded device velocity and submit. No slot check and no lock (the caller
// -- set_rpm after its slot check, or go_for(PV) which owns the slot for its whole run -- holds both).
void ServoController::push_velocity(double rpm) noexcept {
    const double clamped = clamp_rpm(rpm, config_.max_motor_speed_rpm);
    std::int32_t dev = rpm_to_device_velocity(clamped, config_.counts_per_rev, config_.gear_ratio);
    dev = clamp_to_stop_budget(dev);  // stoppable-within-teardown-window guard (0x60FF)
    (void)commands_.push(Command{SetVelocity{dev}});
}

std::int32_t ServoController::clamp_to_stop_budget(std::int32_t vel_cps) const noexcept {
    // Velocity guard: a commanded velocity must be stoppable within the controlled-stop teardown
    // window, else a lifecycle-stop-while-moving would still be ramping when close() de-energizes,
    // cutting torque at speed. Clamp to the budget derived from that window (vel_budget_cps_). Clamp,
    // not reject: the motor turns at the ceiling, observably below the request. Applied to both the
    // PV setpoint (0x60FF/set_rpm) and the PP move speed (0x6081/go_to,go_for) -- same hazard, same
    // formula. Inert (budget 0) when quick-stop is not configured.
    const std::int64_t budget = vel_budget_cps_.load(std::memory_order_acquire);
    if (budget <= 0 || static_cast<std::int64_t>(std::abs(vel_cps)) <= budget) {
        return vel_cps;
    }
    return vel_cps >= 0 ? static_cast<std::int32_t>(budget) : -static_cast<std::int32_t>(budget);
}

bool ServoController::gen_terminal(std::uint32_t gen) const noexcept {
    // A blocking move is terminal once the RT publishes its completion OR its failure for that gen.
    // Generations are monotonic and completed/failed hold the LAST terminal gen, so equality is the
    // test (a stale earlier terminal never masks a live later gen).
    return gen != 0 && (state_.completed_generation.load(std::memory_order_acquire) == gen ||
                        state_.failed_generation.load(std::memory_order_acquire) == gen);
}

bool ServoController::motion_slot_busy() const noexcept {
    const std::uint32_t cur = motion_slot_.load(std::memory_order_acquire);
    return cur != 0 && !gen_terminal(cur);  // a LIVE (non-terminal) blocking move owns the slot
}

void ServoController::release_motion_slot(std::uint32_t gen) noexcept {
    std::uint32_t cur = gen;
    (void)motion_slot_.compare_exchange_strong(cur, 0U, std::memory_order_acq_rel, std::memory_order_acquire);  // only if still ours
}

bool ServoController::try_claim_motion_slot(std::uint32_t gen) noexcept {
    // Single-CAS claim: succeed only if the slot is free or holds an already-terminal gen (reclaim). A
    // concurrent second claimer that read the same terminal `cur` loses the CAS, reloads a live gen,
    // and returns false ("operation ongoing"). No check-then-claim race.
    std::uint32_t cur = motion_slot_.load(std::memory_order_acquire);
    for (;;) {
        if (cur != 0 && !gen_terminal(cur)) {
            return false;  // a LIVE blocking move owns it
        }
        if (motion_slot_.compare_exchange_weak(cur, gen, std::memory_order_acq_rel, std::memory_order_acquire)) {
            return true;
        }
        // CAS failed -> `cur` reloaded with the winner's gen; loop re-evaluates (live -> reject).
    }
}

void ServoController::await_move(std::uint32_t generation) {
    const std::uint32_t g = generation;
    // No timeout: a legitimately-long move must not be killed by a wall clock (there is no no-progress
    // watchdog either; a stuck move waits until the client stops it, the drive faults, or the RT loop
    // exits). A C++20 atomic wait on wake_seq_ (bumped and notified by every terminal transition, fault
    // onset, stop/reconfigure, and the RT loop's on_stop), lost-wakeup-immune by the seq re-check, no
    // mutex, RT never blocks. master_-free.
    for (;;) {
        const std::uint64_t seq = wake_seq_.load(std::memory_order_acquire);
        // Classify (order matters: this move's own completion first, then stop/dead, then its own abort.
        // Handshake-timeout sets failed_generation but not faulted, so check it before the generic
        // drive-fault branch; last_error() carries the precise reason; then a real drive/bus fault.)
        if (state_.completed_generation.load(std::memory_order_acquire) >= g ||
            state_.active_generation.load(std::memory_order_acquire) > g) {
            return;  // completed (or superseded by a newer move -- benign)
        }
        // Every failure path releases the single-in-flight slot: a move that will never complete
        // (RT loop gone, aborted, or the drive faulted before or during it) must not block later commands.
        if (stopping_.load(std::memory_order_acquire) || rt_exited_.load(std::memory_order_acquire)) {
            release_motion_slot(g);
            throw Error("move: controller stopped / RT loop not alive");
        }
        if (state_.failed_generation.load(std::memory_order_acquire) >= g) {
            release_motion_slot(g);
            throw Error("move aborted (" + last_error() + ")");
        }
        if (state_.faulted.load(std::memory_order_acquire)) {
            release_motion_slot(g);
            throw Error("move: drive faulted (" + last_error() + ")");
        }
        wake_seq_.wait(seq, std::memory_order_acquire);  // block until a wake bump (or a spurious wake -> re-check)
    }
}

// SDO-write the standard homing objects, then the configured torque object. Non-RT, run before the
// RT loop is told to start: a refused object fails home() with the drive's abort code.
std::vector<std::byte> ServoController::write_homing_objects() {
    const ServoConfig::Homing& h = config_.homing;
    const auto write = [this](std::uint16_t index, std::uint8_t sub, auto value) {
        std::array<std::byte, sizeof(value)> raw{};
        store_le<decltype(value)>(raw, value);
        sdo_write(index, sub, raw);
    };
    const double cpr = config_.counts_per_rev;
    const double ratio = config_.gear_ratio;
    // Resolve everything the config must supply first: a request it cannot satisfy must fail before
    // any homing object is written, not halfway through.
    if (h.method == 0) {
        throw Error("home: needs the 'homing' config attribute (its 0x6098 method is drive-specific, so it cannot be guessed)");
    }
    const sdo_codec::TypeInfo* torque_type = nullptr;
    if (h.torque_percent > 0.0) {
        torque_type = sdo_codec::find_type(h.torque.type);
        if (torque_type == nullptr) {
            throw Error("home: unknown homing torque object type '" + h.torque.type + "'");
        }
    }
    // Save the torque object before clamping it: on a drive whose only torque limit is the global
    // one (no homing-scoped object), leaving the clamp in place would cap every later move.
    std::vector<std::byte> saved;
    if (torque_type != nullptr) {
        saved.resize(torque_type->width);
        saved.resize(sdo_read(h.torque.index, h.torque.sub, saved));
    }
    // Prove the drive took the method rather than trusting the download: one that ignores an
    // unknown value would otherwise home by whatever method it kept -- into another stop.
    write(0x6098, 0, h.method);
    std::array<std::byte, 4> echo{};  // wider than the INT8 object: a width mismatch is data, not a transport failure
    const std::size_t read = sdo_read(0x6098, 0, echo);
    if (read != 1 || static_cast<std::int8_t>(echo[0]) != h.method) {
        throw Error("home: the drive refused homing method " + std::to_string(h.method) + " (0x6098 reads back " +
                    std::to_string(static_cast<int>(static_cast<std::int8_t>(echo[0]))) + " in " + std::to_string(read) +
                    " bytes) -- it does not support it");
    }
    write(0x6099, 1, static_cast<std::uint32_t>(rpm_to_device_velocity(h.speed_rpm, cpr, ratio)));
    if (h.zero_speed_rpm > 0.0) {
        write(0x6099, 2, static_cast<std::uint32_t>(rpm_to_device_velocity(h.zero_speed_rpm, cpr, ratio)));
    }
    if (h.offset_counts != 0) {
        write(0x607C, 0, h.offset_counts);
    }
    if (h.accel_rpm_per_s > 0.0) {
        write(0x609A, 0, accel_to_device(rpm_to_device_velocity(h.accel_rpm_per_s, cpr, ratio), accel_units_per_cps2()));
    }
    if (torque_type != nullptr) {
        sdo_codec::Input in;
        // Round: the product is a unit conversion, and the codec refuses a fraction. A percent that
        // does not land on an exact multiple of the object's resolution is a rounding question, not
        // an error.
        in.number = std::round(h.torque_percent * h.torque.units_per_percent);
        sdo_write(h.torque.index, h.torque.sub, torque_type->encode(in));
    }
    return saved;
}

// Put the torque object back after the run. Best-effort and never throws: it is cleanup on both the
// success and the failure path, and a refused restore must not mask the homing result.
void ServoController::restore_homing_torque(std::span<const std::byte> saved) noexcept {
    if (saved.empty()) {
        return;
    }
    try {
        sdo_write(config_.homing.torque.index, config_.homing.torque.sub, saved);
    } catch (const std::exception& e) {
        ETHERCAT_LOG_ERROR("servo",
                           "home: restoring the torque limit {} failed -- the drive is still clamped: {}",
                           hex(config_.homing.torque.index),
                           e.what());
    }
}

// A search re-bases what the drive reports: the position jumps by the whole distance to the old
// origin in one cycle, while the encoder under it does not move. Separate the two by magnitude
// against the speed ceiling, and total it so the origin can be put in the encoder's frame.
void ServoController::track_frame_rebase(std::int32_t actual, bool in_homing) noexcept {
    // Only a search re-bases the frame, so only a search is watched. Outside one, drop the history:
    // a bring-up's first process-data frames step from zero to the live position, which is not a
    // re-base and must not be counted as one, and every run must measure from the frame it starts in
    // rather than from whatever the frame was when the driver came up.
    if (!in_homing) {
        have_last_actual_ = false;
        return;
    }
    if (!have_last_actual_) {
        frame_rebase_ = 0;
        last_actual_ = actual;
        have_last_actual_ = true;
        return;
    }
    const std::int64_t delta = static_cast<std::int64_t>(actual) - static_cast<std::int64_t>(last_actual_);
    last_actual_ = actual;
    if (delta > rebase_threshold_ || delta < -rebase_threshold_) {
        frame_rebase_ += delta;  // not travel: the frame moved under us
    }
}

// Select a CiA402 mode on the drive by SDO (non-RT). In a map that carries 0x6060 the RT loop writes
// the same value every cycle and the PDO governs; this is what moves a drive whose map does not.
void ServoController::select_drive_mode(Cia402Mode mode) {
    const std::array<std::byte, 1> v{static_cast<std::byte>(static_cast<std::uint8_t>(mode))};
    sdo_write(0x6060, 0, v);
}

// Park until the RT loop leaves Homing. Same wake protocol as await_move: no clock, the timeout is
// the RT loop's own cycle bound.
bool ServoController::await_homing() {
    for (;;) {
        const std::uint64_t seq = wake_seq_.load(std::memory_order_acquire);
        const auto st = static_cast<HomingState>(state_.homing_state.load(std::memory_order_acquire));
        if (st != HomingState::Running) {
            return st == HomingState::Done;
        }
        if (stopping_.load(std::memory_order_acquire) || rt_exited_.load(std::memory_order_acquire)) {
            throw Error("home: controller stopped / RT loop not alive");
        }
        wake_seq_.wait(seq, std::memory_order_acquire);
    }
}

void ServoController::publish_homing_result(HomingFail fail) noexcept {
    state_.homing_fail.store(static_cast<std::uint8_t>(fail), std::memory_order_relaxed);
    state_.homing_state.store(static_cast<std::uint8_t>(fail == HomingFail::None ? HomingState::Done : HomingState::Failed),
                              std::memory_order_release);
    bump_wake();  // home() is parked on this state, not on a clock
}

void ServoController::finish_homing(HomingFail fail) noexcept {
    publish_homing_result(fail);
    homing_cycles_ = 0;
    homing_echo_wait_ = 0;
    motion_->reset();  // no command survives a home: the run ends in a hold at the new origin
    // Back through the enable ladder, not straight to Operational: the drive may still be running
    // mode 6, and only the ladder waits for 0x6061 to echo the mode home() restores. Until it does,
    // the not-operational path keeps the set-points tracking the actual position, so re-energizing
    // at the new zero cannot lunge. Same path a cleared fault takes.
    lifecycle_ = Enabling{};
    mode_echo_wait_ = 0;
}

// Store the origin a home found where the drive will find it again after power-on:
//
//     new = old - (reported at origin - re-base during the search)
//
// The bracketed term is the origin in the ENCODER's frame; the reported frame is re-based by a
// search, the encoder under it is not. Writing the offset alone leaves it in RAM, so the write is
// followed by the parameter store.
ServoController::EncoderZeroWrite ServoController::save_encoder_zero() {
    const std::optional<EncoderOriginObject> obj = encoder_origin_object();
    if (!obj) {
        return EncoderZeroWrite{};  // a generic CiA402 drive has nowhere to keep an origin; nothing to do
    }
    const EncoderOriginObject& d = *obj;
    if (!homed()) {
        throw Error("save_encoder_zero: the axis is not homed, so there is no origin to keep -- home it first");
    }
    // Required, not forced: the store is refused with the servo on, and de-energizing an axis is the
    // caller's decision to make, not a side effect of asking for the datum to be kept.
    // Read from the process image the RT loop already publishes every cycle, not by asking the
    // drive: the store is refused outside Switch On Disabled, and an SDO poll to find that out adds
    // mailbox traffic exactly when the drive is being asked to change state.
    if (!state_.switch_on_disabled.load(std::memory_order_acquire)) {
        throw Error(
            "save_encoder_zero: the drive must be in Switch On Disabled -- the parameter store is refused in any other state. "
            "Disable the drive first (the `disable` do_command) and do not move the axis in between");
    }
    const sdo_codec::TypeInfo* const type = sdo_codec::find_type(d.type);
    if (type == nullptr) {
        throw Error(std::string("save_encoder_zero: unknown origin object type '") + d.type + "'");
    }
    std::vector<std::byte> buf(type->width);
    buf.resize(sdo_read(d.index, d.sub, buf));
    const std::optional<double> was = type->decode(buf).number;
    if (!was) {
        throw Error("save_encoder_zero: the origin object " + hex(d.index) +
                    " did not read back as a number, so the offset that keeps "
                    "the origin cannot be derived from it");
    }
    if (!state_.home_encoder_rel_valid.load(std::memory_order_acquire)) {
        throw Error(
            "save_encoder_zero: the reported position has already been re-based by an earlier homing run this session, so the "
            "origin cannot be expressed in the encoder's frame. Restart the module (or cycle control power), home once, "
            "and keep the datum from that run");
    }
    const std::int64_t enc_rel = state_.home_encoder_rel.load(std::memory_order_relaxed);
    const double want = *was - static_cast<double>(enc_rel);
    // Stored every run, even when the new origin matches the old one. Skipping a store that would
    // change nothing looked free, but it decides from the derived offset whether the drive is
    // already right -- and a run that derives the offset wrongly would then also skip the store
    // that would have corrected it. The store costs milliseconds and the drive energizes again
    // straight after, so there is nothing to buy.
    sdo_codec::Input in;
    in.number = want;
    sdo_write(d.index, d.sub, type->encode(in));
    // Read back rather than trust the download: a refused offset reads as homed now and as lost on
    // the next power-up, which is the failure this exists to prevent.
    std::vector<std::byte> echo(type->width);
    echo.resize(sdo_read(d.index, d.sub, echo));
    const std::optional<double> got = type->decode(echo).number;
    if (!got || *got != want) {
        throw Error("save_encoder_zero: the drive did not take the encoder origin written to " + hex(d.index) + " (wrote " +
                    std::to_string(want) + ", reads back " + (got ? std::to_string(*got) : std::string("non-numeric")) + ")");
    }
    store_parameters();
    ETHERCAT_LOG_INFO("servo",
                      "slave {}: datum: was {} now {} origin_in_encoder_frame {} object {} -- stored to non-volatile memory; "
                      "cycle control power, or restart the module, before energizing again",
                      config_.slave_id,
                      *was,
                      want,
                      enc_rel,
                      hex(d.index));
    return EncoderZeroWrite{*was, want, enc_rel};
}

// CoE 0x1010:01 "save all parameters". The signature guards against storing by mistake: the drive
// aborts any value but the ASCII "save", so a wrong one is refused rather than silently ignored.
//
// The store is ASYNCHRONOUS, and waiting for it is not optional. The object reads 0 for as long as
// the save is running, so a caller that writes the signature and moves on can take the bus down
// mid-write -- and then the parameter is in RAM only, which reads back correct and looks like a
// successful store right up until the next power cycle, when it is gone.
void ServoController::store_parameters() {
    constexpr std::uint32_t kSaveSignature = 0x65766173;  // 's','a','v','e' little-endian
    std::array<std::byte, 4> raw{};
    store_le<std::uint32_t>(raw, kSaveSignature);
    const auto started = std::chrono::steady_clock::now();
    sdo_write(0x1010, 1, raw);
    // The download is answered when the drive has ACCEPTED the signature, not when the write to
    // non-volatile memory is done -- CiA 301 has the object read back 0 for exactly as long as the
    // save is running. Returning on the download instead leaves the parameter in RAM, where it
    // reads back correct and looks stored right up until the next power cycle, when it is gone.
    for (int i = 0; i < kStorePolls; ++i) {
        std::this_thread::sleep_for(kStorePoll);
        std::array<std::byte, 4> back{};
        try {
            sdo_read(0x1010, 1, back);
        } catch (const Error&) {
            continue;  // a refused mailbox while the write is in flight is the save running, not a failure
        }
        if (load_le<std::uint32_t>(back) != 0) {
            ETHERCAT_LOG_INFO("servo",
                              "slave {}: parameter store complete after {} ms",
                              config_.slave_id,
                              std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - started).count());
            return;
        }
    }
    throw Error("store_parameters: the drive is still reporting a save in progress (1010h:01 reads 0) after " +
                std::to_string(kStorePolls * kStorePoll.count()) +
                " ms -- the parameters are in RAM and will be lost at the next power cycle, so do not cut control power yet");
}

bool ServoController::home() {
    maybe_recover_bus();
    std::uint32_t g = 0;
    std::uint32_t timeout_cycles = 0;
    {
        const std::shared_lock<std::shared_mutex> lk(api_mutex_);
        if (degraded_.load(std::memory_order_acquire)) {
            throw Error("home unavailable: " + (degraded_reason_.empty() ? last_error() : degraded_reason_));
        }
        if (estop_engaged()) {
            throw Error("home: emergency stop engaged -- release it before homing");
        }
        if (!state_.powered.load(std::memory_order_acquire)) {
            throw Error("home: the drive is not energized");
        }
        timeout_cycles = static_cast<std::uint32_t>(config_.homing.timeout_s * static_cast<double>(config_.target_loop_rate_hz));
        g = next_generation_.fetch_add(1, std::memory_order_relaxed) + 1;
        if (!try_claim_motion_slot(g)) {  // homing owns the single-in-flight slot for the whole run
            throw Error("home: a motion operation is already in progress");
        }
    }
    bool attained = false;
    std::vector<std::byte> saved_torque;
    try {
        saved_torque = write_homing_objects();
        select_drive_mode(Cia402Mode::Homing);
        state_.homing_fail.store(static_cast<std::uint8_t>(HomingFail::None), std::memory_order_relaxed);
        state_.homing_state.store(static_cast<std::uint8_t>(HomingState::Running), std::memory_order_release);
        {
            const std::shared_lock<std::shared_mutex> lk(api_mutex_);
            (void)commands_.push(Command{StartHoming{g, timeout_cycles}});
        }
        attained = await_homing();
        if (attained && config_.homing.save_zero_after_home) {
            disable();  // the store is refused with the servo on
            for (int i = 0; i < kStoreDisableWaitCycles && !state_.switch_on_disabled.load(std::memory_order_acquire); ++i) {
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
            }
            save_encoder_zero();
            // The store can re-base what the drive reports, and the axis has not moved since the run
            // attained home -- so whatever it reports now IS the origin, whichever frame the drive
            // decided to serve. Re-point the home reference at it and energize again: home() took the
            // drive down for the store, so home() brings it back, and the caller gets a usable axis
            // instead of a dead one and an instruction to restart.
            state_.home_counts.store(state_.position_counts.load(std::memory_order_acquire), std::memory_order_relaxed);
            enable();
        }
    } catch (...) {
        state_.homing_state.store(static_cast<std::uint8_t>(HomingState::Failed), std::memory_order_release);
        // Restore the running mode whatever happened, so a failed home never leaves the drive in
        // homing mode with the enable ladder waiting for an echo that will not come.
        try {
            select_drive_mode(commanded_cia402_mode());
        } catch (const std::exception& e) {
            ETHERCAT_LOG_ERROR("servo", "home: restoring the running mode failed: {}", e.what());
        }
        restore_homing_torque(saved_torque);
        release_motion_slot(g);
        throw;
    }
    // Free the slot before the restore, which can throw on a refused SDO: a home that attained and
    // then failed to restore the mode must not also leave the slot held, refusing every later move.
    release_motion_slot(g);
    restore_homing_torque(saved_torque);
    select_drive_mode(commanded_cia402_mode());
    ETHERCAT_LOG_INFO("servo", "slave {}: homing {}", config_.slave_id, attained ? "attained" : homing_error());
    return attained;
}

bool ServoController::homed() const noexcept {
    return state_.homed.load(std::memory_order_acquire);
}

std::int32_t ServoController::home_position_counts() const noexcept {
    return state_.home_counts.load(std::memory_order_acquire);
}

double ServoController::home_position_revs() const noexcept {
    const std::shared_lock<std::shared_mutex> lk(api_mutex_);  // config_ conversion params, as position_revs()
    return counts_to_revs(home_position_counts(), config_.counts_per_rev, config_.gear_ratio);
}

HomingState ServoController::homing_state() const noexcept {
    return static_cast<HomingState>(state_.homing_state.load(std::memory_order_acquire));
}

std::string ServoController::homing_error() const {
    if (static_cast<HomingState>(state_.homing_state.load(std::memory_order_acquire)) != HomingState::Failed) {
        return {};
    }
    return to_string(static_cast<HomingFail>(state_.homing_fail.load(std::memory_order_relaxed)));
}

void ServoController::go_to(double rpm, double position) {
    maybe_recover_bus();
    std::uint32_t g = 0;
    {
        const std::shared_lock<std::shared_mutex> lk(api_mutex_);
        if (degraded_.load(std::memory_order_acquire)) {  // §8
            throw Error("go_to unavailable: " + (degraded_reason_.empty() ? last_error() : degraded_reason_));
        }
        if (estop_engaged()) {
            throw Error("go_to: emergency stop engaged -- release it before commanding motion");
        }
        // Always switch-capable: go_to ensures PP at runtime, never rejected on mode. Absolute target
        // in the zeroed frame: add zero_offset_counts to map the user's zeroed position to the raw
        // encoder frame, so go_to(X) lands where position_revs() == X (get_position is zeroed too).
        const std::int32_t counts = static_cast<std::int32_t>(revs_to_counts(position, config_.counts_per_rev, config_.gear_ratio) +
                                                              state_.zero_offset_counts.load(std::memory_order_acquire));
        if (!std::isfinite(rpm) || rpm <= 0.0) {
            // A zero speed can never complete (the drive's profile generator and ours would both sit
            // at the start forever) and a blocking call would park until cancelled: reject it.
            throw Error("go_to: rpm must be a positive number (got " + std::to_string(rpm) + ")");
        }
        const double clamped_rpm = clamp_rpm(rpm, config_.max_motor_speed_rpm);
        const std::int32_t prof = clamp_to_stop_budget(rpm_to_device_velocity(clamped_rpm, config_.counts_per_rev, config_.gear_ratio));
        refuse_mode_switch_while_moving("go_to", MotionCommand::Position);
        g = next_generation_.fetch_add(1, std::memory_order_relaxed) + 1;
        if (!try_claim_motion_slot(g)) {  // single-in-flight: a live blocking move already owns the slot
            throw Error("go_to: a motion operation is already in progress");
        }
        (void)commands_.push(Command{SetTarget{counts, static_cast<std::uint32_t>(std::abs(prof)), false, g}});
    }  // release the shared lock before parking (so reconfigure isn't blocked for the whole move)

    await_move(g);
}

void ServoController::go_for(double rpm, double revs) {
    maybe_recover_bus();
    // Always switch-capable: go_for is always a relative PP move (ensure PP, target = actual + delta).
    // set_rpm remains the PV-jog verb.
    std::uint32_t g = 0;
    {
        const std::shared_lock<std::shared_mutex> lk(api_mutex_);
        if (degraded_.load(std::memory_order_acquire)) {  // §8
            throw Error("go_for unavailable: " + (degraded_reason_.empty() ? last_error() : degraded_reason_));
        }
        if (estop_engaged()) {
            throw Error("go_for: emergency stop engaged -- release it before commanding motion");
        }
        // Relative move (frame-agnostic): push SetTarget{relative=true} so the FSM computes target =
        // actual + delta. Do not route through go_to, which adds zero_offset (absolute frame) and
        // would double-shift a relative move.
        if (!std::isfinite(rpm) || rpm == 0.0) {
            throw Error("go_for: rpm must be non-zero (got " + std::to_string(rpm) + ")");  // a zero profile speed never completes
        }
        const std::int32_t delta =
            revs_to_counts(rpm < 0.0 ? -revs : revs, config_.counts_per_rev, config_.gear_ratio);  // sign(rpm) flips the direction
        const double clamped_rpm = clamp_rpm(std::abs(rpm), config_.max_motor_speed_rpm);
        const std::int32_t prof = rpm_to_device_velocity(clamped_rpm, config_.counts_per_rev, config_.gear_ratio);
        refuse_mode_switch_while_moving("go_for", MotionCommand::Position);
        g = next_generation_.fetch_add(1, std::memory_order_relaxed) + 1;
        if (!try_claim_motion_slot(g)) {  // single-in-flight
            throw Error("go_for: a motion operation is already in progress");
        }
        (void)commands_.push(Command{SetTarget{delta, static_cast<std::uint32_t>(std::abs(prof)), true, g}});
    }
    await_move(g);
}

// A command that needs a different CiA402 mode than the drive currently runs is only accepted with
// the motor at rest: the driver never brings the motor to rest on its own to change mode. Caller
// holds the shared lock (motion_ valid).
void ServoController::refuse_mode_switch_while_moving(const char* verb, MotionCommand cmd) const {
    if (!f_mode_disp_) {
        return;  // no 0x6061: the current mode is unknown; the mode's echo gate is inert too
    }
    const auto want = static_cast<std::int8_t>(motion_->mode_required_by(cmd));
    const std::int8_t current = state_.mode_echo.load(std::memory_order_acquire);
    if (current != 0 && current != want && state_.moving.load(std::memory_order_acquire)) {
        throw Error(std::string(verb) + ": the motor is moving in mode " + std::to_string(static_cast<int>(current)) +
                    " and this command needs mode " + std::to_string(static_cast<int>(want)) + " -- stop the motor before changing mode");
    }
}

void ServoController::halt() noexcept {
    const std::shared_lock<std::shared_mutex> lk(api_mutex_);
    (void)commands_.push(Command{Halt{}});
}

void ServoController::request_fault_reset() noexcept {
    const std::shared_lock<std::shared_mutex> lk(api_mutex_);
    // Always accepted, including while the stop is engaged: a drive that latches the stop keeps
    // reporting its code after release, and this edge is what clears it.
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
    // Make the current actual read offset_revs: zero = current - offset_in_counts. Reads config_
    // conversion params, so takes the shared lock (against reconfigure's swap).
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
    return stopping_.load(std::memory_order_acquire) || rt_exited_.load(std::memory_order_acquire);
}

std::uint64_t ServoController::loop_cycle() const noexcept {
    return state_.loop_cycle.load(std::memory_order_relaxed);
}

std::int32_t ServoController::velocity_counts() const noexcept {
    return state_.velocity.load(std::memory_order_relaxed);
}

std::size_t ServoController::sdo_read(std::uint16_t index, std::uint8_t sub, std::span<std::byte> out, std::chrono::milliseconds timeout) {
    // The CoE read runs directly on this (non-RT) caller thread, concurrent with the RT PDO loop
    // (SOEM v2 port is thread-safe). The shared lock serializes against reconfigure()'s exclusive
    // master_.reset() so master_ cannot be reset mid-transfer; the read blocks holding the shared
    // lock, bounded by the backend's mailbox timeout, so reconfigure() waits at most that long.
    // `timeout` is retained on the signature but no longer drives a wait; the transfer's own SOEM
    // timeout bounds it.
    (void)timeout;
    const std::shared_lock<std::shared_mutex> lk(api_mutex_);
    // Gate on the running state (rt_runner_), not just master_ != null. stop() resets rt_runner_ but
    // keeps master_, and after stop() the bus is closed, so a direct SDO must fail cleanly rather than
    // return stale data or hit a closed port; refuse unless a Runner is live. rt_runner_ is written
    // only under the exclusive api_mutex_ (start/stop/reconfigure), so this shared-lock read is safe.
    if (master_ == nullptr || rt_runner_ == nullptr) {
        throw Error("ServoController::sdo_read: not running -- call while operational (object " + std::to_string(index) + ":" +
                    std::to_string(sub) + ")");
    }
    return master_->sdo_read(config_.slave_id, index, sub, out);
}

void ServoController::sdo_write(std::uint16_t index, std::uint8_t sub, std::span<const std::byte> data) {
    const std::shared_lock<std::shared_mutex> lk(api_mutex_);
    if (master_ == nullptr || rt_runner_ == nullptr) {
        throw Error("ServoController::sdo_write: not running -- call while operational (object " + std::to_string(index) + ":" +
                    std::to_string(sub) + ")");
    }
    master_->sdo_write(config_.slave_id, index, sub, data);
}

double ServoController::rated_current_amps() const noexcept {
    const std::shared_lock<std::shared_mutex> lk(api_mutex_);
    return config_.motor_rated_current_amps;
}

std::string ServoController::fault_description(std::uint16_t /*code*/) const {
    // The generic base has no device gloss, so it returns empty and last_error() shows just the bare
    // hex (never wrong, just less descriptive). A device subclass overrides this to name its
    // vendor codes. Cold path.
    return {};
}

std::string ServoController::last_error() const {
    // Cold but lock-free and master_-free (symmetric with is_powered/is_moving): read the three
    // published tier flags (acquire) and their payloads, composing every active tier so a both-true
    // sync-loss reports root cause and symptom. (fault_description reads config_, taking the shared lock,
    // which is fine on this non-RT path.)
    std::string out;
    const auto append = [&out](const std::string& s) {
        if (!out.empty()) {
            out += "; ";
        }
        out += s;
    };

    // Drive (root cause), pair read: flag acquire, then code relaxed.
    if (state_.drive_faulted.load(std::memory_order_acquire)) {
        const std::uint16_t code = state_.drive_fault_code.load(std::memory_order_relaxed);
        if (code != 0 && code == config_.estop_fault_code) {
            append("emergency stop engaged (0x603F " + hex(code) + ")");
        } else if (code != 0) {
            const std::string gloss = fault_description(code);
            append("drive fault " + hex(code) + (gloss.empty() ? "" : " (" + gloss + ")"));
        } else {
            append("drive fault (code pending)");
        }
    }
    // Bring-up AL refusal: the drive refused an AL state transition at bring-up (e.g. AL 0x0027
    // "Freerun not supported" when a DC-only drive is requested into OP without SYNC0). Distinct from
    // the 0x603F drive fault above; it names the cause that a bare "drive not operational" hides.
    if (const std::uint16_t al = state_.bringup_al_code.load(std::memory_order_relaxed); al != 0) {
        std::string msg = "drive refused OP: AL " + hex(al);
        if (al == 0x0027 && !config_.use_distributed_clocks) {
            msg += " (freerun not supported -- set use_distributed_clocks=true for this DC-only drive)";
        }
        append(msg);
    }
    // Bus (symptom + recovery).
    if (state_.wkc_faulted.load(std::memory_order_acquire)) {
        append("EtherCAT working-counter fault: got " + std::to_string(state_.fault_wkc.load(std::memory_order_relaxed)) + ", expected " +
               std::to_string(state_.expected_wkc.load(std::memory_order_relaxed)) + " -- bus re-init required");
    }
    // Controller (latched controller error).
    switch (rt_error_.load(std::memory_order_acquire)) {
        case RtError::HandshakeTimeout:
            append("Profile-Position set-point acknowledge timed out");
            break;
        case RtError::NotOperational:
            append("drive not operational");
            break;
        case RtError::MotorStopped:
            append("motor stopped");
            break;
        case RtError::MotorDisabled:
            append("motor disabled");
            break;
        case RtError::ModeMismatch:
            append("drive mode-of-operation (0x6061) did not follow the commanded mode");
            break;
        case RtError::RtSetupFailed:
            append(rt_unavailable_message(config_.rt_priority));
            break;
        case RtError::None:
            break;
    }
    return out;
}

}  // namespace ethercat::servo
