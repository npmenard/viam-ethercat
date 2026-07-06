#include "viam/lib/servo_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <utility>

#include "ethercat/errors.hpp"
#include "ethercat/pdo_buffer.hpp"
#include "ethercat/realtime.hpp"
#include "ethercat/soem_backend.hpp"
#include "ethercat/util.hpp"
#include "viam/lib/motion_profile.hpp"

namespace ethercat::servo {

namespace {

// A drive's "no-sync" fault code is a device seam (sync_fault_code(), nullopt in this generic
// base), not a constant here, so this core carries no vendor value. The bring-up gate reads it
// through the seam (nullopt means no detection).
// Controlled-stop watchdog headroom: the teardown window and velocity budget reserve this much
// time below the full window so the ramp finishes before close() (a slave's sync/SM watchdog is
// typically a few tens of ms).
constexpr double kStopWindowMarginS = 0.05;
// Driver-owned mode-switch bounds (cycles; internal constants, no config knob -- the switch is a
// bounded hold, never a de-energize deadline). kModeSwitchStopCycles: max cycles to ramp the current
// mode to rest before switching (a load resisting stop gives up rather than switch mid-motion).
// kModeSwitchSettleCycles: max cycles to await the 0x6061 echo of the new mode before giving up.
constexpr std::uint32_t kModeSwitchStopCycles = 1000;
constexpr std::uint32_t kModeSwitchSettleCycles = 200;
// Bring-up retry: attempt OP up to this many times, clearing drive errors and rebuilding the master
// (INIT bounce) between attempts. Bounded, because a persistent fault must give up rather than hammer
// (repeated failed OP entries wedge some drives' EtherCAT interface until a power cycle).
// kBringupWaitCap is a defensive backstop above the
// Runner's own bring-up bound (120s), so the async-outcome poll never hangs if a signal is missed.
constexpr unsigned kMaxBringupAttempts = 5;
constexpr std::chrono::milliseconds kBringupWaitCap{130'000};

Cia402Mode to_cia402_mode(ControlMode mode) noexcept {
    return mode == ControlMode::ProfileVelocity ? Cia402Mode::ProfileVelocity : Cia402Mode::ProfilePosition;
}

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
    // per-cycle PDO mode from the driver's switch_intent_ governs at runtime; this is only the
    // pre-cycling default.
    slave.default_mode = Cia402Mode::ProfilePosition;
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
        (void)std::fprintf(stderr,
                           "[servo] vendor fault-reset SDO (slave %u 0x%04X:%02X) failed (continuing): %s\n",
                           static_cast<unsigned>(config_.slave_id),
                           static_cast<unsigned>(reset->index),
                           static_cast<unsigned>(reset->subindex),
                           e.what());
    }
}

ServoController::ServoController(ServoConfig config)
    : config_(validated(std::move(config))),
      // The sequencer carries only the two quick-stop values: the 0x6085 decel from config (0 makes
      // configure() skip the quick-stop SDO setup, so stop coasts) and the 0x605A option, asserted
      // to be 2 (decelerate, then auto-transition to SwitchOnDisabled). is-moving/reached and the
      // mode-switch are the driver's.
      sequencer_(config_.quick_stop_decel, 2),
      commands_(config_.command_queue_capacity) {}

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
    if (config_.require_realtime && !realtime::sched_fifo_available(config_.rt_priority)) {
        throw Error(rt_unavailable_message(config_.rt_priority));
    }

    // configure() reaches SAFE-OP and does no memory lock (residency is RT-setup's job, not
    // thread-free bus policy). The RT thread then runs the DC bring-up prelude (settle, request OP,
    // await OP) to Operational; realtime::setup() does the full MCL_CURRENT|MCL_FUTURE in-thread,
    // post-spawn. An in-thread, post-spawn MCL_FUTURE never sees this thread's later jthread stack
    // alloc, and nothing cyclic runs before that in-thread lock, so no SYNC0-critical page-fault
    // window opens.
    master_ = std::make_unique<Master>(build_master_config(config_), std::make_unique<SoemBackend>());
    master_->init();
    master_->configure();  // -> SAFE-OP (may throw Error; the SDK retries)
    bring_up();            // resolve, clear errors, and reach OP, with bounded retry
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
    last_cw_ = 0;
    sequencer_.reset();  // clear the shared sequencer's per-run sequencing state (handshake/latches) for reuse
    prev_actual_ = 0;
    first_cycle_ = true;
    halted_ = false;
    stop_at_rest_ = false;
    switch_phase_ = SwitchPhase::None;  // no mode-switch in flight on a fresh run
    switch_cycles_ = 0;
    at_rest_ = false;
    pending_new_setpoint_ = false;  // no armed handshake on a fresh run
    latched_ctrl_error_ = RtError::None;
    last_sync_code_ = 0;
    rt_exited_.store(false, std::memory_order_release);  // event-based RT aliveness: fresh run, loop is live
}

// The Runner's stopping-window cap (cycles): sized so a quick-stop ramp completes before
// close()->INIT de-energizes (no torque-cut at speed). One source of truth with the velocity budget.
std::uint32_t ServoController::teardown_window_cycles() const noexcept {
    // Opt-out (no controlled stop): disable-voltage coast is instant, so a 2-cycle window.
    if (config_.quick_stop_decel == 0) {
        return 2;
    }
    // decel>0: size the window to the controlled-stop budget so the quick-stop ramp completes before
    // close()->INIT (no torque-cut). window_cycles = controlled_stop_window_ms x loop_rate. The event
    // gate (teardown_complete) exits earlier once at rest; this is the hard cap.
    const std::uint64_t rate = config_.target_loop_rate_hz;
    const std::uint64_t cyc = (static_cast<std::uint64_t>(config_.controlled_stop_window_ms) * rate + 999ULL) / 1000ULL;
    return static_cast<std::uint32_t>(std::max<std::uint64_t>(cyc, 2ULL));
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
        degraded_reason_ = std::string("ServoController degraded at start: ") + e.what();
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
        resolve_fields();
        run_vendor_fault_reset();  // clear drive errors before bring-up (device seam; single port owner)
        if (attempt_bringup()) {
            return;  // reached OP
        }
        if (attempt >= kMaxBringupAttempts) {
            (void)std::fprintf(stderr,
                               "[servo] slave %u: bring-up FAILED after %u attempts -- giving up (Degraded): %s\n",
                               static_cast<unsigned>(config_.slave_id),
                               attempt,
                               last_error().c_str());
            return;  // Degraded: degraded_ and the AL/fault tier are already set by on_stop(BringupAborted)
        }
        (void)std::fprintf(stderr,
                           "[servo] slave %u: bring-up attempt %u failed -- clearing errors + retrying...\n",
                           static_cast<unsigned>(config_.slave_id),
                           attempt);
        // Full recovery for the next attempt: drop the Runner (join the exited RT thread, close()->INIT)
        // and master, then rebuild (INIT bounce, PRE-OP settle, DC re-arm). Requests OP once next attempt.
        rt_runner_.reset();
        master_.reset();
        master_ = std::make_unique<Master>(build_master_config(config_), std::make_unique<SoemBackend>());
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
    if (config_.require_realtime && !realtime::sched_fifo_available(config_.rt_priority)) {
        throw Error(rt_unavailable_message(config_.rt_priority));
    }
    master_ = std::make_unique<Master>(build_master_config(config_), std::make_unique<SoemBackend>());
    master_->init();
    master_->configure();
    bring_up();  // resolve, clear errors, and reach OP, with bounded retry
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
    // A PV motion-hold locks position (not just zero velocity) by switching to PP-at-current-counts.
    // Available when the map carries 0x6060 and 0x607A. The runtime halt handler further gates
    // on the current intent being PV (a PP halt already holds in PP, no switch).
    pv_hold_capable_ = f_mode_wr_.has_value() && master_->try_resolve_rx<cia402::TargetPosition>(s).has_value();

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
    if (config_.quick_stop_decel == 0) {
        (void)std::fprintf(stderr,
                           "[servo] slave %u: quick_stop_decel not set -> STOP is an UNCONTROLLED disable-voltage "
                           "coast (safe, but a load-holding axis will drift/drop). Set quick_stop_decel (0x6085) for a "
                           "controlled ramp-stop.\n",
                           static_cast<unsigned>(s));
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
    // Always switch-capable: command the current intent (go_to/go_for -> PP, set_rpm -> PV). The
    // wrapper (run_mode_switch) runs the switch when this differs from the drive's confirmed 0x6061.
    return to_cia402_mode(switch_intent_);
}

// Driver-owned runtime mode-switch. Holds energized (cw 0x0F) throughout: "no motion" during a switch
// is not a de-energize. Stopping: ramp the current mode to rest (driver rest detection, at_rest_) then
// advance. Settling: command the target mode via the sequencer (which writes 0x6060 = cmd.mode) plus a
// safe seed, and confirm the 0x6061 echo. The hold token is constant through the switch so the sequencer's
// handshake settles to Idle (a clean cw=0x0F hold, no re-armed bit 4). A drive fault during the window
// is handled by step_lifecycle's Fault branch, not here.
std::uint16_t ServoController::run_mode_switch(CycleContext& ctx, std::int32_t actual, Cia402Mode want) noexcept {
    const std::int8_t want_i8 = static_cast<std::int8_t>(want);
    const std::int8_t confirmed = ctx.load<cia402::ModeDisplay::type>(*f_mode_disp_);  // 0x6061 echo THIS cycle (caller guards mapped)
    SequencerCommand pcmd;
    pcmd.enable = true;
    pcmd.halt = false;
    pcmd.profile_velocity = profile_vel_;
    pcmd.target_counts = actual;  // PP hold at the current position (no lunge)
    pcmd.target_velocity = 0;     // PV ramp to 0
    pcmd.new_setpoint = false;    // hold: never arm the handshake during a switch (the sequencer settles to Idle)

    if (switch_phase_ == SwitchPhase::Stopping) {
        // Command the current confirmed mode so the drive stays in its present control loop while it ramps
        // to rest (don't write the new 0x6060 until the motor has stopped, to avoid an in-motion switch).
        pcmd.mode = (confirmed == static_cast<std::int8_t>(Cia402Mode::ProfileVelocity)) ? Cia402Mode::ProfileVelocity
                                                                                         : Cia402Mode::ProfilePosition;
        const std::uint16_t cw = sequencer_.step(ctx, pcmd);
        if (at_rest_) {  // driver rest detection (last publish_state's verdict)
            switch_phase_ = SwitchPhase::Settling;
            switch_cycles_ = 0;
        } else if (++switch_cycles_ >= kModeSwitchStopCycles) {
            mode_switch_give_up(confirmed);  // motor didn't stop; never switch mid-motion
        }
        return cw;
    }
    // Settling: command the target mode plus a safe seed (the sequencer writes 0x6060 = want); confirm the echo.
    pcmd.mode = want;
    const std::uint16_t cw = sequencer_.step(ctx, pcmd);
    if (confirmed == want_i8) {  // confirmed: next cycle runs the target mode's motion body
        switch_phase_ = SwitchPhase::None;
    } else if (++switch_cycles_ >= kModeSwitchSettleCycles) {
        mode_switch_give_up(confirmed);  // 0x6061 never echoed the new mode
    }
    return cw;
}

// Safe give-up disposition for a mode-switch that couldn't confirm. Never throws (the motion API call
// already returned; a switch failure is a stay-safe hold, not an operator error). A PV->PP hold reverts
// to the interim PV-at-0 bit-8 hold (accept small drift, stay energized). An operator switch reverts the
// intent to the drive's confirmed mode so it stops re-requesting (no retry storm) and holds energized at rest.
void ServoController::mode_switch_give_up(std::int8_t confirmed) noexcept {
    switch_phase_ = SwitchPhase::None;
    switch_cycles_ = 0;
    if (pv_hold_as_pp_) {
        pv_hold_as_pp_ = false;
        pv_velocity_ = 0;
    } else {
        switch_intent_ = (confirmed == static_cast<std::int8_t>(Cia402Mode::ProfileVelocity)) ? ControlMode::ProfileVelocity
                                                                                              : ControlMode::ProfilePosition;
    }
}

std::uint16_t ServoController::step_lifecycle(CycleContext& ctx, Status status, const CommandBatch& batch, std::int32_t actual) noexcept {
    const Cia402State dev = status.decode();
    const bool bus_fault = ctx.fault();

    // A new motion command (or enable) clears the sticky Halt.
    if (batch.set_target.has_value() || batch.set_velocity.has_value() || batch.enable) {
        halted_ = false;
        pv_hold_as_pp_ = false;  // a fresh motion intent ends the PV->PP position hold (switches back to PV)
    }
    if (batch.halt) {
        abort_active_move(RtError::MotorStopped);  // cancel any in-flight blocking move; its waiter throws "motor stopped".
                                                   // Fires on any halt (the halt cancels whatever is running), regardless of order.
        // Order-preserving: the sticky halt latches (and the PV->PP hold arms) only when the halt is the
        // latest stop-relevant command in this batch. If a motion command was issued after the halt in the
        // same drain (Stop() then GoTo() coalesced), that move supersedes the halt, so halted_ stays clear
        // (already set false above) and the move runs, instead of being parked under Halt (cw 0x011F,
        // whose bit-4 edge drives never ack). A halt in its own batch has halt_supersedes == true (the
        // common case).
        if (batch.halt_supersedes) {
            halted_ = true;  // sticky: stays asserted across cycles until a new motion command
            // On a switch-capable PV map, hold position via PP (below). The driver mode-switch brings
            // PV->PP; then pending_new_setpoint_ arms the sequencer's PP handshake once for the hold target,
            // which latches the live actual at rest (post stop-first ramp) so there is no lunge. Not
            // switch-capable means pv_hold_as_pp_ stays false and the interim bit-8 zero-velocity hold
            // applies. Only when the drive is currently in PV (a PP-intent halt already holds in PP, no
            // switch). commanded_is_pp() reads the live switch_intent_.
            if (pv_hold_capable_ && !commanded_is_pp()) {
                pv_hold_as_pp_ = true;
                pending_new_setpoint_ = true;  // arm the PP hold handshake once (consumed post-switch)
            }
        }
    }
    if (batch.disable) {
        abort_active_move(RtError::MotorDisabled);  // cancel any in-flight blocking move; its waiter throws "motor disabled"
    }
    // (abort_active_move is a no-op when no move is live -- gen 0 or already terminal -- so cancelling
    //  an already-completed move does not overwrite its success: first-terminal-wins.)

    // Adopt a new PP target (generation rides in the command, post-coalescing). A PP target
    // (go_to/go_for) routes the always-switchable drive to PP intent.
    if (batch.set_target.has_value()) {
        switch_intent_ = ControlMode::ProfilePosition;
        const SetTarget& t = *batch.set_target;
        if (t.generation != state_.active_generation.load(std::memory_order_relaxed)) {
            target_counts_ = t.relative ? static_cast<std::int32_t>(actual + t.counts) : t.counts;
            profile_vel_ = t.profile_velocity;
            latched_ctrl_error_ = RtError::None;  // a fresh move starts with a clean diagnostic slate
            pending_new_setpoint_ = true;         // arm the sequencer's PP handshake for this new target (consumed post-switch)
            state_.active_generation.store(t.generation, std::memory_order_release);
        }
    }
    // A velocity setpoint (set_rpm) routes the always-switchable drive to PV intent.
    if (batch.set_velocity.has_value()) {
        switch_intent_ = ControlMode::ProfileVelocity;
        pv_velocity_ = batch.set_velocity->velocity;
    }

    if (std::holds_alternative<Init>(lifecycle_)) {
        if (dev == Cia402State::Fault) {
            lifecycle_ = Faulted{};
        } else {
            lifecycle_ = Enabling{};
            mode_gate_ = ModeGate::Pending;  // re-arm the mode-echo gate for this bring-up
        }
        return ControlWord::disable_voltage();
    }
    if (std::holds_alternative<Enabling>(lifecycle_)) {
        if (dev == Cia402State::Fault) {
            lifecycle_ = Faulted{};
            return ControlWord::disable_voltage();  // latch the fault; reset is explicit (Faulted handles it)
        }
        // Seed 0x6060 = commanded mode through the enable ladder. The module's own enable FSM (not the
        // sequencer, which is stepped only post-OperationEnabled) drives the ladder, so it must write the
        // mode itself, else on a PDO-mapped-0x6060 map the drive follows the PDO (=0) and enables in mode
        // 0. Inert when 0x6060 is SDO-set only: f_mode_wr_ is nullopt.
        if (f_mode_wr_) {
            ctx.store<cia402::ModeOfOperation::type>(
                *f_mode_wr_, static_cast<std::int8_t>(commanded_cia402_mode()));  // switchable: the current mode intent
        }
        // Mode-echo gate, in the module's own ladder: once the drive is SwitchedOn the commanded mode
        // should be adopted (SDO-set at configure, or PDO-seeded above), so require 0x6061 == commanded
        // before energizing to OperationEnabled. A mismatch (the drive silently ignored the mode) latches
        // Failed, de-energizes, and sets RtError::ModeMismatch (last_error), sticky so there is no
        // ReadyToSwitchOn <-> SwitchedOn oscillation. Inert when 0x6061 is not mapped. Mirrors the
        // sequencer's gate, which the module never reaches (it delegates to the sequencer only post-OperationEnabled).
        if (mode_gate_ == ModeGate::Pending && f_mode_disp_ && (dev == Cia402State::SwitchedOn || dev == Cia402State::OperationEnabled)) {
            const auto want = static_cast<std::int8_t>(commanded_cia402_mode());  // gate on the commanded (intent) mode
            const auto echo = ctx.load<cia402::ModeDisplay::type>(*f_mode_disp_);
            if (echo == want) {
                mode_gate_ = ModeGate::Passed;
            } else {
                mode_gate_ = ModeGate::Failed;
                latched_ctrl_error_ = RtError::ModeMismatch;
                rt_error_.store(RtError::ModeMismatch, std::memory_order_release);
            }
        }
        if (mode_gate_ == ModeGate::Failed) {
            return ControlWord::disable_voltage();  // refuse: stay de-energized (is_powered false)
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
        // Driver-owned runtime mode-switch. The intent's Cia402 mode is PP for a positioned move (or a
        // PV->PP motion-hold), PV for set_rpm. If the drive's confirmed 0x6061 differs, or a switch is
        // already mid-sequence, the wrapper orchestrates the switch (hold energized, bring to rest,
        // command the new mode via the sequencer, await the echo) instead of the motion body. Only when
        // 0x6060 (write) and 0x6061 (echo) are both mapped; else the mode is fixed and this never triggers.
        const Cia402Mode want = pv_hold_as_pp_ ? Cia402Mode::ProfilePosition : commanded_cia402_mode();
        if (f_mode_wr_ && f_mode_disp_) {
            const std::int8_t want_i8 = static_cast<std::int8_t>(want);
            const std::int8_t confirmed = ctx.load<cia402::ModeDisplay::type>(*f_mode_disp_);  // 0x6061 echo this cycle
            if (switch_phase_ == SwitchPhase::None && confirmed != 0 && confirmed != want_i8) {
                switch_phase_ = SwitchPhase::Stopping;
                switch_cycles_ = 0;
            }
            if (switch_phase_ != SwitchPhase::None) {
                return run_mode_switch(ctx, actual, want);
            }
        }

        // No switch in flight: delegate the Operational healthy-path (enable-hold, PP new-set-point
        // handshake, 0x6081 move-speed or PV 0x60FF stream, Halt) to the shared generic sequencer. The
        // wrapper keeps completion generations, the two-tier fault, the fault-reset machine, and
        // is-moving/reached. pending_new_setpoint_ arms the sequencer's handshake on the cycle a new target
        // is adopted. The sequencer writes the controlword and command objects into ctx and returns the cw;
        // publish_state below reads its handshake-idle for the completion gate.
        SequencerCommand pcmd;
        if (pv_hold_as_pp_) {
            // PV motion-hold as PP-at-current-counts. The switch above brought PV->PP (position loop
            // active); command PP with target = the position latched at the halt so the drive's position
            // loop locks the shaft (no drift). pending_new_setpoint_ (armed when the hold began) arms the
            // PP handshake once to latch the live actual at rest, so there is no lunge. halt=false so the
            // handshake actually runs (a bit-8 halt would freeze the profile generator and never latch
            // the set-point). Not a completable move, so completion tracking is untouched.
            pcmd.mode = Cia402Mode::ProfilePosition;
            pcmd.target_counts = actual;  // live actual: the handshake latches it at rest, so no lunge
            pcmd.profile_velocity = profile_vel_;
            pcmd.enable = true;
            pcmd.halt = false;
        } else {
            pcmd.mode = commanded_cia402_mode();  // the current intent (the switch above ensured 0x6061 matches)
            pcmd.target_counts = target_counts_;
            pcmd.profile_velocity = profile_vel_;
            pcmd.target_velocity = pv_velocity_;
            pcmd.enable = true;
            pcmd.halt = halted_;
        }
        pcmd.new_setpoint = pending_new_setpoint_;  // arm the sequencer's handshake on the cycle a new target is adopted
        pending_new_setpoint_ = false;              // consume (a switch defers this -- run_mode_switch returns before here)
        const std::uint16_t cw = sequencer_.step(ctx, pcmd);
        // The four-phase handshake's ack (or ack-clear) timeout is the sequencer's per-cycle signal; the
        // wrapper owns the disposition and aborts the in-flight move (latch and wake the waiter).
        if (sequencer_.state().handshake_timed_out && !pv_hold_as_pp_) {
            abort_active_move(RtError::HandshakeTimeout);
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
        // Debounce the clear: a refault re-arms the streak, so a momentary clear-then-refault never
        // confirms (it is not mistaken for success).
        if (dev != Cia402State::Fault) {
            ++clear_streak_;
        } else {
            clear_streak_ = 0;
        }
        // Success: the clear held for K consecutive cycles, so recovery is confirmed.
        if (clear_streak_ >= config_.fault_reset_clear_confirm_cycles) {
            lifecycle_ = Enabling{};
            clear_streak_ = 0;
            return fsm_.step(status, Cia402State::OperationEnabled);  // hand off to the enable ladder
        }
        // Window remaining: keep working. Decrement every cycle (Fault or confirming), not only on
        // Fault cycles, so a flickering drive's total dwell stays bounded by the window regardless of
        // flicker period. Present the reset edge only while in Fault; while confirming a clear, return
        // a neutral controlword (don't pulse bit 7 at an already-clearing drive).
        if (reset_cycles_remaining_ > 0) {
            --reset_cycles_remaining_;
            return (dev == Cia402State::Fault) ? fault_reset_with_rearm(status) : ControlWord::disable_voltage();
        }
        // Give up: the window expired without a confirmed clear (never cleared, or cleared but never
        // confirmed). Revert to Faulted with a diagnostic; do not re-enter Resetting. Cleared by the
        // next operator fault_reset.
        latched_ctrl_error_ = RtError::FaultResetFailed;
        lifecycle_ = Faulted{};
        clear_streak_ = 0;
        return ControlWord::disable_voltage();
    }
    if (std::holds_alternative<Faulted>(lifecycle_)) {
        if (batch.fault_reset) {
            // Enter Resetting and hold the reset intent for the recovery window: the command-queue
            // fault_reset is a one-shot (consumed this cycle), so the sub-state, not the flag, carries
            // the intent across the drive's clear-reflect latency. A persistent bus WkcFault still
            // reappears next cycle via the live tier; it needs reconfigure, not fault_reset.
            latched_ctrl_error_ = RtError::None;  // clear the prior diagnostic (incl. a prior FaultResetFailed)
            lifecycle_ = Resetting{};
            reset_cycles_remaining_ = config_.fault_reset_window_cycles;
            clear_streak_ = 0;                      // start the debounce fresh
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

void ServoController::publish_state(CycleContext& ctx, Status status, std::int32_t actual, std::int32_t velocity) noexcept {
    state_.position_counts.store(actual, std::memory_order_relaxed);
    state_.velocity.store(velocity, std::memory_order_relaxed);

    const bool powered = status.operation_enabled();
    state_.powered.store(powered, std::memory_order_relaxed);

    const std::uint32_t g = state_.active_generation.load(std::memory_order_relaxed);
    const bool move_active = g != 0 && state_.completed_generation.load(std::memory_order_relaxed) != g &&
                             state_.failed_generation.load(std::memory_order_relaxed) != g;

    // Reach and rest signals. position_stable must be called once per cycle (it advances the ring),
    // so evaluate it unconditionally; its verdict feeds at_rest_ (the mode-switch stop-first gate) and
    // the reach heuristic. The move-complete predicate is a driver seam (reached_target): the
    // generic base trusts statusword bit 10; a device subclass overrides to the position-stability
    // heuristic for drives that tie bit 10 high. velocity_threshold>0 stays an optional PV is-moving gate.
    const bool pos_stable = position_stable(actual);
    at_rest_ = pos_stable;  // the driver's rest verdict, read (1-cycle stale) by run_mode_switch's stop-first gate
    const bool pos_near_target = std::abs(actual - target_counts_) <= config_.position_tolerance_counts;
    const bool at_target = reached_target(pos_near_target, pos_stable, status);

    // is_moving: PP = an active positioned move not yet at target; PV = the drive is not at rest.
    // At-rest is always the position-stability heuristic. target_counts_ is never assigned in PV, so
    // the PP position predicate must not drive PV moving.
    const bool moving =
        commanded_is_pp() ? (powered && move_active && !at_target) : (powered && !pos_stable);  // switchable uses the live intent
    state_.moving.store(moving, std::memory_order_relaxed);

    // PP generation protocol: completion (PP-only via move_active). There is no no-progress watchdog;
    // a stuck move parks in await_move until the client stops it, the drive faults, or the RT loop
    // exits (client-owned cancellation, consistent with the no-timeout wait).
    if (powered && move_active && at_target && sequencer_.state().handshake_idle) {
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
    // Resolve the sequencer's typed fields and run its quick-stop SDO setup here (non-RT, pre-spawn,
    // single port owner -- the one hook that may throw; a throw aborts start() cleanly and goes
    // Degraded). The module's own f_* offsets and vendor reset still resolve in start()/reconfigure();
    // this adds the sequencer's resolution (same Master, same SAFE-OP phase). needs_quick_stop is gated on
    // a configured 0x6085 (quick_stop_decel > 0); the echoed value backs the velocity-window guard. A
    // mismatched 0x605A or absent 0x6085 throws.
    const std::uint32_t echoed = sequencer_.configure(cfg, /*needs_quick_stop=*/config_.quick_stop_decel > 0);
    qs_decel_echoed_.store(echoed, std::memory_order_release);
    // Derive the velocity guard budget from the teardown window (a single source of truth, so the
    // window and the budget cannot disagree): the max velocity the echoed 0x6085 decel can ramp to 0
    // within (teardown_window - margin). 0 makes the guard inert.
    std::int64_t budget = 0;
    if (echoed > 0) {
        const double window_s = static_cast<double>(teardown_window_cycles()) / static_cast<double>(config_.target_loop_rate_hz);
        const double b = static_cast<double>(echoed) * std::max(0.0, window_s - kStopWindowMarginS);
        budget = static_cast<std::int64_t>(b);
    }
    vel_budget_cps_.store(budget, std::memory_order_release);
}

void ServoController::on_operational(CycleContext& ctx) noexcept {
    // No explicit seed: the std::variant lifecycle climbs Init->Enabling->Operational inside
    // step_lifecycle off the drive state.
    (void)ctx;
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
        // Two-level stop: when quick-stop is configured (quick_stop_decel>0), delegate to the sequencer's
        // controlled quick-stop (ramp via 0x6085, auto SwitchOnDisabled, then disable-voltage backstop
        // once |vel| is sub-threshold for the debounce). Opt-out (no decel): a straight disable-voltage
        // coast. Both are defined safe stops; the controlled one is opt-in. The sequencer's step() takes
        // its ctx.stopping() branch and writes the cw.
        if (config_.quick_stop_decel > 0) {
            SequencerCommand scmd;
            scmd.mode = commanded_cia402_mode();
            scmd.enable = false;               // stopping is not a motion intent; the sequencer's stopping branch owns the cw
            (void)sequencer_.step(ctx, scmd);  // writes cw (kQuickStopCw / disable backstop) into ctx
        } else {
            ctx.store<std::uint16_t>(f_ctrlword_, ControlWord::disable_voltage());
        }
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

    const std::uint16_t cw = step_lifecycle(ctx, status, batch, actual);
    ctx.store<std::uint16_t>(f_ctrlword_, cw);
    last_cw_ = cw;

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
        const std::string al_msg = master_ != nullptr ? master_->describe_al_code(al) : std::string{};
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
        // One-shot operator log line. Cold teardown path, so fprintf here is fine (not the hot loop).
        (void)std::fprintf(stderr,
                           "[servo] bring-up FAILED: drive not operational%s%s\n",
                           al != 0 ? (" -- drive refused OP: AL " + hex(al) + " (" + al_msg + ")").c_str() : "",
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
        master_ = std::make_unique<Master>(build_master_config(config_), std::make_unique<SoemBackend>());
        master_->init();
        master_->configure();  // -> SAFE-OP
        bring_up();            // -> OP with bounded retry (gives up by setting degraded_)
    } catch (const Error& e) {
        // Rebuild failed outright (typically: the interface is still down). Stay torn down and
        // recoverable; the next motion call retries.
        rt_runner_.reset();
        master_.reset();
        bus_lost_.store(true, std::memory_order_release);
        throw Error("bus lost and recovery failed (" + std::string(e.what()) + ") -- will retry on the next motion call");
    }
    if (degraded_.load(std::memory_order_acquire)) {
        // bring_up() exhausted its attempts: the bus enumerates but the drive would not reach OP.
        // Stay torn down and recoverable rather than parked on a half-alive master.
        const std::string cause = last_error();
        rt_runner_.reset();
        master_.reset();
        bus_lost_.store(true, std::memory_order_release);
        throw Error("bus lost and recovery failed (" + cause + ") -- will retry on the next motion call");
    }
    (void)std::fprintf(stderr, "[servo] bus recovered -- drive back to OPERATIONAL\n");
}

void ServoController::set_rpm(double rpm) {
    maybe_recover_bus();
    const std::shared_lock<std::shared_mutex> lk(api_mutex_);
    if (degraded_.load(std::memory_order_acquire)) {  // §8 Degraded-but-alive: motion APIs throw, never act
        throw Error("set_rpm unavailable: " + (degraded_reason_.empty() ? last_error() : degraded_reason_));
    }
    // Always switch-capable: set_rpm ensures PV at runtime, never rejected on mode. A PV setpoint
    // yields to a live blocking move (a go_for timed run) and is rejected "operation ongoing"
    // (including set_rpm(0); halt() is the stop verb). PV setpoints are latest-wins among themselves
    // (no slot), so this rejects only under a live blocking move.
    if (motion_slot_busy()) {
        throw Error("set_rpm: a motion operation is already in progress");
    }
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
        if (stopping_.load(std::memory_order_acquire) || rt_exited_.load(std::memory_order_acquire)) {
            throw Error("move: controller stopped / RT loop not alive");  // RT loop exited (stop / bus fault / bring-up abort)
        }
        if (state_.failed_generation.load(std::memory_order_acquire) >= g) {
            throw Error("move aborted (" + last_error() + ")");
        }
        if (state_.faulted.load(std::memory_order_acquire)) {
            throw Error("move: drive faulted during the move");
        }
        wake_seq_.wait(seq, std::memory_order_acquire);  // block until a wake bump (or a spurious wake -> re-check)
    }
}

void ServoController::go_to(double rpm, double position) {
    maybe_recover_bus();
    std::uint32_t g = 0;
    {
        const std::shared_lock<std::shared_mutex> lk(api_mutex_);
        if (degraded_.load(std::memory_order_acquire)) {  // §8
            throw Error("go_to unavailable: " + (degraded_reason_.empty() ? last_error() : degraded_reason_));
        }
        // Always switch-capable: go_to ensures PP at runtime, never rejected on mode. Absolute target
        // in the zeroed frame: add zero_offset_counts to map the user's zeroed position to the raw
        // encoder frame, so go_to(X) lands where position_revs() == X (get_position is zeroed too).
        const std::int32_t counts = static_cast<std::int32_t>(revs_to_counts(position, config_.counts_per_rev, config_.gear_ratio) +
                                                              state_.zero_offset_counts.load(std::memory_order_acquire));
        const double clamped_rpm = clamp_rpm(rpm, config_.max_motor_speed_rpm);
        const std::int32_t prof = clamp_to_stop_budget(rpm_to_device_velocity(clamped_rpm, config_.counts_per_rev, config_.gear_ratio));
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
        // Relative move (frame-agnostic): push SetTarget{relative=true} so the FSM computes target =
        // actual + delta. Do not route through go_to, which adds zero_offset (absolute frame) and
        // would double-shift a relative move.
        const std::int32_t delta = revs_to_counts(revs, config_.counts_per_rev, config_.gear_ratio);
        const double clamped_rpm = clamp_rpm(rpm, config_.max_motor_speed_rpm);
        const std::int32_t prof = rpm_to_device_velocity(clamped_rpm, config_.counts_per_rev, config_.gear_ratio);
        g = next_generation_.fetch_add(1, std::memory_order_relaxed) + 1;
        if (!try_claim_motion_slot(g)) {  // single-in-flight
            throw Error("go_for: a motion operation is already in progress");
        }
        (void)commands_.push(Command{SetTarget{delta, static_cast<std::uint32_t>(std::abs(prof)), true, g}});
    }
    await_move(g);
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
        if (code != 0) {
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
        case RtError::FaultResetFailed:
            append("fault-reset ineffective -- cause persists");
            break;
        case RtError::MotorStopped:
            append("motor stopped");
            break;
        case RtError::MotorDisabled:
            append("motor disabled");
            break;
        case RtError::ModeMismatch:
            append("drive mode-of-operation (0x6061) did not match the commanded mode -- refused to energize");
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
