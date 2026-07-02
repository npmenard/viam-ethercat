#include "viam/lib/servo_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <utility>

#include "ethercat/errors.hpp"
#include "ethercat/hex.hpp"
#include "ethercat/pdo_buffer.hpp"
#include "ethercat/realtime.hpp"
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
constexpr std::uint16_t kModeOfOp = 0x6060;    // runtime mode-of-operation (RxPDO); present => PV->PP hold-switch (M6)
constexpr std::uint16_t kModeDisplay = 0x6061;  // mode display (TxPDO); present => enable-time mode-echo gate (#45/#57)
constexpr std::uint16_t kFaultCode = 0x603F;   // drive error code (TxPDO, optional feedback)
// #TODO-4: the A6's "no-SYNC0" code (0x8700 / Er74.1) is NO LONGER a constant here --
// it's CONFIG DATA (ServoConfig::sync_fault_code), so this generic core carries no
// vendor value. The bring-up gate reads it from config (nullopt ⇒ no detection).
constexpr std::uint16_t kVelActual = 0x606C;  // velocity actual value (TxPDO, optional feedback)
constexpr std::uint64_t kNsPerSec = 1'000'000'000ULL;
// #47-P3b R1 controlled-stop watchdog headroom: the teardown window / VEL budget reserve this much
// time below the full window so the ramp finishes strictly BEFORE close() (A6 sync watchdog ~50ms).
constexpr double kStopWindowMarginS = 0.05;

// #40 item 7: ONE clock helper -- alias the shared realtime::monotonic_ns (the local
// duplicate is gone; watchdog + last_cycle_time are the users).
using realtime::monotonic_ns;

Cia402Mode to_cia402_mode(ControlMode mode) noexcept {
    return mode == ControlMode::ProfileVelocity ? Cia402Mode::ProfileVelocity : Cia402Mode::ProfilePosition;
}

ServoConfig validated(ServoConfig config) {
    config.validate();
    // #61: control_mode INTENT -> derive the standard CiA402 PDO map when the user didn't supply one
    // (an explicit rxpdo/txpdo is an advanced override, left verbatim). Idempotent.
    config.apply_derived_pdo_maps();
    // #59: reached/is_moving is noise-robust position-delta (see publish_state); position_tolerance_counts
    // is the "close enough + stable" band. DEFAULT it to counts_per_rev/720 (0.5 deg) when unset (<=0) --
    // self-documenting + scales with encoder resolution. (Was 0, which forced an EXACT-match reached
    // predicate -> a move never completed under encoder noise. Latent bug, #59.) velocity_threshold stays
    // an OPTIONAL override: >0 => a velocity gate; 0 => the position-delta method (the default).
    if (config.position_tolerance_counts <= 0) {
        config.position_tolerance_counts = static_cast<std::int32_t>(config.counts_per_rev / 720.0 + 0.5);
        if (config.position_tolerance_counts < 1) {
            config.position_tolerance_counts = 1;  // floor for a tiny-count encoder
        }
    }
    return config;
}

// #39: CONSUMER-side vendor fault-reset, run while this thread is still the SINGLE
// port owner (after Master::configure(), before the RT thread spawns). The vendor
// datum (A6: 0x2031:01 = 1) comes from the hardware JSON, never code. Best-effort:
// a failed clear is logged, not fatal -- the bring-up gate still guards OP entry.
void run_vendor_fault_reset(Master& m, const ServoConfig& c) {
    if (!c.vendor_fault_reset.has_value()) {
        return;
    }
    const SdoWrite& fr = *c.vendor_fault_reset;
    try {
        m.sdo_write(c.slave_id, fr.index, fr.subindex, fr.data);
    } catch (const Error& e) {
        (void)std::fprintf(stderr,
                           "[servo] vendor fault-reset SDO (slave %u 0x%04X:%02X) failed (continuing): %s\n",
                           static_cast<unsigned>(c.slave_id),
                           static_cast<unsigned>(fr.index),
                           static_cast<unsigned>(fr.subindex),
                           e.what());
    }
}

MasterConfig build_master_config(const ServoConfig& c) {
    SlaveConfig slave;
    slave.slave_id = c.slave_id;
    slave.rxpdo = c.rxpdo;
    slave.txpdo = c.txpdo;
    slave.default_mode = to_cia402_mode(c.mode);
    // #39: NO slave.fault_reset -- the vendor reset is consumer-side now (see
    // run_vendor_fault_reset above; executed pre-RT-spawn in start()/reconfigure()).
    slave.sync_cycle_granularity_ns = c.sync_cycle_granularity_ns;  // #44: Master validates rate vs granularity up front

    MasterConfig mc;
    mc.ifname = c.ifname;
    mc.target_loop_rate_hz = c.target_loop_rate_hz;
    mc.slaves = {slave};
    mc.max_consecutive_wkc_errors = static_cast<std::uint32_t>(c.max_consecutive_wkc_errors);
    mc.use_distributed_clocks = c.use_distributed_clocks;
    // Post-OP DC settle grace (cycles) while the SYNC0 phase finishes locking: suppress
    // the WKC-fault latch so a residual transient doesn't trip a spurious BusError. The
    // bring-up SETTLE bound uses MasterConfig's own default (dc_op_gate_cycles);
    // bench-tune it at first light if needed.
    constexpr std::uint32_t kDefaultDcSettleCycles = 1000;
    mc.dc_settle_cycles = c.use_distributed_clocks ? kDefaultDcSettleCycles : 0;
    return mc;
}

}  // namespace

ServoController::ServoController(ServoConfig config)
    : ServoController(std::move(config), [] { return std::unique_ptr<EcatBackend>(std::make_unique<SoemBackend>()); }) {}

DeviceProfile ServoController::make_module_profile(const ServoConfig& c) noexcept {
    DeviceProfile p;
    // Fault-reset mechanism: a vendor SDO (e.g. A6 0x2031:01) when the config carries one, else the
    // standard CiA402 controlword bit7. (The module runs its own #18 fault machine in the wrapper;
    // this field is for the policy's future in-loop reset -- R2/sub-step-4.)
    p.fault_reset = c.vendor_fault_reset.has_value() ? DeviceProfile::FaultReset::VendorSdo : DeviceProfile::FaultReset::Cia402Bit7;
    p.vendor_fault_reset = c.vendor_fault_reset;
    p.position_tolerance = c.position_tolerance_counts;  // effective value (validated() defaulted 0 -> counts_per_rev/720)
    // #59 (2nd consumer): the policy's zero_vel_threshold gates the quick-stop-AT-REST de-energize
    // (:209/:335) -- a BACKSTOP; the PRIMARY is 0x605A=2 auto-SwitchOnDisabled (P3c-proven). Keep it a
    // velocity gate (position-delta lives in the wrapper, not the pure-counts generic policy -- #41). Only
    // override the profile's sane 500 default when the config set an explicit velocity_threshold (>0);
    // do NOT propagate the 0 "unset" sentinel (that would zero the gate -> break the backstop for a
    // non-auto-disable device). Flagged to team-lead + DA.
    if (c.velocity_threshold > 0) {
        p.zero_vel_threshold = c.velocity_threshold;
    }
    p.quick_stop_decel = c.quick_stop_decel;  // 0 => configure() skips the quick-stop SDO setup
    // --- MODULE behavior flags (vs the bench A6 defaults): full 4-phase new-setpoint handshake
    //     with the module's ack timeout; Stop = CiA402 bit8 Halt; no PV position mirror. ---
    p.handshake_timeout_cycles = c.handshake_timeout_cycles;
    // A MOTION-stop (Halt) of a PV move. On a SWITCH-CAPABLE map (0x6060 + 0x607A mapped) the wrapper
    // instead switches the drive to PP-at-current-counts (M6, resolve_fields/step_lifecycle) so the
    // POSITION loop locks the shaft. This bit8 setting is the FALLBACK for a NON-switch-capable PV map:
    // Halt asserts CiA402 bit8 (the drive's own halt ramp) -> holds zero VELOCITY, not zero POSITION, so
    // under an external load the axis drifts (safe on the no-load sim / bench). (spec §A R1 / M6.)
    p.halt_uses_bit8 = true;
    p.pv_mirror_position = false;
    return p;
}

ServoController::ServoController(ServoConfig config, BackendFactory backend_factory)
    : config_(validated(std::move(config))),
      backend_factory_(std::move(backend_factory)),
      policy_(make_module_profile(config_)),
      commands_(config_.command_queue_capacity) {
    if (!backend_factory_) {
        throw ConfigError("ServoController: null backend factory");
    }
}

ServoController::~ServoController() {
    stop();
}

void ServoController::start() {
    const std::unique_lock<std::shared_mutex> lk(api_mutex_);

    // NOTE: configure() reaches SAFE-OP and does NO memory lock (TODO-6: residency is
    // RT-setup's job, not thread-free bus policy). The RT thread then runs the DC
    // bring-up prelude (SETTLE -> request OP -> AWAIT_OP) to OPERATIONAL; its
    // setup_realtime() = realtime::setup() does the full MCL_CURRENT|MCL_FUTURE
    // IN-THREAD, post-spawn -- which is ALSO where the EAGAIN-avoidance now lives: an
    // in-thread/post-spawn MCL_FUTURE never sees this thread's later jthread stack
    // alloc (the trap the old pre-spawn MCL_CURRENT-only carve-out worked around), and
    // nothing cyclic runs before that in-thread lock, so no SYNC0-critical page-fault
    // window opens.
    master_ = std::make_unique<Master>(build_master_config(config_), backend_factory_());
    master_->init();
    master_->configure();  // -> SAFE-OP (may throw InitError; propagated as today -- the SDK retries)
    resolve_fields();
    run_vendor_fault_reset(*master_, config_);  // #39: consumer-side, single port owner (pre-Runner-start)
    spawn_runner();
}

// Zero the per-run published atomics + RT-only working state (shared by start()/reconfigure()).
void ServoController::reset_run_state() {
    stopping_.store(false, std::memory_order_release);
    rt_error_.store(RtError::None, std::memory_order_relaxed);
    state_.faulted.store(false, std::memory_order_relaxed);
    state_.loop_cycle.store(0, std::memory_order_relaxed);
    state_.last_cycle_time_ns.store(0, std::memory_order_relaxed);
    state_.active_generation.store(0, std::memory_order_relaxed);
    state_.completed_generation.store(0, std::memory_order_relaxed);
    state_.failed_generation.store(0, std::memory_order_relaxed);
    next_generation_.store(0, std::memory_order_relaxed);
    motion_slot_.store(0, std::memory_order_relaxed);  // R3: free the single-in-flight slot on (re)start
    state_.expected_wkc.store(master_->expected_wkc(), std::memory_order_relaxed);  // constant; read lock-free by last_error()
    lifecycle_ = Init{};
    last_cw_ = 0;
    policy_.reset();  // #47-P3b: clear the shared policy's per-run sequencing state (handshake/latches) for reuse
    prev_actual_ = 0;
    first_cycle_ = true;
    halted_ = false;
    stop_at_rest_ = false;
    latched_ctrl_error_ = RtError::None;
    last_progress_actual_ = 0;
    stall_cycles_ = 0;
    last_sync_code_ = 0;
    const std::uint64_t period_ns = kNsPerSec / config_.target_loop_rate_hz;
    const std::uint64_t stall_ns = config_.stall_threshold_cycles * period_ns;
    watchdog_ns_.store(std::max<std::uint64_t>(stall_ns, 20'000'000ULL), std::memory_order_release);
}

// Construct the one-shot Runner BORROWING master_, attach *this as the SlaveControl, and
// start it. The Runner owns realtime setup + the DC bring-up pump + pacing + teardown (the
// old run_rt_loop's job). #39: the Runner owns the set_rt_active bracket now. §8: a start-time
// failure -> Degraded-but-alive (APIs throw, process stays up), never rethrown past here.
std::uint32_t ServoController::teardown_window_cycles() const noexcept {
    // Opt-out (no controlled stop): disable-voltage coast is instant -> the old 2-cycle window.
    if (config_.quick_stop_decel == 0) {
        return 2;
    }
    // decel>0: size the window to the controlled-stop budget so the Quick-Stop ramp COMPLETES before
    // close()->INIT (no torque-cut). window_cycles = controlled_stop_window_ms x loop_rate. The event
    // gate (teardown_complete) exits earlier once at rest; this is the hard CAP.
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
    // The Er74 OP-entry gate (bringup_step -> Aborted) decides a failed bring-up, not this
    // wall bound -- keep it well above Master's own op-await window (the pump backstop).
    rc.bringup_timeout = std::chrono::milliseconds(120'000);
    // Teardown window (#47-P3b R1): for a controlled Quick-Stop (quick_stop_decel>0) size it to the
    // controlled-stop window so the ramp reaches REST before master.close()->INIT de-energizes (NO
    // torque-cut at speed); the event gate (teardown_complete) exits as soon as the drive is at rest
    // so the common already-stopped case doesn't pay the full window. Opt-out coast = 2 cycles.
    rc.teardown_cycles = teardown_window_cycles();
    rt_runner_ = std::make_unique<Runner>(*master_, rc);
    try {
        rt_runner_->attach(config_.slave_id, *this);
        rt_runner_->start();  // on_configured (no-op) -> set_rt_active(true) -> spawn the RT thread
    } catch (const Error& e) {
        // §8: refusal/attach failure at start -> Degraded; drop the un-started Runner.
        degraded_reason_ = std::string("ServoController degraded at start: ") + e.what();
        degraded_.store(true, std::memory_order_release);
        rt_runner_.reset();  // ~Runner: never-started -> no join/close, just frees
    }
}

void ServoController::stop() noexcept {
    const std::unique_lock<std::shared_mutex> lk(api_mutex_);
    stopping_.store(true, std::memory_order_release);
    completion_cv_.notify_all();  // wake any parked go_to/go_for waiters
    // Drop the Runner: ~Runner runs the BOUNDED teardown (join the RT thread -> set_rt_active(false)
    // -> master.close()->INIT). A WEDGED step() fail-stops the process (#52), not an unbounded hang.
    rt_runner_.reset();
}

void ServoController::reconfigure(ServoConfig config) {
    ServoConfig next = validated(std::move(config));
    const std::unique_lock<std::shared_mutex> lk(api_mutex_);
    // Drop the Runner FIRST (its ~Runner joins the RT thread + close()->INIT) before touching
    // master_ -- the RT thread is master_'s only cyclic user, so this is the join barrier.
    stopping_.store(true, std::memory_order_release);
    completion_cv_.notify_all();
    rt_runner_.reset();
    master_.reset();  // safe: Runner (master_'s only cyclic user) is destroyed
    config_ = std::move(next);

    // Restart with the new config (same body as start(), lock already held).
    master_ = std::make_unique<Master>(build_master_config(config_), backend_factory_());
    master_->init();
    master_->configure();
    resolve_fields();
    run_vendor_fault_reset(*master_, config_);  // #39: consumer-side, single port owner (pre-Runner-start)
    spawn_runner();
}

void ServoController::resolve_fields() {
    // Resolve each RT field ONCE here (non-RT, at start) to a cached offset; the RT loop then
    // reads/writes at the cached byte_offset with literal widths -- NO per-cycle resolve, throw,
    // or map-walk. rx_field/tx_field now return an offset-only FieldLocation (#30 P2c byte_width
    // drop); an absent OPTIONAL field caches a default FieldLocation{} -> !mapped().
    const std::uint16_t s = config_.slave_id;
    f_ctrlword_ = master_->rx_field(s, kCtrlword, 0);
    f_statusword_ = master_->tx_field(s, kStatusword, 0);
    f_actual_ = master_->tx_field(s, kActualPos, 0);
    // #61: resolve whichever command objects the (derived or overridden) map carries -- PP
    // {0x607A,0x6081}, PV {0x60FF}, switchable ALL. Map-driven (rxpdo_has) so all three control_modes
    // work; an absent object caches !mapped() (the policy guards every write on mapped()).
    f_target_ = rxpdo_has(kTargetPos) ? master_->rx_field(s, kTargetPos, 0) : FieldLocation{};
    f_profile_velocity_ = rxpdo_has(kProfileVel) ? master_->rx_field(s, kProfileVel, 0) : FieldLocation{};
    f_velocity_ = rxpdo_has(kTargetVel) ? master_->rx_field(s, kTargetVel, 0) : FieldLocation{};
    // #47-P3b M6 / #61: a PV motion-hold locks POSITION (not just zero velocity) ONLY for a PV or
    // switchable config (NOT a fixed PP config -- a PP halt already holds in PP, no switch) AND when the
    // map is switch-capable (0x6060 runtime-mode + 0x607A PP-target both present). Absent -> the interim
    // bit8 zero-VELOCITY hold. The RUNTIME halt handler further gates on the current intent being PV.
    pv_hold_capable_ = config_.mode != ControlMode::ProfilePosition && rxpdo_has(kModeOfOp) && rxpdo_has(kTargetPos);
    // OPTIONAL TxPDO feedback (spec #16) -- both modes. !mapped() => not in the map, so the
    // RT loop falls back (velocity estimate) / omits the tier (fault code).
    f_fault_code_ = txpdo_has(kFaultCode) ? master_->tx_field(s, kFaultCode, 0) : FieldLocation{};
    f_velocity_actual_ = txpdo_has(kVelActual) ? master_->tx_field(s, kVelActual, 0) : FieldLocation{};
    // #47-P3c/#57 enable-ladder mode fields: 0x6060 (write, seed the mode through the ladder) is present
    // only in a PDO-mapped-0x6060 map (bench/M6); 0x6061 (read, the mode-echo gate) is present in the A6
    // production map. Both optional -> !mapped() makes the respective enable-ladder step inert.
    f_mode_wr_ = rxpdo_has(kModeOfOp) ? master_->rx_field(s, kModeOfOp, 0) : FieldLocation{};
    f_mode_disp_ = txpdo_has(kModeDisplay) ? master_->tx_field(s, kModeDisplay, 0) : FieldLocation{};

    // #59: size the position-stability window to ~20 ms at the loop rate (>=3 cycles), reset it. Pre-
    // allocated here (non-RT, pre-spawn) so the RT loop never allocates. Re-sized on each start/reconfigure.
    const std::uint32_t win = std::max<std::uint32_t>(3, static_cast<std::uint32_t>(config_.target_loop_rate_hz) / 50);
    pos_hist_.assign(win, 0);
    pos_hist_idx_ = 0;
    pos_hist_filled_ = 0;

    // #47-P3b R1 opt-out OBSERVABILITY (DA): a module WITHOUT a configured quick_stop_decel stops
    // via UNCONTROLLED disable-voltage coast -- a known, predictable coast, safe BECAUSE we won't
    // Quick-Stop against an unverified/unsized decel. But on a load-holding / vertical axis a coast
    // drifts/drops the load, so surface the opt-out at bring-up (non-RT, once) rather than let an
    // operator find out the hard way. Not a hard require (opt-in philosophy) -- discoverable, same
    // "observable, not silent" philosophy as the VEL clamp-below-requested.
    if (config_.quick_stop_decel == 0) {
        (void)std::fprintf(stderr,
                           "[servo] slave %u: quick_stop_decel not set -> STOP is an UNCONTROLLED disable-voltage "
                           "coast (safe, but a load-holding axis will drift/drop). Set quick_stop_decel (0x6085) for a "
                           "controlled ramp-stop.\n",
                           static_cast<unsigned>(s));
    }
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

bool ServoController::position_stable(std::int32_t actual) noexcept {
    // #59: push `actual` into the ring; STABLE once the window is full AND its range (max-min) is within
    // position_tolerance_counts -- encoder jitter at rest stays within tol => stable; real motion widens
    // the range => not stable. Not-yet-full => not stable (still settling). O(N), N ~ 20ms of cycles.
    if (pos_hist_.empty()) {
        return false;  // never sized (pre-start) -- treat as moving, fail-safe
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
    // #61: a switchable config commands the current intent (go_to->PP, set_rpm->PV); a fixed PP/PV
    // config always commands that mode. The policy runs the §6 switch when 0x6060 is mapped and the
    // commanded mode differs from the drive's confirmed 0x6061 (switchable maps 0x6060 -> switch live).
    if (config_.mode == ControlMode::Switchable) {
        return switch_intent_ == ControlMode::ProfileVelocity ? Cia402Mode::ProfileVelocity : Cia402Mode::ProfilePosition;
    }
    return to_cia402_mode(config_.mode);
}

std::uint16_t ServoController::step_lifecycle(CycleContext& ctx, Status status, const CommandBatch& batch, std::int32_t actual) noexcept {
    const Cia402State dev = status.decode();
    const bool bus_fault = ctx.fault();

    // A new motion command (or enable) clears the sticky Halt.
    if (batch.set_target.has_value() || batch.set_velocity.has_value() || batch.enable) {
        halted_ = false;
        pv_hold_as_pp_ = false;  // M6: a fresh motion intent ends the PV->PP position hold (switches back to PV)
    }
    if (batch.halt) {
        halted_ = true;                             // STICKY: stays asserted across cycles until a new motion command
        abort_active_move(RtError::MotorStopped);   // R3: CANCEL any in-flight blocking move -> its waiter throws "motor stopped"
        // M6: on a switch-capable PV map, hold POSITION via PP (below). pv_hold_token_ kicks the policy's
        // PP handshake for the hold target WITHOUT disturbing the move-generation space. The hold target is
        // the LIVE actual (passed each cycle) -- the PP handshake latches it once, on its bit4 edge, which
        // fires only AFTER the mode-switch stop-first ramp has brought the motor to REST -> the latched
        // target IS the rest position (spec M6 "seed 0x607A=ACTUAL counts"), so no back-jump/lunge. Not
        // switch-capable -> pv_hold_as_pp_ stays false -> interim bit8 zero-velocity hold.
        // #61: only when the drive is currently in PV (fixed-PV always; switchable only after set_rpm) --
        // a PP-intent halt already holds in PP, no switch. commanded_is_pp() reads the live switch_intent_.
        if (pv_hold_capable_ && !commanded_is_pp()) {
            pv_hold_as_pp_ = true;
            ++pv_hold_token_;
        }
    }
    if (batch.disable) {
        abort_active_move(RtError::MotorDisabled);  // R3: CANCEL any in-flight blocking move -> its waiter throws "motor disabled"
    }
    // (abort_active_move is a no-op when no move is live -- gen 0 or already terminal -- so a
    //  cancel of an already-completed move does NOT overwrite its success: first-terminal-wins.)

    // Adopt a new PP target (generation rides in the command, post-coalescing).
    // #61: a PP target arrives for a PP config OR a switchable config (go_to/go_for) -> intent PP.
    if (config_.mode != ControlMode::ProfileVelocity && batch.set_target.has_value()) {
        switch_intent_ = ControlMode::ProfilePosition;  // switchable: route to PP (no-op for a fixed PP config)
        const SetTarget& t = *batch.set_target;
        if (t.generation != state_.active_generation.load(std::memory_order_relaxed)) {
            target_counts_ = t.relative ? static_cast<std::int32_t>(actual + t.counts) : t.counts;
            profile_vel_ = t.profile_velocity;
            last_progress_actual_ = actual;
            stall_cycles_ = 0;
            latched_ctrl_error_ = RtError::None;  // a fresh move starts with a clean diagnostic slate
            state_.active_generation.store(t.generation, std::memory_order_release);
            // The policy restarts its new-setpoint handshake off the token (= this generation)
            // change on the next step() -- no wrapper-side handshake state to prime (#47-P3b).
        }
    }
    // #61: a velocity setpoint arrives for a PV config OR a switchable config (set_rpm) -> intent PV.
    if (config_.mode != ControlMode::ProfilePosition && batch.set_velocity.has_value()) {
        switch_intent_ = ControlMode::ProfileVelocity;  // switchable: route to PV (no-op for a fixed PV config)
        pv_velocity_ = batch.set_velocity->velocity;
    }

    if (std::holds_alternative<Init>(lifecycle_)) {
        if (dev == Cia402State::Fault) {
            lifecycle_ = Faulted{};
        } else {
            lifecycle_ = Enabling{};
            mode_gate_ = ModeGate::Pending;  // #57: re-arm the mode-echo gate for this bring-up
        }
        return ControlWord::disable_voltage();
    }
    if (std::holds_alternative<Enabling>(lifecycle_)) {
        if (dev == Cia402State::Fault) {
            lifecycle_ = Faulted{};
            return ControlWord::disable_voltage();  // latch the fault; reset is EXPLICIT (Faulted handles it)
        }
        // #47-P3c/#57: SEED 0x6060 = commanded mode through the enable ladder. The module's OWN enable FSM
        // (not the policy, which is stepped only post-OE) drives the ladder, so it must write the mode
        // itself -- else on a PDO-mapped-0x6060 map the drive follows the PDO (=0) and enables in mode 0.
        // Inert when 0x6060 is SDO-set only (production): f_mode_wr_ !mapped().
        if (f_mode_wr_.mapped()) {
            ctx.store<cia402::ModeOfOperation::type>(f_mode_wr_, static_cast<std::int8_t>(commanded_cia402_mode()));  // #61: switchable -> current intent
        }
        // #47-P3c/#57 MODE-ECHO GATE (#45 fail-closed, in the module's OWN ladder): once the drive is
        // SwitchedOn the commanded mode should be adopted (SDO-set at configure, or PDO-seeded above), so
        // require 0x6061 == commanded BEFORE energizing to OE. A mismatch (the A6 silently ignored the
        // mode) latches Failed -> de-energize + RtError::ModeMismatch (last_error), STICKY (no RTSO<->SO
        // oscillation). Inert when 0x6061 isn't mapped. Mirrors the policy's line-232 gate, which the
        // module never reaches (it delegates to the policy only post-OE).
        if (mode_gate_ == ModeGate::Pending && f_mode_disp_.mapped() &&
            (dev == Cia402State::SwitchedOn || dev == Cia402State::OperationEnabled)) {
            const auto want = static_cast<std::int8_t>(commanded_cia402_mode());  // #61: gate on the commanded (intent) mode
            const auto echo = ctx.load<cia402::ModeDisplay::type>(f_mode_disp_);
            if (echo == want) {
                mode_gate_ = ModeGate::Passed;
            } else {
                mode_gate_ = ModeGate::Failed;
                latched_ctrl_error_ = RtError::ModeMismatch;
                rt_error_.store(RtError::ModeMismatch, std::memory_order_release);
            }
        }
        if (mode_gate_ == ModeGate::Failed) {
            return ControlWord::disable_voltage();  // REFUSE: fail-closed, stay de-energized (is_powered false)
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
        // #47-P3b: DELEGATE the Operational healthy-path (enable-hold + PP new-setpoint handshake +
        // 0x6081 move-speed / PV 0x60FF stream + Halt) to the shared generic policy. The wrapper
        // KEEPS completion-generations, the two-tier fault, #18 fault-reset, and the stall watchdog
        // (rev-6 boundary). The opaque token = the active move generation (the policy resets its
        // reached-latch + restarts the handshake when it changes -- FOLD 3). The policy writes the
        // controlword + command objects into ctx and returns the cw; publish_state below reads its
        // handshake-idle for the completion gate.
        PolicyCommand pcmd;
        if (pv_hold_as_pp_) {
            // M6: PV motion-hold as PP-at-current-counts. Command PP with target = the position latched
            // at the halt; the generic mode-switch ramps PV->0, switches 0x6060=PP, then the PP handshake
            // (kicked by pv_hold_token_) latches the hold target so the drive's position loop LOCKS the
            // shaft (no drift). halt=false so the handshake actually runs (a bit8 halt would freeze the
            // profile generator and never latch the setpoint). Not a completable move -> pv_hold_token_,
            // not active_generation, so completion tracking is untouched (the halt already failed it).
            pcmd.mode = Cia402Mode::ProfilePosition;
            pcmd.target_counts = actual;  // LIVE actual: the handshake latches it at REST (post ramp) -> no lunge
            pcmd.profile_velocity = profile_vel_;
            pcmd.enable = true;
            pcmd.halt = false;
            pcmd.token = pv_hold_token_;
        } else {
            pcmd.mode = commanded_cia402_mode();  // #61: switchable -> current intent (policy runs the §6 switch when it differs from 0x6061)
            pcmd.target_counts = target_counts_;
            pcmd.profile_velocity = profile_vel_;
            pcmd.target_velocity = pv_velocity_;
            pcmd.enable = true;
            pcmd.halt = halted_;
            pcmd.token = state_.active_generation.load(std::memory_order_relaxed);
        }
        const std::uint16_t cw = policy_.step(ctx, pcmd);
        // The 4-phase handshake's ack (or ack-clear) timeout is the policy's per-cycle signal; the
        // WRAPPER owns the disposition -> abort the in-flight move (latch + wake the waiter), same as
        // the old step_handshake abort_active_move(HandshakeTimeout).
        if (policy_.state().handshake_timed_out && !pv_hold_as_pp_) {
            abort_active_move(RtError::HandshakeTimeout);
        }
        // M6 failure disposition (spec §A R1): if the PV->PP hold-switch can't confirm (motor won't
        // stop / 0x6061 never echoes PP), DON'T throw -- this is an internal hold, not an operator
        // command. Revert to the interim PV-at-0 bit8 hold (accept small drift, stay energized). Next
        // cycle commands PV+halt; current_mode is still PV (switch failed pre-echo) so no re-switch.
        if (policy_.state().mode_switch_failed && pv_hold_as_pp_) {
            pv_hold_as_pp_ = false;
            pv_velocity_ = 0;  // spec §A R1 "PV-AT-0": command zero velocity for the reverted hold (don't resume the pre-halt rpm)
        } else if (policy_.state().mode_switch_failed && config_.mode == ControlMode::Switchable) {
            // #61: a switchable operator switch (go_to/set_rpm intent) couldn't confirm the new 0x6061 --
            // revert the intent to the drive's CONFIRMED mode so we stop re-requesting (no retry storm);
            // stay energized at rest, no throw (the motion API call already returned). SAFE disposition.
            switch_intent_ = (policy_.state().current_mode == static_cast<std::int8_t>(Cia402Mode::ProfileVelocity))
                                 ? ControlMode::ProfileVelocity
                                 : ControlMode::ProfilePosition;
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

void ServoController::publish_state(CycleContext& ctx, Status status, std::int32_t actual, std::int32_t velocity) noexcept {
    state_.position_counts.store(actual, std::memory_order_relaxed);
    state_.velocity.store(velocity, std::memory_order_relaxed);

    const bool powered = status.operation_enabled();
    state_.powered.store(powered, std::memory_order_relaxed);

    const std::uint32_t g = state_.active_generation.load(std::memory_order_relaxed);
    const bool move_active = g != 0 && state_.completed_generation.load(std::memory_order_relaxed) != g &&
                             state_.failed_generation.load(std::memory_order_relaxed) != g;

    // #59 noise-robust "stopped" (NEVER bit10): position STABLE over the last N cycles (its range <=
    // position_tolerance_counts) -- immune to encoder jitter at rest. velocity_threshold>0 is an OPTIONAL
    // override (a classic velocity gate); 0 (the default) uses the position-delta method. position_stable
    // MUST be called once per cycle (it advances the ring), so evaluate it unconditionally.
    const bool pos_stable = position_stable(actual);
    const bool stopped = (config_.velocity_threshold > 0) ? (std::abs(velocity) <= config_.velocity_threshold) : pos_stable;

    // Move-complete predicate: |target - actual| <= tol AND the axis has come to rest (stopped).
    // Only meaningful in PP (move_active implies a go_to generation).
    const bool at_target = std::abs(actual - target_counts_) <= config_.position_tolerance_counts && stopped;

    // is_moving: PP = an active positioned move not yet at target; PV = the drive is not at rest.
    // target_counts_ is never assigned in PV, so the PP position predicate must NOT drive PV moving.
    const bool moving =
        commanded_is_pp() ? (powered && move_active && !at_target) : (powered && !stopped);  // #61: switchable uses the live intent
    state_.moving.store(moving, std::memory_order_relaxed);

    // PP generation protocol: completion + no-progress watchdog (PP-only via move_active).
    if (powered && move_active && at_target && policy_.state().handshake_idle) {
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
    const bool bus_fault = ctx.fault();
    // BUS tier: payload (the raw last-exchange WKC -- the actual bad value at fault, ==
    // master_->last_wkc()) then flag.
    state_.fault_wkc.store(ctx.wkc().last, std::memory_order_relaxed);
    state_.wkc_faulted.store(bus_fault, std::memory_order_release);
    // DRIVE tier: live-read 0x603F from THIS cycle's owned snapshot EVERY faulted cycle (not
    // edge-captured) so a code the drive latches a frame or two after it sets bit3 is
    // still picked up ("code pending" collapses to the rare hard-drop race only).
    const std::uint16_t drive_code = f_fault_code_.mapped() ? ctx.load<std::uint16_t>(f_fault_code_) : 0;
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

// --- SlaveControl hooks (#54 P3a): the old run_rt_loop body, split across the Runner's
// lifecycle. The Runner owns realtime setup + the DC bring-up pump + the one DcPacer +
// pacing + the steady cadence + the stopping window + master.close(). What remains is POLICY.

void ServoController::on_configured(ConfigContext& cfg) {
    // #47-P3b: resolve the policy's typed fields + run its quick-stop SDO setup here (NON-RT,
    // pre-spawn, single port owner -- the ONE hook that may throw; a throw aborts start() cleanly
    // -> Degraded §8). The module's own f_* offsets + #39 vendor reset still resolve in
    // start()/reconfigure(); this ADDS the policy's resolution (same Master, same SAFE-OP phase).
    // needs_quick_stop gated on a configured 0x6085 (quick_stop_decel > 0); the echoed value backs
    // the future velocity-window guard. A mode-mismatched 0x605A / absent 0x6085 THROWS (fail-closed).
    const std::uint32_t echoed = policy_.configure(cfg, /*needs_quick_stop=*/config_.quick_stop_decel > 0);
    qs_decel_echoed_.store(echoed, std::memory_order_release);
    // DERIVE the velocity guard budget FROM the teardown window (architect FLAG 1 -- single source of
    // truth, so the window and the budget can never disagree): the max velocity the echoed 0x6085
    // decel can ramp to 0 within (teardown_window - margin). 0 -> guard inert.
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
    // step_lifecycle off the drive state, exactly as the old steady loop did.
    (void)ctx;
}

bool ServoController::sync_faulted(const CycleContext& ctx) const noexcept {
    // The old bring-up gate (#TODO-4): drive-sync-faulted = mapped 0x603F == the configured
    // no-sync code; nullopt (none declared) => always false. Stash the read code for on_stop's
    // bring-up-abort diagnostic (sync_faulted + on_stop both run on the RT thread).
    const std::uint16_t code = f_fault_code_.mapped() ? ctx.load<std::uint16_t>(f_fault_code_) : 0;
    last_sync_code_ = code;
    return config_.sync_fault_code.has_value() && code == *config_.sync_fault_code;
}

void ServoController::step(CycleContext& ctx) noexcept {
    if (ctx.stopping()) {
        // LIFECYCLE-stop (spec §A R1 / FOLD 4): the Runner enters its stopping window on ANY stop
        // cause -- including a BUS fault (master_.fault()), which the OLD loop did NOT treat as an
        // exit: it kept running + publishing the fault tiers every cycle. So during stopping we
        // de-energize (safe) but KEEP PUBLISHING, so last_error() composes the bus/drive fault
        // that triggered the stop (#16 compose-both tier liveness). The Runner ships the final
        // process + close()->INIT after the window.
        const Status sstatus{ctx.load<std::uint16_t>(f_statusword_)};
        const std::int32_t sactual = ctx.load<std::int32_t>(f_actual_);
        const std::int32_t svel =
            f_velocity_actual_.mapped()
                ? ctx.load<std::int32_t>(f_velocity_actual_)
                : static_cast<std::int32_t>(static_cast<std::int64_t>(sactual - prev_actual_) * config_.target_loop_rate_hz);
        prev_actual_ = sactual;
        // #47-P3b R1 TWO-LEVEL stop: when quick-stop is configured (quick_stop_decel>0), delegate
        // to the policy's controlled Quick-Stop (ramp via 0x6085 -> auto SwitchOnDisabled ->
        // disable-voltage BACKSTOP once |vel| is sub-threshold for the debounce). OPT-OUT (no
        // decel): P3a straight disable-voltage coast -- both are defined safe stops; the controlled
        // one is opt-in. The policy's step() takes its ctx.stopping() branch and writes the cw.
        if (config_.quick_stop_decel > 0) {
            PolicyCommand scmd;
            scmd.mode = commanded_cia402_mode();  // #61
            scmd.enable = false;  // stopping is not a motion intent; the policy's stopping branch owns the cw
            scmd.token = state_.active_generation.load(std::memory_order_relaxed);
            (void)policy_.step(ctx, scmd);  // writes cw (kQuickStopCw / disable backstop) into ctx
        } else {
            ctx.store<std::uint16_t>(f_ctrlword_, ControlWord::disable_voltage());
        }
        // Event-driven teardown early-out (#47-P3b R1): once the drive is de-energized AT REST
        // (SwitchOnDisabled -- the 0x605A==2 auto-transition at zero, or the disable-voltage
        // backstop / opt-out coast landing), signal the Runner it may end the teardown window. A
        // moving stop keeps this false until the controlled ramp reaches rest, so close() never
        // cuts torque at speed; the generous decel>0 window is the hard cap.
        stop_at_rest_ = sstatus.switch_on_disabled();
        publish_state(ctx, sstatus, sactual, svel);
        return;
    }

    const CommandBatch batch = commands_.drain();

    // ONE input snapshot per cycle (spec #16 §10): statusword/bit3, actual, 0x603F, 0x606C
    // all read from THIS cycle's owned image (the Runner copied it in before step()), so the
    // fault code matches the fault state it is reported with -- structurally, as before.
    const Status status{ctx.load<std::uint16_t>(f_statusword_)};
    const std::int32_t actual = ctx.load<std::int32_t>(f_actual_);
    if (first_cycle_) {
        prev_actual_ = actual;  // avoid a spurious huge velocity on cycle 0
        first_cycle_ = false;
    }
    // Velocity from the wire (0x606C) when mapped, else the instantaneous estimate
    // (actual-delta * loop rate). One branch; identical fallback when unmapped.
    const std::int32_t velocity =
        f_velocity_actual_.mapped()
            ? ctx.load<std::int32_t>(f_velocity_actual_)
            : static_cast<std::int32_t>(static_cast<std::int64_t>(actual - prev_actual_) * config_.target_loop_rate_hz);
    prev_actual_ = actual;

    const std::uint16_t cw = step_lifecycle(ctx, status, batch, actual);
    ctx.store<std::uint16_t>(f_ctrlword_, cw);
    last_cw_ = cw;

    // process() ships the cw + latches the next input -- Runner-owned, AROUND step() (the
    // documented 1-cycle store latency, P2b HW-verified). publish reads THIS cycle's snapshot.
    publish_state(ctx, status, actual, velocity);
}

void ServoController::on_stop(StopReason reason) noexcept {
    if (reason == StopReason::BringupAborted) {
        // SYNC0 did not take (Er74.1 in the gate). Surface root cause (drive tier) + symptom
        // (not operational) -- the old run_rt_loop bring-up-abort branch, now here. No
        // auto-retry (repeated Er74 OP-entry wedges the A6); recovery = explicit reconfigure.
        state_.drive_fault_code.store(last_sync_code_ != 0 ? last_sync_code_ : config_.sync_fault_code.value_or(0),
                                      std::memory_order_relaxed);
        state_.drive_faulted.store(true, std::memory_order_release);
        rt_error_.store(RtError::NotOperational, std::memory_order_release);
        state_.faulted.store(true, std::memory_order_release);
        degraded_.store(true, std::memory_order_release);  // §8: bring-up failed -> Degraded (APIs throw via last_error())
    } else if (reason == StopReason::RtSetupFailed) {
        // §8: realtime scheduling unavailable && require_realtime -> Degraded-but-alive (the
        // old start() InitError throw is REPLACED by this, the task's explicit §8 addition).
        rt_error_.store(RtError::NotOperational, std::memory_order_release);
        degraded_.store(true, std::memory_order_release);
    }
    // Requested / BusFault: clean teardown -- the steady loop published the bus tier each
    // cycle; parked waiters are woken by stop()'s notify_all.
}

void ServoController::set_rpm(double rpm) {
    const std::shared_lock<std::shared_mutex> lk(api_mutex_);
    if (degraded_.load(std::memory_order_acquire)) {  // §8 Degraded-but-alive: motion APIs throw, never act
        throw BusError("set_rpm unavailable: " + (degraded_reason_.empty() ? last_error() : degraded_reason_));
    }
    if (config_.mode == ControlMode::ProfilePosition) {  // #61: PV and switchable accept set_rpm; only a fixed PP config rejects
        throw ConfigError("set_rpm requires Profile Velocity (PV) or switchable mode; this servo is configured PP -- use go_to/go_for");
    }
    // R3 exclusion matrix (§3): a PV setpoint yields to a LIVE blocking move (a go_for timed run) --
    // reject "operation ongoing" (set_rpm(0) too; halt() is the stop verb). PV setpoints are
    // latest-wins AMONG THEMSELVES (no slot), so this rejects ONLY under a live blocking move.
    if (motion_slot_busy()) {
        throw BusError("set_rpm: a motion operation is already in progress");
    }
    push_velocity(rpm);
}

// Private: convert rpm -> guarded device velocity + submit. NO slot check / NO lock (the caller --
// set_rpm after its slot check, or go_for(PV) which OWNS the slot for its whole run -- holds both).
void ServoController::push_velocity(double rpm) noexcept {
    const double clamped = clamp_rpm(rpm, config_.max_motor_speed_rpm);
    std::int32_t dev = rpm_to_device_velocity(clamped, config_.counts_per_rev, config_.gear_ratio);
    dev = clamp_to_stop_budget(dev);  // #47-P3b R1: stoppable-within-teardown-window guard (0x60FF)
    (void)commands_.push(Command{SetVelocity{dev}});
}

std::int32_t ServoController::clamp_to_stop_budget(std::int32_t vel_cps) const noexcept {
    // #47-P3b R1 VELOCITY GUARD: a commanded velocity must be STOPPABLE within the controlled-stop
    // teardown window -- else a lifecycle-stop-while-moving would still be ramping when close()
    // de-energizes = torque-cut. Clamp to the budget DERIVED from that window (vel_budget_cps_).
    // CLAMP not reject: the motor turns at the ceiling, observably below the request. Applied to
    // BOTH the PV setpoint (0x60FF/set_rpm) AND the PP move speed (0x6081/go_to,go_for) -- same
    // hazard, same formula (architect FLAG 2). Inert (budget 0) when quick-stop isn't configured.
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
    // SINGLE-CAS claim (M7b): succeed only if the slot is FREE or holds an ALREADY-TERMINAL gen
    // (reclaim). A concurrent second claimer that read the same terminal `cur` loses the CAS ->
    // reloads a live gen -> returns false ("operation ongoing"). No check-then-claim TOCTOU.
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
        if (degraded_.load(std::memory_order_acquire)) {  // §8
            throw BusError("go_to unavailable: " + (degraded_reason_.empty() ? last_error() : degraded_reason_));
        }
        if (config_.mode == ControlMode::ProfileVelocity) {  // #61: PP and switchable accept go_to; only a fixed PV config rejects
            throw ConfigError("go_to requires Profile Position (PP) or switchable mode; this servo is configured PV -- use set_rpm");
        }
        // ABSOLUTE target in the ZEROED frame: add zero_offset_counts to map the
        // user's zeroed position to the raw encoder frame, so go_to(X) lands where
        // position_revs()==X after reset_zero (get_position is zeroed too).
        const std::int32_t counts = static_cast<std::int32_t>(revs_to_counts(position, config_.counts_per_rev, config_.gear_ratio) +
                                                              state_.zero_offset_counts.load(std::memory_order_acquire));
        const double clamped_rpm = clamp_rpm(rpm, config_.max_motor_speed_rpm);
        const std::int32_t prof = clamp_to_stop_budget(rpm_to_device_velocity(clamped_rpm, config_.counts_per_rev, config_.gear_ratio));
        g = next_generation_.fetch_add(1, std::memory_order_relaxed) + 1;
        if (!try_claim_motion_slot(g)) {  // R3 single-in-flight: a live blocking move already owns the slot
            throw BusError("go_to: a motion operation is already in progress");
        }
        move_timeout = config_.move_timeout_ms != 0 ? std::chrono::milliseconds(config_.move_timeout_ms) : std::chrono::milliseconds(30000);
        (void)commands_.push(Command{SetTarget{counts, static_cast<std::uint32_t>(std::abs(prof)), false, g}});
    }  // release the shared lock BEFORE parking (so reconfigure isn't blocked for the whole move)

    await_move(g, move_timeout);
}

void ServoController::go_for(double rpm, double revs) {
    if (config_.mode != ControlMode::ProfileVelocity) {  // #61: PP + switchable -> relative PP move; PV -> timed run (below)
        std::uint32_t g = 0;
        std::chrono::milliseconds move_timeout{0};
        {
            const std::shared_lock<std::shared_mutex> lk(api_mutex_);
            if (degraded_.load(std::memory_order_acquire)) {  // §8
                throw BusError("go_for unavailable: " + (degraded_reason_.empty() ? last_error() : degraded_reason_));
            }
            // RELATIVE move (frame-agnostic): push SetTarget{relative=true} so the FSM
            // computes target = actual + delta. Do NOT route through go_to -- go_to now
            // adds zero_offset (absolute frame), which would double-shift a relative move.
            const std::int32_t delta = revs_to_counts(revs, config_.counts_per_rev, config_.gear_ratio);
            const double clamped_rpm = clamp_rpm(rpm, config_.max_motor_speed_rpm);
            const std::int32_t prof = rpm_to_device_velocity(clamped_rpm, config_.counts_per_rev, config_.gear_ratio);
            g = next_generation_.fetch_add(1, std::memory_order_relaxed) + 1;
            if (!try_claim_motion_slot(g)) {  // R3 single-in-flight
                throw BusError("go_for: a motion operation is already in progress");
            }
            move_timeout =
                config_.move_timeout_ms != 0 ? std::chrono::milliseconds(config_.move_timeout_ms) : std::chrono::milliseconds(30000);
            (void)commands_.push(Command{SetTarget{delta, static_cast<std::uint32_t>(std::abs(prof)), true, g}});
        }
        await_move(g, move_timeout);
        return;
    }
    // PV: run at rpm for the time to cover `revs`, then halt. R3: HOLD the single-in-flight slot for
    // the whole timed run (a concurrent go_to/go_for/set_rpm rejects "operation ongoing"), releasing
    // it at the end. Uses push_velocity (bypasses set_rpm's slot check -- we OWN the slot).
    double duration_s = 0.0;
    std::uint32_t g = 0;
    {
        const std::shared_lock<std::shared_mutex> lk(api_mutex_);
        if (degraded_.load(std::memory_order_acquire)) {  // §8
            throw BusError("go_for unavailable: " + (degraded_reason_.empty() ? last_error() : degraded_reason_));
        }
        g = next_generation_.fetch_add(1, std::memory_order_relaxed) + 1;
        if (!try_claim_motion_slot(g)) {
            throw BusError("go_for: a motion operation is already in progress");
        }
        const double effective_rpm = clamp_rpm(rpm, config_.max_motor_speed_rpm);
        if (effective_rpm != 0.0) {
            duration_s = std::abs(revs / (effective_rpm / 60.0));
        }
        push_velocity(rpm);
    }
    if (duration_s > 0.0) {
        std::this_thread::sleep_for(std::chrono::duration<double>(duration_s));
    }
    {
        const std::shared_lock<std::shared_mutex> lk(api_mutex_);
        push_velocity(0.0);  // command zero velocity so the run actually stops
    }
    // Release the slot -- a timed PV run has no gen-terminal, so we (the single owner) free it via a
    // CAS that clears ONLY our own claim (a concurrent reconfigure that reset it doesn't get clobbered).
    motion_slot_.compare_exchange_strong(g, 0, std::memory_order_acq_rel, std::memory_order_relaxed);
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

std::uint64_t ServoController::loop_cycle() const noexcept {
    return state_.loop_cycle.load(std::memory_order_relaxed);
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
            append("drive fault " + hex(code) + (gloss.empty() ? "" : " (" + gloss + ")"));
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
        case RtError::MotorStopped:
            append("motor stopped");
            break;
        case RtError::MotorDisabled:
            append("motor disabled");
            break;
        case RtError::ModeMismatch:
            append("drive mode-of-operation (0x6061) did not match the commanded mode -- refused to energize");
            break;
        case RtError::None:
            break;
    }
    return out;
}

}  // namespace ethercat::servo
