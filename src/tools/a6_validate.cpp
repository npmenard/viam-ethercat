// a6_validate -- TEMPORARY hardware bring-up validation for the A6-EC servo.
//
// Exercises the ethercat master library against a REAL A6 drive in safe stages:
//   Stage A (always):  open NIC -> enumerate -> SDO identity + key objects
//                       (PRE-OP, read-only; no PDO config, no motion).
//   Stage B (always):  configure (PDO remap + 0x6060 mode + OP) -> cyclic
//                       process() -> drive controlword to READY-TO-SWITCH-ON
//                       (motor NOT energized) and read live feedback.
//   --enable           additionally step the CiA402 ladder to OPERATION-ENABLED
//                       and HOLD at the current position (motor energizes,
//                       holding torque, NO commanded motion).
//   --move-pp R [RPM]  after enabling, command a RELATIVE PP move of R revs at
//                       RPM (default 60) via the bit4 new-setpoint handshake and
//                       watch convergence. *** MOTION -- opt-in only. ***
//   --move-sine        after enabling, stream a CSP soft-started sine.
//                       *** MOTION -- opt-in only. ***
//
// #47 P2: this tool now runs on ethercat::Runner. The hand-rolled Phase-1 pump
// (run_to_operational + observer), the Phase-2 steady loop, the local DcPacer,
// and the two teardown loops are DELETED -- the Runner owns the RT thread,
// realtime setup, the one pacer, bring-up, the steady cadence, the stopping
// window, and master.close(). What remains HERE is pure POLICY:
//   - A6Control::step()        = the old Phase-2 CiA402 branch tree, verbatim
//   - A6Control::sync_faulted() = the old 0x603F==0x8700 bring-up gate (A6
//                                 knowledge stays in the CONSUMER -- #41 layering)
//   - the stopping window      = the old graceful teardown (CSP hold-then-disable)
//   - main()                   = a NON-RT printer polling Runner status + the
//                                 control's atomic telemetry (the old in-loop
//                                 prints, moved off the RT thread)
//
// Defaults are non-energizing and motionless. Runtime needs CAP_NET_RAW (raw
// socket). NOT a production path; the real driver is the Viam module.

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <span>
#include <string>
#include <thread>
#include <vector>

#include "ethercat/cia402.hpp"
#include "ethercat/errors.hpp"
#include "ethercat/master.hpp"
#include "ethercat/pdo_buffer.hpp"
#include "ethercat/pdo_mapping.hpp"
#include "ethercat/runner.hpp"
#include "ethercat/soem_backend.hpp"
#include "tools/sine_move.hpp"

namespace {

using namespace ethercat;

// --- CiA402 / A6 object indices (hex; the JSON config carries them in decimal) ---
constexpr std::uint16_t kControlword = 0x6040;
constexpr std::uint16_t kStatusword = 0x6041;
constexpr std::uint16_t kModeDisplay = 0x6061;
constexpr std::uint16_t kTargetPosition = 0x607A;
constexpr std::uint16_t kPositionActual = 0x6064;
constexpr std::uint16_t kProfileVelocity = 0x6081;
constexpr std::uint16_t kVelocityActual = 0x606C;
constexpr std::uint16_t kTorqueActual = 0x6077;
constexpr std::uint16_t kFaultCode = 0x603F;
// NOTE: this tool reads NO raw CoE objects -- identity comes from Master::slave_info()
// and live status/position from the control's PDO loads. The DC sync-type / health-
// counter objects (0x1C32 / 0x1C33) are the LIBRARY's to own; reaching OP via the
// bring-up FSM is the DC-confirmation.

constexpr std::uint16_t kErr741NoSync = 0x8700;  // A6 Er74.1 "no sync signal" (normal pre-OP)

constexpr double kCountsPerRev = 131072.0;  // A6 single-turn encoder = 2^17
constexpr std::uint32_t kLoopHz = 1000;

// Little-endian SDO-value bytes via the shared store_le (no hand-rolled packing).
// For the one config-time SDO value (fault-reset 0x2031:01).
template <PdoScalar T>
std::vector<std::byte> sdo_value(T v) {
    std::vector<std::byte> b(sizeof(T));
    store_le<T>(b, v);
    return b;
}

std::atomic<bool> g_stop{false};
extern "C" void on_sigint(int) {
    g_stop.store(true);
}

// Build the MasterConfig for one A6, mirroring etc/a6-hardware.example.json (RxPDO
// 0x1600 = ctrl + target-pos + profile-vel; TxPDO 0x1A00 = fault + status +
// mode-display + pos + vel + torque). `mode` selects 0x6060: ProfilePosition (the
// bit4-handshake --move-pp) or CyclicSyncPosition (the streamed --move-sine). Both
// reuse the SAME PDO map -- 0x607A serves the PP target AND the CSP streamed target --
// so one builder covers both; only the post-enable control semantics differ.
MasterConfig build_a6_config(const std::string& ifname, Cia402Mode mode) {
    MasterConfig cfg;
    cfg.ifname = ifname;
    cfg.target_loop_rate_hz = kLoopHz;  // 1 ms SYNC0 = 4 x 250 us (A6-legal)
    cfg.use_distributed_clocks = true;  // A6 supports ONLY DC sync
    cfg.dc_settle_cycles = 1000;        // ~1 s post-OP grace while the phase finishes locking
    cfg.max_consecutive_wkc_errors = 5;
    // The bring-up SETTLE bound uses MasterConfig's default (dc_op_gate_cycles). SYNC0 is
    // armed in PRE-OP inside configure() (before config_map_group); the Runner's bring-up
    // pump then runs SETTLE (phase-locked PD) -> request OP once -> AWAIT_OP, all gapless.

    SlaveConfig a6;
    a6.slave_id = 1;
    a6.default_mode = mode;  // 0x6060 set in configure(); PP=1 (handshake) or CSP=8 (streamed sine)
    // #44: the A6 accepts only 250 us-multiple SYNC0 cycles (else Er74.0 at OP entry).
    a6.sync_cycle_granularity_ns = 250'000;
    // #39: the vendor fault-reset is CONSUMER policy -- this tool runs it itself
    // post-configure via Master::sdo_write (see --reset-fault in main), BEFORE
    // Runner::start() (still the single port owner; the #39 bracket is Runner-owned).
    // #20: we DELIBERATELY do NOT write 0x1C32:01 (SM sync-type) -- CLAUDE.md.

    a6.rxpdo.assign_index = 0x1C12;
    a6.rxpdo.pdo_indices = {0x1600};
    a6.rxpdo.entries[0x1600] = {
        {kControlword, 0, 16},
        {kTargetPosition, 0, 32},
        {kProfileVelocity, 0, 32},
    };

    a6.txpdo.assign_index = 0x1C13;
    a6.txpdo.pdo_indices = {0x1A00};
    a6.txpdo.entries[0x1A00] = {
        {kFaultCode, 0, 16},
        {kStatusword, 0, 16},
        {kModeDisplay, 0, 8},
        {kPositionActual, 0, 32},
        {kVelocityActual, 0, 32},
        {kTorqueActual, 0, 16},
    };

    cfg.slaves.push_back(std::move(a6));
    return cfg;
}

struct Options {
    std::string ifname = "enp86s0";
    bool enable = false;
    bool move_pp = false;
    bool move_sine = false;  // --move-sine: CSP streamed soft-started sine (energized)
    bool csp_probe = false;  // --csp-probe: bring up in CSP mode (0x6060=8) but DO NOT enable --
                             // just read+print feedback. Non-energizing; safe.
    bool reset_fault = false;
    double move_revs = 0.0;
    double move_rpm = 60.0;
    double sine_amplitude = 20000.0;       // counts (peak); --sine-amplitude
    double sine_period = 4.0;              // seconds; --sine-period
    std::int32_t follow_err_limit = 5000;  // counts; CSP tool-level following-error abort
    int seconds = 6;
};

// RT->main telemetry: each field is an independent relaxed atomic. Cross-field skew
// of a cycle is fine for a 5 Hz console print; no field tears. Written every step()
// (cheap), read by main's printer.
struct Telemetry {
    std::atomic<std::uint16_t> sw{0};
    std::atomic<std::uint16_t> fc{0};
    std::atomic<std::uint16_t> cw{0};
    std::atomic<std::int8_t> mode{0};
    std::atomic<std::int32_t> pos{0};
    std::atomic<std::int32_t> vel{0};
    std::atomic<std::int32_t> target{0};
    std::atomic<std::uint64_t> cycle{0};
    std::atomic<std::int64_t> dc_phase_ns{0};
    std::atomic<std::uint64_t> bad_wkc{0};
    std::atomic<bool> enabled{false};
};

// The #47 P2 control: ALL the old Phase-2 policy, as a SlaveControl. One instance,
// one slave. RT-thread-only state lives in plain members (step/hooks are single-
// threaded by construction); main() reads only the Telemetry atomics + Runner status.
//
// Console I/O note: edge events (fault edge, OE reached, safety abort, move done)
// print ONE-SHOT from the RT thread -- technically blocking I/O in step(), accepted
// for this validation tool exactly as the old loop accepted it (rare, bounded); the
// PERIODIC telemetry print moved to main() where it belongs.
class A6Control final : public SlaveControl {
   public:
    A6Control(const Options& opt, Telemetry& tel) noexcept : opt_(opt), tel_(tel) {
        goal_ = opt_.enable ? Cia402State::OperationEnabled : Cia402State::ReadyToSwitchOn;
        profile_vel_ = static_cast<std::uint32_t>(opt_.move_rpm / 60.0 * kCountsPerRev);
    }

    // NON-RT, pre-spawn, may throw: resolve every typed field ONCE (configure-time
    // width asserts). This map is built in this file, so all eight are mapped.
    void on_configured(Master& master, std::uint16_t slave_id) override {
        cw_loc_ = master.resolve_rx<cia402::ControlWord>(slave_id);
        target_loc_ = master.resolve_rx<cia402::TargetPosition>(slave_id);
        pv_loc_ = master.resolve_rx<cia402::ProfileVelocity>(slave_id);
        sw_loc_ = master.resolve_tx<cia402::Statusword>(slave_id);
        pos_loc_ = master.resolve_tx<cia402::PositionActual>(slave_id);
        vel_loc_ = master.resolve_tx<cia402::VelocityActual>(slave_id);
        fc_loc_ = master.resolve_tx<cia402::FaultCode>(slave_id);
        mode_loc_ = master.resolve_tx<cia402::ModeDisplay>(slave_id);
    }

    // RT, every bring-up cycle: the old run_to_operational gate, verbatim -- Er74.1
    // pending == 0x603F reads 0x8700. A6 knowledge lives HERE (consumer), keeping the
    // Runner vendor-free (#41). Also stashes the DC phase so main's printer can show
    // bring-up phase-lock progress without touching the master (single port owner).
    bool sync_faulted(const CycleContext& ctx) const noexcept override {
        tel_.dc_phase_ns.store(ctx.dc_time_ns() % static_cast<std::int64_t>(1'000'000'000ULL / kLoopHz), std::memory_order_relaxed);
        return ctx.load<cia402::FaultCode::type>(fc_loc_) == kErr741NoSync;
    }

    void on_operational(CycleContext& ctx) noexcept override {
        std::cout << "[B] *** OPERATIONAL *** wkc=" << ctx.wkc().last << "/" << ctx.wkc().expected
                  << " -- DC bring-up complete (no Er74.1), entering CiA402 control\n";
    }

    void on_stop(StopReason reason) noexcept override {
        stop_reason_ = reason;
        if (reason == StopReason::BringupAborted) {
            std::cerr << "[B] !!! BRING-UP ABORTED: OP did not hold within the await window -- the drive did not reach\n"
                      << "    OP with full WKC + Er74.1 cleared (SYNC0 likely not truly established). OP was requested\n"
                      << "    ONCE + not re-requested (repeated Er74 OP-entry wedges the A6). Power-cycle + check DC\n"
                      << "    wiring/cycle.\n";
        } else {
            std::cout << "\n[B] stopping (" << to_string(reason) << ") -- running the teardown window (hold/disable policy)...\n";
        }
    }

    // RT, every steady cycle: the old Phase-2 branch tree. Loads at the top (this
    // cycle's latched feedback), stores at the bottom (ship with the NEXT exchange --
    // the same +1-cycle latency the old make_tpdo/submit had).
    void step(CycleContext& ctx) noexcept override {
        const Status status{ctx.load<cia402::Statusword::type>(sw_loc_)};
        const std::int32_t pos = ctx.load<cia402::PositionActual::type>(pos_loc_);
        const std::int32_t vel = ctx.load<cia402::VelocityActual::type>(vel_loc_);
        const std::uint16_t fc = ctx.load<cia402::FaultCode::type>(fc_loc_);
        const std::uint64_t cycle = ctx.cycle();

        if (!hold_captured_) {
            hold_pos_ = pos;
            target_ = pos;
            hold_captured_ = true;
        }

        // --- the stopping WINDOW (#47 §5): the old graceful teardown as POLICY.
        // CSP clean stop: hold the last commanded position ~100 cycles (cw 0x0F,
        // target frozen -- do NOT snap, that yanks the shaft), then disable (0x00)
        // for the rest of the window. Any unclean cause (safety abort, bus fault,
        // drive fault) or a non-energized run: disable immediately.
        if (ctx.stopping()) {
            ++stopping_steps_;
            const bool clean_csp_hold = opt_.move_sine && announced_op_ && !safety_abort_ && stop_reason_ == StopReason::Requested &&
                                        !status.fault() && stopping_steps_ <= 100;
            if (clean_csp_hold) {
                ctx.store<cia402::TargetPosition::type>(target_loc_, last_sine_target_);
                ctx.store<cia402::ControlWord::type>(cw_loc_, ControlWord::enable_operation());
            } else {
                ctx.store<cia402::ControlWord::type>(cw_loc_, ControlWord::disable_voltage());  // 0x00
            }
            publish(status.raw, fc, last_cw_, pos, vel, cycle, ctx);
            return;
        }

        // --- duration / completion stops (the old loop's exit conditions).
        if (cycle >= static_cast<std::uint64_t>(opt_.seconds) * kLoopHz) {
            ctx.request_stop();
        }
        if (opt_.move_pp && move_done_ && cycle % 500 == 0) {
            ctx.request_stop();  // let it settle a moment, then finish
        }

        const bool faulted = status.decode() == Cia402State::Fault;
        if (faulted && !was_faulted_) {
            std::cout << "[B] !!! DRIVE FAULT t=" << cycle / kLoopHz << "s: 0x603F=0x" << std::hex << fc << " sw=0x" << status.raw
                      << std::dec << '\n';
        } else if (!faulted && was_faulted_) {
            std::cout << "[B] *** FAULT CLEARED *** -> " << to_string(status.decode()) << '\n';
        }
        was_faulted_ = faulted;

        // CSP energized-motion safety net (#24): once we're STREAMING the sine, ANY
        // drive-unhappy signal -- Fault (bit3), following-error (bit13), a nonzero
        // 0x603F, or the tool-level follow-err limit -- aborts and disables NOW (the
        // stopping branch above sees safety_abort_ and skips the hold). We do NOT
        // auto-reset + re-energize mid-motion.
        if (opt_.move_sine && announced_op_) {
            const std::int32_t follow_err = (cycle > enable_cycle_) ? (last_sine_target_ - pos) : 0;
            if (status.fault() || status.following_error() || fc != 0 || std::abs(follow_err) > opt_.follow_err_limit) {
                std::cerr << "[B] !!! CSP SAFETY ABORT: fault(bit3)=" << status.fault()
                          << " followingError(bit13)=" << status.following_error() << " 0x603F=0x" << std::hex << fc << std::dec
                          << " follow_err=" << follow_err << " (limit " << opt_.follow_err_limit
                          << ") -- stopping the sine + disabling immediately.\n";
                safety_abort_ = true;
                ctx.request_stop();
                ctx.store<cia402::ControlWord::type>(cw_loc_, ControlWord::disable_voltage());
                publish(status.raw, fc, ControlWord::disable_voltage(), pos, vel, cycle, ctx);
                return;
            }
        }

        // --- the old Phase-2 CiA402 branch tree, verbatim (policy only).
        std::uint16_t cw = fsm_.step(status, goal_);
        if (faulted) {
            // Generic CiA402 bit7 fault-reset edge (the A6's real reset is the vendor
            // 0x2031:01 SDO, issued pre-start; this is the in-loop steady-state fallback).
            cw = (last_cw_ & ControlWord::kFaultResetBit) ? 0x0000 : ControlWord::fault_reset();
        } else if (opt_.enable && status.operation_enabled()) {
            if (!announced_op_) {
                pos_enable_ = pos;  // CSP-safe origin: actual position the cycle OE is reached
                enable_cycle_ = cycle;
                announced_op_ = true;
                tel_.enabled.store(true, std::memory_order_relaxed);
                std::cout << "[B] *** OPERATION ENABLED *** (motor energized at pos=" << pos_enable_ << ")\n";
            }
            if (opt_.move_sine) {
                // CSP: stream the soft-started relative-to-enable sine every cycle; cw held
                // at 0x0F, NO bit4 handshake (that's PP). CSP-safe: sin(0)=0 -> the first
                // target == pos_enable (zero jump); amplitude ramps over the first period.
                const double t = static_cast<double>(cycle - enable_cycle_) / static_cast<double>(kLoopHz);
                target_ = ethercat::tools::csp_target_counts(true, pos, pos_enable_, opt_.sine_amplitude, opt_.sine_period, t);
                last_sine_target_ = target_;
                ctx.store<cia402::TargetPosition::type>(target_loc_, target_);
                cw = ControlWord::enable_operation();  // 0x0F
            } else {
                // --move-pp (bit4 handshake) or plain hold at the enable position.
                if (opt_.move_pp && !move_done_) {
                    target_ = hold_pos_ + static_cast<std::int32_t>(opt_.move_revs * kCountsPerRev);
                }
                ctx.store<cia402::TargetPosition::type>(target_loc_, target_);
                ctx.store<cia402::ProfileVelocity::type>(pv_loc_, profile_vel_);
                std::uint16_t base = ControlWord::enable_operation();  // 0x0F
                if (opt_.move_pp && !move_done_) {
                    // bit4 handshake: assert new-setpoint, hold until the drive acks (bit12),
                    // then drop it so the next move can re-arm.
                    if (!setpoint_latched_) {
                        base = ControlWord::with_new_setpoint(base, true);  // 0x1F
                        if (status.setpoint_acknowledged()) {
                            setpoint_latched_ = true;
                        }
                    } else {
                        base = ControlWord::with_new_setpoint(base, false);  // back to 0x0F
                        if (std::abs(pos - target_) < 300) {
                            move_done_ = true;
                            std::cout << "[B] move complete: pos=" << pos << " (target " << target_ << ")\n";
                        }
                    }
                }
                cw = base;
            }
        } else if (opt_.enable && opt_.move_sine) {
            // CSP enable-jump fix: climbing the ladder (06->07->0F) toward OE. In CSP the
            // drive latches its FIRST setpoint from the 0x607A on the cw=0x0F frame that
            // TRIGGERS OE -- one of THESE ladder frames. Stream target = live actual every
            // ladder cycle -> genuine zero jump. Without this, 0x607A stays 0 and the drive
            // slews from the absolute-encoder position toward 0 on enable.
            target_ = ethercat::tools::csp_target_counts(false, pos, pos_enable_, opt_.sine_amplitude, opt_.sine_period, 0.0);
            last_sine_target_ = target_;  // keep coherent for the graceful-hold + follow-err seed
            ctx.store<cia402::TargetPosition::type>(target_loc_, target_);
            // cw left as fsm_.step()'s ladder climb (06/07/0F as appropriate).
        } else if (!opt_.enable) {
            cw = ControlWord::shutdown();  // 0x06 -> ReadyToSwitchOn, NOT energized
        }
        ctx.store<cia402::ControlWord::type>(cw_loc_, cw);
        last_cw_ = cw;

        publish(status.raw, fc, cw, pos, vel, cycle, ctx);
    }

    bool safety_abort() const noexcept {
        return safety_abort_;
    }

   private:
    void publish(std::uint16_t sw,
                 std::uint16_t fc,
                 std::uint16_t cw,
                 std::int32_t pos,
                 std::int32_t vel,
                 std::uint64_t cycle,
                 const CycleContext& ctx) noexcept {
        tel_.sw.store(sw, std::memory_order_relaxed);
        tel_.fc.store(fc, std::memory_order_relaxed);
        tel_.cw.store(cw, std::memory_order_relaxed);
        tel_.mode.store(ctx.load<cia402::ModeDisplay::type>(mode_loc_), std::memory_order_relaxed);
        tel_.pos.store(pos, std::memory_order_relaxed);
        tel_.vel.store(vel, std::memory_order_relaxed);
        tel_.target.store(target_, std::memory_order_relaxed);
        tel_.cycle.store(cycle, std::memory_order_relaxed);
        tel_.dc_phase_ns.store(ctx.dc_time_ns() % static_cast<std::int64_t>(1'000'000'000ULL / kLoopHz), std::memory_order_relaxed);
        tel_.bad_wkc.store(ctx.wkc().bad_cycles, std::memory_order_relaxed);
    }

    const Options& opt_;
    Telemetry& tel_;
    Cia402Fsm fsm_;
    Cia402State goal_ = Cia402State::ReadyToSwitchOn;
    std::uint32_t profile_vel_ = 0;

    // resolved field handles (on_configured)
    FieldLocation cw_loc_, target_loc_, pv_loc_;
    FieldLocation sw_loc_, pos_loc_, vel_loc_, fc_loc_, mode_loc_;

    // RT-thread-only policy state (single-threaded by construction)
    std::uint16_t last_cw_ = 0;
    bool announced_op_ = false;
    bool setpoint_latched_ = false;
    bool move_done_ = false;
    bool was_faulted_ = false;
    bool hold_captured_ = false;
    bool safety_abort_ = false;
    std::int32_t hold_pos_ = 0;
    std::int32_t target_ = 0;
    std::int32_t pos_enable_ = 0;
    std::uint64_t enable_cycle_ = 0;
    std::int32_t last_sine_target_ = 0;
    std::uint32_t stopping_steps_ = 0;
    StopReason stop_reason_ = StopReason::None;
};

}  // namespace

int main(int argc, char** argv) {
    Options opt;
    std::vector<std::string> args(argv + 1, argv + argc);
    for (std::size_t i = 0; i < args.size(); ++i) {
        const std::string& a = args[i];
        if (a == "--enable") {
            opt.enable = true;
        } else if (a == "--reset-fault") {
            opt.reset_fault = true;
        } else if (a == "--move-pp" && i + 1 < args.size()) {
            opt.move_pp = true;  // requires an EXPLICIT --enable (checked below) -- no implicit energize
            opt.move_revs = std::stod(args[++i]);
            if (i + 1 < args.size() && args[i + 1].rfind("--", 0) != 0) {
                opt.move_rpm = std::stod(args[++i]);
            }
        } else if (a == "--move-sine") {
            opt.move_sine = true;  // requires an EXPLICIT --enable (checked below) -- no implicit energize
        } else if (a == "--csp-probe") {
            opt.csp_probe = true;  // CSP mode, NO enable -- read+print feedback only (diagnostic)
        } else if (a == "--sine-amplitude" && i + 1 < args.size()) {
            opt.sine_amplitude = std::stod(args[++i]);
        } else if (a == "--sine-period" && i + 1 < args.size()) {
            opt.sine_period = std::stod(args[++i]);
        } else if (a == "--follow-err-limit" && i + 1 < args.size()) {
            opt.follow_err_limit = std::stoi(args[++i]);
        } else if (a == "--seconds" && i + 1 < args.size()) {
            opt.seconds = std::stoi(args[++i]);
        } else if (a.rfind("--", 0) != 0) {
            opt.ifname = a;
        } else {
            std::cerr << "usage: a6_validate [ifname] [--enable] [--reset-fault] [--move-pp REVS [RPM]]\n"
                      << "                   [--move-sine [--sine-amplitude N] [--sine-period S]] [--csp-probe] [--seconds N]\n"
                      << "  The DC bring-up is automatic (Runner-owned, #47): configure() arms SYNC0 in PRE-OP, the\n"
                      << "  Runner's pump runs SETTLE -> request OP once -> AWAIT_OP, gapless + phase-locked.\n"
                      << "  Er74.1 in SAFE-OP is normal pre-sync, clears at OP.\n"
                      << "  --enable: energize to OperationEnabled (holding torque). REQUIRED for any move below --\n"
                      << "            --move-pp/--move-sine no longer imply it, so a forgotten --enable fails closed.\n"
                      << "  --move-sine: *** ENERGIZED MOTION (needs --enable) *** CSP-mode soft-started position sine,\n"
                      << "               relative to the enable position. pos(t)=pos_enable + A*min(1,t/T)*sin(2*pi*t/T);\n"
                      << "               A=--sine-amplitude (counts, def 20000), T=--sine-period (s, def 4.0). CSP-safe\n"
                      << "               (no jump) + ramped (no velocity step). --follow-err-limit N (counts, def 5000):\n"
                      << "               abort+disable if |commanded-actual| exceeds it. Mutually exclusive with --move-pp.\n"
                      << "  --move-pp REVS [RPM]: *** MOTION (needs --enable) *** PP-mode relative move via the bit4 handshake.\n"
                      << "  --csp-probe: NON-energizing diagnostic -- bring up in CSP mode (0x6060=8), hold at\n"
                      << "               ReadyToSwitchOn (NO enable), print feedback.\n"
                      << "  --reset-fault: clear a latent drive fault at bring-up via the A6 vendor SDO 0x2031:01=1.\n";
            return 2;
        }
    }

    const int mode_flags = static_cast<int>(opt.move_pp) + static_cast<int>(opt.move_sine) + static_cast<int>(opt.csp_probe);
    if (mode_flags > 1) {
        std::cerr << "error: --move-pp / --move-sine / --csp-probe are mutually exclusive (one mode of operation at a time)\n";
        return 2;
    }
    // SAFETY: a move must NOT silently energize. --move-pp/--move-sine require an explicit
    // --enable, so a forgotten --enable FAILS CLOSED instead of moving the shaft.
    if ((opt.move_pp || opt.move_sine) && !opt.enable) {
        std::cerr << "error: --move-pp / --move-sine command ENERGIZED MOTION and require an explicit --enable\n"
                  << "       (safety: motion must be a deliberate opt-in -- a forgotten --enable will not silently move the shaft).\n"
                  << "       For a non-energizing CSP feedback read, use --csp-probe instead.\n";
        return 2;
    }
    const Cia402Mode mode = (opt.move_sine || opt.csp_probe) ? Cia402Mode::CyclicSyncPosition : Cia402Mode::ProfilePosition;

    (void)std::signal(SIGINT, on_sigint);

    std::cout << "=== A6-EC validation on '" << opt.ifname << "' (Runner-based, #47 P2) ===\n"
              << "mode: " << to_string(mode) << " | enable=" << (opt.enable ? "YES (motor energizes)" : "no") << " | move="
              << (opt.move_sine ? "SINE A=" + std::to_string(static_cast<long>(opt.sine_amplitude)) +
                                      "ct T=" + std::to_string(opt.sine_period) + "s (CSP, soft-started)"
                  : opt.move_pp ? std::to_string(opt.move_revs) + " rev @ " + std::to_string(opt.move_rpm) + " rpm (PP)"
                                : "none (hold)")
              << "\n\n";

    // DC SYNC0 cycle = loop period; the A6 requires an integer multiple of 250 us.
    constexpr std::uint32_t kCycleNs = 1'000'000'000U / kLoopHz;
    static_assert(kCycleNs % 250'000U == 0, "SYNC0 cycle must be a 250us multiple for the A6");

    std::cout << "[rt] realtime setup (mlockall/SCHED_FIFO prio 80) is Runner-owned now (#47); best-effort\n"
              << "     (require_realtime=false) -- run with sudo for DC-safe timing.\n"
              << "[dc] phase-lock target = auto(mid-cycle) | SYNC0 CyclShift = 0ns (config knob)"
              << " | fault-reset at bring-up = " << (opt.reset_fault ? "ON (vendor 0x2031:01=1)" : "off") << "\n\n";

    Master master(build_a6_config(opt.ifname, mode), std::make_unique<SoemBackend>());

    // --- Stage A: open + enumerate + read-only SDO identity (PRE-OP) ---
    try {
        master.init();
    } catch (const Error& e) {
        std::cerr << "init failed: " << e.what() << '\n';
        return 1;
    }
    std::cout << "[A] bus up: A6 enumerated (1 slave, matches config).\n";
    const SlaveInfo info = master.slave_info(1);
    std::cout << "    identity: vendor=0x" << std::hex << info.vendor_id << " product=0x" << info.product_code << " rev=0x" << info.revision
              << std::dec << " name=\"" << info.name << "\" (Rx " << info.output_bytes << "B / Tx " << info.input_bytes << "B)\n"
              << "[A] OK -- EtherCAT enumeration confirmed.\n\n";

    const std::uint16_t slave = 1;

    // --- Stage B: configure to SAFE-OP + DC; the Runner owns everything after start().
    try {
        master.configure();
    } catch (const Error& e) {
        std::cerr << "[B] configure failed: " << e.what() << "\n"
                  << "    (an AL-reject at the SAFE-OP transition lands here; the AL status code in the message names why.)\n";
        return 1;
    }
    std::cout << "[B] configured to SAFE-OP, DC enabled (expected WKC=" << master.expected_wkc()
              << "); handing the bus to the Runner (bring-up pump: SETTLE -> request OP -> AWAIT_OP)...\n";

    // #39 consumer-side vendor fault-reset: BEFORE Runner::start() this thread is the
    // single port owner, so a plain blocking SDO is safe (after start() the #39
    // rt_active bracket -- Runner-owned -- makes it throw). Read 0x603F; if a latent
    // fault is present, clear it via the A6 VENDOR SDO 0x2031:01 = 1 (NOT CiA402 bit7).
    if (opt.reset_fault) {
        try {
            std::array<std::byte, 2> fc_raw{};
            const std::size_t n = master.sdo_read(slave, kFaultCode, 0, fc_raw);
            const std::uint16_t fc0 = n >= 2 ? load_le<std::uint16_t>(fc_raw) : 0;
            if (fc0 != 0) {
                std::cout << "[B] latent drive fault 0x" << std::hex << fc0 << std::dec
                          << " -- clearing via vendor SDO 0x2031:01=1 (consumer-side, #39)...\n";
                master.sdo_write(slave, 0x2031, 0x01, sdo_value<std::uint16_t>(1));
            } else {
                std::cout << "[B] --reset-fault: no latent fault (0x603F=0), nothing to clear.\n";
            }
        } catch (const Error& e) {
            std::cerr << "[B] --reset-fault SDO failed (continuing; the bring-up gate still guards OP): " << e.what() << '\n';
        }
    }

    // --- the Runner (#47 P2): library-owned RT thread / pacer / bring-up / window / close.
    Telemetry tel;
    A6Control control(opt, tel);
    RunnerConfig rc;
    rc.rt_priority = 80;
    rc.require_realtime = false;  // validation tool runs best-effort (matches the old behavior)
    rc.bringup_timeout = std::chrono::milliseconds(120'000);
    // 150 stopping cycles: the CSP clean stop holds the last commanded position for the
    // first 100 (no shaft yank), then 50 of cw=0x00 disable -- the old two teardown loops
    // as window policy inside A6Control::step().
    rc.teardown_cycles = 150;

    Runner runner(master, rc);
    try {
        runner.attach(slave, control);
        runner.start();  // on_configured (field resolution) runs here; throws abort cleanly
    } catch (const Error& e) {
        std::cerr << "[B] runner start failed: " << e.what() << '\n';
        master.close();
        return 1;
    }

    // --- main = the NON-RT printer + SIGINT relay (the old in-loop prints, off-thread).
    auto last_print = std::chrono::steady_clock::now() - std::chrono::seconds(1);
    while (runner.status().phase != RunnerPhase::Stopped) {
        if (g_stop.load()) {
            runner.request_stop();  // SIGINT -> graceful stop (the window runs the disable policy)
        }
        const auto now = std::chrono::steady_clock::now();
        if (now - last_print >= std::chrono::milliseconds(200)) {  // ~5 Hz
            last_print = now;
            const RunnerStatus st = runner.status();
            if (st.phase == RunnerPhase::BringingUp) {
                std::cout << "[B] bring-up... dcPhase=" << tel.dc_phase_ns.load(std::memory_order_relaxed) << "ns\n";
            } else if (st.phase == RunnerPhase::Running || st.phase == RunnerPhase::Stopping) {
                const Status status{tel.sw.load(std::memory_order_relaxed)};
                std::cout << "    t=" << tel.cycle.load(std::memory_order_relaxed) / kLoopHz << "s " << to_string(status.decode())
                          << " sw=0x" << std::hex << status.raw << " 0x603F=0x" << tel.fc.load(std::memory_order_relaxed) << std::dec
                          << " mode=" << static_cast<int>(tel.mode.load(std::memory_order_relaxed)) << " Rx.cw=0x" << std::hex
                          << tel.cw.load(std::memory_order_relaxed) << std::dec
                          << " cmdTarget=" << tel.target.load(std::memory_order_relaxed)
                          << " pos=" << tel.pos.load(std::memory_order_relaxed)
                          << " followErr=" << (tel.target.load(std::memory_order_relaxed) - tel.pos.load(std::memory_order_relaxed))
                          << " vel=" << tel.vel.load(std::memory_order_relaxed) << " badWKC=" << tel.bad_wkc.load(std::memory_order_relaxed)
                          << " dcPhase=" << tel.dc_phase_ns.load(std::memory_order_relaxed) << "ns"
                          << (st.phase == RunnerPhase::Stopping ? " [STOPPING]" : "") << '\n';
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    const StopReason reason = runner.status().reason;
    runner.stop();  // join -> rt_active(false) -> master.close() (the proven INIT teardown)

    const WkcStats stats = master.wkc_stats();  // library-side tally (incl. window cycles)
    std::cout << "\n=== done. stop=" << to_string(reason) << (control.safety_abort() ? " (CSP SAFETY ABORT)" : "")
              << " | bad-WKC cycles: " << stats.bad_cycles << " / " << stats.total_cycles
              << " | contract violations: " << runner.contract_violations() << " ===\n";
    // Exit code: bring-up/rt-setup failures are hard errors (the old return 1 paths);
    // a completed run -- including a safety abort that cleanly disabled -- reports 0
    // with the cause printed (matches the old tool's behavior).
    return (reason == StopReason::BringupAborted || reason == StopReason::RtSetupFailed) ? 1 : 0;
}
