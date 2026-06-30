#pragma once

// a6_control.hpp -- A6Control (the a6_validate bench SlaveControl policy) + Options +
// Telemetry + the A6 object-index constants, extracted from a6_validate.cpp to a header
// (#53) so the energized PP/PV control logic is offline-testable against a sim
// Master+Runner (mirrors sine_move.hpp's pure-header pattern). a6_validate.cpp keeps
// main() + build_a6_config() + the SIGINT relay and includes this.
//
// Lives in `namespace ethercat::tools`, so the library names (SlaveControl,
// CycleContext, Cia402Fsm, ControlWord, cia402::*, ...) resolve unqualified from the
// enclosing `ethercat` scope -- no `using namespace` at header scope.

#include <array>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <vector>

#include "ethercat/cia402.hpp"
#include "ethercat/errors.hpp"
#include "ethercat/master.hpp"
#include "ethercat/pdo_buffer.hpp"
#include "ethercat/runner.hpp"
#include "tools/sine_move.hpp"

namespace ethercat::tools {

// --- CiA402 / A6 object indices (hex; the JSON config carries them in decimal) ---
constexpr std::uint16_t kControlword = 0x6040;
constexpr std::uint16_t kStatusword = 0x6041;
constexpr std::uint16_t kModeDisplay = 0x6061;
constexpr std::uint16_t kTargetPosition = 0x607A;
constexpr std::uint16_t kTargetVelocity = 0x60FF;  // PV target (#53)
constexpr std::uint16_t kPositionActual = 0x6064;
constexpr std::uint16_t kProfileVelocity = 0x6081;
constexpr std::uint16_t kVelocityActual = 0x606C;
constexpr std::uint16_t kTorqueActual = 0x6077;
constexpr std::uint16_t kFaultCode = 0x603F;
constexpr std::uint16_t kQuickStopDecel = 0x6085;    // PV quick-stop deceleration (counts/s^2)
constexpr std::uint16_t kQuickStopOption = 0x605A;   // quick-stop option code (must read == 2)

constexpr std::uint16_t kErr741NoSync = 0x8700;  // A6 Er74.1 "no sync signal" (normal pre-OP)

constexpr double kCountsPerRev = 131072.0;  // A6 single-turn encoder = 2^17
constexpr std::uint32_t kLoopHz = 1000;

// --- #53 PV / quick-stop / reached tuning ---------------------------------------------
// Quick-stop controlword: enable-operation (0x0F) with bit2 (Quick-Stop) CLEARED -> 0x0B,
// the CiA402 T11 OperationEnabled->QuickStopActive command (spec §PV). POLARITY: the QS
// command is bit2=0, and 0x0F already has bit2 SET, so 0x0F & ~0x04 = 0x0B. (0x02 is the
// textbook-minimal exact value; if the A6 decodes QS by exact match it's the fallback.)
constexpr std::uint16_t kQuickStopCw = ControlWord::enable_operation() & ~std::uint16_t{0x0004};
// 0x606C velocity-estimate "stopped" threshold (just above the noise floor, DA) + debounce.
constexpr std::int32_t kZeroVelThresh = 500;       // counts/s (~0.23 rev/s @ 2^17 c/rev)
constexpr std::uint32_t kZeroVelDebounce = 5;      // consecutive sub-thresh cycles before the disable backstop
// 0x6085 quick-stop decel default = 50 rev/s^2 (stops 600 rpm in ~200 ms; well inside W).
constexpr std::uint32_t kQuickStopDecelDefault = 6'553'600;  // counts/s^2
constexpr std::int16_t kQuickStopOptionRequired = 2;         // 0x605A must read == 2 (decel on 0x6085 -> auto SwitchOnDisabled)
constexpr std::uint32_t kPvTeardownCycles = 2000;            // PV stopping window = 2 s @ 1 kHz (generous upper bound)
constexpr double kVelGuardMarginS = 0.1;                     // VEL-guard t_margin (s)

// Little-endian SDO-value bytes via the shared store_le (no hand-rolled packing).
template <PdoScalar T>
std::vector<std::byte> sdo_value(T v) {
    std::vector<std::byte> b(sizeof(T));
    store_le<T>(b, v);
    return b;
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
    // --- #53 new modes ---
    bool move_pos = false;            // --move-pos POS [VEL]: absolute Profile-Position move-to (PP)
    bool move_vel = false;           // --move-vel VEL: continuous Profile-Velocity until Ctrl-C (PV)
    std::int32_t pos_target = 0;     // --move-pos POS (absolute, counts)
    std::int32_t pp_vel_cps = 0;     // --move-pos optional VEL (profile velocity, counts/s); 0 => default from move_rpm
    std::int32_t pv_vel_cps = 0;     // --move-vel VEL (counts/s)
    std::int32_t pos_tol = 300;      // --pos-tol (DA-C: proven HW default 300; tighten only with a bench deadband measurement)
};

// Count the mutually-exclusive move modes set (>1 => CLI conflict). Pure + testable.
inline int mode_flag_count(const Options& o) noexcept {
    return static_cast<int>(o.move_pp) + static_cast<int>(o.move_sine) + static_cast<int>(o.csp_probe) +
           static_cast<int>(o.move_pos) + static_cast<int>(o.move_vel);
}

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
    // `mode` is the commanded 0x6060 (the CLI mode); its int8 value is what 0x6061 must
    // echo before we enable (#53 DA-B). build_a6_config() set the SAME mode at configure.
    A6Control(const Options& opt, Telemetry& tel, Cia402Mode mode) noexcept : opt_(opt), tel_(tel) {
        goal_ = opt_.enable ? Cia402State::OperationEnabled : Cia402State::ReadyToSwitchOn;
        profile_vel_ = static_cast<std::uint32_t>(opt_.move_rpm / 60.0 * kCountsPerRev);
        commanded_mode_disp_ = static_cast<std::int8_t>(mode);  // 0x6061 echo target (PP=1, PV=3, CSP=8)
        // PP profile velocity for --move-pos: explicit --move-pos VEL, else the move_rpm default.
        pp_profile_vel_ = opt_.pp_vel_cps > 0 ? static_cast<std::uint32_t>(opt_.pp_vel_cps) : profile_vel_;
    }

    // NON-RT, pre-spawn, may throw: resolve every typed field ONCE (configure-time
    // width asserts). This map is built in this file, so all eight are mapped. Uses the
    // restricted ConfigContext (#TODO-10) -- resolve_rx/tx bound to the slave, no Master&.
    void on_configured(ConfigContext& cfg) override {
        cw_loc_ = cfg.resolve_rx<cia402::ControlWord>();
        target_loc_ = cfg.resolve_rx<cia402::TargetPosition>();
        pv_loc_ = cfg.resolve_rx<cia402::ProfileVelocity>();
        sw_loc_ = cfg.resolve_tx<cia402::Statusword>();
        pos_loc_ = cfg.resolve_tx<cia402::PositionActual>();
        vel_loc_ = cfg.resolve_tx<cia402::VelocityActual>();
        fc_loc_ = cfg.resolve_tx<cia402::FaultCode>();
        mode_loc_ = cfg.resolve_tx<cia402::ModeDisplay>();
        tv_loc_ = cfg.resolve_rx<cia402::TargetVelocity>();  // #53: PV target (also over-mapped in PP/CSP, inert)

        // #53 PV configure-time, pre-energize refusals (fail-closed -- a throw here aborts
        // start() cleanly, before any motion). Only for --move-vel.
        if (opt_.move_vel) {
            // 0x605A (quick-stop option) ASSERT == 2 (DA Finding 2 / Q4): the 0x6085-decel ->
            // auto-SwitchOnDisabled regime the PV stop relies on. Do NOT write it (a warm
            // write needs a control-power cycle to apply) -- read-and-require.
            std::array<std::byte, 2> qso{};
            const std::size_t n = cfg.sdo_read(kQuickStopOption, 0, qso);
            const std::int16_t qs_opt = n >= 2 ? load_le<std::int16_t>(qso) : std::int16_t{-1};
            if (qs_opt != kQuickStopOptionRequired) {
                throw ConfigError("A6Control: 0x605A (quick-stop option) = " + std::to_string(qs_opt) +
                                  ", require == 2 (decel on 0x6085 then auto SwitchOnDisabled). Refusing to energize "
                                  "PV -- a different option silently breaks the controlled-stop premise (0=coast, "
                                  "1=decel on 0x6084). Fix the drive's 0x605A (control-power-cycle it) and retry.");
            }
            // 0x6085 (quick-stop decel) write + readback-echo (DA-A): use the ECHOED value
            // for the VEL guard + the HW decel-slope expectation, never the commanded value.
            cfg.sdo_write(kQuickStopDecel, 0, sdo_value<std::uint32_t>(kQuickStopDecelDefault));
            std::array<std::byte, 4> qd{};
            const std::size_t m = cfg.sdo_read(kQuickStopDecel, 0, qd);
            const std::uint32_t echoed = m >= 4 ? load_le<std::uint32_t>(qd) : 0U;
            if (echoed == 0U) {
                throw ConfigError("A6Control: 0x6085 (quick-stop decel) readback = 0 / absent after write. Refusing to "
                                  "energize PV -- without a real decel the quick-stop can't stop the motor in the window.");
            }
            qs_decel_echoed_ = echoed;
            // VEL guard (DA-H): with the echoed decel and the window W, a too-large VEL can't
            // ramp to 0 in time. Both are known now -> refuse at configure (fail-closed),
            // making "the window covers the worst-case decel" TRUE by construction.
            const double window_s = static_cast<double>(kPvTeardownCycles) / static_cast<double>(kLoopHz);
            const double vel_max = static_cast<double>(echoed) * (window_s - kVelGuardMarginS);
            if (std::abs(static_cast<double>(opt_.pv_vel_cps)) > vel_max) {
                throw ConfigError("A6Control: --move-vel " + std::to_string(opt_.pv_vel_cps) +
                                  " counts/s exceeds the quick-stop window budget (|VEL| <= 0x6085 * (W - margin) = " +
                                  std::to_string(static_cast<long long>(vel_max)) + " counts/s for 0x6085=" +
                                  std::to_string(echoed) + ", W=" + std::to_string(window_s) +
                                  "s). Refusing to energize -- it could not ramp to 0 before de-energize.");
            }
        }
    }

    // RT, every bring-up cycle: the old run_to_operational gate, verbatim -- Er74.1
    // pending == 0x603F reads 0x8700. A6 knowledge lives HERE (consumer), keeping the
    // Runner vendor-free (#41). NOTE const-with-side-effect: the dc_phase store below
    // is a deliberate TELEMETRY SIDE-CHANNEL (an atomic in the referenced Telemetry,
    // not logical state of this control) -- it's how main's printer shows bring-up
    // phase-lock progress without touching the master (single port owner).
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
            // #53 PV clean stop = CiA402 Quick-Stop (NOT torque-cut-at-speed): command
            // cw=0x0B (T11 OperationEnabled->QuickStopActive); the drive ramps via 0x6085.
            const bool clean_pv_stop = opt_.move_vel && announced_op_ && !safety_abort_ &&
                                       stop_reason_ == StopReason::Requested && !status.fault();
            const bool clean_csp_hold = opt_.move_sine && announced_op_ && !safety_abort_ && stop_reason_ == StopReason::Requested &&
                                        !status.fault() && stopping_steps_ <= 100;
            if (clean_pv_stop) {
                // Hold cw=0x0B and observe. Under 0x605A=2 the DRIVE decelerates and
                // auto-transitions to SwitchOnDisabled at its own zero (primary). The
                // event-driven cw->0x00 is the BACKSTOP: only once |0x606C| is sub-threshold
                // for kZeroVelDebounce cycles (so a velocity-estimate dip can't disable
                // mid-decel) -- which also covers the 0x605A in {5,6,7} "stay in QSA" regime.
                std::uint16_t qcw = kQuickStopCw;  // 0x0B
                if (std::abs(vel) < kZeroVelThresh) {
                    ++zerovel_cycles_;
                } else {
                    zerovel_cycles_ = 0;
                }
                if (status.switch_on_disabled() || zerovel_cycles_ >= kZeroVelDebounce) {
                    qcw = ControlWord::disable_voltage();  // 0x00 (no-op under =2 once auto-disabled; the disable under {5,6,7})
                }
                ctx.store<cia402::TargetVelocity::type>(tv_loc_, 0);
                ctx.store<cia402::ControlWord::type>(cw_loc_, qcw);
                last_cw_ = qcw;
            } else if (clean_csp_hold) {
                ctx.store<cia402::TargetPosition::type>(target_loc_, last_sine_target_);
                ctx.store<cia402::ControlWord::type>(cw_loc_, ControlWord::enable_operation());
            } else {
                ctx.store<cia402::ControlWord::type>(cw_loc_, ControlWord::disable_voltage());  // 0x00
            }
            publish(status.raw, fc, last_cw_, pos, vel, cycle, ctx);
            return;
        }

        // --- completion stop: move-to modes finish on their own when reached;
        // continuous modes (hold / sine / PV) run until SIGINT (Ctrl-C -> request_stop).
        if ((opt_.move_pp || opt_.move_pos) && move_done_ && cycle % 500 == 0) {
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

        // #53 DA-B -- MODE-ECHO fail-closed (load-bearing for PV). The A6 SILENTLY ignores
        // unsupported 0x6060 mode-sets (#45), so a PV (0x6060=3) the drive doesn't honor would
        // leave it in PP/CSP while we stream 0x60FF -> undefined ENERGIZED behavior. Before the
        // FSM is allowed to climb to OperationEnabled, require 0x6061 == the commanded mode.
        // Check ONCE at SwitchedOn (0x6061 is populated cyclically by then); a mismatch refuses
        // to enable -- hold at ReadyToSwitchOn + end the run. (Applies to every --enable mode.)
        if (opt_.enable && !mode_checked_ && !mode_refused_ && status.switched_on() && !status.operation_enabled()) {
            const std::int8_t md = ctx.load<cia402::ModeDisplay::type>(mode_loc_);
            if (md == commanded_mode_disp_) {
                mode_checked_ = true;  // echo confirmed -> allow the enable ladder
            } else {
                mode_refused_ = true;
                goal_ = Cia402State::ReadyToSwitchOn;  // REFUSE: do NOT energize
                std::cerr << "[B] !!! MODE ECHO MISMATCH: 0x6061=" << static_cast<int>(md) << " != commanded 0x6060="
                          << static_cast<int>(commanded_mode_disp_)
                          << " -- the A6 did not accept the commanded mode (#45); REFUSING to enable.\n";
                ctx.request_stop();
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
            } else if (opt_.move_vel) {
                // #53 PV: stream 0x60FF = VEL each cycle; the drive ramps via 0x6083 (accel).
                // (DA-G) mirror 0x607A = live actual position so the over-mapped target stays
                // benign if the A6 cross-supervises position. cw held at 0x0F.
                ctx.store<cia402::TargetVelocity::type>(tv_loc_, opt_.pv_vel_cps);
                ctx.store<cia402::TargetPosition::type>(target_loc_, pos);
                target_ = opt_.pv_vel_cps;  // telemetry shows the commanded velocity
                cw = ControlWord::enable_operation();  // 0x0F
            } else if (opt_.move_pos) {
                // #53 absolute Profile-Position move-to: bit6 = 0 (NOT relative -- with_new_setpoint
                // never sets kRelativeBit), 0x607A = absolute POS, 0x6081 = profile velocity.
                target_ = opt_.pos_target;
                ctx.store<cia402::TargetPosition::type>(target_loc_, target_);
                ctx.store<cia402::ProfileVelocity::type>(pv_loc_, pp_profile_vel_);
                std::uint16_t base = ControlWord::enable_operation();  // 0x0F (bit6 stays 0)
                if (!move_done_) {
                    // True rising-edge new-setpoint handshake, exactly ONE bit4 0->1 edge
                    // (DA-I): assert bit4 -> await bit12 ack -> clear bit4 -> watch reached.
                    if (!setpoint_latched_) {
                        base = ControlWord::with_new_setpoint(base, true);  // 0x1F
                        if (!bit4_high_) {
                            ++bit4_edges_;  // count the 0->1 edge once
                            bit4_high_ = true;
                        }
                        if (status.setpoint_acknowledged()) {
                            setpoint_latched_ = true;  // bit12 acked
                        }
                    } else {
                        base = ControlWord::with_new_setpoint(base, false);  // clear bit4 (re-armable)
                        bit4_high_ = false;
                        // Reached = actual within tol AND velocity ~0 (debounced). NEVER bit10
                        // (A6 ties it high, #43). kPosReachedTol default 300 (DA-C proven).
                        const bool pos_ok = std::abs(pos - target_) <= opt_.pos_tol;
                        if (std::abs(vel) < kZeroVelThresh) {
                            ++zerovel_cycles_;
                        } else {
                            zerovel_cycles_ = 0;
                        }
                        if (pos_ok && zerovel_cycles_ >= kZeroVelDebounce) {
                            move_done_ = true;
                            std::cout << "[B] move-pos reached: pos=" << pos << " target=" << target_ << " (|d|<=" << opt_.pos_tol
                                      << " counts, vel~0)\n";
                        }
                    }
                }
                cw = base;
            } else {
                // --move-pp (bit4 handshake) or plain hold at the enable position.
                // UNCHANGED (DA-C: --move-pp is a green HW-tested path; #53 does NOT flip it).
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

    // --- read-after-stop accessors (offline tests; the join is the happens-before edge) ---
    bool move_done() const noexcept {
        return move_done_;
    }
    bool mode_refused() const noexcept {
        return mode_refused_;
    }
    int bit4_edges() const noexcept {
        return bit4_edges_;
    }
    std::uint32_t qs_decel_echoed() const noexcept {
        return qs_decel_echoed_;
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
    FieldLocation cw_loc_, target_loc_, pv_loc_, tv_loc_;
    FieldLocation sw_loc_, pos_loc_, vel_loc_, fc_loc_, mode_loc_;

    // #53 commanded mode echo + PV/PP-abs config (set in ctor / on_configured)
    std::int8_t commanded_mode_disp_ = 1;  // 0x6061 echo target (PP=1, PV=3, CSP=8)
    std::uint32_t pp_profile_vel_ = 0;     // 0x6081 for --move-pos
    std::uint32_t qs_decel_echoed_ = 0;    // 0x6085 readback (DA-A; downstream math uses this)

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
    // #53 mode-echo gate + PP-abs handshake-edge + zero-vel debounce
    bool mode_checked_ = false;          // 0x6061 echo confirmed -> enable allowed
    bool mode_refused_ = false;          // echo mismatch -> refused to enable (one-shot)
    bool bit4_high_ = false;             // tracks the bit4 level for edge counting
    int bit4_edges_ = 0;                 // count of bit4 0->1 rising edges (#53 PP, DA-I: must be 1)
    std::uint32_t zerovel_cycles_ = 0;   // consecutive |vel|<thresh cycles (PV stop backstop + PP reached)
};

}  // namespace ethercat::tools
