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

}  // namespace ethercat::tools
