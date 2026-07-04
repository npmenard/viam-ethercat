#pragma once

// cia402_policy.hpp -- the GENERIC CiA402 motion policy (#47-P3b sub-step 1).
//
// The proven bench sequencing (a6_validate A6Control, #53) collapsed to a library-grade,
// device-AGNOSTIC policy: BOTH a6_validate's A6Control AND the Viam module's ServoController
// wrap THIS. The genericity thesis (spec §1): a correct CiA402 driver needs almost no
// per-device code.
//
// BOUNDARY (architect-reviewed, spec §2/§4/§9; #17/#18 diet):
//  * The policy is PURE-COUNTS + unit-agnostic -- it never sees a rev / rpm / gear ratio
//    (the wrapper converts, §4 / FOLD 1). It does NO motion monitoring: is-moving / reached
//    are the DRIVER's job (statusword bit10 for a generic drive, a position-stability
//    heuristic for the A6 whose bit10 is broken) -- the policy owns only the CiA402
//    SEQUENCING (enable ladder, mode-echo gate, the 4-phase new-setpoint handshake,
//    quick-stop). Runtime mode-SWITCH orchestration is the DRIVER's job too (#18): the
//    policy is a DUMB per-mode executor -- it writes 0x6060 = cmd.mode + that mode's command
//    objects every cycle and never decides to switch.
//  * The only device residuals are the two quick-stop VALUES (0x6085 decel, 0x605A option),
//    passed to the ctor. Standard CiA402 object indices + controlword/statusword bit
//    semantics ARE the standard and live here. Disposition is STRUCTURAL by hook -- a Halt
//    intent HOLDS (CiA402 bit8), ctx.stopping()/on_stop() DE-ENERGIZE via Quick-Stop (FOLD 4).
//
// The policy operates on a CycleContext (library RT boundary) + typed cia402::Field aliases.

#include <cstddef>
#include <cstdint>
#include <vector>

#include "ethercat/cia402.hpp"
#include "ethercat/errors.hpp"
#include "ethercat/master.hpp"  // FieldLocation
#include "ethercat/pdo_buffer.hpp"
#include "ethercat/runner.hpp"  // CycleContext, ConfigContext, StopReason

namespace ethercat {

// --- What the wrapper commands the policy each cycle (pure counts). ---
struct PolicyCommand {
    Cia402Mode mode = Cia402Mode::ProfilePosition;  // 0x6060 mode to command THIS cycle
    std::int32_t target_counts = 0;                 // PP absolute target (counts)
    std::uint32_t profile_velocity = 0;             // PP move speed 0x6081 (counts/s)
    std::int32_t target_velocity = 0;               // PV target (counts/s)
    bool enable = false;                            // energize to OperationEnabled
    bool halt = false;                              // R1 motion-stop: assert CiA402 Halt (bit8), HOLD energized
    // WRAPPER signal: (re)arm the PP new-setpoint handshake THIS cycle (a new target was adopted).
    // Replaces the old opaque token -- the driver, not the policy, decides when a move begins.
    bool new_setpoint = false;
};

// --- What the policy publishes each cycle (the wrapper reads pos/vel from ctx directly). ---
struct PolicyState {
    std::int8_t current_mode = 0;  // 0x6061 echo (confirmed device mode)
    std::uint16_t fault_code = 0;  // raw 0x603F (device error code; generic never names it)
    bool mode_mismatch = false;    // mode-echo fail-closed refused to enable (#45)
    // --- PP 4-phase new-setpoint handshake signals ---
    bool handshake_idle = true;        // the new-setpoint handshake is quiescent (WRAPPER gates completion on this + its own at-target)
    bool handshake_timed_out = false;  // PER-CYCLE: the ack (or ack-clear) timed out THIS cycle -> WRAPPER maps to abort_active_move
};

// The generic policy. Owns its resolved FieldLocations + the RT-only sequencing state.
// Not a SlaveControl -- the wrapper's SlaveControl hooks delegate here.
class Cia402Policy {
   public:
    // The two device residuals (spec §1): the quick-stop deceleration VALUE written to 0x6085
    // (counts/s^2; 0 => quick-stop not configured, configure() skips the SDO setup) and the
    // 0x605A option the drive is ASSERTED to already hold (2 = decel-then-auto-SwitchOnDisabled).
    explicit Cia402Policy(std::uint32_t quick_stop_decel, std::int16_t quick_stop_option = 2) noexcept
        : quick_stop_decel_(quick_stop_decel), quick_stop_option_(quick_stop_option) {}

    // NON-RT, pre-spawn, may throw (ConfigError -> Runner start aborts, wrapper -> Degraded):
    // resolve the standard fields + the quick-stop SDO setup (0x605A assert-==required;
    // 0x6085 write + readback-echo). Returns the ECHOED 0x6085 (the wrapper does the VEL guard
    // against the commanded velocity -- a wrapper/units concern). `needs_quick_stop` gates the
    // SDO setup (a consumer that stops by coast skips it).
    std::uint32_t configure(ConfigContext& cfg, bool needs_quick_stop) {
        // REQUIRED fields (any CiA402 drive maps them; a missing one is a real misconfig -> throw).
        cw_loc_ = cfg.resolve_rx<cia402::ControlWord>();
        sw_loc_ = cfg.resolve_tx<cia402::Statusword>();
        // OPTIONAL / mode-conditional fields: a generic consumer maps only what its mode drives
        // (a PV-only map omits 0x607A/0x6081; a PP-only map omits 0x60FF; feedback objects 0x603F/
        // 0x6061 are optional). Resolve tolerantly; every per-cycle access guards on mapped().
        target_loc_ = cfg.resolve_rx_optional<cia402::TargetPosition>();
        pv_loc_ = cfg.resolve_rx_optional<cia402::ProfileVelocity>();
        tv_loc_ = cfg.resolve_rx_optional<cia402::TargetVelocity>();
        mode_wr_loc_ = cfg.resolve_rx_optional<cia402::ModeOfOperation>();  // 0x6060 RxPDO; absent -> mode is SDO-set only
        fc_loc_ = cfg.resolve_tx_optional<cia402::FaultCode>();
        mode_loc_ = cfg.resolve_tx_optional<cia402::ModeDisplay>();

        std::uint32_t echoed = 0;
        if (needs_quick_stop) {
            // 0x605A (quick-stop option) ASSERT == required (do NOT write it -- a warm write
            // needs a control-power cycle). A different option silently breaks the controlled
            // stop premise (0=coast, 1=decel on 0x6084 not 0x6085).
            std::array<std::byte, 2> qso{};
            const std::size_t n = cfg.sdo_read(kQuickStopOption, 0, qso);
            const std::int16_t qs_opt = n >= 2 ? load_le<std::int16_t>(qso) : std::int16_t{-1};
            if (qs_opt != quick_stop_option_) {
                throw ConfigError("Cia402Policy: 0x605A (quick-stop option) = " + std::to_string(qs_opt) +
                                  ", require == " + std::to_string(quick_stop_option_) +
                                  " (decel on 0x6085 -> auto SwitchOnDisabled). Refusing to energize.");
            }
            // 0x6085 (quick-stop decel) write + readback-echo; use the ECHOED value downstream.
            cfg.sdo_write(kQuickStopDecel, 0, sdo_bytes<std::uint32_t>(quick_stop_decel_));
            std::array<std::byte, 4> qd{};
            const std::size_t m = cfg.sdo_read(kQuickStopDecel, 0, qd);
            echoed = m >= 4 ? load_le<std::uint32_t>(qd) : 0U;
            if (echoed == 0U) {
                throw ConfigError("Cia402Policy: 0x6085 (quick-stop decel) readback = 0/absent after write. Refusing to energize.");
            }
        }
        qs_decel_echoed_ = echoed;
        return echoed;
    }

    std::uint32_t qs_decel_echoed() const noexcept {
        return qs_decel_echoed_;
    }
    int bit4_edges() const noexcept {
        return bit4_edges_;
    }
    const PolicyState& state() const noexcept {
        return state_;
    }

    // RT, every steady cycle. Consumes `cmd` + the cycle feedback (ctx), writes the controlword
    // + the mode's command objects, updates state_. Returns the controlword written (telemetry).
    // ctx.stopping() -> Quick-Stop de-energize (FOLD 4); Halt intent -> HOLD (CiA402 bit8, cw 0x0F).
    std::uint16_t step(CycleContext& ctx, const PolicyCommand& cmd) noexcept {
        const Status status{ctx.load<cia402::Statusword::type>(sw_loc_)};
        state_.fault_code = fc_loc_.mapped() ? ctx.load<cia402::FaultCode::type>(fc_loc_) : std::uint16_t{0};
        state_.current_mode = mode_loc_.mapped() ? ctx.load<cia402::ModeDisplay::type>(mode_loc_) : std::int8_t{0};
        state_.handshake_timed_out = false;  // PER-CYCLE signal: re-armed each step, set only on the timeout cycle

        // #70: count consecutive cycles the WIRE controlword had Halt (bit8) clear, off last_cw_ (the
        // cw we WROTE last cycle -- exactly what the drive observed). The PP handshake gates its bit4
        // raise on this so a new-setpoint edge never coincides with a halt release (the A6 ignores it).
        if ((last_cw_ & ControlWord::kHaltBit) != 0U) {
            halt_clear_cycles_ = 0;
        } else if (halt_clear_cycles_ < kSetpointHaltSettleCycles) {
            ++halt_clear_cycles_;
        }

        // WRAPPER-signalled new setpoint: (re)arm the 4-phase handshake for the new target.
        if (cmd.new_setpoint) {
            handshake_ = Handshake::WriteTarget;
            state_.handshake_idle = false;
            bit4_high_ = false;
        }

        // FOLD 4: the stopping window (ctx.stopping) DE-ENERGIZES via CiA402 Quick-Stop -- the drive
        // ramps via 0x6085 then auto-SwitchOnDisabled (0x605A==required). The status-based cw->0x00 is
        // the backstop: once the drive reports SwitchOnDisabled (reached rest + auto-disabled) drop to
        // disable-voltage. (No velocity debounce -- the 0x605A auto-transition is the live de-energize.)
        if (ctx.stopping()) {
            std::uint16_t qcw = kQuickStopCw;  // 0x0B: enable_operation() with bit2 (QS) cleared
            if (!announced_op_ || status.fault() || status.switch_on_disabled()) {
                qcw = ControlWord::disable_voltage();  // never energized / faulted / already at rest -> straight off
            }
            if (tv_loc_.mapped()) {
                ctx.store<cia402::TargetVelocity::type>(tv_loc_, 0);
            }
            ctx.store<cia402::ControlWord::type>(cw_loc_, qcw);
            return qcw;
        }

        const bool faulted = status.decode() == Cia402State::Fault;

        // #47-P3c (WIRE-CONFIRMED bug fix): a MAPPED 0x6060 must carry the commanded mode from CYCLE 0
        // -- BEFORE the mode-echo gate below and through the WHOLE enable ladder. The A6 follows the
        // RxPDO mode-of-operation over the SDO default once it is cycling, so if 0x6060 is only written
        // post-OperationEnabled it stays 0 through the ladder and the gate sees 0x6061 != commanded ->
        // SILENT request_stop. Seed it here every cycle: the policy is a DUMB per-mode executor (#18) --
        // it always writes 0x6060 = cmd.mode; the WRAPPER owns the runtime mode-switch orchestration and
        // picks cmd.mode (the transitional mode during a stop-first/settle, the target mode once confirmed).
        if (mode_wr_loc_.mapped()) {
            ctx.store<cia402::ModeOfOperation::type>(mode_wr_loc_, static_cast<std::int8_t>(cmd.mode));
        }

        // MODE-ECHO fail-closed (#45, load-bearing for PV): before climbing to OperationEnabled,
        // at SwitchedOn require 0x6061 == the commanded mode; a mismatch refuses to enable.
        if (cmd.enable && !mode_checked_ && !state_.mode_mismatch && status.switched_on() && !status.operation_enabled()) {
            if (state_.current_mode == static_cast<std::int8_t>(cmd.mode)) {
                mode_checked_ = true;
            } else {
                state_.mode_mismatch = true;
                ctx.request_stop();
            }
        }

        std::uint16_t cw =
            fsm_.step(status, cmd.enable && !state_.mode_mismatch ? Cia402State::OperationEnabled : Cia402State::ReadyToSwitchOn);
        if (faulted) {
            // In-loop CiA402 bit7 fault-reset EDGE (the standard reset; the vendor mechanism is
            // pre-start SDO). Level-toggle so the drive sees a rising edge, no spin.
            cw = (last_cw_ & ControlWord::kFaultResetBit) ? std::uint16_t{0x0000} : ControlWord::fault_reset();
        } else if (cmd.enable && !state_.mode_mismatch && status.operation_enabled()) {
            announced_op_ = true;
            cw = drive_operational_(ctx, cmd, status);
        } else if (!cmd.enable) {
            cw = ControlWord::shutdown();  // 0x06 -> ReadyToSwitchOn, not energized
        }
        ctx.store<cia402::ControlWord::type>(cw_loc_, cw);
        last_cw_ = cw;
        return cw;
    }

    void on_stop(StopReason /*reason*/) noexcept {}

    // Reset the per-run RT SEQUENCING state (published state + handshake + latches) for REUSE
    // across a wrapper stop/restart. Leaves the resolved FieldLocations + qs_decel_echoed_ intact
    // (configure() owns those and re-runs before the next RT phase). A single-shot wrapper
    // (a6_validate builds a fresh A6Control per run) never needs this; the module (persistent
    // ServoController across reconfigure) calls it from reset_run_state().
    void reset() noexcept {
        state_ = PolicyState{};
        handshake_ = Handshake::Idle;
        handshake_cycles_remaining_ = 0;
        last_cw_ = 0;
        announced_op_ = false;
        mode_checked_ = false;
        bit4_high_ = false;
        bit4_edges_ = 0;
        halt_clear_cycles_ = kSetpointHaltSettleCycles;  // #70: fresh run starts "halt settled"
    }

   private:
    // The OperationEnabled body: PP absolute move (4-phase bit4 handshake) or PV stream, + Halt (R1
    // HOLD via CiA402 bit8). Pure counts; NO reach predicate (the DRIVER owns is-moving/reached).
    std::uint16_t drive_operational_(CycleContext& ctx, const PolicyCommand& cmd, Status status) noexcept {
        std::uint16_t base = ControlWord::enable_operation();  // 0x0F
        if (mode_wr_loc_.mapped()) {
            ctx.store<cia402::ModeOfOperation::type>(mode_wr_loc_, static_cast<std::int8_t>(cmd.mode));
        }
        if (cmd.mode == Cia402Mode::ProfileVelocity) {
            // PV: stream target velocity. Halt asserts the drive's own Halt ramp (bit8).
            if (tv_loc_.mapped()) {
                ctx.store<cia402::TargetVelocity::type>(tv_loc_, cmd.target_velocity);
            }
            if (cmd.halt) {
                base = ControlWord::with_halt(base, true);
            }
            return base;
        }
        // ProfilePosition absolute move-to.
        if (target_loc_.mapped()) {
            ctx.store<cia402::TargetPosition::type>(target_loc_, cmd.target_counts);
        }
        if (pv_loc_.mapped()) {
            ctx.store<cia402::ProfileVelocity::type>(pv_loc_, cmd.profile_velocity);  // 0x6081 move speed (optional)
        }

        // FULL 4-phase CiA402 new-setpoint handshake (WriteTarget -> AwaitAck -> ClearBit4 ->
        // AwaitAckClear), armed by cmd.new_setpoint and bounded by kHandshakeTimeoutCycles. On an
        // ack/ack-clear timeout it sets handshake_timed_out (WRAPPER owns the abort). Halt asserts bit8
        // (below) and does NOT gate the handshake.
        switch (handshake_) {
            case Handshake::Idle:
                break;  // quiescent: bit4 low
            case Handshake::WriteTarget:
                if (halt_clear_cycles_ < kSetpointHaltSettleCycles) {
                    break;  // #70: halt not yet observed clear a full cycle -- hold bit4 LOW, stay in WriteTarget
                }
                base = ControlWord::with_new_setpoint(base, true);
                if (!bit4_high_) {
                    ++bit4_edges_;  // count the 0->1 edge once (DA-I)
                    bit4_high_ = true;
                }
                handshake_ = Handshake::AwaitAck;
                handshake_cycles_remaining_ = kHandshakeTimeoutCycles;
                break;
            case Handshake::AwaitAck:
                if (status.setpoint_acknowledged()) {
                    handshake_ = Handshake::ClearBit4;
                    base = ControlWord::with_new_setpoint(base, true);
                } else if (handshake_cycles_remaining_ == 0) {
                    state_.handshake_timed_out = true;  // ack never arrived -> WRAPPER aborts the move
                    handshake_ = Handshake::Idle;
                    bit4_high_ = false;
                } else {
                    --handshake_cycles_remaining_;
                    base = ControlWord::with_new_setpoint(base, true);
                }
                break;
            case Handshake::ClearBit4:
                handshake_ = Handshake::AwaitAckClear;
                handshake_cycles_remaining_ = kHandshakeTimeoutCycles;
                bit4_high_ = false;
                break;  // bit4 dropped
            case Handshake::AwaitAckClear:
                if (!status.setpoint_acknowledged()) {
                    handshake_ = Handshake::Idle;
                } else if (handshake_cycles_remaining_ == 0) {
                    state_.handshake_timed_out = true;
                    handshake_ = Handshake::Idle;
                } else {
                    --handshake_cycles_remaining_;
                }
                break;  // bit4 low
        }
        state_.handshake_idle = (handshake_ == Handshake::Idle);
        if (cmd.halt) {
            base = ControlWord::with_halt(base, true);
        }
        return base;
    }

    // Standard CiA402 object indices used by the policy (THE STANDARD, not device facts).
    static constexpr std::uint16_t kQuickStopDecel = 0x6085;
    static constexpr std::uint16_t kQuickStopOption = 0x605A;
    // Quick-stop controlword: enable_operation() with bit2 (Quick-Stop) CLEARED = 0x0B.
    static constexpr std::uint16_t kQuickStopCw = ControlWord::enable_operation() & ~std::uint16_t{0x0004};
    // #70 (WIRE-PROVEN, task #9): the A6 does NOT honor a new-setpoint (bit4) rising edge until it
    // has observed Halt (bit8) CLEAR for a full cycle. Require this many consecutive cycles with the
    // WRITTEN controlword's Halt bit clear before raising bit4 (2 => ~2 ms @ 1 kHz, small with margin).
    static constexpr std::uint32_t kSetpointHaltSettleCycles = 2;
    // 4-phase new-setpoint ack (and ack-clear) timeout, in cycles. Internal constant (#17: the config
    // knob died) -- 100 ms @ 1 kHz is generous; the A6 acks within a few cycles.
    static constexpr std::uint32_t kHandshakeTimeoutCycles = 100;

    template <PdoScalar T>
    static std::vector<std::byte> sdo_bytes(T v) {
        std::vector<std::byte> b(sizeof(T));
        store_le<T>(b, v);
        return b;
    }

    // 4-phase PP new-setpoint handshake sub-FSM.
    enum class Handshake : std::uint8_t { Idle, WriteTarget, AwaitAck, ClearBit4, AwaitAckClear };

    Cia402Fsm fsm_;
    PolicyState state_;
    Handshake handshake_ = Handshake::Idle;
    std::uint32_t handshake_cycles_remaining_ = 0;
    // #70: consecutive recent cycles the WRITTEN controlword had Halt (bit8) clear (capped at
    // kSetpointHaltSettleCycles). Init "settled" so the first move isn't delayed; reset to 0 on any
    // written Halt. Gates the bit4 raise so a coincident halt-release + bit4 edge never reaches the wire.
    std::uint32_t halt_clear_cycles_ = kSetpointHaltSettleCycles;

    FieldLocation cw_loc_, target_loc_, pv_loc_, tv_loc_, mode_wr_loc_;
    FieldLocation sw_loc_, fc_loc_, mode_loc_;

    const std::uint32_t quick_stop_decel_;  // 0x6085 write value (0 = quick-stop not configured)
    const std::int16_t quick_stop_option_;  // 0x605A asserted value
    std::uint32_t qs_decel_echoed_ = 0;
    std::uint16_t last_cw_ = 0;
    bool announced_op_ = false;
    bool mode_checked_ = false;
    bool bit4_high_ = false;
    int bit4_edges_ = 0;
};

}  // namespace ethercat
