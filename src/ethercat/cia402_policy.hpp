#pragma once

// cia402_policy.hpp -- the GENERIC CiA402 motion policy (#47-P3b sub-step 1).
//
// The proven bench sequencing (a6_validate A6Control, #53) collapsed to a library-grade,
// device-AGNOSTIC policy: BOTH a6_validate's A6Control AND the Viam module's SlaveControl
// wrap THIS, parameterized only by a tiny DeviceProfile. The genericity thesis (spec §1):
// a correct CiA402 driver needs almost no per-device code.
//
// BOUNDARY (architect-reviewed, spec §2/§4/§9):
//  * The policy is PURE-COUNTS + unit-agnostic -- it never sees a rev / rpm / gear ratio
//    (the wrapper converts, §4 / FOLD 1). It never knows completion-generations -- an
//    intent carries an OPAQUE token the policy echoes in its state (FOLD 3). Disposition is
//    STRUCTURAL by hook -- a Halt intent HOLDS, ctx.stopping()/on_stop() DE-ENERGIZE via
//    Quick-Stop, always, both consumers (FOLD 4); no disposition parameter.
//  * DeviceProfile carries ONLY device CODES/VALUES/mechanism -- NEVER baked into the policy
//    (TODO-41 discipline): standard CiA402 object indices + controlword/statusword bit
//    semantics ARE the standard and live here; device fault/sync CODES, vendor reset objects,
//    and the quick-stop deceleration/option VALUES arrive via DeviceProfile. No coded sync gate
//    (FOLD 2): bring-up success = reach+hold-OP; sync_faulted is informational.
//
// The policy operates on a CycleContext (library RT boundary) + typed cia402::Field aliases.

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

#include "ethercat/cia402.hpp"
#include "ethercat/errors.hpp"
#include "ethercat/master.hpp"     // FieldLocation, SdoWrite
#include "ethercat/pdo_buffer.hpp"
#include "ethercat/runner.hpp"     // CycleContext, ConfigContext, StopReason

namespace ethercat {

// --- The ONE device residual (spec §1). Config DATA, never code. Unit-agnostic. ---
struct DeviceProfile {
    // THE genuine device-specific: how a fault clears. On some drives the standard CiA402
    // controlword bit7 reset SILENTLY fails and a vendor SDO reset is required instead. bit7 =
    // the standard in-loop reset. (Also selects R2 recovery LOCATION at P3b sub-step 4.)
    enum class FaultReset : std::uint8_t { Cia402Bit7, VendorSdo };
    FaultReset fault_reset = FaultReset::Cia402Bit7;
    std::optional<SdoWrite> vendor_fault_reset;  // the vendor reset SDO, when fault_reset == VendorSdo

    // Standard CiA402 tunables (VALUES arrive here; the standard OBJECTS are in the policy).
    std::int32_t position_tolerance = 300;      // reached: |target-actual| <= this
    std::int32_t zero_vel_threshold = 500;      // reached / quick-stop: |vel| <= this (counts/s)
    std::uint32_t zero_vel_debounce = 5;        // consecutive sub-thresh cycles before disable/reached latch
    std::uint32_t quick_stop_decel = 0;  // 0x6085 write (counts/s^2); 0 = unset -> a quick-stop consumer MUST set it (fail-closed at configure)
    std::int16_t quick_stop_option = 2;         // 0x605A required (decel-then-auto-SwitchOnDisabled)
    std::uint32_t mode_switch_settle_cycles = 200;  // T_switch (§6 step 4; the transition undefined-feedback window)
    // §6 step 1 stop-first bound: cycles to let the motor ramp to |vel| <= zero_vel_threshold in the
    // CURRENT mode before a mode-switch. If it never stops in this window (a load resisting stop) the
    // switch FAILS "motor didn't stop" (don't switch mid-motion, don't hang the queue). DISTINCT from
    // controlled_stop_window (the LIFECYCLE de-energize budget) -- this is a motion-hold ramp bound.
    std::uint32_t mode_switch_ramp_stop_cycles = 1000;

    // --- PP new-setpoint handshake shape (P3b sub-step 2/3, profile-gated superset) ---
    // 0 => the SIMPLE 2-phase handshake (raise bit4 until the drive acks, then clear + a
    //   debounced reach-check). No ack-clear wait, no timeout. The bench (a6_validate) default.
    // >0 => the FULL CiA402 4-phase new-setpoint handshake (WriteTarget -> AwaitAck ->
    //   ClearBit4 -> AwaitAckClear) with THIS ack timeout (cycles). On timeout the policy sets
    //   PolicyState.handshake_timed_out (a per-cycle signal); the WRAPPER owns the abort/gen
    //   disposition (FOLD 3 / rev-6 boundary -- the policy never touches completion). The Viam
    //   module (config_.handshake_timeout_cycles) uses this.
    std::uint32_t handshake_timeout_cycles = 0;
    // Halt (R1 HOLD) mechanism. false => setpoint-hold: command the target = current position
    //   (PP) / target velocity = 0 (PV) -- the drive ramps to a stop at the current point (bench
    //   default). true => assert CiA402 controlword bit8 (Halt) and LEAVE the command objects
    //   (the drive's own halt ramp stops it) -- the Viam module's Stop semantics.
    bool halt_uses_bit8 = false;
    // PV over-mapped 0x607A (target position): true => mirror it to the live actual each cycle so
    //   a position target mapped alongside PV stays benign (DA-G, bench default). false => leave it
    //   (the module maps only the fields it drives).
    bool pv_mirror_position = true;
};

// --- What the wrapper commands the policy each cycle (pure counts). ---
struct PolicyCommand {
    Cia402Mode mode = Cia402Mode::ProfilePosition;  // 0x6060 mode this command wants
    std::int32_t target_counts = 0;                 // PP absolute target (counts)
    std::uint32_t profile_velocity = 0;             // PP move speed 0x6081 (counts/s)
    std::int32_t target_velocity = 0;               // PV target (counts/s)
    bool enable = false;                            // energize to OperationEnabled
    bool halt = false;                              // R1 motion-stop: HOLD energized (ramp->0)
    bool fault_reset = false;                        // operator fault-reset intent
    // OPAQUE correlation token (FOLD 3): the policy echoes it in state.active_token + resets
    // the reached-latch when it changes; it NEVER interprets it. The wrapper maps token->gen.
    std::uint32_t token = 0;
};

// --- What the policy publishes each cycle (the wrapper reads pos/vel from ctx directly). ---
struct PolicyState {
    enum class Phase : std::uint8_t { Init, Enabling, Holding, Moving, Switching, Faulted, Resetting };
    Phase phase = Phase::Init;
    bool reached = false;              // move-complete for active_token (|d|<=tol && |vel|~0, NOT bit10)
    std::uint32_t active_token = 0;    // the token `reached` refers to
    std::int8_t current_mode = 0;      // 0x6061 echo (confirmed device mode)
    std::uint16_t fault_code = 0;      // raw 0x603F (device error code; generic never names it)
    bool mode_mismatch = false;        // mode-echo fail-closed refused to enable (#45)
    // --- PP 4-phase handshake signals (only meaningful when handshake_timeout_cycles > 0) ---
    bool handshake_idle = true;        // the new-setpoint handshake is quiescent (WRAPPER gates completion on this + its own at-target)
    bool handshake_timed_out = false;  // PER-CYCLE: the ack (or ack-clear) timed out THIS cycle -> WRAPPER maps to abort_active_move (FOLD 3)
    // --- §6 runtime mode-switch (sub-step 5) ---
    bool switching = false;            // a mode-switch sequence is in progress (stop-first / write / settle)
    bool mode_switch_failed = false;   // PER-CYCLE: the switch could NOT confirm (motor-didn't-stop | 0x6061 never echoed within T_switch | drive error) -> WRAPPER throws "mode-switch failed"; policy holds SAFE (energized, at rest, no lunge)
};

// The generic policy. Owns its resolved FieldLocations + the RT-only sequencing state.
// Not a SlaveControl -- the wrapper's SlaveControl hooks delegate here.
class Cia402Policy {
   public:
    explicit Cia402Policy(DeviceProfile profile) noexcept : profile_(profile) {}

    // NON-RT, pre-spawn, may throw (ConfigError -> Runner start aborts, wrapper -> Degraded):
    // resolve the standard fields + the PV quick-stop SDO setup (0x605A assert-==required;
    // 0x6085 write + readback-echo). Returns the ECHOED 0x6085 (the wrapper does the VEL guard
    // against the commanded velocity -- a wrapper/units concern). `needs_quick_stop` gates the
    // PV SDO setup (a PP-only consumer skips it).
    std::uint32_t configure(ConfigContext& cfg, bool needs_quick_stop) {
        // REQUIRED fields (any CiA402 drive maps them; a missing one is a real misconfig -> throw).
        cw_loc_ = cfg.resolve_rx<cia402::ControlWord>();
        sw_loc_ = cfg.resolve_tx<cia402::Statusword>();
        pos_loc_ = cfg.resolve_tx<cia402::PositionActual>();
        // OPTIONAL / mode-conditional fields: a generic consumer maps only what its mode drives
        // (a PV-only map omits 0x607A/0x6081; a PP-only map omits 0x60FF; feedback objects 0x606C/
        // 0x603F/0x6061 are optional). Resolve tolerantly; every per-cycle access guards on mapped().
        // (The bench A6 maps them all -> these resolve present -> behavior is unchanged.)
        target_loc_ = cfg.resolve_rx_optional<cia402::TargetPosition>();
        pv_loc_ = cfg.resolve_rx_optional<cia402::ProfileVelocity>();
        tv_loc_ = cfg.resolve_rx_optional<cia402::TargetVelocity>();
        mode_wr_loc_ = cfg.resolve_rx_optional<cia402::ModeOfOperation>();  // 0x6060 RxPDO (sub-step 5 runtime mode-switch); absent -> mode is SDO-set only
        vel_loc_ = cfg.resolve_tx_optional<cia402::VelocityActual>();
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
            if (qs_opt != profile_.quick_stop_option) {
                throw ConfigError("Cia402Policy: 0x605A (quick-stop option) = " + std::to_string(qs_opt) + ", require == " +
                                  std::to_string(profile_.quick_stop_option) +
                                  " (decel on 0x6085 -> auto SwitchOnDisabled). Refusing to energize.");
            }
            // 0x6085 (quick-stop decel) write + readback-echo; use the ECHOED value downstream.
            cfg.sdo_write(kQuickStopDecel, 0, sdo_bytes<std::uint32_t>(profile_.quick_stop_decel));
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
    // ctx.stopping() -> Quick-Stop de-energize (FOLD 4); Halt intent -> HOLD (ramp->0, cw 0x0F).
    std::uint16_t step(CycleContext& ctx, const PolicyCommand& cmd) noexcept {
        const Status status{ctx.load<cia402::Statusword::type>(sw_loc_)};
        const std::int32_t pos = ctx.load<cia402::PositionActual::type>(pos_loc_);
        // Optional feedback: 0 when unmapped (the wrapper may compute its own velocity estimate;
        // the policy's reached/quick-stop use this when present).
        const std::int32_t vel = vel_loc_.mapped() ? ctx.load<cia402::VelocityActual::type>(vel_loc_) : 0;
        state_.fault_code = fc_loc_.mapped() ? ctx.load<cia402::FaultCode::type>(fc_loc_) : std::uint16_t{0};
        state_.current_mode = mode_loc_.mapped() ? ctx.load<cia402::ModeDisplay::type>(mode_loc_) : std::int8_t{0};
        state_.handshake_timed_out = false;  // PER-CYCLE signal: re-armed each step, set only on the timeout cycle
        state_.mode_switch_failed = false;   // PER-CYCLE signal: set only on the cycle a mode-switch gives up

        // FOLD 3: a new token resets the reached latch (a stale reach can't resolve a new move).
        if (cmd.token != state_.active_token) {
            state_.active_token = cmd.token;
            state_.reached = false;
            setpoint_latched_ = false;
            bit4_high_ = false;
            zerovel_cycles_ = 0;
            // 4-phase mode: restart the new-setpoint handshake for the new target (WRITE -> ACK ->
            // CLEAR -> ACK-CLEAR). Simple 2-phase mode leaves handshake_ Idle (unused).
            if (profile_.handshake_timeout_cycles != 0) {
                handshake_ = Handshake::WriteTarget;
                state_.handshake_idle = false;
            }
        }

        // FOLD 4: the stopping window (ctx.stopping) DE-ENERGIZES via CiA402 Quick-Stop -- the
        // drive ramps via 0x6085 then auto-SwitchOnDisabled (0x605A==required); the event-driven
        // cw->0x00 is the backstop once |vel| is sub-threshold for the debounce.
        if (ctx.stopping()) {
            std::uint16_t qcw = kQuickStopCw;  // 0x0B: enable_operation() with bit2 (QS) cleared
            if (announced_op_ && !status.fault()) {
                if (std::abs(vel) < profile_.zero_vel_threshold) {
                    ++zerovel_cycles_;
                } else {
                    zerovel_cycles_ = 0;
                }
                if (status.switch_on_disabled() || zerovel_cycles_ >= profile_.zero_vel_debounce) {
                    qcw = ControlWord::disable_voltage();
                }
                if (tv_loc_.mapped()) {
                    ctx.store<cia402::TargetVelocity::type>(tv_loc_, 0);
                }
            } else {
                qcw = ControlWord::disable_voltage();  // never energized / faulted -> straight off
            }
            ctx.store<cia402::ControlWord::type>(cw_loc_, qcw);
            return qcw;
        }

        const bool faulted = status.decode() == Cia402State::Fault;
        state_.phase = faulted ? PolicyState::Phase::Faulted : state_.phase;

        // MODE-ECHO fail-closed (#45, load-bearing for PV): before climbing to OperationEnabled,
        // at SwitchedOn require 0x6061 == the commanded mode; a mismatch refuses to enable.
        if (cmd.enable && !mode_checked_ && !state_.mode_mismatch && status.switched_on() && !status.operation_enabled()) {
            if (state_.current_mode == static_cast<std::int8_t>(cmd.mode)) {
                mode_checked_ = true;
            } else {
                state_.mode_mismatch = true;
                goal_ = Cia402State::ReadyToSwitchOn;  // REFUSE: hold, do not energize
                ctx.request_stop();
            }
        }

        std::uint16_t cw = fsm_.step(status, cmd.enable && !state_.mode_mismatch ? Cia402State::OperationEnabled : Cia402State::ReadyToSwitchOn);
        if (faulted) {
            // In-loop CiA402 bit7 fault-reset EDGE (the standard reset; the vendor mechanism is
            // pre-start SDO). Level-toggle so the drive sees a rising edge, no spin.
            cw = (last_cw_ & ControlWord::kFaultResetBit) ? std::uint16_t{0x0000} : ControlWord::fault_reset();
            state_.phase = PolicyState::Phase::Resetting;
        } else if (cmd.enable && !state_.mode_mismatch && status.operation_enabled()) {
            if (!announced_op_) {
                announced_op_ = true;
                enable_pos_ = pos;
            }
            cw = drive_operational_(ctx, cmd, status, pos, vel);
        } else if (!cmd.enable) {
            cw = ControlWord::shutdown();  // 0x06 -> ReadyToSwitchOn, not energized
        }
        ctx.store<cia402::ControlWord::type>(cw_loc_, cw);
        last_cw_ = cw;
        return cw;
    }

    void on_stop(StopReason reason) noexcept {
        last_stop_reason_ = reason;
    }

    // Reset the per-run RT SEQUENCING state (published state + handshake + latches) for REUSE
    // across a wrapper stop/restart. Leaves profile_ + the resolved FieldLocations + qs_decel_echoed_
    // intact (configure() owns those and re-runs before the next RT phase). A single-shot wrapper
    // (a6_validate builds a fresh A6Control per run) never needs this; the module (persistent
    // ServoController across reconfigure) calls it from reset_run_state().
    void reset() noexcept {
        state_ = PolicyState{};
        goal_ = Cia402State::OperationEnabled;
        handshake_ = Handshake::Idle;
        handshake_cycles_remaining_ = 0;
        ms_phase_ = ModeSwitch::None;
        ms_cycles_ = 0;
        last_cw_ = 0;
        announced_op_ = false;
        mode_checked_ = false;
        setpoint_latched_ = false;
        bit4_high_ = false;
        bit4_edges_ = 0;
        zerovel_cycles_ = 0;
        enable_pos_ = 0;
        last_stop_reason_ = StopReason::None;
    }

   private:
    // The OperationEnabled body: PP absolute move (bit4 handshake + reached) or PV stream, +
    // Halt (R1 HOLD). Pure counts; the reached predicate is actual-vs-target + vel~0 (NEVER bit10).
    std::uint16_t drive_operational_(CycleContext& ctx, const PolicyCommand& cmd, Status status, std::int32_t pos, std::int32_t vel) noexcept {
        // §6 RUNTIME MODE-SWITCH: if the drive's CONFIRMED mode (0x6061) != the commanded mode, OR a
        // switch is mid-sequence, run the switch (HOLD energized, no motion) INSTEAD of the motion
        // body. Only when 0x6060 is RxPDO-mapped -- else runtime switching isn't possible and the mode
        // is fixed (byte-identical: A6Control doesn't map 0x6060, so this never triggers).
        if (mode_wr_loc_.mapped()) {
            const std::int8_t want = static_cast<std::int8_t>(cmd.mode);
            if (ms_phase_ != ModeSwitch::None || (state_.current_mode != 0 && state_.current_mode != want)) {
                return run_mode_switch_(ctx, cmd, pos, vel, want);
            }
        }
        std::uint16_t base = ControlWord::enable_operation();  // 0x0F
        // sub-step 5: when 0x6060 is RxPDO-mapped, keep it = the commanded mode on the wire each
        // steady cycle (no switch in progress -- the mode is confirmed and stable).
        if (mode_wr_loc_.mapped()) {
            ctx.store<cia402::ModeOfOperation::type>(mode_wr_loc_, static_cast<std::int8_t>(cmd.mode));
        }
        const bool bit8_halt = cmd.halt && profile_.halt_uses_bit8;
        if (cmd.mode == Cia402Mode::ProfileVelocity) {
            // PV: stream target velocity. Halt: setpoint-halt (halt_uses_bit8==false) ramps the
            // command to 0; bit8-halt leaves the command and asserts the drive's Halt ramp.
            const std::int32_t v = (cmd.halt && !profile_.halt_uses_bit8) ? 0 : cmd.target_velocity;
            if (tv_loc_.mapped()) {
                ctx.store<cia402::TargetVelocity::type>(tv_loc_, v);
            }
            if (profile_.pv_mirror_position && target_loc_.mapped()) {
                ctx.store<cia402::TargetPosition::type>(target_loc_, pos);  // benign over-mapped 0x607A (DA-G)
            }
            if (bit8_halt) {
                base = ControlWord::with_halt(base, true);
            }
            state_.phase = (std::abs(vel) > profile_.zero_vel_threshold) ? PolicyState::Phase::Moving : PolicyState::Phase::Holding;
            return base;
        }
        // ProfilePosition absolute move-to. setpoint-halt holds at the current point (target=pos);
        // bit8-halt keeps the latched target and lets the drive's Halt ramp stop it.
        const bool setpoint_halt = cmd.halt && !profile_.halt_uses_bit8;
        const std::int32_t tgt = setpoint_halt ? pos : cmd.target_counts;
        if (target_loc_.mapped()) {
            ctx.store<cia402::TargetPosition::type>(target_loc_, tgt);
        }
        if (pv_loc_.mapped()) {
            ctx.store<cia402::ProfileVelocity::type>(pv_loc_, cmd.profile_velocity);  // 0x6081 move speed (optional)
        }

        if (profile_.handshake_timeout_cycles == 0) {
            // --- SIMPLE 2-phase handshake (bench default): raise bit4 until ack, then clear +
            //     debounced reach. Halt (setpoint-hold) skips the handshake and holds. ---
            if (!state_.reached && !cmd.halt) {
                if (!setpoint_latched_) {
                    base = ControlWord::with_new_setpoint(base, true);  // bit4 rising edge (0x1F)
                    if (!bit4_high_) {
                        ++bit4_edges_;  // count the 0->1 edge once (DA-I)
                        bit4_high_ = true;
                    }
                    if (status.setpoint_acknowledged()) {
                        setpoint_latched_ = true;
                    }
                    state_.phase = PolicyState::Phase::Moving;
                } else {
                    base = ControlWord::with_new_setpoint(base, false);  // clear bit4 (re-armable)
                    bit4_high_ = false;
                    update_reached_(pos, vel, tgt);
                    state_.phase = state_.reached ? PolicyState::Phase::Holding : PolicyState::Phase::Moving;
                }
            } else {
                state_.phase = PolicyState::Phase::Holding;  // reached or halted -> hold at target
            }
            return base;
        }

        // --- FULL 4-phase CiA402 new-setpoint handshake (module): WriteTarget -> AwaitAck ->
        //     ClearBit4 -> AwaitAckClear. Ack/ack-clear timeout -> handshake_timed_out (WRAPPER
        //     owns the abort). The reach predicate runs INDEPENDENTLY every cycle. Halt asserts
        //     bit8 (below) and does NOT gate the handshake. ---
        switch (handshake_) {
            case Handshake::Idle:
                break;  // quiescent: bit4 low
            case Handshake::WriteTarget:
                base = ControlWord::with_new_setpoint(base, true);
                if (!bit4_high_) {
                    ++bit4_edges_;
                    bit4_high_ = true;
                }
                handshake_ = Handshake::AwaitAck;
                handshake_cycles_remaining_ = profile_.handshake_timeout_cycles;
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
                handshake_cycles_remaining_ = profile_.handshake_timeout_cycles;
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
        update_reached_(pos, vel, tgt);  // independent of the handshake FSM
        state_.phase = state_.reached ? PolicyState::Phase::Holding : PolicyState::Phase::Moving;
        if (bit8_halt) {
            base = ControlWord::with_halt(base, true);
        }
        return base;
    }

    // §6 canonical mode-switch sequence. HOLDS energized (cw 0x0F) throughout -- "no motion" during a
    // switch is NOT de-energize (R1). On give-up sets state_.mode_switch_failed (per-cycle); the drive
    // stays SAFE (at rest, energized, no lunge) and the WRAPPER throws "mode-switch failed". (A drive
    // FAULT during the window is NOT handled here -- step()'s faulted branch takes it: a fault is an
    // involuntary de-energize + the two-tier fault path, not a stay-safe hold.)
    std::uint16_t run_mode_switch_(CycleContext& ctx, const PolicyCommand& cmd, std::int32_t pos, std::int32_t vel, std::int8_t want) noexcept {
        (void)cmd;
        const std::uint16_t cw = ControlWord::enable_operation();  // 0x0F -- hold energized (§6 steps 1+4)
        const std::int8_t pp = static_cast<std::int8_t>(Cia402Mode::ProfilePosition);
        if (ms_phase_ == ModeSwitch::None) {  // ENTER: begin the stop-first ramp
            ms_phase_ = ModeSwitch::StopFirst;
            ms_cycles_ = 0;
            zerovel_cycles_ = 0;
        }
        state_.switching = true;

        if (ms_phase_ == ModeSwitch::StopFirst) {
            // Step 1 -- PRECONDITION: ramp to rest in the CURRENT mode (command NO motion), bounded.
            if (state_.current_mode == pp) {
                if (target_loc_.mapped()) {
                    ctx.store<cia402::TargetPosition::type>(target_loc_, pos);  // hold current position
                }
            } else if (tv_loc_.mapped()) {
                ctx.store<cia402::TargetVelocity::type>(tv_loc_, 0);  // ramp velocity -> 0
            }
            if (mode_wr_loc_.mapped()) {
                ctx.store<cia402::ModeOfOperation::type>(mode_wr_loc_, state_.current_mode);  // NOT switched yet
            }
            if (std::abs(vel) <= profile_.zero_vel_threshold) {
                ++zerovel_cycles_;
            } else {
                zerovel_cycles_ = 0;
            }
            if (zerovel_cycles_ >= profile_.zero_vel_debounce) {
                seed_and_write_mode_(ctx, pos, want);  // steps 2+3: seed safe + WRITE 0x6060
                ms_phase_ = ModeSwitch::Settle;
                ms_cycles_ = 0;
            } else if (++ms_cycles_ >= profile_.mode_switch_ramp_stop_cycles) {
                state_.mode_switch_failed = true;  // motor didn't stop -> never switch mid-motion
                ms_phase_ = ModeSwitch::None;
                state_.switching = false;
            }
            ctx.store<cia402::ControlWord::type>(cw_loc_, cw);
            return cw;
        }
        // Steps 4+5 -- SETTLE: hold the new mode + safe seed, NO motion; confirm 0x6061==want or fail.
        seed_and_write_mode_(ctx, pos, want);
        if (state_.current_mode == want) {  // CONFIRMED -> next cycle runs the new mode's motion body
            ms_phase_ = ModeSwitch::None;
            state_.switching = false;
        } else if (++ms_cycles_ >= profile_.mode_switch_settle_cycles) {
            state_.mode_switch_failed = true;  // 0x6061 never echoed the new mode (silent-mismatch, #45 shape)
            ms_phase_ = ModeSwitch::None;
            state_.switching = false;
        }
        ctx.store<cia402::ControlWord::type>(cw_loc_, cw);
        return cw;
    }

    // Seed the NEW mode's RxPDO command to a SAFE no-lunge value + write 0x6060 = new mode.
    void seed_and_write_mode_(CycleContext& ctx, std::int32_t pos, std::int8_t want) noexcept {
        if (want == static_cast<std::int8_t>(Cia402Mode::ProfilePosition)) {
            if (target_loc_.mapped()) {
                ctx.store<cia402::TargetPosition::type>(target_loc_, pos);  // 0x607A = THIS-cycle actual (no lunge)
            }
        } else if (tv_loc_.mapped()) {
            ctx.store<cia402::TargetVelocity::type>(tv_loc_, 0);  // 0x60FF = 0
        }
        if (mode_wr_loc_.mapped()) {
            ctx.store<cia402::ModeOfOperation::type>(mode_wr_loc_, want);
        }
    }

    // Latch state_.reached once |pos-target| <= tol AND |vel| sub-threshold for the debounce.
    void update_reached_(std::int32_t pos, std::int32_t vel, std::int32_t tgt) noexcept {
        const bool pos_ok = std::abs(pos - tgt) <= profile_.position_tolerance;
        if (std::abs(vel) < profile_.zero_vel_threshold) {
            ++zerovel_cycles_;
        } else {
            zerovel_cycles_ = 0;
        }
        if (!state_.reached && pos_ok && zerovel_cycles_ >= profile_.zero_vel_debounce) {
            state_.reached = true;
        }
    }

    // Standard CiA402 object indices used by the policy (THE STANDARD, not device facts).
    static constexpr std::uint16_t kQuickStopDecel = 0x6085;
    static constexpr std::uint16_t kQuickStopOption = 0x605A;
    // Quick-stop controlword: enable_operation() with bit2 (Quick-Stop) CLEARED = 0x0B.
    static constexpr std::uint16_t kQuickStopCw = ControlWord::enable_operation() & ~std::uint16_t{0x0004};

    template <PdoScalar T>
    static std::vector<std::byte> sdo_bytes(T v) {
        std::vector<std::byte> b(sizeof(T));
        store_le<T>(b, v);
        return b;
    }

    // 4-phase PP new-setpoint handshake sub-FSM (used only when handshake_timeout_cycles > 0).
    enum class Handshake : std::uint8_t { Idle, WriteTarget, AwaitAck, ClearBit4, AwaitAckClear };
    // §6 runtime mode-switch sub-FSM (sub-step 5): StopFirst (ramp to rest in the current mode) ->
    // Settle (0x6060 written, hold energized, wait for the 0x6061 confirm or T_switch/error fail).
    enum class ModeSwitch : std::uint8_t { None, StopFirst, Settle };

    DeviceProfile profile_;
    Cia402Fsm fsm_;
    PolicyState state_;
    Cia402State goal_ = Cia402State::OperationEnabled;
    Handshake handshake_ = Handshake::Idle;
    std::uint32_t handshake_cycles_remaining_ = 0;
    ModeSwitch ms_phase_ = ModeSwitch::None;  // §6 runtime mode-switch sub-state
    std::uint32_t ms_cycles_ = 0;             // stop-first / settle window counter

    FieldLocation cw_loc_, target_loc_, pv_loc_, tv_loc_, mode_wr_loc_;
    FieldLocation sw_loc_, pos_loc_, vel_loc_, fc_loc_, mode_loc_;

    std::uint32_t qs_decel_echoed_ = 0;
    std::uint16_t last_cw_ = 0;
    bool announced_op_ = false;
    bool mode_checked_ = false;
    bool setpoint_latched_ = false;
    bool bit4_high_ = false;
    int bit4_edges_ = 0;
    std::uint32_t zerovel_cycles_ = 0;
    std::int32_t enable_pos_ = 0;
    StopReason last_stop_reason_ = StopReason::None;
};

}  // namespace ethercat
