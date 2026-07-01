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
    std::uint32_t mode_switch_settle_cycles = 200;  // T_switch (P3c; the transition undefined window)
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
        cw_loc_ = cfg.resolve_rx<cia402::ControlWord>();
        target_loc_ = cfg.resolve_rx<cia402::TargetPosition>();
        pv_loc_ = cfg.resolve_rx<cia402::ProfileVelocity>();
        tv_loc_ = cfg.resolve_rx<cia402::TargetVelocity>();
        sw_loc_ = cfg.resolve_tx<cia402::Statusword>();
        pos_loc_ = cfg.resolve_tx<cia402::PositionActual>();
        vel_loc_ = cfg.resolve_tx<cia402::VelocityActual>();
        fc_loc_ = cfg.resolve_tx<cia402::FaultCode>();
        mode_loc_ = cfg.resolve_tx<cia402::ModeDisplay>();

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
        const std::int32_t vel = ctx.load<cia402::VelocityActual::type>(vel_loc_);
        state_.fault_code = ctx.load<cia402::FaultCode::type>(fc_loc_);
        state_.current_mode = ctx.load<cia402::ModeDisplay::type>(mode_loc_);

        // FOLD 3: a new token resets the reached latch (a stale reach can't resolve a new move).
        if (cmd.token != state_.active_token) {
            state_.active_token = cmd.token;
            state_.reached = false;
            setpoint_latched_ = false;
            zerovel_cycles_ = 0;
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
                ctx.store<cia402::TargetVelocity::type>(tv_loc_, 0);
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

   private:
    // The OperationEnabled body: PP absolute move (bit4 handshake + reached) or PV stream, +
    // Halt (R1 HOLD). Pure counts; the reached predicate is actual-vs-target + vel~0 (NEVER bit10).
    std::uint16_t drive_operational_(CycleContext& ctx, const PolicyCommand& cmd, Status status, std::int32_t pos, std::int32_t vel) noexcept {
        std::uint16_t base = ControlWord::enable_operation();  // 0x0F
        if (cmd.mode == Cia402Mode::ProfileVelocity) {
            // PV: stream target velocity; halt -> ramp to 0 (HOLD energized, R1). Mirror 0x607A
            // = live actual so the over-mapped position target stays benign (DA-G).
            const std::int32_t v = cmd.halt ? 0 : cmd.target_velocity;
            ctx.store<cia402::TargetVelocity::type>(tv_loc_, v);
            ctx.store<cia402::TargetPosition::type>(target_loc_, pos);
            state_.phase = (std::abs(vel) > profile_.zero_vel_threshold) ? PolicyState::Phase::Moving : PolicyState::Phase::Holding;
            return base;
        }
        // ProfilePosition absolute move-to: bit6=0, 0x607A=target, 0x6081=(profile vel unchanged
        // -- the wrapper seeds it via the command? kept in target for now). Halt -> hold current.
        const std::int32_t tgt = cmd.halt ? pos : cmd.target_counts;
        ctx.store<cia402::TargetPosition::type>(target_loc_, tgt);
        ctx.store<cia402::ProfileVelocity::type>(pv_loc_, cmd.profile_velocity);  // 0x6081 move speed
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
                const bool pos_ok = std::abs(pos - tgt) <= profile_.position_tolerance;
                if (std::abs(vel) < profile_.zero_vel_threshold) {
                    ++zerovel_cycles_;
                } else {
                    zerovel_cycles_ = 0;
                }
                if (pos_ok && zerovel_cycles_ >= profile_.zero_vel_debounce) {
                    state_.reached = true;
                    state_.phase = PolicyState::Phase::Holding;
                } else {
                    state_.phase = PolicyState::Phase::Moving;
                }
            }
        } else {
            state_.phase = PolicyState::Phase::Holding;  // reached or halted -> hold at target
        }
        return base;
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

    DeviceProfile profile_;
    Cia402Fsm fsm_;
    PolicyState state_;
    Cia402State goal_ = Cia402State::OperationEnabled;

    FieldLocation cw_loc_, target_loc_, pv_loc_, tv_loc_;
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
