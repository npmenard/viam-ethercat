#pragma once

// Generic, device-agnostic CiA402 motion policy
//
// Scope:
//  * The policy works in pure counts and is unit-agnostic The policy owns only
//    the CiA402 sequencing: enable ladder, mode-echo, the four-phase new-set-point
//    handshake (WriteTarget -> AwaitAck -> ClearBit4 -> AwaitAckClear), and quick-stop. Runtime mode switching is also the driver's job;
//    the policy is a per-mode executor that writes 0x6060 = cmd.mode plus that mode's command objects every cycle and never decides to
//    switch.
//  * The only device-specific values are the two quick-stop settings (0x6085 decel, 0x605A
//    option), passed to the constructor. Standard CiA402 object indices and controlword/
//    statusword bit semantics live here. A Halt intent holds position (CiA402 bit 8);
//    ctx.stopping()/on_stop() de-energize via quick-stop.
//
// The policy operates on a CycleContext (the library RT boundary) plus typed cia402::Field aliases.

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

#include "ethercat/cia402.hpp"
#include "ethercat/errors.hpp"
#include "ethercat/master.hpp"
#include "ethercat/pdo_buffer.hpp"
#include "ethercat/runner.hpp"

namespace ethercat {

struct PolicyCommand {
    Cia402Mode mode = Cia402Mode::ProfilePosition;  // 0x6060 mode
    std::int32_t target_counts = 0;                 // PP absolute target (counts)
    std::uint32_t profile_velocity = 0;             // PP move speed (counts/s)
    std::int32_t target_velocity = 0;               // PV target (counts/s)
    bool enable = false;
    bool halt = false;  // assert CiA402 Halt (bit 8) and hold position, stay energized
    bool new_setpoint = false;
};

struct PolicyState {
    std::int8_t current_mode = 0;  // 0x6061 echo (confirmed device mode)
    std::uint16_t fault_code = 0;  // raw 0x603F device error code
    bool mode_mismatch = false;
    bool handshake_idle = true;
    bool handshake_timed_out = false;
};

class Cia402Policy {
   public:
    // The two device-specific values: the quick-stop deceleration written to 0x6085
    // (counts/s^2; 0 means quick-stop is not configured and configure() skips the SDO setup)
    // and the 0x605A option the drive is asserted to already hold (2 = decelerate, then
    // auto-transition to SwitchOnDisabled).
    explicit Cia402Policy(std::uint32_t quick_stop_decel, std::int16_t quick_stop_option = 2) noexcept
        : quick_stop_decel_(quick_stop_decel), quick_stop_option_(quick_stop_option) {}

    // Non-RT, called before the RT thread spawns; may throw Error (which aborts Runner start,
    // or moves the wrapper to Degraded). Resolves the standard fields and does the quick-stop
    // SDO setup: assert 0x605A == required, then write and read back 0x6085. Returns the
    // read-back 0x6085.
    std::uint32_t configure(ConfigContext& cfg, bool needs_quick_stop) {
        cw_loc_ = cfg.resolve_rx<cia402::ControlWord>();
        sw_loc_ = cfg.resolve_tx<cia402::Statusword>();

        target_loc_ = cfg.try_resolve_rx<cia402::TargetPosition>();
        pv_loc_ = cfg.try_resolve_rx<cia402::ProfileVelocity>();
        tv_loc_ = cfg.try_resolve_rx<cia402::TargetVelocity>();
        mode_wr_loc_ = cfg.try_resolve_rx<cia402::ModeOfOperation>();  // 0x6060 RxPDO; nullopt -> mode is SDO-set only
        fc_loc_ = cfg.try_resolve_tx<cia402::FaultCode>();
        mode_loc_ = cfg.try_resolve_tx<cia402::ModeDisplay>();

        std::uint32_t echoed = 0;
        if (needs_quick_stop) {
            // Assert 0x605A (quick-stop option) equals the required value; do not write it, as
            // a warm rewrite needs a control-power cycle. A different option silently breaks the
            // controlled-stop premise (0 = coast, 1 = decelerate on 0x6084 rather than 0x6085).
            std::array<std::byte, 2> qso{};
            const std::size_t n = cfg.sdo_read(kQuickStopOption, 0, qso);
            const std::int16_t qs_opt = n >= 2 ? load_le<std::int16_t>(qso) : std::int16_t{-1};
            if (qs_opt != quick_stop_option_) {
                throw Error("Cia402Policy: 0x605A (quick-stop option) = " + std::to_string(qs_opt) + ", require == " +
                            std::to_string(quick_stop_option_) + " (decel on 0x6085 -> auto SwitchOnDisabled). Refusing to energize.");
            }
            // Write 0x6085 (quick-stop decel), then read it back and use the read-back value downstream.
            cfg.sdo_write(kQuickStopDecel, 0, sdo_bytes<std::uint32_t>(quick_stop_decel_));
            std::array<std::byte, 4> qd{};
            const std::size_t m = cfg.sdo_read(kQuickStopDecel, 0, qd);
            echoed = m >= 4 ? load_le<std::uint32_t>(qd) : 0U;
            if (echoed == 0U) {
                throw Error("Cia402Policy: 0x6085 (quick-stop decel) readback = 0/absent after write. Refusing to energize.");
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

    // RT, every steady cycle. Consumes `cmd` and the cycle feedback (ctx), writes the
    // controlword and the mode's command objects, and updates state_. Returns the controlword
    // written. ctx.stopping() de-energizes via quick-stop; a Halt intent holds position
    // (CiA402 bit 8, controlword 0x0F).
    std::uint16_t step(CycleContext& ctx, const PolicyCommand& cmd) noexcept {
        const Status status{ctx.load<cia402::Statusword::type>(sw_loc_)};
        state_.fault_code = fc_loc_ ? ctx.load<cia402::FaultCode::type>(*fc_loc_) : std::uint16_t{0};
        state_.current_mode = mode_loc_ ? ctx.load<cia402::ModeDisplay::type>(*mode_loc_) : std::int8_t{0};
        state_.handshake_timed_out = false;  // re-armed each step; set only on the cycle the handshake times out

        // Count consecutive cycles the written controlword had Halt (bit 8) clear, off last_cw_
        // (the controlword the drive actually observed). The PP handshake gates its bit-4 raise
        // on this so a new-set-point edge never coincides with a halt release -- a workaround for
        // drives that ignore a bit-4 edge arriving on the same cycle the Halt bit drops.
        if ((last_cw_ & ControlWord::kHaltBit) != 0U) {
            halt_clear_cycles_ = 0;
        } else if (halt_clear_cycles_ < kSetpointHaltSettleCycles) {
            ++halt_clear_cycles_;
        }

        // New set-point signalled by the wrapper
        if (cmd.new_setpoint) {
            handshake_ = Handshake::WriteTarget;
            state_.handshake_idle = false;
            bit4_high_ = false;
        }

        // The stopping window de-energizes via CiA402 quick-stop: the drive ramps down on
        // 0x6085, then auto-transitions to SwitchOnDisabled (0x605A must equal the required
        // option). Once it reports SwitchOnDisabled, drop to disable-voltage. The 0x605A
        // auto-transition is the actual de-energize, so no velocity debounce is needed.
        if (ctx.stopping()) {
            std::uint16_t qcw = kQuickStopCw;  // 0x0B: enable_operation() with bit 2 (quick-stop) cleared
            if (!announced_op_ || status.fault() || status.switch_on_disabled()) {
                qcw = ControlWord::disable_voltage();  // never energized, faulted, or already at rest: go straight off
            }
            if (tv_loc_) {
                ctx.store<cia402::TargetVelocity::type>(*tv_loc_, 0);
            }
            ctx.store<cia402::ControlWord::type>(cw_loc_, qcw);
            return qcw;
        }

        const bool faulted = status.decode() == Cia402State::Fault;

        // A mapped 0x6060 must carry the commanded mode from the first cycle, before the
        // mode-echo gate below and throughout the enable ladder: a drive with 0x6060 in its
        // RxPDO follows the cyclic value over the SDO default once cycling, so if 0x6060 is
        // written only after OperationEnabled it reads 0 through the ladder and the gate sees
        // 0x6061 != the commanded mode and stops. Seed it every cycle; the wrapper owns mode-switch
        // orchestration and picks cmd.mode.
        if (mode_wr_loc_) {
            ctx.store<cia402::ModeOfOperation::type>(*mode_wr_loc_, static_cast<std::int8_t>(cmd.mode));
        }

        // Mode-echo check: before climbing to OperationEnabled, at SwitchedOn require 0x6061 ==
        // the commanded mode. A mismatch refuses to enable.
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
            // CiA402 bit-7 fault-reset edge (the standard reset; the vendor mechanism is a
            // pre-start SDO). Toggle the level so the drive sees a rising edge.
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

    // Reset the per-run RT sequencing state (published state, handshake, latches) so the policy
    // can be reused across a wrapper stop/restart. Leaves the resolved FieldLocations and
    // qs_decel_echoed_ intact; configure() owns those and re-runs before the next RT phase.
    // Only a persistent wrapper needs this (the module's ServoController, across reconfigure);
    // a one-shot consumer just builds a fresh policy per run.
    void reset() noexcept {
        state_ = PolicyState{};
        handshake_ = Handshake::Idle;
        handshake_cycles_remaining_ = 0;
        last_cw_ = 0;
        announced_op_ = false;
        mode_checked_ = false;
        bit4_high_ = false;
        bit4_edges_ = 0;
        halt_clear_cycles_ = kSetpointHaltSettleCycles;  // a fresh run starts already halt-settled
    }

   private:
    std::uint16_t drive_operational_(CycleContext& ctx, const PolicyCommand& cmd, Status status) noexcept {
        std::uint16_t base = ControlWord::enable_operation();  // 0x0F
        if (mode_wr_loc_) {
            ctx.store<cia402::ModeOfOperation::type>(*mode_wr_loc_, static_cast<std::int8_t>(cmd.mode));
        }
        if (cmd.mode == Cia402Mode::ProfileVelocity) {
            // PV: stream target velocity. Halt asserts the drive's own Halt ramp (bit8).
            if (tv_loc_) {
                ctx.store<cia402::TargetVelocity::type>(*tv_loc_, cmd.target_velocity);
            }
            if (cmd.halt) {
                base = ControlWord::with_halt(base, true);
            }
            return base;
        }
        // ProfilePosition absolute move-to.
        if (target_loc_) {
            ctx.store<cia402::TargetPosition::type>(*target_loc_, cmd.target_counts);
        }
        if (pv_loc_) {
            ctx.store<cia402::ProfileVelocity::type>(*pv_loc_, cmd.profile_velocity);  // 0x6081 move speed (optional)
        }

        // Four-phase CiA402 new-set-point handshake (WriteTarget -> AwaitAck -> ClearBit4 ->
        // AwaitAckClear)
        switch (handshake_) {
            case Handshake::Idle:
                break;
            case Handshake::WriteTarget:
                if (halt_clear_cycles_ < kSetpointHaltSettleCycles) {
                    break;  // Halt not yet observed clear a full cycle: hold bit 4 low, stay in WriteTarget
                }
                base = ControlWord::with_new_setpoint(base, true);
                if (!bit4_high_) {
                    ++bit4_edges_;  // count the 0->1 edge once
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
                    state_.handshake_timed_out = true;  // ack never arrived; the wrapper aborts the move
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

    // Standard CiA402 object indices used by the policy.
    static constexpr std::uint16_t kQuickStopDecel = 0x6085;
    static constexpr std::uint16_t kQuickStopOption = 0x605A;
    // Quick-stop controlword: enable_operation() with bit 2 (quick-stop) cleared = 0x0B.
    static constexpr std::uint16_t kQuickStopCw = ControlWord::enable_operation() & ~std::uint16_t{0x0004};
    // Workaround for drives that drop a new-set-point (bit 4) rising edge unless Halt (bit 8)
    // has been clear on the wire for at least one full cycle -- on such drives a move issued
    // right after a stop would never be acknowledged. Require this many consecutive cycles with
    // the written controlword's Halt bit clear before raising bit 4 (2 is about 2 ms at 1 kHz).
    static constexpr std::uint32_t kSetpointHaltSettleCycles = 2;
    // Four-phase new-set-point ack (and ack-clear) timeout, in cycles. 100 ms at 1 kHz is
    // generous; drives typically ack within a few cycles.
    static constexpr std::uint32_t kHandshakeTimeoutCycles = 100;

    template <PdoScalar T>
    static std::vector<std::byte> sdo_bytes(T v) {
        std::vector<std::byte> b(sizeof(T));
        store_le<T>(b, v);
        return b;
    }

    // Four-phase PP new-set-point handshake sub-FSM.
    enum class Handshake : std::uint8_t { Idle, WriteTarget, AwaitAck, ClearBit4, AwaitAckClear };

    Cia402Fsm fsm_;
    PolicyState state_;
    Handshake handshake_ = Handshake::Idle;
    std::uint32_t handshake_cycles_remaining_ = 0;
    // Consecutive recent cycles the written controlword had Halt (bit 8) clear, capped at
    // kSetpointHaltSettleCycles. Starts settled so the first move isn't delayed; resets to 0 on
    // any written Halt. Gates the bit-4 raise so a halt-release and bit-4 edge never coincide on the wire.
    std::uint32_t halt_clear_cycles_ = kSetpointHaltSettleCycles;

    FieldLocation cw_loc_, sw_loc_;                                            // required fields (throwing resolve)
    std::optional<FieldLocation> target_loc_, pv_loc_, tv_loc_, mode_wr_loc_;  // optional RxPDO command fields
    std::optional<FieldLocation> fc_loc_, mode_loc_;                           // optional TxPDO feedback fields

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
