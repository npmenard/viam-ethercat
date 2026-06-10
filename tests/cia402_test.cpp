#include <cstdint>

#include "ethercat/cia402.hpp"
#include "test_harness.hpp"

using ethercat::Cia402Fsm;
using ethercat::Cia402Mode;
using ethercat::Cia402State;
using ethercat::ControlWord;
using ethercat::FsmError;
using ethercat::next_cw;
using ethercat::SetpointPhase;
using ethercat::Status;
using ethercat::sustain_cw;

namespace {

Status sw(std::uint16_t raw) {
    return Status{raw};
}

// Canonical statusword for a state (#38 §2 patterns), remote (bit9) set by default;
// `extra` adds bits (bit12 ack = 0x1000; pass extra=0 via sw_of_raw for bit9-clear).
constexpr std::uint16_t sw_bits(Cia402State s) {
    switch (s) {
        case Cia402State::NotReadyToSwitchOn:
            return 0x0000;
        case Cia402State::SwitchOnDisabled:
            return 0x0040;
        case Cia402State::ReadyToSwitchOn:
            return 0x0021;
        case Cia402State::SwitchedOn:
            return 0x0023;
        case Cia402State::OperationEnabled:
            return 0x0027;
        case Cia402State::QuickStopActive:
            return 0x0007;
        case Cia402State::FaultReactionActive:
            return 0x000F;
        case Cia402State::Fault:
            return 0x0008;
    }
    return 0x0008;
}
constexpr std::uint16_t sw_of(Cia402State s, std::uint16_t extra = 0) {
    return static_cast<std::uint16_t>(sw_bits(s) | 0x0200U | extra);  // bit9 remote set
}

// Minimal DS402-CONFORMANT drive model for the walking tests: applies at most ONE
// legal transition per controlword level (no transition-skipping -- matches the A6's
// documented sequencing). CRITICALLY honors T2-on-0x06 from SwitchOnDisabled (DS402 +
// bench-proven on the A6) -- the behavior the B1 HOLD-at-SOD regression test rides on.
// `qsa_holds` models 0x605A in 5..7 (drive STAYS in QuickStopActive -> T16/T12 apply);
// false = the A6 default (0x605A=2): auto-T12 out of QSA regardless of cw.
Cia402State drive_step(Cia402State s, std::uint16_t cw, bool qsa_holds = false) {
    const std::uint16_t base = cw & 0x008FU;  // bits0-3 + bit7; bit4/6/8 (handshake/halt) don't transition
    switch (s) {
        case Cia402State::NotReadyToSwitchOn:
            return Cia402State::SwitchOnDisabled;  // T1 auto
        case Cia402State::SwitchOnDisabled:
            return base == 0x06 ? Cia402State::ReadyToSwitchOn : s;  // T2 fires on 0x06 (the B1 hazard)
        case Cia402State::ReadyToSwitchOn:
            if (base == 0x07 || base == 0x0F) {
                return Cia402State::SwitchedOn;  // T3 (one step only; 0x0F's switch-on bits apply)
            }
            return (base & 0x02U) == 0 ? Cia402State::SwitchOnDisabled : s;  // T7 disable voltage
        case Cia402State::SwitchedOn:
            if (base == 0x0F) {
                return Cia402State::OperationEnabled;  // T4
            }
            if (base == 0x06) {
                return Cia402State::ReadyToSwitchOn;  // T6
            }
            return (base & 0x02U) == 0 ? Cia402State::SwitchOnDisabled : s;  // T10
        case Cia402State::OperationEnabled:
            if (base == 0x07) {
                return Cia402State::SwitchedOn;  // T5
            }
            if (base == 0x06) {
                return Cia402State::ReadyToSwitchOn;  // T8
            }
            if (base == 0x02) {
                return Cia402State::QuickStopActive;  // T11
            }
            return (base & 0x02U) == 0 ? Cia402State::SwitchOnDisabled : s;  // T9
        case Cia402State::QuickStopActive:
            if (!qsa_holds) {
                return Cia402State::SwitchOnDisabled;  // A6 default: auto-T12
            }
            if (base == 0x0F) {
                return Cia402State::OperationEnabled;  // T16 (605A in 5..7)
            }
            return (base & 0x02U) == 0 ? Cia402State::SwitchOnDisabled : s;  // T12 on disable voltage
        case Cia402State::FaultReactionActive:
            return Cia402State::Fault;  // T14 auto
        case Cia402State::Fault:
            return (cw & ControlWord::kFaultResetBit) != 0 ? Cia402State::SwitchOnDisabled : s;  // T15
    }
    return s;
}

}  // namespace

TEST("statusword decodes to each DS402 state (canonical bit patterns)") {
    CHECK_EQ(sw(0x0000).decode(), Cia402State::NotReadyToSwitchOn);
    CHECK_EQ(sw(0x0040).decode(), Cia402State::SwitchOnDisabled);
    CHECK_EQ(sw(0x0021).decode(), Cia402State::ReadyToSwitchOn);
    CHECK_EQ(sw(0x0023).decode(), Cia402State::SwitchedOn);
    CHECK_EQ(sw(0x0027).decode(), Cia402State::OperationEnabled);
    CHECK_EQ(sw(0x0007).decode(), Cia402State::QuickStopActive);
    CHECK_EQ(sw(0x000F).decode(), Cia402State::FaultReactionActive);
    CHECK_EQ(sw(0x0008).decode(), Cia402State::Fault);
}

TEST("decode ignores bits outside the state mask (voltage/warning/target/etc)") {
    // OperationEnabled (0x27) with voltage-enabled(4), warning(7), remote(9),
    // target-reached(10), setpoint-ack(12) all set must still decode as OE.
    CHECK_EQ(sw(0x1627).decode(), Cia402State::OperationEnabled);
    // SwitchOnDisabled with high noise bits set.
    CHECK_EQ(sw(0xFF40).decode(), Cia402State::SwitchOnDisabled);
    // Fault stays Fault even with warning + target-reached set.
    CHECK_EQ(sw(0x0488).decode(), Cia402State::Fault);
}

TEST("status bit helpers read the documented bits") {
    const Status s = sw(0x1E37);  // bits 0,1,2,4,5,9,10,11,12 set: 0x1E37
    CHECK(s.ready_to_switch_on());
    CHECK(s.switched_on());
    CHECK(s.operation_enabled());
    CHECK(!s.fault());
    CHECK(s.voltage_enabled());
    CHECK(s.quick_stop());
    CHECK(!s.switch_on_disabled());
    CHECK(!s.warning());
    CHECK(s.remote());
    CHECK(s.target_reached());         // bit10
    CHECK(s.internal_limit_active());  // bit11
    CHECK(s.setpoint_acknowledged());  // bit12
    CHECK(!s.following_error());       // bit13 clear
    // bit13 set:
    CHECK(sw(0x2000).following_error());
}

TEST("controlword encoders return the documented levels") {
    CHECK_EQ(ControlWord::shutdown(), std::uint16_t{0x0006});
    CHECK_EQ(ControlWord::switch_on(), std::uint16_t{0x0007});
    CHECK_EQ(ControlWord::enable_operation(), std::uint16_t{0x000F});
    CHECK_EQ(ControlWord::disable_voltage(), std::uint16_t{0x0000});
    CHECK_EQ(ControlWord::quick_stop(), std::uint16_t{0x0002});
    CHECK_EQ(ControlWord::fault_reset(), std::uint16_t{0x0080});
}

TEST("new-set-point bit4 handshake (0x0F -> 0x1F) and halt bit8") {
    CHECK_EQ(ControlWord::with_new_setpoint(ControlWord::enable_operation(), true), std::uint16_t{0x001F});
    CHECK_EQ(ControlWord::with_new_setpoint(ControlWord::enable_operation(), false), std::uint16_t{0x000F});
    // Idempotent clear.
    CHECK_EQ(ControlWord::with_new_setpoint(0x001F, false), std::uint16_t{0x000F});
    // Halt set/clear on the enable level.
    CHECK_EQ(ControlWord::with_halt(ControlWord::enable_operation(), true), std::uint16_t{0x010F});
    CHECK_EQ(ControlWord::with_halt(0x010F, false), std::uint16_t{0x000F});
}

TEST("enable ladder: one legal transition per cycle toward OperationEnabled") {
    const Cia402Fsm fsm;
    // SwitchOnDisabled -> shutdown -> ReadyToSwitchOn -> switch_on -> SwitchedOn
    // -> enable_operation -> OperationEnabled (then hold).
    CHECK_EQ(fsm.step(sw(0x0040), Cia402State::OperationEnabled), ControlWord::shutdown());
    CHECK_EQ(fsm.step(sw(0x0021), Cia402State::OperationEnabled), ControlWord::switch_on());
    CHECK_EQ(fsm.step(sw(0x0023), Cia402State::OperationEnabled), ControlWord::enable_operation());
    CHECK_EQ(fsm.step(sw(0x0027), Cia402State::OperationEnabled), ControlWord::enable_operation());
}

TEST("fault recovery: step returns fault-reset LEVEL while in Fault") {
    const Cia402Fsm fsm;
    CHECK_EQ(sw(0x0008).decode(), Cia402State::Fault);
    CHECK_EQ(fsm.step(sw(0x0008), Cia402State::OperationEnabled), ControlWord::fault_reset());
    // FaultReactionActive: hold with voltage disabled until it settles.
    CHECK_EQ(fsm.step(sw(0x000F), Cia402State::OperationEnabled), ControlWord::disable_voltage());
}

TEST("disable and quick-stop goals") {
    const Cia402Fsm fsm;
    // From OperationEnabled, asking for SwitchOnDisabled -> disable voltage (0x00).
    CHECK_EQ(fsm.step(sw(0x0027), Cia402State::SwitchOnDisabled), ControlWord::disable_voltage());
    // From OperationEnabled, asking for QuickStopActive -> quick_stop (0x02).
    CHECK_EQ(fsm.step(sw(0x0027), Cia402State::QuickStopActive), ControlWord::quick_stop());
    // From QuickStopActive, asking for OperationEnabled -> resume (0x0F).
    CHECK_EQ(fsm.step(sw(0x0007), Cia402State::OperationEnabled), ControlWord::enable_operation());
}

TEST("mode and state names round-trip to strings") {
    CHECK(std::string("OperationEnabled") == ethercat::to_string(Cia402State::OperationEnabled));
    CHECK(std::string("Fault") == ethercat::to_string(Cia402State::Fault));
    CHECK(std::string("ProfilePosition") == ethercat::to_string(Cia402Mode::ProfilePosition));
    CHECK(std::string("ProfileVelocity") == ethercat::to_string(Cia402Mode::ProfileVelocity));
}

// ===========================================================================
// #38 goal-walking FSM tests (spec §10, all 14 -- numbered to match the spec)
// ===========================================================================

// (1) Exhaustive routing enumeration: every §3 cell, value-for-value, plus the
// invalid-goal columns. Written as an explicit table so it diffs against §3 visually.
TEST("#38.1: next_cw matches the spec routing table cell-for-cell") {
    using S = Cia402State;
    struct Cell {
        S current;
        S goal;
        std::uint16_t cw;  // expected value for OK cells
    };
    // The 8x4 valid-goal grid (§3): WAIT cells resolve to the CURRENT state's sustain,
    // HOLD cells to the goal's sustain -- all deterministic values.
    constexpr Cell kTable[] = {
        // NotReadyToSwitchOn row: all WAIT (T1 auto) -> sustain(NRTSO)=0x00
        {S::NotReadyToSwitchOn, S::SwitchOnDisabled, 0x00},
        {S::NotReadyToSwitchOn, S::ReadyToSwitchOn, 0x00},
        {S::NotReadyToSwitchOn, S::SwitchedOn, 0x00},
        {S::NotReadyToSwitchOn, S::OperationEnabled, 0x00},
        // SwitchOnDisabled row: HOLD 0x00 (B1) / T2
        {S::SwitchOnDisabled, S::SwitchOnDisabled, 0x00},
        {S::SwitchOnDisabled, S::ReadyToSwitchOn, 0x06},
        {S::SwitchOnDisabled, S::SwitchedOn, 0x06},
        {S::SwitchOnDisabled, S::OperationEnabled, 0x06},
        // ReadyToSwitchOn row: T7 / HOLD / T3 / T3
        {S::ReadyToSwitchOn, S::SwitchOnDisabled, 0x00},
        {S::ReadyToSwitchOn, S::ReadyToSwitchOn, 0x06},
        {S::ReadyToSwitchOn, S::SwitchedOn, 0x07},
        {S::ReadyToSwitchOn, S::OperationEnabled, 0x07},
        // SwitchedOn row: T10 / T6 / HOLD / T4
        {S::SwitchedOn, S::SwitchOnDisabled, 0x00},
        {S::SwitchedOn, S::ReadyToSwitchOn, 0x06},
        {S::SwitchedOn, S::SwitchedOn, 0x07},
        {S::SwitchedOn, S::OperationEnabled, 0x0F},
        // OperationEnabled row: T9 / T8 / T5 / HOLD
        {S::OperationEnabled, S::SwitchOnDisabled, 0x00},
        {S::OperationEnabled, S::ReadyToSwitchOn, 0x06},
        {S::OperationEnabled, S::SwitchedOn, 0x07},
        {S::OperationEnabled, S::OperationEnabled, 0x0F},
        // QuickStopActive row: T12 / T12-override / T12-override / T16
        {S::QuickStopActive, S::SwitchOnDisabled, 0x00},
        {S::QuickStopActive, S::ReadyToSwitchOn, 0x00},
        {S::QuickStopActive, S::SwitchedOn, 0x00},
        {S::QuickStopActive, S::OperationEnabled, 0x0F},
        // FaultReactionActive row: WAIT (sustain 0x06) for the three; OE -> ERROR below
        {S::FaultReactionActive, S::SwitchOnDisabled, 0x06},
        {S::FaultReactionActive, S::ReadyToSwitchOn, 0x06},
        {S::FaultReactionActive, S::SwitchedOn, 0x06},
    };
    for (const Cell& c : kTable) {
        const auto r = next_cw(c.current, c.goal);
        CHECK(r.has_value());
        CHECK_EQ(*r, c.cw);
    }
    // ERROR cells: FRA x OE, and the whole Fault row.
    CHECK(!next_cw(S::FaultReactionActive, S::OperationEnabled).has_value());
    CHECK(next_cw(S::FaultReactionActive, S::OperationEnabled).error() == FsmError::FaultActive);
    for (const S g : {S::SwitchOnDisabled, S::ReadyToSwitchOn, S::SwitchedOn, S::OperationEnabled}) {
        CHECK(!next_cw(S::Fault, g).has_value());
        CHECK(next_cw(S::Fault, g).error() == FsmError::FaultActive);
    }
    // Invalid goals (QSA/Fault/NotReady/FRA) -> InvalidGoal from EVERY current state.
    for (const S cur : {S::NotReadyToSwitchOn,
                        S::SwitchOnDisabled,
                        S::ReadyToSwitchOn,
                        S::SwitchedOn,
                        S::OperationEnabled,
                        S::QuickStopActive,
                        S::FaultReactionActive,
                        S::Fault}) {
        for (const S g : {S::QuickStopActive, S::Fault, S::NotReadyToSwitchOn, S::FaultReactionActive}) {
            CHECK(!next_cw(cur, g).has_value());
            CHECK(next_cw(cur, g).error() == FsmError::InvalidGoal);
        }
    }
}

// (2) Multi-hop walk SOD->OE in exactly 3 cw emissions; stalled sw holds the cw and
// grows cycles_in_state.
TEST("#38.2: SOD->OE walks 0x06,0x07,0x0F in 3 updates; a stalled drive holds the cw") {
    Cia402Fsm f;
    Cia402State drive = Cia402State::SwitchOnDisabled;
    f.update(sw_of(drive));  // adopts SOD
    CHECK(f.enable().has_value());
    const std::uint16_t expect[] = {0x06, 0x07, 0x0F};
    for (const std::uint16_t e : expect) {
        CHECK_EQ(f.get_cw(), e);
        drive = drive_step(drive, f.get_cw());
        f.update(sw_of(drive));
    }
    CHECK_EQ(f.state(), Cia402State::OperationEnabled);
    CHECK_EQ(f.get_cw(), std::uint16_t{0x0F});  // HOLD at goal
    // Stalled drive: re-walk from SOD but freeze the sw at ReadyToSwitchOn.
    Cia402Fsm g;
    g.update(sw_of(Cia402State::ReadyToSwitchOn));
    CHECK(g.enable().has_value());
    for (std::uint32_t i = 2; i <= 5; ++i) {
        g.update(sw_of(Cia402State::ReadyToSwitchOn));
        CHECK_EQ(g.get_cw(), std::uint16_t{0x07});  // same next-hop, never escalates
        CHECK_EQ(g.cycles_in_state(), i);
    }
}

// (3) Re-planning: a drive auto-transition (QSA auto-T12 -> SOD) re-routes next cycle;
// the goal is preserved (non-fault case).
TEST("#38.3: re-planning absorbs a drive auto-transition; goal preserved") {
    Cia402Fsm f;
    f.update(sw_of(Cia402State::QuickStopActive));  // first update: QSA isn't a goal -> hold-current
    CHECK(f.set_state(Cia402State::OperationEnabled).has_value());
    CHECK_EQ(f.get_cw(), std::uint16_t{0x0F});  // T16 attempt from QSA
    // The A6-default drive auto-T12s to SOD underneath us:
    f.update(sw_of(Cia402State::SwitchOnDisabled));
    CHECK_EQ(f.get_cw(), std::uint16_t{0x06});  // re-routed fresh from SOD; goal intact
    f.update(sw_of(Cia402State::ReadyToSwitchOn));
    CHECK_EQ(f.get_cw(), std::uint16_t{0x07});  // still walking to OE
}

// (4) bit-7 edge: exactly one update-cycle of clean 0x80, then base with bit7=0 (the
// falling edge); no-op without a fault; no auto-route after the clear.
TEST("#38.4: fault.reset() pulses 0x80 for exactly one cycle; no-op sans fault; no auto-route") {
    Cia402Fsm f;
    f.update(sw_of(Cia402State::Fault));
    CHECK(f.fault.active());
    CHECK_EQ(f.get_cw(), std::uint16_t{0x06});  // fault sustain, inert
    f.fault.reset();
    CHECK_EQ(f.get_cw(), std::uint16_t{0x80});  // the clean one-cycle pulse (#18 form)
    f.update(sw_of(Cia402State::Fault));        // drive hasn't cleared yet
    CHECK_EQ(f.get_cw(), std::uint16_t{0x06});  // bit7 FALLING edge -- re-arms the drive's detector
    CHECK(f.fault.active());                    // reset ineffective so far -- honest surface
    // Drive clears to SOD (T15): NO auto-route -- the goal stays cancelled.
    f.update(sw_of(Cia402State::SwitchOnDisabled));
    CHECK(!f.fault.active());
    CHECK_EQ(f.get_cw(), std::uint16_t{0x00});  // hold-current (B1-inert), not walking anywhere
    // reset() without a fault is a no-op.
    f.fault.reset();
    CHECK_EQ(f.get_cw(), std::uint16_t{0x00});
}

// (5) Fault cancels the goal; post-T15 the FSM holds; a fresh enable() walks again.
TEST("#38.5: a fault cancels the active goal; recovery requires a fresh enable()") {
    Cia402Fsm f;
    f.update(sw_of(Cia402State::ReadyToSwitchOn));
    CHECK(f.enable().has_value());
    CHECK_EQ(f.get_cw(), std::uint16_t{0x07});  // walking
    f.update(sw_of(Cia402State::FaultReactionActive));
    CHECK_EQ(f.get_cw(), std::uint16_t{0x06});                       // WAIT through the auto fault reaction
    CHECK(!f.set_state(Cia402State::OperationEnabled).has_value());  // refused while faulted
    CHECK(f.set_state(Cia402State::OperationEnabled).error() == FsmError::FaultActive);
    f.update(sw_of(Cia402State::Fault));
    f.fault.reset();
    f.update(sw_of(Cia402State::SwitchOnDisabled));  // T15 landed
    CHECK_EQ(f.get_cw(), std::uint16_t{0x00});       // holds; goal was cancelled
    CHECK(f.enable().has_value());                   // fresh, deliberate re-energize decision
    CHECK_EQ(f.get_cw(), std::uint16_t{0x06});       // walks again
}

// (6) Full 4-phase bit-4 handshake + every set_point error + observability.
TEST("#38.6: PP set_point 4-phase handshake, errors, and caller observability") {
    Cia402Fsm f;
    f.set_mode(Cia402Mode::ProfilePosition);
    // NotOperational outside OE:
    f.update(sw_of(Cia402State::SwitchedOn));
    CHECK(f.set_point().error() == FsmError::NotOperational);
    // Reach OE (and make OE the goal -- the first update above ADOPTED SwitchedOn per
    // D3, which would otherwise correctly walk DOWN, composing bit4 over 0x07):
    f.update(sw_of(Cia402State::OperationEnabled));
    CHECK(f.enable().has_value());
    CHECK_EQ(f.setpoint_phase(), SetpointPhase::Idle);
    CHECK(f.set_point().has_value());
    CHECK_EQ(f.setpoint_phase(), SetpointPhase::Arm);
    CHECK_EQ(f.get_cw(), std::uint16_t{0x1F});       // 0x0F | bit4
    CHECK(f.set_point().error() == FsmError::Busy);  // double set_point
    f.update(sw_of(Cia402State::OperationEnabled));  // no ack yet
    CHECK_EQ(f.setpoint_phase(), SetpointPhase::AwaitAck);
    CHECK_EQ(f.get_cw(), std::uint16_t{0x1F});  // bit4 held
    // Caller observability: phase parks in AwaitAck while bit12 never arrives.
    for (int i = 0; i < 3; ++i) {
        f.update(sw_of(Cia402State::OperationEnabled));
        CHECK_EQ(f.setpoint_phase(), SetpointPhase::AwaitAck);
    }
    f.update(sw_of(Cia402State::OperationEnabled, 0x1000));  // bit12 ack
    CHECK_EQ(f.setpoint_phase(), SetpointPhase::Drop);
    CHECK_EQ(f.get_cw(), std::uint16_t{0x0F});               // bit4 dropped
    f.update(sw_of(Cia402State::OperationEnabled, 0x1000));  // ack still high
    CHECK_EQ(f.setpoint_phase(), SetpointPhase::AwaitAckClear);
    f.update(sw_of(Cia402State::OperationEnabled));  // ack cleared
    CHECK_EQ(f.setpoint_phase(), SetpointPhase::Idle);
    // Relative ride-along (bit6):
    CHECK(f.set_point(/*relative=*/true).has_value());
    CHECK_EQ(f.get_cw(), std::uint16_t{0x5F});  // 0x0F | bit4 | bit6
    // WrongMode outside PP:
    Cia402Fsm pv;
    pv.set_mode(Cia402Mode::ProfileVelocity);
    pv.update(sw_of(Cia402State::OperationEnabled));
    CHECK(pv.set_point().error() == FsmError::WrongMode);
    CHECK_EQ(pv.get_cw(), std::uint16_t{0x0F});  // PV cw never carries bit4/bit6
}

// (7) quick_stop: 0x02 override in OE; goal cancelled; intent consumed on leaving OE;
// the A6-default landing (SOD) does NOT resume; outside OE it is a no-op.
TEST("#38.7: quick_stop overrides in OE, cancels the goal, never lingers or resumes") {
    Cia402Fsm f;
    Cia402State drive = Cia402State::SwitchOnDisabled;
    f.update(sw_of(drive));
    CHECK(f.enable().has_value());
    for (int i = 0; i < 3; ++i) {  // walk to OE
        drive = drive_step(drive, f.get_cw());
        f.update(sw_of(drive));
    }
    CHECK(f.operation_enabled());
    f.quick_stop();
    CHECK_EQ(f.get_cw(), std::uint16_t{0x02});  // override (suppresses everything else)
    drive = drive_step(drive, f.get_cw());      // -> QSA
    f.update(sw_of(drive));                     // leaving OE consumes the intent
    drive = drive_step(drive, f.get_cw());      // A6 default: auto-T12 -> SOD
    f.update(sw_of(drive));
    CHECK_EQ(f.state(), Cia402State::SwitchOnDisabled);
    for (int i = 0; i < 5; ++i) {  // goal was cancelled -> hold SOD inert, NO resume
        CHECK_EQ(f.get_cw(), std::uint16_t{0x00});
        drive = drive_step(drive, f.get_cw());
        f.update(sw_of(drive));
        CHECK_EQ(f.state(), Cia402State::SwitchOnDisabled);
    }
    // Outside OE: no-op -- a held goal is NOT cancelled, nothing lingers armed.
    Cia402Fsm g;
    g.update(sw_of(Cia402State::SwitchedOn));
    CHECK(g.enable().has_value());
    g.quick_stop();                             // state != OE -> no-op
    CHECK_EQ(g.get_cw(), std::uint16_t{0x0F});  // still walking to OE (T4), no 0x02
}

// (8) T16 + the T12-override on a HOLDING drive (605A in 5..7 simulated).
TEST("#38.8: holding-QSA drive: T16 resumes to OE; goal=RTSO routes T12 and re-walks") {
    // (a) goal=OE re-issued from held QSA -> 0x0F (T16) works where the drive supports it.
    Cia402Fsm f;
    Cia402State drive = Cia402State::SwitchOnDisabled;
    f.update(sw_of(drive));
    CHECK(f.enable().has_value());
    for (int i = 0; i < 3; ++i) {
        drive = drive_step(drive, f.get_cw(), /*qsa_holds=*/true);
        f.update(sw_of(drive));
    }
    CHECK(f.operation_enabled());
    f.quick_stop();
    drive = drive_step(drive, f.get_cw(), true);  // -> QSA, and it HOLDS
    f.update(sw_of(drive));
    CHECK_EQ(f.state(), Cia402State::QuickStopActive);
    CHECK(f.set_state(Cia402State::OperationEnabled).has_value());  // fresh override post-stop
    CHECK_EQ(f.get_cw(), std::uint16_t{0x0F});                      // T16
    drive = drive_step(drive, f.get_cw(), true);
    f.update(sw_of(drive));
    CHECK(f.operation_enabled());
    // (b) goal=RTSO from held QSA -> 0x00 (T12) -> SOD -> re-walk to RTSO (the override
    // principle: WAIT would emit 0x02 and sustain the stop forever).
    f.quick_stop();
    drive = drive_step(drive, f.get_cw(), true);  // -> QSA (holds)
    f.update(sw_of(drive));
    CHECK(f.set_state(Cia402State::ReadyToSwitchOn).has_value());
    CHECK_EQ(f.get_cw(), std::uint16_t{0x00});  // T12 routed, not WAIT
    drive = drive_step(drive, f.get_cw(), true);
    f.update(sw_of(drive));
    CHECK_EQ(f.state(), Cia402State::SwitchOnDisabled);
    CHECK_EQ(f.get_cw(), std::uint16_t{0x06});  // re-walking
    drive = drive_step(drive, f.get_cw(), true);
    f.update(sw_of(drive));
    CHECK_EQ(f.state(), Cia402State::ReadyToSwitchOn);
    CHECK_EQ(f.get_cw(), std::uint16_t{0x06});  // HOLD at the goal
}

// (9) remote() gate: bit9=0 -> the cw is being ignored; state frozen; cycles grow.
TEST("#38.9: remote()==false surfaces an ignoring drive; cw stays stable") {
    Cia402Fsm f;
    const std::uint16_t local_sw = sw_bits(Cia402State::ReadyToSwitchOn);  // bit9 CLEAR
    f.update(local_sw);
    CHECK(!f.remote());
    CHECK(f.enable().has_value());
    for (std::uint32_t i = 2; i <= 5; ++i) {
        f.update(local_sw);
        CHECK(!f.remote());
        CHECK_EQ(f.get_cw(), std::uint16_t{0x07});  // same composed cw (harmless; ignored)
        CHECK_EQ(f.cycles_in_state(), i);           // the caller's timeout primitive grows
    }
    CHECK_EQ(f.state(), Cia402State::ReadyToSwitchOn);  // frozen
}

// (10) Decode hygiene: bit10/bit14 randomized -> identical decode + behavior.
TEST("#38.10: bit10/bit14 noise never changes decode or the emitted cw") {
    for (const Cia402State s : {Cia402State::SwitchOnDisabled,
                                Cia402State::ReadyToSwitchOn,
                                Cia402State::SwitchedOn,
                                Cia402State::OperationEnabled,
                                Cia402State::QuickStopActive,
                                Cia402State::Fault}) {
        Cia402Fsm clean;
        Cia402Fsm noisy;
        clean.update(sw_of(s));
        noisy.update(sw_of(s, 0x4400));  // bit10 | bit14 set
        CHECK_EQ(clean.state(), noisy.state());
        CHECK_EQ(clean.get_cw(), noisy.get_cw());
        (void)clean.set_state(Cia402State::OperationEnabled);
        (void)noisy.set_state(Cia402State::OperationEnabled);
        CHECK_EQ(clean.get_cw(), noisy.get_cw());
    }
}

// (11) get_cw idempotence: N calls between updates return the identical value.
TEST("#38.11: get_cw is idempotent within a cycle") {
    Cia402Fsm f;
    f.update(sw_of(Cia402State::SwitchedOn));
    CHECK(f.enable().has_value());
    const std::uint16_t first = f.get_cw();
    for (int i = 0; i < 10; ++i) {
        CHECK_EQ(f.get_cw(), first);
    }
}

// (12) B1 regression: against a drive that HONORS T2-on-0x06 (DS402-conformant + the
// A6 per bench), goal=SOD must STAY SOD -- a 0x06 hold would ping-pong SOD<->RTSO.
// NON-VACUITY validated during dev: flipping sustain_cw(SwitchOnDisabled) to 0x06
// makes this test FAIL (drive_step fires T2) -- see the commit message.
TEST("#38.12: HOLD-at-SOD emits 0x00 and stays put on a T2-honoring drive (B1)") {
    Cia402Fsm f;
    Cia402State drive = Cia402State::SwitchOnDisabled;
    f.update(sw_of(drive));                                         // adopts SOD as the goal
    CHECK(f.set_state(Cia402State::SwitchOnDisabled).has_value());  // explicit too
    for (int i = 0; i < 10; ++i) {
        CHECK_EQ(f.get_cw(), std::uint16_t{0x00});  // B1: the inert hold
        drive = drive_step(drive, f.get_cw());      // honors T2-on-0x06
        CHECK_EQ(drive, Cia402State::SwitchOnDisabled);
        f.update(sw_of(drive));
    }
}

// (13) B2 handshake abort: a fault mid-AwaitAck resets the phase; after recovery a
// fresh set_point succeeds (no Busy-forever).
TEST("#38.13: fault mid-AwaitAck aborts the handshake; set_point works after recovery") {
    Cia402Fsm f;
    f.set_mode(Cia402Mode::ProfilePosition);
    f.update(sw_of(Cia402State::OperationEnabled));
    CHECK(f.set_point().has_value());
    f.update(sw_of(Cia402State::OperationEnabled));  // Arm -> AwaitAck
    CHECK_EQ(f.setpoint_phase(), SetpointPhase::AwaitAck);
    f.update(sw_of(Cia402State::Fault));                // fault mid-handshake
    CHECK_EQ(f.setpoint_phase(), SetpointPhase::Idle);  // B2 abort -- no wedge
    f.fault.reset();
    f.update(sw_of(Cia402State::SwitchOnDisabled));  // cleared
    CHECK(f.enable().has_value());
    f.update(sw_of(Cia402State::ReadyToSwitchOn));
    f.update(sw_of(Cia402State::SwitchedOn));
    f.update(sw_of(Cia402State::OperationEnabled));
    CHECK(f.set_point().has_value());  // NOT Busy -- full recovery
    CHECK_EQ(f.setpoint_phase(), SetpointPhase::Arm);
}

// (14) D3 adoption: the first update adopts the current state (sustain emitted; OE
// notably SUSTAINS 0x0F -- an FSM swap-in must not de-energize); no transition is
// commanded until a set_state arrives.
TEST("#38.14: first update adopts the current state; OE keeps 0x0F; nothing walks") {
    for (const Cia402State s : {Cia402State::NotReadyToSwitchOn,
                                Cia402State::SwitchOnDisabled,
                                Cia402State::ReadyToSwitchOn,
                                Cia402State::SwitchedOn,
                                Cia402State::OperationEnabled,
                                Cia402State::QuickStopActive,
                                Cia402State::FaultReactionActive,
                                Cia402State::Fault}) {
        Cia402Fsm f;
        f.update(sw_of(s));
        CHECK_EQ(f.state(), s);
        CHECK_EQ(f.get_cw(), sustain_cw(s));  // hold-current; no transition commanded
        CHECK_EQ(f.cycles_in_state(), std::uint32_t{1});
    }
    // The load-bearing case spelled out: constructed while the drive is already
    // OperationEnabled -> 0x0F is SUSTAINED (energization maintained, not initiated).
    Cia402Fsm oe;
    oe.update(sw_of(Cia402State::OperationEnabled));
    CHECK_EQ(oe.get_cw(), std::uint16_t{0x0F});
    // And the adopted goal is real: a drive sag to SwitchedOn re-walks back to OE
    // (the pre-existing operational intent is preserved across the swap-in).
    oe.update(sw_of(Cia402State::SwitchedOn));
    CHECK_EQ(oe.get_cw(), std::uint16_t{0x0F});  // T4 back toward the adopted OE goal
}

TEST_MAIN()
