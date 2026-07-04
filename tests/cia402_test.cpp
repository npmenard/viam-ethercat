#include <cstdint>

#include "ethercat/cia402.hpp"
#include "test_harness.hpp"

using ethercat::Cia402Fsm;
using ethercat::Cia402Mode;
using ethercat::Cia402State;
using ethercat::ControlWord;
using ethercat::Status;

namespace {

Status sw(std::uint16_t raw) {
    return Status{raw};
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

TEST_MAIN()
