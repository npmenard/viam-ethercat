#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

#include "ethercat/cia402.hpp"
#include "ethercat/errors.hpp"
#include "ethercat/pdo_buffer.hpp"
#include "ethercat/sim_backend.hpp"
#include "test_harness.hpp"

using ethercat::Cia402Mode;
using ethercat::Cia402State;
using ethercat::ControlWord;
using ethercat::EcatState;
using ethercat::SimBackend;
using ethercat::SimSlaveModel;
using ethercat::Status;

namespace {

// A minimal A6-ish image: RxPDO = ctrl(0x6040)@0 + target(0x607A)@2 (6 bytes);
// TxPDO = status(0x6041)@0 + actual(0x6064)@2 (6 bytes).
SimSlaveModel a6_like_model() {
    SimSlaveModel m;
    m.output_bytes = 6;
    m.input_bytes = 6;
    m.ctrlword_off = 0;
    m.target_off = 2;
    m.statusword_off = 0;
    m.actual_off = 2;
    m.mode = Cia402Mode::ProfilePosition;
    m.counts_per_step = 1000;
    return m;
}

// Bring a freshly-constructed backend to OP (SimBackend is non-movable -- it
// inherits EcatBackend's deleted move -- so configure it in place by reference).
void bring_up_to_op(SimBackend& be) {
    be.scan("sim0");
    be.map_process_image();
    be.request_state(0, EcatState::Op);
}

void write_ctrl(SimBackend& be, std::uint16_t cw) {
    ethercat::store_le<std::uint16_t>(be.outputs(1).subspan(0, 2), cw);
}

Status read_status(SimBackend& be) {
    return Status{ethercat::load_le<std::uint16_t>(be.inputs(1).subspan(0, 2))};
}

}  // namespace

TEST("SimBackend: device walks the DS402 enable ladder; bit10 always 1") {
    SimBackend be{std::vector<SimSlaveModel>{a6_like_model()}};
    bring_up_to_op(be);

    // First exchange auto-advances NotReady -> SwitchOnDisabled.
    write_ctrl(be, ControlWord::disable_voltage());
    be.send_receive();
    CHECK_EQ(read_status(be).decode(), Cia402State::SwitchOnDisabled);

    // Shutdown (0x06) -> ReadyToSwitchOn.
    write_ctrl(be, ControlWord::shutdown());
    be.send_receive();
    CHECK_EQ(read_status(be).decode(), Cia402State::ReadyToSwitchOn);

    // Switch on (0x07) -> SwitchedOn.
    write_ctrl(be, ControlWord::switch_on());
    be.send_receive();
    CHECK_EQ(read_status(be).decode(), Cia402State::SwitchedOn);

    // Enable operation (0x0F) -> OperationEnabled.
    write_ctrl(be, ControlWord::enable_operation());
    be.send_receive();
    CHECK_EQ(read_status(be).decode(), Cia402State::OperationEnabled);

    // The A6 quirk: bit10 must be set in every statusword observed above.
    CHECK(read_status(be).target_reached());  // bit10 held high
}

TEST("SimBackend: PP set-point-acknowledge bit12 handshake") {
    SimBackend be{std::vector<SimSlaveModel>{a6_like_model()}};
    bring_up_to_op(be);
    // Drive to OperationEnabled.
    for (std::uint16_t cw :
         {ControlWord::disable_voltage(), ControlWord::shutdown(), ControlWord::switch_on(), ControlWord::enable_operation()}) {
        write_ctrl(be, cw);
        be.send_receive();
    }
    CHECK_EQ(read_status(be).decode(), Cia402State::OperationEnabled);
    CHECK(!read_status(be).setpoint_acknowledged());

    // Set a target, then raise bit4 (new set-point) -> ack (bit12) within a cycle.
    ethercat::store_le<std::int32_t>(be.outputs(1).subspan(2, 4), 50000);
    write_ctrl(be, ControlWord::with_new_setpoint(ControlWord::enable_operation(), true));  // 0x1F
    be.send_receive();
    CHECK(read_status(be).setpoint_acknowledged());  // bit12

    // Drop bit4 -> ack clears.
    write_ctrl(be, ControlWord::enable_operation());  // 0x0F
    be.send_receive();
    CHECK(!read_status(be).setpoint_acknowledged());
}

TEST("SimBackend: PP actual position chases the latched target") {
    SimBackend be{std::vector<SimSlaveModel>{a6_like_model()}};
    bring_up_to_op(be);
    for (std::uint16_t cw :
         {ControlWord::disable_voltage(), ControlWord::shutdown(), ControlWord::switch_on(), ControlWord::enable_operation()}) {
        write_ctrl(be, cw);
        be.send_receive();
    }
    // Target = 2500 counts; counts_per_step = 1000 -> reached in 3 cycles.
    ethercat::store_le<std::int32_t>(be.outputs(1).subspan(2, 4), 2500);
    write_ctrl(be, ControlWord::with_new_setpoint(ControlWord::enable_operation(), true));
    for (int i = 0; i < 5; ++i) {
        be.send_receive();
    }
    const std::int32_t actual = ethercat::load_le<std::int32_t>(be.inputs(1).subspan(2, 4));
    CHECK_EQ(actual, std::int32_t{2500});
}

TEST("SimBackend: fault inject decodes Fault; fault-reset edge recovers") {
    SimBackend be{std::vector<SimSlaveModel>{a6_like_model()}};
    bring_up_to_op(be);
    write_ctrl(be, ControlWord::disable_voltage());
    be.send_receive();
    CHECK_EQ(read_status(be).decode(), Cia402State::SwitchOnDisabled);

    be.inject_fault(1);
    be.send_receive();
    CHECK_EQ(read_status(be).decode(), Cia402State::Fault);

    // Fault reset requires a RISING edge on bit7: clear, then set.
    write_ctrl(be, 0x0000);
    be.send_receive();
    CHECK_EQ(read_status(be).decode(), Cia402State::Fault);  // still faulted, no edge yet
    write_ctrl(be, ControlWord::fault_reset());              // 0x80 rising
    be.send_receive();
    CHECK_EQ(read_status(be).decode(), Cia402State::SwitchOnDisabled);
}

TEST("SimBackend: double scan throws; short-WKC hook fires once") {
    SimBackend be{std::vector<SimSlaveModel>{a6_like_model()}};
    CHECK_EQ(be.scan("sim0"), std::size_t{1});
    CHECK_THROWS(be.scan("sim0"), ethercat::ConfigError);

    be.map_process_image();
    be.request_state(0, EcatState::Op);
    const int expected = be.expected_wkc();
    CHECK(expected > 0);
    be.force_short_wkc_once();
    CHECK_EQ(be.send_receive(), expected - 1);  // short once
    CHECK_EQ(be.send_receive(), expected);      // back to normal
}

TEST_MAIN()
