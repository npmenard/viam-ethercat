#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

#include "ethercat/cia402.hpp"
#include "ethercat/errors.hpp"
#include "ethercat/pdo_buffer.hpp"
#include "ethercat/pdo_mapping.hpp"
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
    m.target_reached_always_set = true;  // the A6 bit10 quirk -- model DATA, not sim code (#43)
    return m;
}

// Bring a freshly-constructed backend to OP (SimBackend is non-movable -- it
// inherits EcatBackend's deleted move -- so configure it in place by reference).
void bring_up_to_op(SimBackend& be) {
    be.open("sim0");
    be.map_process_data();
    be.request_state(0, EcatState::Op);
}

void write_ctrl(SimBackend& be, std::uint16_t cw) {
    ethercat::store_le<std::uint16_t>(be.slave_io(1).outputs.subspan(0, 2), cw);
}

Status read_status(SimBackend& be) {
    return Status{ethercat::load_le<std::uint16_t>(be.slave_io(1).inputs.subspan(0, 2))};
}

// Set modes-of-operation 0x6060 (U8) via SDO -- the master does this in configure();
// the device only moves once it's set (de-masked from model.mode).
void set_mode(SimBackend& be, Cia402Mode mode) {
    const std::array<std::byte, 1> data{static_cast<std::byte>(static_cast<std::uint8_t>(mode))};
    be.sdo_write(1, 0x6060, 0, data);
}

}  // namespace

TEST("SimBackend: device walks the DS402 enable ladder; bit10 always 1") {
    SimBackend be{std::vector<SimSlaveModel>{a6_like_model()}};
    bring_up_to_op(be);

    // First exchange auto-advances NotReady -> SwitchOnDisabled.
    write_ctrl(be, ControlWord::disable_voltage());
    be.exchange();
    CHECK_EQ(read_status(be).decode(), Cia402State::SwitchOnDisabled);

    // Shutdown (0x06) -> ReadyToSwitchOn.
    write_ctrl(be, ControlWord::shutdown());
    be.exchange();
    CHECK_EQ(read_status(be).decode(), Cia402State::ReadyToSwitchOn);

    // Switch on (0x07) -> SwitchedOn.
    write_ctrl(be, ControlWord::switch_on());
    be.exchange();
    CHECK_EQ(read_status(be).decode(), Cia402State::SwitchedOn);

    // Enable operation (0x0F) -> OperationEnabled.
    write_ctrl(be, ControlWord::enable_operation());
    be.exchange();
    CHECK_EQ(read_status(be).decode(), Cia402State::OperationEnabled);

    // The A6 quirk: bit10 must be set in every statusword observed above.
    CHECK(read_status(be).target_reached());  // bit10 held high
}

TEST("SimBackend: PP set-point-acknowledge bit12 handshake") {
    SimBackend be{std::vector<SimSlaveModel>{a6_like_model()}};
    bring_up_to_op(be);
    set_mode(be, Cia402Mode::ProfilePosition);  // 0x6060 = PP (else no handshake)
    // Drive to OperationEnabled.
    for (std::uint16_t cw :
         {ControlWord::disable_voltage(), ControlWord::shutdown(), ControlWord::switch_on(), ControlWord::enable_operation()}) {
        write_ctrl(be, cw);
        be.exchange();
    }
    CHECK_EQ(read_status(be).decode(), Cia402State::OperationEnabled);
    CHECK(!read_status(be).setpoint_acknowledged());

    // Set a target, then raise bit4 (new set-point) -> ack (bit12) within a cycle.
    ethercat::store_le<std::int32_t>(be.slave_io(1).outputs.subspan(2, 4), 50000);
    write_ctrl(be, ControlWord::with_new_setpoint(ControlWord::enable_operation(), true));  // 0x1F
    be.exchange();
    CHECK(read_status(be).setpoint_acknowledged());  // bit12

    // Drop bit4 -> ack clears.
    write_ctrl(be, ControlWord::enable_operation());  // 0x0F
    be.exchange();
    CHECK(!read_status(be).setpoint_acknowledged());
}

TEST("SimBackend: PP actual position chases the latched target") {
    SimBackend be{std::vector<SimSlaveModel>{a6_like_model()}};
    bring_up_to_op(be);
    set_mode(be, Cia402Mode::ProfilePosition);  // 0x6060 = PP (else no motion)
    for (std::uint16_t cw :
         {ControlWord::disable_voltage(), ControlWord::shutdown(), ControlWord::switch_on(), ControlWord::enable_operation()}) {
        write_ctrl(be, cw);
        be.exchange();
    }
    // Target = 2500 counts; counts_per_step = 1000 -> reached in 3 cycles.
    ethercat::store_le<std::int32_t>(be.slave_io(1).outputs.subspan(2, 4), 2500);
    write_ctrl(be, ControlWord::with_new_setpoint(ControlWord::enable_operation(), true));
    for (int i = 0; i < 5; ++i) {
        be.exchange();
    }
    const std::int32_t actual = ethercat::load_le<std::int32_t>(be.slave_io(1).inputs.subspan(2, 4));
    CHECK_EQ(actual, std::int32_t{2500});
}

TEST("SimBackend: mode-0 guard -- no 0x6060 write means no motion even when enabled") {
    // The de-mask: without the master's 0x6060 SDO the device stays in mode 0, so a
    // real drive (and now the sim) won't move. This is the bug that was previously
    // masked by SimBackend reading model.mode directly.
    SimBackend be{std::vector<SimSlaveModel>{a6_like_model()}};
    bring_up_to_op(be);
    // NOTE: deliberately NO set_mode() here.
    for (std::uint16_t cw :
         {ControlWord::disable_voltage(), ControlWord::shutdown(), ControlWord::switch_on(), ControlWord::enable_operation()}) {
        write_ctrl(be, cw);
        be.exchange();
    }
    CHECK_EQ(read_status(be).decode(), Cia402State::OperationEnabled);  // enabled...
    ethercat::store_le<std::int32_t>(be.slave_io(1).outputs.subspan(2, 4), 2500);
    write_ctrl(be, ControlWord::with_new_setpoint(ControlWord::enable_operation(), true));
    for (int i = 0; i < 5; ++i) {
        be.exchange();
    }
    // ...but mode 0 => actual NEVER advances (and the PP handshake never acks).
    CHECK_EQ(ethercat::load_le<std::int32_t>(be.slave_io(1).inputs.subspan(2, 4)), std::int32_t{0});
    CHECK(!read_status(be).setpoint_acknowledged());
}

TEST("SimBackend: fault inject decodes Fault; fault-reset edge recovers") {
    SimBackend be{std::vector<SimSlaveModel>{a6_like_model()}};
    bring_up_to_op(be);
    write_ctrl(be, ControlWord::disable_voltage());
    be.exchange();
    CHECK_EQ(read_status(be).decode(), Cia402State::SwitchOnDisabled);

    be.inject_fault(1);
    be.exchange();
    CHECK_EQ(read_status(be).decode(), Cia402State::Fault);

    // Fault reset requires a RISING edge on bit7: clear, then set.
    write_ctrl(be, 0x0000);
    be.exchange();
    CHECK_EQ(read_status(be).decode(), Cia402State::Fault);  // still faulted, no edge yet
    write_ctrl(be, ControlWord::fault_reset());              // 0x80 rising
    be.exchange();
    CHECK_EQ(read_status(be).decode(), Cia402State::SwitchOnDisabled);
}

TEST("SimBackend: double scan throws; short-WKC hook fires once") {
    SimBackend be{std::vector<SimSlaveModel>{a6_like_model()}};
    CHECK_EQ(be.open("sim0"), std::size_t{1});
    CHECK_THROWS(be.open("sim0"), ethercat::ConfigError);

    be.map_process_data();
    be.request_state(0, EcatState::Op);
    const int expected = be.expected_wkc();
    CHECK(expected > 0);
    be.force_short_wkc_once();
    CHECK_EQ(be.exchange(), expected - 1);  // short once
    CHECK_EQ(be.exchange(), expected);      // back to normal
}

// --- #32 hygiene fixes -------------------------------------------------------

// #32.1: backend sdo_read/sdo_write bounds-check an out-of-range slave with a
// clear-text ConfigError (a precondition/programming error -- the SAME tier
// slave_info throws, NOT a BusError/WKC fault) naming the configured count.
TEST("#32.1: sdo bounds-check throws ConfigError naming the configured count") {
    SimBackend be(std::vector<SimSlaveModel>{a6_like_model()});  // exactly 1 slave
    (void)be.open("sim0");
    std::array<std::byte, 2> buf{};
    CHECK_THROWS_MSG(be.sdo_read(99, 0x6041, 0, buf), ethercat::ConfigError, "configured 1");
    CHECK_THROWS_MSG(be.sdo_write(99, 0x6040, 0, buf), ethercat::ConfigError, "configured 1");
    CHECK_THROWS_MSG(be.sdo_read(0, 0x6041, 0, buf), ethercat::ConfigError, "out of range");  // 0 is not a 1-based id
    // In-range still works (no throw): write then read back the same object.
    const std::array<std::byte, 2> val{std::byte{0x34}, std::byte{0x12}};
    be.sdo_write(1, 0x6040, 0, val);
    CHECK_EQ(be.sdo_read(1, 0x6040, 0, buf), std::size_t{2});
}

// #32.2: a GENERIC (non-mapping) SDO abort surfaces as SdoError; the SAME abort
// on a MAPPING object, routed through apply_pdo_map, surfaces as PdoMappingError.
TEST("#32.2: generic SDO abort -> SdoError; mapping-object abort -> PdoMappingError") {
    SimBackend be(std::vector<SimSlaveModel>{a6_like_model()});
    (void)be.open("sim0");

    // (a) a non-mapping object (mode 0x6060) aborts -> SdoError (NOT PdoMappingError).
    be.set_sdo_write_abort(1, 0x6060, 0);
    const std::array<std::byte, 1> mode{std::byte{8}};
    CHECK_THROWS(be.sdo_write(1, 0x6060, 0, mode), ethercat::SdoError);

    // (b) a mapping-object write (0x1C12:00, the first write apply_pdo_map issues)
    // aborts -> apply_pdo_map re-tags it PdoMappingError (the name is correct there).
    be.set_sdo_write_abort(1, 0x1C12, 0);
    ethercat::PdoMap rx;
    rx.pdo_indices = {0x1600};
    rx.entries[0x1600] = {{0x6040, 0, 16}, {0x607A, 0, 32}};
    CHECK_THROWS(ethercat::apply_pdo_map(be, 1, rx, ethercat::PdoDirection::Rx), ethercat::PdoMappingError);
}

TEST_MAIN()
