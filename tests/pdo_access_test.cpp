// Offline tests for the #30 PDO access API (P2a): Field<>/cia402 aliases, Rpdo
// (immutable frame-consistent read snapshot) and Tpdo (seeded write builder,
// submit-to-transmit). Driven by a Master over a SimBackend -- no hardware.
//
// Covers spec #30 §8 P2a tests 1-7:
//   (1) snapshot frame-consistency   (5) resolve-throw on not-in-map
//   (2) seeded-Tpdo carry-over       (6) bounds-throw past the frame
//   (3) unsubmitted never transmits  (7) round-trip via the cia402:: aliases
//   (4) submit -> next-cycle transmit

#include <cstdint>
#include <memory>
#include <vector>

#include "ethercat/cia402.hpp"
#include "ethercat/errors.hpp"
#include "ethercat/field.hpp"
#include "ethercat/master.hpp"
#include "ethercat/pdo_buffer.hpp"
#include "ethercat/sim_backend.hpp"
#include "test_harness.hpp"

using ethercat::BringupStatus;
using ethercat::Cia402Fsm;
using ethercat::Cia402Mode;
using ethercat::Cia402State;
using ethercat::Field;
using ethercat::load_le;
using ethercat::Master;
using ethercat::MasterConfig;
using ethercat::Rpdo;
using ethercat::SimBackend;
using ethercat::SimSlaveModel;
using ethercat::SlaveConfig;
using ethercat::Status;
using ethercat::store_le;
using ethercat::Tpdo;
namespace cia402 = ethercat::cia402;

namespace {

// A6-ish image: RxPDO = ctrl(0x6040)@0 u16 + target(0x607A)@2 i32 (6 bytes);
// TxPDO = status(0x6041)@0 u16 + actual(0x6064)@2 i32 (6 bytes). 0x603F / 0x6081
// are deliberately NOT mapped (so the not-in-map throw paths have a target).
MasterConfig make_config() {
    SlaveConfig sc;
    sc.slave_id = 1;
    sc.rxpdo.assign_index = 0x1C12;
    sc.rxpdo.pdo_indices = {0x1600};
    sc.rxpdo.entries[0x1600] = {{0x6040, 0, 16}, {0x607A, 0, 32}};
    sc.txpdo.assign_index = 0x1C13;
    sc.txpdo.pdo_indices = {0x1A00};
    sc.txpdo.entries[0x1A00] = {{0x6041, 0, 16}, {0x6064, 0, 32}};
    sc.default_mode = Cia402Mode::ProfilePosition;

    MasterConfig cfg;
    cfg.ifname = "sim0";
    cfg.slaves = {sc};
    return cfg;
}

std::vector<SimSlaveModel> make_models() {
    SimSlaveModel m;
    m.output_bytes = 6;
    m.input_bytes = 6;
    m.ctrlword_off = 0;
    m.target_off = 2;
    m.statusword_off = 0;
    m.actual_off = 2;
    m.mode = Cia402Mode::ProfilePosition;
    m.counts_per_step = 1000;
    return {m};
}

// Bring the master to EtherCAT OP via the bring-up FSM.
void to_op(Master& m) {
    m.init();
    m.configure();
    for (int c = 0; c < 2000; ++c) {
        const BringupStatus bs = m.bringup_step(/*drive_sync_faulted=*/false);
        if (bs == BringupStatus::Operational || bs == BringupStatus::Aborted) {
            break;
        }
    }
    CHECK(m.all_operational());
}

// Step the CiA402 enable ladder one cycle, fully through the new API: read status
// via Rpdo, step toward OperationEnabled, stream the controlword via a seeded Tpdo,
// submit, process.
void step_ladder(Master& m) {
    const Status st{m.read_rpdo(1).get<cia402::Statusword>()};
    Tpdo t = m.make_tpdo(1);
    t.put<cia402::ControlWord>(Cia402Fsm{}.step(st, Cia402State::OperationEnabled));
    t.submit();
    m.process();
}

}  // namespace

// (7) round-trip via the cia402:: aliases: a seeded Tpdo put through submit+process
// lands the right bytes at the right offsets/widths in the command image, read back
// via load_le. (Exercises ControlWord@0/u16 and TargetPosition@2/i32 aliases.)
TEST("#30 P2a.7: Tpdo put<alias> + submit -> command image round-trips at the right offset/width") {
    Master m{make_config(), std::make_unique<SimBackend>(make_models())};
    to_op(m);

    Tpdo t = m.make_tpdo(1);
    t.put<cia402::ControlWord>(std::uint16_t{0x000F});
    t.put<cia402::TargetPosition>(std::int32_t{0x12345678});
    t.submit();
    m.process();  // RT take_outputs drains the staged frame into the command image, then sends

    const auto out = m.outputs(1);
    CHECK_EQ(load_le<std::uint16_t>(out.subspan(0, 2)), std::uint16_t{0x000F});    // ControlWord -> 0x6040 @ 0
    CHECK_EQ(load_le<std::int32_t>(out.subspan(2, 4)), std::int32_t{0x12345678});  // TargetPosition -> 0x607A @ 2
}

// (2) seeded-Tpdo carry-over: a field you DON'T put() retains the seed (the current
// command image), while the one you put() updates.
TEST("#30 P2a.2: a seeded Tpdo carries over un-put fields and updates put ones") {
    Master m{make_config(), std::make_unique<SimBackend>(make_models())};
    to_op(m);

    // Establish a known command image: ctrl=0x0006, target=111.
    store_le<std::uint16_t>(m.outputs(1).subspan(0, 2), std::uint16_t{0x0006});
    store_le<std::int32_t>(m.outputs(1).subspan(2, 4), std::int32_t{111});
    m.process();  // nothing staged -> take_outputs is a no-op; the direct writes stand + are sent

    Tpdo t = m.make_tpdo(1);                           // seeds from {0x0006, 111}
    t.put<cia402::TargetPosition>(std::int32_t{222});  // change ONLY the target
    t.submit();
    m.process();

    const auto out = m.outputs(1);
    CHECK_EQ(load_le<std::uint16_t>(out.subspan(0, 2)), std::uint16_t{0x0006});  // ControlWord carried over from the seed
    CHECK_EQ(load_le<std::int32_t>(out.subspan(2, 4)), std::int32_t{222});       // TargetPosition updated
}

// (3)+(4) unsubmitted-never-transmits and submit->next-cycle-transmit.
TEST("#30 P2a.3/4: an unsubmitted Tpdo never reaches the bus; submit() lands next process()") {
    Master m{make_config(), std::make_unique<SimBackend>(make_models())};
    to_op(m);
    store_le<std::int32_t>(m.outputs(1).subspan(2, 4), std::int32_t{111});
    m.process();

    // (3) put but DO NOT submit -> the command image is unchanged after a cycle.
    {
        Tpdo t = m.make_tpdo(1);
        t.put<cia402::TargetPosition>(std::int32_t{999});
        CHECK(!t.submitted());
        m.process();
        CHECK_EQ(load_le<std::int32_t>(m.outputs(1).subspan(2, 4)), std::int32_t{111});  // 999 never left the Tpdo
    }
    // (4) submit -> staged, not yet on the image; the NEXT process() drains it.
    {
        Tpdo t = m.make_tpdo(1);
        t.put<cia402::TargetPosition>(std::int32_t{777});
        t.submit();
        CHECK(t.submitted());
        CHECK_EQ(load_le<std::int32_t>(m.outputs(1).subspan(2, 4)), std::int32_t{111});  // staged, not yet taken
        m.process();
        CHECK_EQ(load_le<std::int32_t>(m.outputs(1).subspan(2, 4)), std::int32_t{777});  // transmitted this cycle
    }
}

// (1) snapshot frame-consistency: an Rpdo is a frozen COPY -- its get<>s keep
// returning the captured frame's values even as the bus advances under it, and a
// fresh read_rpdo sees the newer frame. Demonstrated with the statusword, which
// CHANGES across the enable ladder (SwitchedOn -> OperationEnabled) -- no motion
// needed (the sim's PP chase needs a bit4 handshake; the ladder is cleaner here).
TEST("#30 P2a.1: an Rpdo is a frozen, frame-consistent snapshot") {
    Master m{make_config(), std::make_unique<SimBackend>(make_models())};
    to_op(m);

    // Advance the CiA402 ladder until a fresh snapshot shows SwitchedOn (one rung below
    // OperationEnabled), then freeze that Rpdo copy.
    Rpdo mid = m.read_rpdo(1);
    for (int guard = 0; guard < 50 && Status{mid.get<cia402::Statusword>()}.decode() != Cia402State::SwitchedOn; ++guard) {
        step_ladder(m);
        mid = m.read_rpdo(1);
    }
    CHECK(Status{mid.get<cia402::Statusword>()}.decode() == Cia402State::SwitchedOn);
    const std::uint16_t sw_mid = mid.get<cia402::Statusword>();
    const std::uint64_t cyc = mid.snapshot().cycle;

    // Drive on to OperationEnabled -- the live statusword changes underneath `mid`.
    for (int c = 0; c < 50; ++c) {
        const bool enabled = Status{m.read_rpdo(1).get<cia402::Statusword>()}.decode() == Cia402State::OperationEnabled;
        if (enabled) {
            break;
        }
        step_ladder(m);
    }

    // `mid` is a COPY: its get<>s + metadata are still the SwitchedOn frame's (frozen).
    CHECK_EQ(mid.get<cia402::Statusword>(), sw_mid);
    CHECK_EQ(mid.snapshot().cycle, cyc);

    // A fresh snapshot sees the advanced frame: newer cycle AND a changed statusword.
    const Rpdo fresh = m.read_rpdo(1);
    CHECK(fresh.snapshot().cycle > cyc);
    CHECK(fresh.get<cia402::Statusword>() != sw_mid);  // ladder advanced past SwitchedOn
}

// (5) resolve-throw on NOT-IN-MAP -> PdoMappingError. The throw-tier split: not-in-map is a
// MAP-MEMBERSHIP concern (PdoMappingError, spanning apply_pdo_map + this runtime access of an
// un-mapped object); a MALFORMED access (wrong width / past frame) is PdoAccessError (test 6/8).
TEST("#30 P2a.5: get/put of an unmapped object throws PdoMappingError") {
    Master m{make_config(), std::make_unique<SimBackend>(make_models())};
    m.init();
    m.configure();  // field tables are built here; OP not required for resolution

    const Rpdo r = m.read_rpdo(1);
    CHECK_THROWS(r.get<cia402::FaultCode>(), ethercat::PdoMappingError);  // 0x603F not in this TxPDO

    Tpdo t = m.make_tpdo(1);
    CHECK_THROWS(t.put<cia402::ProfileVelocity>(std::uint32_t{5}), ethercat::PdoMappingError);  // 0x6081 not in this RxPDO
}

// (6) an over-wide T on a mapped object -> PdoAccessError. (With the §5 width assert this trips
// at RESOLVE -- sizeof(T) != the mapped width -- before any read; the in-Rpdo/Tpdo offset+sizeof
// bounds check is the defense-in-depth fallback if the provisional width assert is ever pulled.)
TEST("#30 P2a.6: an over-wide Field on a mapped object throws PdoAccessError") {
    Master m{make_config(), std::make_unique<SimBackend>(make_models())};
    m.init();
    m.configure();

    // Aliased (the CHECK_THROWS macro can't take the commas inside Field<...>). 0x6064/0x607A are
    // 32-bit-mapped at offset 2 in a 6-byte frame; an int64 (8 B) is both wrong-width AND [2,10) past it.
    using ActualAsI64 = Field<0x6064, 0, std::int64_t>;
    using TargetAsI64 = Field<0x607A, 0, std::int64_t>;

    const Rpdo r = m.read_rpdo(1);
    CHECK_THROWS(r.get<ActualAsI64>(), ethercat::PdoAccessError);

    Tpdo t = m.make_tpdo(1);
    CHECK_THROWS(t.put<TargetAsI64>(std::int64_t{0}), ethercat::PdoAccessError);
}

// (8) width-mismatch resolve-throw (#30 §5): an UNDER-wide alias on a mapped object -- in
// bounds, but the wrong number of bytes -- throws clear-text at RESOLVE, instead of silently
// reading/writing 1 of 2 (or 2 of 4) bytes. This is the silent-wrong-read gap the width assert closes.
TEST("#30 P2a.8: an under-wide Field on a mapped object throws PdoAccessError at resolve") {
    Master m{make_config(), std::make_unique<SimBackend>(make_models())};
    m.init();
    m.configure();

    // statusword (0x6041) and controlword (0x6040) are 16-bit mapped; a uint8 alias is 1 of 2 bytes
    // -- IN bounds (offset 0, frame >= 1) but the wrong width, so it must throw at resolve, not read silently.
    using StatusAsU8 = Field<0x6041, 0, std::uint8_t>;
    using CtrlAsU8 = Field<0x6040, 0, std::uint8_t>;

    const Rpdo r = m.read_rpdo(1);
    CHECK_THROWS(r.get<StatusAsU8>(), ethercat::PdoAccessError);

    Tpdo t = m.make_tpdo(1);
    CHECK_THROWS(t.put<CtrlAsU8>(std::uint8_t{1}), ethercat::PdoAccessError);
}

TEST_MAIN()
