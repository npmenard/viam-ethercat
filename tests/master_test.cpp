#include <cstdint>
#include <cstdlib>
#include <memory>
#include <span>
#include <vector>

#include "ethercat/cia402.hpp"
#include "ethercat/errors.hpp"
#include "ethercat/master.hpp"
#include "ethercat/pdo_buffer.hpp"
#include "ethercat/sim_backend.hpp"
#include "test_harness.hpp"

using ethercat::Cia402Fsm;
using ethercat::Cia402Mode;
using ethercat::Cia402State;
using ethercat::ControlWord;
using ethercat::Master;
using ethercat::MasterConfig;
using ethercat::PdoSnapshot;
using ethercat::SimBackend;
using ethercat::SimSlaveModel;
using ethercat::SlaveConfig;
using ethercat::Status;

namespace {

// One A6-like slave: RxPDO 0x1600 = ctrl(0x6040,16)+target(0x607A,32);
// TxPDO 0x1A00 = status(0x6041,16)+actual(0x6064,32).
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
    cfg.max_consecutive_wkc_errors = 5;
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

Status status_of(const PdoSnapshot& snap) {
    return Status{ethercat::load_le<std::uint16_t>(std::span<const std::byte>(snap.bytes).subspan(0, 2))};
}

std::int32_t actual_of(const PdoSnapshot& snap) {
    return ethercat::load_le<std::int32_t>(std::span<const std::byte>(snap.bytes).subspan(2, 4));
}

void write_ctrl(Master& m, std::uint16_t cw) {
    ethercat::store_le<std::uint16_t>(m.outputs(1).subspan(0, 2), cw);
}

// Run the CiA402 enable ladder until OperationEnabled (or the cycle budget).
// Returns true if OperationEnabled was reached.
bool drive_to_op(Master& m) {
    for (int cycle = 0; cycle < 50; ++cycle) {
        // NOTE: an un-published cache reads valid=true with an all-zero frame
        // (seq starts even/stable), which decodes to NotReadyToSwitchOn -- fine
        // for the ladder. Use snap.cycle>0 to distinguish a real published frame.
        const Status st = status_of(m.read_inputs(1));
        write_ctrl(m, Cia402Fsm{}.step(st, Cia402State::OperationEnabled));
        m.process();
        if (st.decode() == Cia402State::OperationEnabled) {
            return true;
        }
    }
    return false;
}

}  // namespace

TEST("Master+SimBackend: full RT loop reaches OperationEnabled, WKC + bit10 hold") {
    Master master{make_config(), std::make_unique<SimBackend>(make_models())};
    master.init();
    master.configure();
    CHECK_EQ(master.slave_count(), std::size_t{1});

    bool reached = false;
    for (int cycle = 0; cycle < 50; ++cycle) {
        const PdoSnapshot snap = master.read_inputs(1);
        const Status st = status_of(snap);
        if (snap.is_live()) {
            CHECK(st.target_reached());  // A6 bit10 held =1 on every real published frame
        }
        write_ctrl(master, Cia402Fsm{}.step(st, Cia402State::OperationEnabled));
        master.process();
        CHECK_EQ(master.working_counter(), master.expected_wkc());  // healthy WKC every cycle
        if (st.decode() == Cia402State::OperationEnabled) {
            reached = true;
            break;
        }
    }
    CHECK(reached);
    CHECK(master.all_operational());
    CHECK(!master.fault());
}

TEST("Master+SimBackend: PP SetTarget propagates; actual converges to target") {
    Master master{make_config(), std::make_unique<SimBackend>(make_models())};
    master.init();
    master.configure();
    CHECK(drive_to_op(master));

    constexpr std::int32_t target = 12345;
    ethercat::store_le<std::int32_t>(master.outputs(1).subspan(2, 4), target);

    // New-set-point handshake: raise bit4 until the drive acks with bit12.
    bool acked = false;
    for (int cycle = 0; cycle < 10 && !acked; ++cycle) {
        const std::uint16_t cw = ControlWord::with_new_setpoint(ControlWord::enable_operation(), true);  // 0x1F
        write_ctrl(master, cw);
        master.process();
        acked = status_of(master.read_inputs(1)).setpoint_acknowledged();
    }
    CHECK(acked);

    // Drop bit4 -> ack clears; then let the motion converge.
    write_ctrl(master, ControlWord::enable_operation());  // 0x0F
    master.process();
    CHECK(!status_of(master.read_inputs(1)).setpoint_acknowledged());

    bool converged = false;
    for (int cycle = 0; cycle < 50 && !converged; ++cycle) {
        write_ctrl(master, ControlWord::enable_operation());
        master.process();
        converged = std::abs(actual_of(master.read_inputs(1)) - target) == 0;
    }
    CHECK(converged);
    // Move-complete by the position predicate, NOT statusword bit10 (which is
    // stuck high on the A6).
    CHECK_EQ(actual_of(master.read_inputs(1)), target);
}

TEST("Master: configure() sets modes-of-operation 0x6060 from default_mode (SDO)") {
    auto sim = std::make_unique<SimBackend>(make_models());
    SimBackend* sim_ptr = sim.get();
    Master master{make_config(), std::move(sim)};  // default_mode = ProfilePosition
    master.init();
    master.configure();
    const std::vector<std::byte> mode = sim_ptr->recorded_sdo(1, 0x6060, 0);
    CHECK_EQ(mode.size(), std::size_t{1});
    CHECK_EQ(static_cast<int>(std::to_integer<std::uint8_t>(mode[0])), 1);  // PP = 1
}

TEST("Master: configure() re-applies the PDO map every call (power-cycle safe)") {
    auto sim = std::make_unique<SimBackend>(make_models());
    SimBackend* sim_ptr = sim.get();
    Master master{make_config(), std::move(sim)};
    master.init();

    master.configure();
    const std::size_t writes_after_first = sim_ptr->sdo_log(1).size();
    CHECK(writes_after_first > 0);
    CHECK(drive_to_op(master));

    master.configure();                                      // simulated power-cycle: the A6 map isn't in EEPROM
    CHECK(sim_ptr->sdo_log(1).size() > writes_after_first);  // the remap ran again
    CHECK(drive_to_op(master));
    CHECK(master.all_operational());
}

TEST("Master: a sustained short WKC latches fault() without throwing") {
    auto sim = std::make_unique<SimBackend>(make_models());
    SimBackend* sim_ptr = sim.get();
    Master master{make_config(), std::move(sim)};
    master.init();
    master.configure();
    CHECK(drive_to_op(master));
    CHECK(!master.fault());

    // One transient short cycle must NOT latch a fault.
    sim_ptr->force_short_wkc_once();
    write_ctrl(master, ControlWord::enable_operation());
    master.process();
    CHECK(!master.fault());

    // max_consecutive_wkc_errors consecutive short cycles -> latched fault.
    for (std::uint32_t i = 0; i < 5; ++i) {
        sim_ptr->force_short_wkc_once();
        write_ctrl(master, ControlWord::enable_operation());
        master.process();  // must not throw
    }
    CHECK(master.fault());
    CHECK(!master.all_operational());
    CHECK(!master.last_error().empty());
}

TEST("Master: ctor validates config (clear-text ConfigError)") {
    MasterConfig bad = make_config();
    bad.ifname.clear();
    CHECK_THROWS_MSG(Master(bad, std::make_unique<SimBackend>(make_models())), ethercat::ConfigError, "interface name");

    MasterConfig bad_rate = make_config();
    bad_rate.target_loop_rate_hz = 5000;
    CHECK_THROWS_MSG(Master(bad_rate, std::make_unique<SimBackend>(make_models())), ethercat::ConfigError, "target_loop_rate_hz");
}

TEST_MAIN()
