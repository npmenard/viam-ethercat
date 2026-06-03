#include <algorithm>
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

using ethercat::BringupStatus;
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
    // Phase 1 (#20): configure() leaves the bus at SAFE-OP; drive the bring-up FSM
    // (SETTLE -> ARM -> GATE -> request OP -> AWAIT_OP) to EtherCAT OPERATIONAL. On the
    // sim, drive_sync_faulted is always false and full WKC is immediate, so this passes
    // in ~dc_arm_settle_cycles + dc_op_gate_cycles (+1) cycles. A generous budget covers
    // the default bounds.
    bool op = false;
    for (int cycle = 0; cycle < 500; ++cycle) {
        const BringupStatus bs = m.bringup_step(/*drive_sync_faulted=*/false);
        if (bs == BringupStatus::Operational) {
            op = true;
            break;
        }
        if (bs == BringupStatus::Aborted) {
            return false;
        }
    }
    if (!op || !m.all_operational()) {
        return false;
    }
    // Phase 2: drive the CiA402 ladder to OperationEnabled.
    // NOTE: an un-published cache reads valid=true with an all-zero frame
    // (seq starts even/stable), which decodes to NotReadyToSwitchOn -- fine
    // for the ladder. Use snap.cycle>0 to distinguish a real published frame.
    for (int cycle = 0; cycle < 50; ++cycle) {
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

    // #20: configure() stops at SAFE-OP; bring the bus to EtherCAT OP via the FSM first.
    for (int cycle = 0; cycle < 500; ++cycle) {
        const BringupStatus bs = master.bringup_step(/*drive_sync_faulted=*/false);
        if (bs == BringupStatus::Operational || bs == BringupStatus::Aborted) {
            break;
        }
    }
    CHECK(master.all_operational());

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

TEST("Master: configure() applies preop_sdo_writes (drive tuning) before the remap") {
    MasterConfig cfg = make_config();
    // Two driver-supplied PRE-OP writes (mirrors the A6 C13 sync-tolerance tune).
    cfg.slaves[0].preop_sdo_writes = {
        {0x2013, 0x06, {std::byte{0x02}, std::byte{0x00}}},  // U16 = 2
        {0x2013, 0x07, {std::byte{0x70}, std::byte{0x17}}},  // U16 = 6000
    };
    auto sim = std::make_unique<SimBackend>(make_models());
    SimBackend* sim_ptr = sim.get();
    Master master{cfg, std::move(sim)};
    master.init();
    master.configure();

    // The raw values were written verbatim.
    const std::vector<std::byte> a = sim_ptr->recorded_sdo(1, 0x2013, 0x06);
    const std::vector<std::byte> b = sim_ptr->recorded_sdo(1, 0x2013, 0x07);
    CHECK_EQ(a.size(), std::size_t{2});
    CHECK_EQ(static_cast<int>(std::to_integer<std::uint8_t>(a[0])), 0x02);
    CHECK_EQ(static_cast<int>(std::to_integer<std::uint8_t>(b[1])), 0x17);

    // ...and they landed BEFORE the remap: their keys lead the SDO log.
    const std::vector<std::uint32_t> log = sim_ptr->sdo_log(1);
    const auto first = log.front();
    CHECK_EQ(first, (std::uint32_t{0x2013} << 8U) | 0x06U);
}

TEST("Master: configure() applies postremap_sdo_writes AFTER the PDO assignment") {
    MasterConfig cfg = make_config();
    // An SM-sync-type write (mirrors the A6 0x1C32:01 = DC SYNC0): must land after the
    // 0x1C12/0x1C13 assignment or the drive re-defaults it.
    cfg.slaves[0].postremap_sdo_writes = {
        {0x1C32, 0x01, {std::byte{0x02}, std::byte{0x00}}},  // sync type = DC SYNC0
    };
    auto sim = std::make_unique<SimBackend>(make_models());
    SimBackend* sim_ptr = sim.get();
    Master master{cfg, std::move(sim)};
    master.init();
    master.configure();

    const std::vector<std::uint32_t> log = sim_ptr->sdo_log(1);
    const auto key = [](std::uint16_t idx, std::uint8_t sub) { return (static_cast<std::uint32_t>(idx) << 8U) | sub; };
    const auto sync_pos = std::find(log.begin(), log.end(), key(0x1C32, 0x01));
    const auto assign_pos = std::find(log.begin(), log.end(), key(0x1C12, 0x00));  // RxPDO SM assignment
    CHECK(sync_pos != log.end());                                                  // the sync-type write happened
    CHECK(assign_pos != log.end());                                                // the assignment happened
    CHECK(assign_pos < sync_pos);                                                  // ...and the assignment came FIRST
}

TEST("Master+SimBackend: dc_time() advances across process() (DC phase-lock input)") {
    Master master{make_config(), std::make_unique<SimBackend>(make_models())};
    master.init();
    master.configure();
    const std::int64_t t0 = master.dc_time();
    master.process();
    master.process();
    CHECK(master.dc_time() > t0);  // the phase-lock PI controller needs a monotonic DC clock
}

TEST("Master: the DC bring-up arms SYNC0 only when use_distributed_clocks is set") {
    {  // DC off -> arm_dc_sync is never called, even after a full bring-up to OP
        auto sim = std::make_unique<SimBackend>(make_models());
        SimBackend* sim_ptr = sim.get();
        Master master{make_config(), std::move(sim)};
        master.init();
        master.configure();
        CHECK(drive_to_op(master));
        CHECK_EQ(sim_ptr->configured_dc_cycle_ns(), std::uint32_t{0});
    }
    {  // DC on -> the bring-up ARM phase calls arm_dc_sync with cycle = 1e9/rate (1 kHz -> 1 ms)
        MasterConfig cfg = make_config();
        cfg.use_distributed_clocks = true;
        cfg.target_loop_rate_hz = 1000;
        auto sim = std::make_unique<SimBackend>(make_models());
        SimBackend* sim_ptr = sim.get();
        Master master{cfg, std::move(sim)};
        master.init();
        master.configure();
        CHECK(drive_to_op(master));
        CHECK_EQ(sim_ptr->configured_dc_cycle_ns(), std::uint32_t{1'000'000});
    }
}

TEST("Master(#20): bringup_step drives SAFE-OP -> OPERATIONAL through the FSM phases") {
    Master master{make_config(), std::make_unique<SimBackend>(make_models())};
    master.init();
    master.configure();
    CHECK(!master.all_operational());  // #20: configure() stops at SAFE-OP, never requests OP

    BringupStatus bs = BringupStatus::Settling;
    bool saw_settling = false;
    bool saw_arming = false;
    bool saw_gating = false;
    for (int cycle = 0; cycle < 500; ++cycle) {
        bs = master.bringup_step(/*drive_sync_faulted=*/false);
        saw_settling = saw_settling || bs == BringupStatus::Settling;
        saw_arming = saw_arming || bs == BringupStatus::Arming;
        saw_gating = saw_gating || bs == BringupStatus::Gating;
        if (bs == BringupStatus::Operational) {
            break;
        }
    }
    CHECK(bs == BringupStatus::Operational);
    CHECK(saw_settling);                // SETTLE pumped PD before arming
    CHECK(saw_arming);                  // passed through the ARM transient
    CHECK(saw_gating);                  // gated on (PD flowing + no Er74.1)
    CHECK(master.all_operational());    // OP reached only after the gate + AWAIT_OP
    CHECK(!master.fault());
}

TEST("Master(#20): bringup_step aborts on Er74.1 in the gate and does NOT retry (no hammer)") {
    MasterConfig cfg = make_config();
    cfg.use_distributed_clocks = true;  // exercise the SYNC0 ARM path
    cfg.dc_arm_settle_cycles = 3;       // short SETTLE for the test
    cfg.dc_op_gate_cycles = 50;
    Master master{cfg, std::make_unique<SimBackend>(make_models())};
    master.init();
    master.configure();

    // Pump through SETTLE + ARM into the GATE with no fault...
    for (int cycle = 0; cycle < 6; ++cycle) {
        (void)master.bringup_step(/*drive_sync_faulted=*/false);
    }
    // ...then the drive reports Er74.1 (no SYNC0) during the gate -> abort, no OP request.
    const BringupStatus aborted = master.bringup_step(/*drive_sync_faulted=*/true);
    CHECK(aborted == BringupStatus::Aborted);
    CHECK(!master.all_operational());

    // NO-HAMMER: even once the fault clears, the FSM stays Aborted -- it never silently
    // re-enters bring-up / re-requests OP (repeated Er74 OP-entry wedges the A6). A
    // re-attempt requires an explicit reconfigure. Many cycles, always Aborted.
    for (int cycle = 0; cycle < 200; ++cycle) {
        CHECK(master.bringup_step(/*drive_sync_faulted=*/false) == BringupStatus::Aborted);
    }
    CHECK(!master.all_operational());
}

TEST("Master(#20): configure() issues the vendor fault-reset SDO when configured (else not)") {
    {  // configured -> the vendor SDO (A6 0x2031:01 = 1) is written once at bring-up
        MasterConfig cfg = make_config();
        cfg.slaves[0].fault_reset = ethercat::SdoWrite{0x2031, 0x01, {std::byte{0x01}}};
        auto sim = std::make_unique<SimBackend>(make_models());
        SimBackend* sim_ptr = sim.get();
        Master master{cfg, std::move(sim)};
        master.init();
        master.configure();
        const std::vector<std::byte> rec = sim_ptr->recorded_sdo(1, 0x2031, 0x01);
        CHECK_EQ(rec.size(), std::size_t{1});
        CHECK(!rec.empty() && rec[0] == std::byte{0x01});
    }
    {  // absent -> no vendor reset write (generic CiA402 bit7 path is used instead)
        auto sim = std::make_unique<SimBackend>(make_models());
        SimBackend* sim_ptr = sim.get();
        Master master{make_config(), std::move(sim)};
        master.init();
        master.configure();
        CHECK(sim_ptr->recorded_sdo(1, 0x2031, 0x01).empty());
    }
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
