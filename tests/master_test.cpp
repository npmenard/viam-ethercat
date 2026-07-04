#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <memory>
#include <span>
#include <thread>
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
    sc.rxpdo.pdo_indices = {0x1600};
    sc.rxpdo.entries[0x1600] = {{0x6040, 0, 16}, {0x607A, 0, 32}};
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
    // Phase 1 (#20): configure() leaves the bus at SAFE-OP with SYNC0 armed (PRE-OP);
    // drive the bring-up FSM (GATE -> request OP -> AWAIT_OP) to EtherCAT OPERATIONAL. On
    // the sim, drive_sync_faulted is always false and full WKC is immediate, so this
    // passes in ~dc_op_gate_cycles (+1) cycles. A generous budget covers the default.
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

TEST("Master+SimBackend: full RT loop reaches OperationEnabled, WKC holds") {
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

// the generic preop_sdo_writes / postremap_sdo_writes orchestration was
// EVICTED from Master (setup-SDO policy is the consumer's, run via Master::sdo_write
// post-configure -- the #39 pattern). The two tests that exercised those lists are
// gone with the mechanism; the surviving consumer-side path (vendor_fault_reset) is
// covered in controller_offline_test. The STRUCTURAL remap order is still pinned by
// pdo_mapping_test (the 5-step apply_pdo_map sequence) + the mode-set test below.

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

    BringupStatus bs = BringupStatus::Gating;
    bool saw_gating = false;
    for (int cycle = 0; cycle < 500; ++cycle) {
        bs = master.bringup_step(/*drive_sync_faulted=*/false);
        saw_gating = saw_gating || bs == BringupStatus::Gating;
        if (bs == BringupStatus::Operational) {
            break;
        }
    }
    CHECK(bs == BringupStatus::Operational);
    CHECK(saw_gating);                // SYNC0 armed in configure(); the loop gated on (PD flowing + no Er74.1)
    CHECK(master.all_operational());  // OP reached only after the gate + AWAIT_OP
    CHECK(!master.fault());
}

TEST("Master(#20): bringup_step waits out OP (ec_sample patience) and aborts only after the generous window") {
    MasterConfig cfg = make_config();
    cfg.use_distributed_clocks = true;  // SYNC0 armed in configure(); the loop SETTLEs then awaits OP
    cfg.dc_op_gate_cycles = 2;          // short SETTLE for the test
    Master master{cfg, std::make_unique<SimBackend>(make_models())};
    master.init();
    master.configure();

    // SETTLE does NOT abort on Er74.1 (it's the normal pre-sync state) -- even with the
    // drive reporting Er74.1, the settle completes and the initial OP request goes out.
    BringupStatus bs = BringupStatus::Gating;
    for (int cycle = 0; cycle < 3; ++cycle) {
        bs = master.bringup_step(/*drive_sync_faulted=*/true);
        CHECK(bs != BringupStatus::Aborted);  // no pre-OP abort on Er74.1
    }
    // AWAIT_OP: Er74.1 never clears -> the held-synced state (full WKC && !Er74.1) is never
    // reached. The FSM does NOT abort early -- it waits out the A6's slow SAFE-OP->OP
    // (ec_sample reaches OP >11s in), re-requesting OP each nudge interval while PD flows --
    // and only gives up after the generous kAwaitOpBound. So it must still be AwaitingOp well
    // past the OLD ~500-cycle bound...
    for (int cycle = 0; cycle < 2'000; ++cycle) {
        bs = master.bringup_step(/*drive_sync_faulted=*/true);
        CHECK(bs == BringupStatus::AwaitingOp);  // patient: not aborted at 500/600/2000 cycles
    }
    // ...and eventually aborts once the wide window elapses (kAwaitOpBound ~30k cycles).
    for (int cycle = 0; cycle < 40'000 && bs != BringupStatus::Aborted; ++cycle) {
        bs = master.bringup_step(/*drive_sync_faulted=*/true);
    }
    CHECK(bs == BringupStatus::Aborted);
    CHECK(!master.all_operational());

    // Aborted is TERMINAL -- a re-attempt requires an explicit reconfigure(), so the FSM stays
    // Aborted even once the fault clears (it does not silently re-enter bring-up).
    for (int cycle = 0; cycle < 200; ++cycle) {
        CHECK(master.bringup_step(/*drive_sync_faulted=*/false) == BringupStatus::Aborted);
    }
    CHECK(!master.all_operational());
}

TEST("Master(#42): AWAIT_OP bounds are MasterConfig fields; the give-up window is wall-time, rate-independent") {
    // The give-up bound is op_await_timeout_ms converted at target_loop_rate_hz (the old
    // 30'000-cycle constant silently meant 30 s only at 1 kHz). Same 100 ms timeout at two
    // rates -> proportionally different CYCLE budgets, same wall-clock patience.
    const auto cycles_to_abort = [](std::uint32_t rate_hz, std::uint32_t timeout_ms) -> int {
        MasterConfig cfg = make_config();
        cfg.use_distributed_clocks = true;
        cfg.dc_op_gate_cycles = 2;  // short SETTLE
        cfg.target_loop_rate_hz = rate_hz;
        cfg.op_await_timeout_ms = timeout_ms;
        Master master{cfg, std::make_unique<SimBackend>(make_models())};
        master.init();
        master.configure();
        int cycles = 0;
        BringupStatus bs = BringupStatus::Gating;
        while (bs != BringupStatus::Aborted && cycles < 10'000) {  // Er74.1 never clears -> must give up
            bs = master.bringup_step(/*drive_sync_faulted=*/true);
            ++cycles;
        }
        CHECK(bs == BringupStatus::Aborted);
        return cycles;
    };
    const int at_1khz = cycles_to_abort(1000, 100);  // 100 ms @ 1 kHz -> ~100 await cycles (+2 settle)
    const int at_250hz = cycles_to_abort(250, 100);  // 100 ms @ 250 Hz -> ~25 await cycles (+2 settle)
    CHECK(at_1khz >= 100);
    CHECK(at_1khz <= 110);
    CHECK(at_250hz >= 25);
    CHECK(at_250hz <= 35);  // SAME wall-time patience, quarter the cycles -- not 4x the wall time

    // Degenerate values are safe: 0-count fields clamp to 1 (no div-by-zero nudge, no
    // zero-evidence hold-confirm) and a 0 ms timeout still leaves a >= 1-cycle window --
    // a healthy sim drive still reaches Operational.
    MasterConfig cfg = make_config();
    cfg.op_hold_confirm_cycles = 0;
    cfg.op_nudge_interval_cycles = 0;
    cfg.op_await_timeout_ms = 0;
    Master master{cfg, std::make_unique<SimBackend>(make_models())};
    master.init();
    master.configure();
    BringupStatus bs = BringupStatus::Gating;
    for (int cycle = 0; cycle < 50 && bs != BringupStatus::Operational && bs != BringupStatus::Aborted; ++cycle) {
        bs = master.bringup_step(/*drive_sync_faulted=*/false);
    }
    CHECK(bs == BringupStatus::Operational);  // healthy drive: confirms within the clamped 1-cycle hold
}

TEST("Master(#44): a declared SYNC0 cycle granularity rejects a non-multiple loop rate at config time") {
    // 600 Hz -> 1'666'666 ns cycle: NOT a 250 us multiple -> Error naming the slave,
    // the granularity, the cycle, and the nearest valid rates -- instead of the drive
    // faulting cryptically at OP entry (the A6 Er74.0 failure mode this prevents).
    MasterConfig bad = make_config();
    bad.use_distributed_clocks = true;
    bad.target_loop_rate_hz = 600;
    bad.slaves[0].sync_cycle_granularity_ns = 250'000;
    CHECK_THROWS(Master(bad, std::make_unique<SimBackend>(make_models())), ethercat::Error);

    // Valid multiples pass (1 kHz = 4 x 250 us), as does the same bad rate when the slave
    // declares NO granularity (0 = no constraint) or DC is off (no SYNC0 in play).
    MasterConfig ok = make_config();
    ok.use_distributed_clocks = true;
    ok.target_loop_rate_hz = 1000;
    ok.slaves[0].sync_cycle_granularity_ns = 250'000;
    Master m_ok{ok, std::make_unique<SimBackend>(make_models())};

    MasterConfig no_decl = make_config();
    no_decl.use_distributed_clocks = true;
    no_decl.target_loop_rate_hz = 600;  // bad rate, but nothing declared -> no check
    Master m_nodecl{no_decl, std::make_unique<SimBackend>(make_models())};

    MasterConfig no_dc = make_config();
    no_dc.target_loop_rate_hz = 600;  // DC off -> no SYNC0 -> granularity moot
    no_dc.slaves[0].sync_cycle_granularity_ns = 250'000;
    Master m_nodc{no_dc, std::make_unique<SimBackend>(make_models())};
}

TEST("Master(#40): wkc_stats counts steady cycles + bad WKC; bring-up excluded; configure resets") {
    auto sim = std::make_unique<SimBackend>(make_models());
    SimBackend* sim_ptr = sim.get();
    Master m2{make_config(), std::move(sim)};
    m2.init();
    m2.configure();
    // Bring-up exchanges must NOT count (partial WKC is normal pre-OP).
    for (int c = 0; c < 200; ++c) {
        if (m2.bringup_step(false) == ethercat::BringupStatus::Operational) {
            break;
        }
    }
    CHECK_EQ(m2.wkc_stats().total_cycles, std::uint64_t{0});
    // Steady cycles: 10 good + 3 forced-short.
    for (int i = 0; i < 10; ++i) {
        m2.process();
    }
    sim_ptr->force_short_wkc(true);
    for (int i = 0; i < 3; ++i) {
        m2.process();
    }
    sim_ptr->force_short_wkc(false);
    const ethercat::WkcStats st = m2.wkc_stats();
    CHECK_EQ(st.total_cycles, std::uint64_t{13});
    CHECK_EQ(st.bad_cycles, std::uint64_t{3});
    CHECK_EQ(st.expected, m2.expected_wkc());
    // configure() resets the counters (per-power-on stats).
    m2.configure();
    CHECK_EQ(m2.wkc_stats().total_cycles, std::uint64_t{0});
    CHECK_EQ(m2.wkc_stats().bad_cycles, std::uint64_t{0});
}

TEST("Master(#39): configure() fires ZERO vendor SDO traffic -- only map/assign/mode objects") {
    // The vendor fault-reset that used to fire in configure() is consumer-side now.
    // Assert the library bring-up writes touch ONLY the PDO mapping sub-protocol
    // (0x1C12/0x1C13 assigns, 0x1600/0x1A00 entries) and modes-of-operation (0x6060)
    // -- in particular, no 0x2031-class vendor object, ever (#41 lens).
    auto sim = std::make_unique<SimBackend>(make_models());
    SimBackend* sim_ptr = sim.get();
    Master master{make_config(), std::move(sim)};
    master.init();
    master.configure();
    CHECK(sim_ptr->recorded_sdo(1, 0x2031, 0x01).empty());
    for (const std::uint32_t key : sim_ptr->sdo_log(1)) {
        const auto index = static_cast<std::uint16_t>(key >> 8U);
        const bool mapping_or_mode = index == 0x1C12 || index == 0x1C13 || index == 0x1600 || index == 0x1A00 || index == 0x6060;
        CHECK(mapping_or_mode);  // any other object = a library vendor-leak
    }
}

TEST("Master(#39): public sdo_write/sdo_read forward to the backend; the RT-phase guard throws") {
    auto sim = std::make_unique<SimBackend>(make_models());
    SimBackend* sim_ptr = sim.get();
    Master master{make_config(), std::move(sim)};
    master.init();
    master.configure();

    // Pre-RT (no declared phase): the consumer-side write goes through...
    const std::array<std::byte, 2> one{std::byte{0x01}, std::byte{0x00}};
    master.sdo_write(1, 0x2031, 0x01, one);
    const std::vector<std::byte> rec = sim_ptr->recorded_sdo(1, 0x2031, 0x01);
    CHECK_EQ(rec.size(), std::size_t{2});
    CHECK(!rec.empty() && rec[0] == std::byte{0x01});
    // ...and reads back through sdo_read.
    std::array<std::byte, 2> back{};
    CHECK_EQ(master.sdo_read(1, 0x2031, 0x01, back), std::size_t{2});
    CHECK(back[0] == std::byte{0x01});

    // #15: the RT-phase gate is gone -- sdo_read/sdo_write run the mailbox transfer directly on the
    // caller's thread and are concurrency-safe against a running RT PDO loop (SOEM v2 port is
    // thread-safe). A subsequent write/read still round-trips through the backend.
    master.sdo_write(1, 0x2031, 0x01, one);
    CHECK_EQ(master.sdo_read(1, 0x2031, 0x01, back), std::size_t{2});
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

TEST("Master: ctor validates config (clear-text Error)") {
    MasterConfig bad = make_config();
    bad.ifname.clear();
    CHECK_THROWS_MSG(Master(bad, std::make_unique<SimBackend>(make_models())), ethercat::Error, "interface name");

    MasterConfig bad_rate = make_config();
    bad_rate.target_loop_rate_hz = 5000;
    CHECK_THROWS_MSG(Master(bad_rate, std::make_unique<SimBackend>(make_models())), ethercat::Error, "target_loop_rate_hz");
}

// #32.3: the DC settle-cycle counts are MasterConfig fields (documented + tunable
// without a recompile). Defaults reproduce today's values, and a Settle-phase
// override (dc_op_gate_cycles) is honored by bringup_step.
TEST("#32.3: dc settle cycles are MasterConfig fields (defaults + Settle override honored)") {
    CHECK_EQ(MasterConfig{}.dc_op_gate_cycles, std::uint32_t{400});  // today's SETTLE count
    CHECK_EQ(MasterConfig{}.dc_settle_cycles, std::uint32_t{0});     // today's post-OP grace

    MasterConfig cfg = make_config();
    cfg.use_distributed_clocks = true;
    cfg.dc_op_gate_cycles = 5;  // short, explicit SETTLE
    Master m{cfg, std::make_unique<SimBackend>(make_models())};
    m.init();
    m.configure();
    // SETTLE pumps exactly dc_op_gate_cycles phase-locked cycles (Gating) before requesting OP,
    // so the first 5 steps are Gating and the settle is done only after the 5th.
    for (int c = 0; c < 5; ++c) {
        CHECK(m.bringup_step(/*drive_sync_faulted=*/false) == BringupStatus::Gating);
    }
    CHECK(m.bringup_step(/*drive_sync_faulted=*/false) != BringupStatus::Gating);  // OP requested after exactly 5
}

// #32.4: dc_sync0_shift_ns (the ecx_dcsync0 CyclShift) is a MasterConfig field that
// threads through configure() to the backend's arm_dc_sync call unchanged.
TEST("#32.4: dc_sync0_shift_ns threads through configure() to the backend arm call") {
    CHECK_EQ(MasterConfig{}.dc_sync0_shift_ns, std::int32_t{0});  // default: SYNC0 on the DC base

    MasterConfig cfg = make_config();
    cfg.use_distributed_clocks = true;
    cfg.dc_sync0_shift_ns = 12345;
    auto backend = std::make_unique<SimBackend>(make_models());
    SimBackend* raw = backend.get();
    Master m{cfg, std::move(backend)};
    m.init();
    m.configure();  // arms SYNC0 in PRE-OP (DC on) -> backend records the CyclShift it received
    CHECK_EQ(raw->configured_dc_sync0_shift_ns(), std::int32_t{12345});
}

TEST_MAIN()
