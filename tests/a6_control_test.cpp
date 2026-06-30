// Offline sim tests for #53 -- a6_validate's A6Control PP-absolute (--move-pos) and
// Profile-Velocity (--move-vel) energized modes, driven by a SimBackend-backed Master+Runner
// (no hardware). The HW bench is the behavioral backstop; these pin the safety-critical
// control logic: the DA-B mode-echo refuse, the PP rising-edge handshake + reached predicate,
// and the PV CiA402 Quick-Stop ramp-then-disable (both 0x605A regimes) + configure-time
// refusals (0x605A==2, 0x6085 readback, VEL guard).
//
// Reads of A6Control / SimBackend state happen AFTER the Runner is stopped+joined (the join is
// the happens-before edge), exactly like runner_test.

#include <chrono>
#include <cstdint>
#include <memory>
#include <thread>
#include <vector>

#include "ethercat/errors.hpp"
#include "ethercat/master.hpp"
#include "ethercat/runner.hpp"
#include "ethercat/sim_backend.hpp"
#include "test_harness.hpp"
#include "tools/a6_control.hpp"

using ethercat::Cia402Mode;
using ethercat::ConfigError;
using ethercat::Master;
using ethercat::MasterConfig;
using ethercat::Runner;
using ethercat::RunnerConfig;
using ethercat::RunnerPhase;
using ethercat::SimBackend;
using ethercat::SimSlaveModel;
using ethercat::SlaveConfig;
using ethercat::tools::A6Control;
using ethercat::tools::kZeroVelThresh;
using ethercat::tools::Options;
using ethercat::tools::Telemetry;

namespace {

// The a6_validate superset PDO map, byte-for-byte (build_a6_config):
//   RxPDO 0x1600: ctrl(16)@0, 0x607A(32)@2, 0x6081(32)@6, 0x60FF(32)@10   -> 14 B
//   TxPDO 0x1A00: 0x603F(16)@0, status(16)@2, 0x6061(8)@4, 0x6064(32)@5,
//                 0x606C(32)@9, 0x6077(16)@13                              -> 15 B
MasterConfig make_cfg(Cia402Mode mode) {
    MasterConfig cfg;
    cfg.ifname = "sim0";
    cfg.use_distributed_clocks = false;  // tests don't need DC; bring-up is faster
    cfg.dc_op_gate_cycles = 2;
    SlaveConfig a6;
    a6.slave_id = 1;
    a6.default_mode = mode;  // 0x6060 written at configure -> the sim echoes it on 0x6061
    a6.rxpdo.pdo_indices = {0x1600};
    a6.rxpdo.entries[0x1600] = {{0x6040, 0, 16}, {0x607A, 0, 32}, {0x6081, 0, 32}, {0x60FF, 0, 32}};
    a6.txpdo.pdo_indices = {0x1A00};
    a6.txpdo.entries[0x1A00] = {{0x603F, 0, 16}, {0x6041, 0, 16}, {0x6061, 0, 8}, {0x6064, 0, 32}, {0x606C, 0, 32}, {0x6077, 0, 16}};
    cfg.slaves.push_back(std::move(a6));
    return cfg;
}

// Sim model with offsets matching make_cfg(Cia402Mode::ProfileVelocity)'s map. A6-shaped: bit10 always set (#43).
SimSlaveModel make_model() {
    SimSlaveModel m;
    m.output_bytes = 14;
    m.input_bytes = 15;
    m.ctrlword_off = 0;
    m.target_off = 2;             // 0x607A
    m.profile_velocity_off = 6;   // 0x6081
    m.velocity_off = 10;          // 0x60FF
    m.statusword_off = 2;
    m.mode_display_off = 4;       // 0x6061
    m.actual_off = 5;             // 0x6064
    m.velocity_actual_off = 9;    // 0x606C
    m.fault_code_off = 0;         // 0x603F
    m.target_reached_always_set = true;
    return m;
}

RunnerConfig fast_rc(std::uint32_t teardown = 60) {
    RunnerConfig rc;
    rc.bringup_timeout = std::chrono::milliseconds(5000);
    rc.teardown_cycles = teardown;
    return rc;
}

// Start, wait for `ready(tel)` (or timeout), then request_stop and wait for Stopped. The
// Runner is scoped INSIDE so its dtor join is the happens-before edge before the caller reads
// sim/control state. Returns true if `ready` was observed before the timeout.
template <class Pred>
bool run_and_stop(Master& m, A6Control& ctrl, Telemetry& tel, RunnerConfig rc, Pred ready, int ready_ms = 3000, int settle_ms = 0) {
    bool saw_ready = false;
    {
        Runner r{m, rc};
        r.attach(1, ctrl);
        r.start();
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(ready_ms);
        while (std::chrono::steady_clock::now() < deadline) {
            if (ready(tel)) {
                saw_ready = true;
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        // settle: let the control finish its move/debounce (e.g. PP reached) before stopping.
        if (saw_ready && settle_ms > 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(settle_ms));
        }
        r.request_stop();
        const auto stop_deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(3000);
        while (r.status().phase != RunnerPhase::Stopped && std::chrono::steady_clock::now() < stop_deadline) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }  // ~Runner: join -> close
    return saw_ready;
}

}  // namespace

// --- exclusivity (CLI) --------------------------------------------------------------------
TEST("#53 exclusivity: two move flags -> conflict (mode_flag_count > 1)") {
    Options o;
    CHECK_EQ(ethercat::tools::mode_flag_count(o), 0);  // none
    o.move_vel = true;
    CHECK_EQ(ethercat::tools::mode_flag_count(o), 1);  // one is fine
    o.move_pos = true;
    CHECK(ethercat::tools::mode_flag_count(o) > 1);  // two -> the CLI rejects it
    Options p;
    p.move_pp = true;
    p.move_sine = true;
    CHECK(ethercat::tools::mode_flag_count(p) > 1);
}

// --- DA-B: mode-echo fail-closed ----------------------------------------------------------
TEST("#53 DA-B: 0x6061 != commanded -> REFUSE to enable (fail-closed)") {
    auto models = std::vector<SimSlaveModel>{make_model()};
    models[0].mode_echo_forced = true;
    models[0].mode_echo_value = 1;  // report PP(1) while we command PV(3) -> the A6 silently ignored the mode-set
    auto sim = std::make_unique<SimBackend>(models);
    Master m{make_cfg(Cia402Mode::ProfileVelocity), std::move(sim)};
    m.init();
    m.configure();
    Options o;
    o.enable = true;
    o.move_vel = true;
    o.pv_vel_cps = 5000;
    Telemetry tel;
    A6Control ctrl(o, tel, Cia402Mode::ProfileVelocity);
    // Never reaches enabled (refused at SwitchedOn); `ready` never fires -> times out fast.
    const bool enabled = run_and_stop(m, ctrl, tel, fast_rc(), [](Telemetry& t) { return t.enabled.load(); }, 800);
    CHECK(!enabled);                 // never energized
    CHECK(ctrl.mode_refused());      // the echo gate fired
    CHECK(!tel.enabled.load());      // and stayed un-energized
}

TEST("#53 DA-B: 0x6061 == commanded -> enable proceeds (the gate is not vacuous)") {
    auto sim = std::make_unique<SimBackend>(std::vector<SimSlaveModel>{make_model()});  // echoes effective_mode
    SimBackend* simp = sim.get();
    Master m{make_cfg(Cia402Mode::ProfileVelocity), std::move(sim)};
    m.init();
    m.configure();
    Options o;
    o.enable = true;
    o.move_vel = true;
    o.pv_vel_cps = 5000;
    Telemetry tel;
    A6Control ctrl(o, tel, Cia402Mode::ProfileVelocity);
    const bool enabled = run_and_stop(m, ctrl, tel, fast_rc(), [](Telemetry& t) { return t.enabled.load(); });
    CHECK(enabled);              // correct echo -> energized
    CHECK(!ctrl.mode_refused());
    CHECK(simp->received_target_velocity(1) == 5000);  // and streamed 0x60FF == VEL
}

// --- PV: Quick-Stop ramp-then-disable, both 0x605A regimes --------------------------------
TEST("#53 PV 0x605A=2 (primary): drive auto-disables at its zero, AFTER vel ramps below thresh") {
    auto models = std::vector<SimSlaveModel>{make_model()};
    models[0].quick_stop_option = 2;        // factory default: decel on 0x6085 -> auto SwitchOnDisabled
    models[0].quick_stop_decel_step = 1000;  // ramp 5000 -> 0 over ~5 QSA cycles (so it's a RAMP, not instant)
    auto sim = std::make_unique<SimBackend>(models);
    SimBackend* simp = sim.get();
    Master m{make_cfg(Cia402Mode::ProfileVelocity), std::move(sim)};
    m.init();
    m.configure();
    Options o;
    o.enable = true;
    o.move_vel = true;
    o.pv_vel_cps = 5000;
    Telemetry tel;
    A6Control ctrl(o, tel, Cia402Mode::ProfileVelocity);
    // Wait until it's enabled AND actually moving (vel >= VEL) before stopping.
    const bool moving = run_and_stop(m, ctrl, tel, fast_rc(200),
                                     [](Telemetry& t) { return t.enabled.load() && t.vel.load() >= 5000; });
    CHECK(moving);
    CHECK(simp->entered_qsa(1));                                   // it Quick-Stopped (not a torque-cut)
    CHECK(simp->velocity_at_qsa_exit(1) < kZeroVelThresh);        // de-energized only AFTER vel ~0 (ramp-then-disable)
    CHECK(simp->velocity_at_qsa_exit(1) >= 0);
}

TEST("#53 PV backstop (SYNTHETIC defensive coverage): control's cw->0x00 disables AFTER vel below thresh") {
    // SYNTHETIC, NOT a HW-reachable regime (architect-confirmed framing): a real 0x605A=2 drive
    // ALWAYS auto-disables at its zero, and the control REFUSES an actual 0x605A!=2 at configure
    // -- so a "reports 2 but never auto-disables" drive cannot occur on HW. This test fabricates
    // exactly that (report 0x605A=2 so configure passes; suppress the auto-disable) purely to get
    // CODE COVERAGE of the control's cw->0x00 backstop, which is DEFENSIVE dead-code-at-the-default
    // (DA: "keep it as uniform both-regimes code, defensive not load-bearing"). The reachable-on-HW
    // path is the PRIMARY test above (drive self-disables). A future reader: this does NOT imply a
    // =2 drive can fail to auto-disable.
    auto models = std::vector<SimSlaveModel>{make_model()};
    models[0].quick_stop_option = 2;
    models[0].quick_stop_suppress_auto_disable = true;
    models[0].quick_stop_decel_step = 1000;
    auto sim = std::make_unique<SimBackend>(models);
    SimBackend* simp = sim.get();
    Master m{make_cfg(Cia402Mode::ProfileVelocity), std::move(sim)};
    m.init();
    m.configure();
    Options o;
    o.enable = true;
    o.move_vel = true;
    o.pv_vel_cps = 5000;
    Telemetry tel;
    A6Control ctrl(o, tel, Cia402Mode::ProfileVelocity);
    const bool moving = run_and_stop(m, ctrl, tel, fast_rc(200),
                                     [](Telemetry& t) { return t.enabled.load() && t.vel.load() >= 5000; });
    CHECK(moving);
    CHECK(simp->entered_qsa(1));
    // The control disabled (cw->0x00) only after the velocity event -> vel at the QSA exit is sub-threshold.
    // *Baseline (declared): a fixed-cycle-counter disable would fire at residual speed -> this FAILS.*
    CHECK(simp->velocity_at_qsa_exit(1) < kZeroVelThresh);
}

// --- PV configure-time refusals (fail-closed, pre-energize) -------------------------------
TEST("#53 PV 0x605A != 2 -> configure REFUSES to energize (start throws)") {
    auto models = std::vector<SimSlaveModel>{make_model()};
    models[0].quick_stop_option = 0;  // coast (no decel!) -- breaks the controlled-stop premise
    auto sim = std::make_unique<SimBackend>(models);
    Master m{make_cfg(Cia402Mode::ProfileVelocity), std::move(sim)};
    m.init();
    m.configure();
    Options o;
    o.enable = true;
    o.move_vel = true;
    o.pv_vel_cps = 5000;
    Telemetry tel;
    A6Control ctrl(o, tel, Cia402Mode::ProfileVelocity);
    Runner r{m, fast_rc()};
    r.attach(1, ctrl);
    CHECK_THROWS(r.start(), ConfigError);  // on_configured's 0x605A assert throws -> no spawn
}

TEST("#53 PV 0x6085 readback = 0 / absent -> configure REFUSES (start throws)") {
    auto models = std::vector<SimSlaveModel>{make_model()};
    models[0].quick_stop_decel_echo_forced = true;
    models[0].quick_stop_decel_echo = 0;  // drive didn't accept the decel write
    auto sim = std::make_unique<SimBackend>(models);
    Master m{make_cfg(Cia402Mode::ProfileVelocity), std::move(sim)};
    m.init();
    m.configure();
    Options o;
    o.enable = true;
    o.move_vel = true;
    o.pv_vel_cps = 5000;
    Telemetry tel;
    A6Control ctrl(o, tel, Cia402Mode::ProfileVelocity);
    Runner r{m, fast_rc()};
    r.attach(1, ctrl);
    CHECK_THROWS(r.start(), ConfigError);
}

TEST("#53 PV 0x6085 CLAMPED -> the control uses the ECHOED value (DA-A)") {
    auto models = std::vector<SimSlaveModel>{make_model()};
    models[0].quick_stop_decel_echo_forced = true;
    models[0].quick_stop_decel_echo = 1'000'000;  // drive clamps our 6.5M down to 1.0M
    auto sim = std::make_unique<SimBackend>(models);
    Master m{make_cfg(Cia402Mode::ProfileVelocity), std::move(sim)};
    m.init();
    m.configure();
    Options o;
    o.enable = true;
    o.move_vel = true;
    o.pv_vel_cps = 5000;  // well within the clamped guard (1.0M * ~1.9s)
    Telemetry tel;
    A6Control ctrl(o, tel, Cia402Mode::ProfileVelocity);
    const bool moving = run_and_stop(m, ctrl, tel, fast_rc(200), [](Telemetry& t) { return t.enabled.load(); });
    CHECK(moving);
    CHECK_EQ(ctrl.qs_decel_echoed(), std::uint32_t{1'000'000});  // downstream math uses the echoed (clamped) value
}

TEST("#53 PV VEL guard: VEL too large for the window -> configure REFUSES (start throws)") {
    auto sim = std::make_unique<SimBackend>(std::vector<SimSlaveModel>{make_model()});  // 0x6085 echoes the written 6.5M
    Master m{make_cfg(Cia402Mode::ProfileVelocity), std::move(sim)};
    m.init();
    m.configure();
    Options o;
    o.enable = true;
    o.move_vel = true;
    o.pv_vel_cps = 2'000'000'000;  // 2e9 counts/s >> 0x6085 * (W - margin) ~ 12.4e6 -> can't ramp in time
    Telemetry tel;
    A6Control ctrl(o, tel, Cia402Mode::ProfileVelocity);
    Runner r{m, fast_rc()};
    r.attach(1, ctrl);
    CHECK_THROWS(r.start(), ConfigError);
}

// --- PP absolute move-to ------------------------------------------------------------------
TEST("#53 PP move-to: reaches target within tol, single bit4 edge, zero-jump-free") {
    auto sim = std::make_unique<SimBackend>(std::vector<SimSlaveModel>{make_model()});
    SimBackend* simp = sim.get();
    Master m{make_cfg(Cia402Mode::ProfilePosition), std::move(sim)};
    m.init();
    m.configure();
    Options o;
    o.enable = true;
    o.move_pos = true;
    o.pos_target = 20000;   // absolute counts
    o.pp_vel_cps = 4000;    // 0x6081 chase speed (sim chases at the wire profile velocity)
    o.pos_tol = 300;
    Telemetry tel;
    A6Control ctrl(o, tel, Cia402Mode::ProfilePosition);
    // Wait until settled (within tol AND ~0 velocity), then a settle window lets the reached
    // debounce latch move_done before we stop.
    const bool reached = run_and_stop(
        m, ctrl, tel, fast_rc(),
        [](Telemetry& t) { return std::abs(t.pos.load() - 20000) <= 300 && std::abs(t.vel.load()) < kZeroVelThresh; }, 3000,
        /*settle_ms=*/40);
    CHECK(reached);
    CHECK(ctrl.move_done());                  // reached predicate fired (actual-vs-target + vel~0, NOT bit10)
    CHECK(std::abs(tel.pos.load() - 20000) <= 300);  // settled within tol (no overshoot/jump past it)
    CHECK_EQ(ctrl.bit4_edges(), 1);           // exactly ONE new-setpoint rising edge (DA-I)
    (void)simp;
}

TEST_MAIN()
