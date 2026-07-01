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

// --- #47-P3b sub-step 5 (P3c): the canonical runtime PP->PV mode-switch (§6), through A6Control ---
// Config variant: 0x6060 (mode-of-op, i8) appended to the RxPDO so the switch can write it cyclically.
namespace {
MasterConfig make_cfg_switch() {
    MasterConfig cfg = make_cfg(Cia402Mode::ProfilePosition);
    cfg.slaves[0].rxpdo.entries[0x1600].push_back({0x6060, 0, 8});  // #47-P3b 5a: mode-of-op in RxPDO (14->15 B)
    return cfg;
}
SimSlaveModel make_model_switch() {
    SimSlaveModel m = make_model();
    m.output_bytes = 15;     // + 0x6060 @ 14
    m.mode_of_op_off = 14;   // consume 0x6060 from the RxPDO -> effective_mode (echoed on 0x6061)
    return m;
}
}  // namespace

TEST("#47-P3c REGRESSION: a 0x6060-MAPPED enable ladder SEEDS the mode from cycle 0 -> reaches OperationEnabled") {
    // WIRE-CONFIRMED bug (tcpdump, energized P3c run): 0x6060 was only written INSIDE drive_operational_
    // (post-OperationEnabled), so with 0x6060 RxPDO-mapped the drive sat at mode 0 through the entire
    // enable ladder -- the A6 follows the PDO mode over the SDO default once cycling -> 0x6061 != commanded
    // PP -> the mode-echo gate SILENTLY request_stop'd (cw stuck at 0x07, the move never happened). The fix
    // seeds 0x6060 = commanded mode from cycle 0 (before the gate). This test asserts the ladder REACHES
    // OperationEnabled with the mode seeded on the wire. FAILS on the pre-fix policy (never energizes);
    // relies on the faithful sim (0x6061 FOLLOWS the PDO mode) -- together they make the bug reproducible.
    auto sim = std::make_unique<SimBackend>(std::vector<SimSlaveModel>{make_model_switch()});
    SimBackend* simp = sim.get();
    Master m{make_cfg_switch(), std::move(sim)};
    m.init();
    m.configure();
    Options o;
    o.enable = true;
    o.move_pos = true;
    o.pos_target = 20000;
    o.pp_vel_cps = 4000;
    o.pos_tol = 300;
    Telemetry tel;
    A6Control ctrl(o, tel, Cia402Mode::ProfilePosition);
    const bool energized = run_and_stop(m, ctrl, tel, fast_rc(), [](Telemetry& t) { return t.enabled.load(); }, 3000);  // generous deadline (siblings use 3000) -- bring-up is ~tens of ms; headroom vs scheduler hiccups
    CHECK(energized);                                                 // reached OperationEnabled (cw hit 0x0F) -- the fix
    CHECK(!ctrl.mode_refused());                                      // the mode-echo gate did NOT refuse
    CHECK(simp->effective_mode(1) == Cia402Mode::ProfilePosition);    // 0x6060 seeded to PP on the wire through the ladder
    CHECK_EQ(tel.mode.load(), 1);                                     // 0x6061 echoes PP (the drive adopted the commanded mode)
}

TEST("#47-P3b P3c: runtime PP->PV mode-switch confirms (0x6061=PV) + gives up safe on silent-mismatch") {
    // SUCCESS: PP move -> reached -> §6 switch (stop-first -> write 0x6060=PV -> 0x6061 echoes PV -> confirm) -> jog.
    auto sim = std::make_unique<SimBackend>(std::vector<SimSlaveModel>{make_model_switch()});
    SimBackend* simp = sim.get();
    Master m{make_cfg_switch(), std::move(sim)};
    m.init();
    m.configure();
    Options o;
    o.enable = true;
    o.move_pos = true;
    o.pos_target = 20000;
    o.pp_vel_cps = 4000;
    o.pos_tol = 300;
    o.then_jog_vel = true;  // after the PP move reaches -> SWITCH to PV and jog
    o.pv_vel_cps = 3000;
    Telemetry tel;
    A6Control ctrl(o, tel, Cia402Mode::ProfilePosition);
    // Ready = 0x6061 (published tel.mode) echoes PV(3): the canonical switch confirmed on the wire.
    const bool confirmed = run_and_stop(m, ctrl, tel, fast_rc(), [](Telemetry& t) { return t.mode.load() == 3; }, 3000, /*settle_ms=*/20);
    CHECK(confirmed);                    // the §6 switch completed: 0x6061 confirmed PV
    CHECK(ctrl.switched_to_vel());       // A6Control latched PP-reached -> requested the switch
    CHECK_EQ(ctrl.confirmed_mode(), 3);  // policy's last 0x6061 read == PV(3)
    CHECK(!ctrl.mode_switch_failed());   // clean success, no give-up
    // DA NO-LUNGE (§6 step 2 seed): the switch must NOT jump the axis. The over-mapped 0x607A tracks the
    // drive's ACTUAL every cycle (the PV mirror), so it is NEVER a stale target that would lunge on the
    // switch -- |0x607A - actual| stays within ONE control cycle's jog motion (the drive is jogging at
    // pv_vel_cps counts/cycle here). A stale/coast regression parks 0x607A tens of thousands of counts
    // from actual -> this bound (2 cycles' jog) catches it; the 1-cycle skew is deterministic (post-join).
    CHECK(std::abs(simp->received_target_position(1) - tel.pos.load()) < 2 * o.pv_vel_cps);
}

TEST("#47-P3b P3c: mode-switch FAILURE -- 0x6061 never echoes (silent-mismatch, #45) -> 'mode-switch failed' + SAFE") {
    // The drive SILENTLY IGNORES the mode-write (0x6061 stays PP even after 0x6060=PV) -- the #45
    // shape. The §6 confirm never fires -> within T_switch the switch FAILS -> mode_switch_failed +
    // the SAFE disposition: give up, revert to the confirmed (PP) mode, stay ENERGIZED at rest (NOT
    // a de-energize, NOT a lunge, NOT a retry storm).
    SimSlaveModel model = make_model_switch();
    model.mode_echo_forced = true;  // force 0x6061 = the OLD mode regardless of 0x6060 (silent-ignore)
    model.mode_echo_value = 1;      // always echo PP(1)
    auto sim = std::make_unique<SimBackend>(std::vector<SimSlaveModel>{model});
    SimBackend* simp = sim.get();
    Master m{make_cfg_switch(), std::move(sim)};
    m.init();
    m.configure();
    Options o;
    o.enable = true;
    o.move_pos = true;
    o.pos_target = 20000;
    o.pp_vel_cps = 4000;
    o.pos_tol = 300;
    o.then_jog_vel = true;
    o.pv_vel_cps = 3000;
    Telemetry tel;
    A6Control ctrl(o, tel, Cia402Mode::ProfilePosition);
    // Run to a timeout (0x6061 never becomes PV -> the switch can't confirm; give it time to reach PP,
    // attempt the switch, and hit the T_switch settle timeout -> fail), then stop + inspect (post-join).
    run_and_stop(m, ctrl, tel, fast_rc(), [](Telemetry&) { return false; }, 800);
    CHECK(ctrl.switched_to_vel() == false);   // gave up: reverted to the confirmed mode (not stuck requesting PV)
    CHECK(ctrl.mode_switch_failed());         // the switch reported failure (silent-mismatch)
    CHECK_EQ(ctrl.confirmed_mode(), 1);       // still PP(1) -- never entered PV
    CHECK_EQ(tel.mode.load(), 1);             // 0x6061 held PP the whole time (energized, no mode change)
    // DA NO-LUNGE: the FAILED switch must not jump the axis either. Direct checks (post-join): the drive
    // was NEVER commanded a non-zero velocity (0x60FF stayed at the seeded 0 -- no velocity lunge), the
    // last-written 0x607A tracks actual (no stale positional target), and the axis HELD its reached
    // position (~pos_target) -- it did not lunge/drift while the switch failed + reverted.
    CHECK_EQ(simp->received_target_velocity(1), 0);
    CHECK(std::abs(simp->received_target_position(1) - tel.pos.load()) < 200);
    CHECK(std::abs(tel.pos.load() - o.pos_target) < o.pos_tol);
}

// --- #47-P3b M56S: THE GENERICITY PAYOFF -- a SECOND device via a PROFILE SWAP, no policy/consumer code.
// The SAME A6Control + SAME generic Cia402Policy drive a DIFFERENT servo (M56S: its runtime mode-switch
// "takes time" -> 0x6061 lags the 0x6060 write). Only the DeviceProfile changes (longer T_switch). ---
namespace {
Options make_switch_opts() {
    Options o;
    o.enable = true;
    o.move_pos = true;
    o.pos_target = 20000;
    o.pp_vel_cps = 4000;
    o.pos_tol = 300;
    o.then_jog_vel = true;  // PP move -> reach -> §6 switch to PV -> jog
    o.pv_vel_cps = 3000;
    return o;
}
}  // namespace

TEST("#47-P3b M56S: SECOND device via PROFILE SWAP -- a SLOW runtime mode-switch confirms under the M56S T_switch") {
    // The M56S transition "takes time": 0x6061 lags the 0x6060=PV write by 40 cycles. The SAME generic
    // policy + A6Control confirm the switch because the M56S PROFILE's longer T_switch (60) covers the lag.
    SimSlaveModel model = make_model_switch();
    model.mode_switch_latency = 40;  // M56S: a RUNTIME mode change applies 40 cycles after the 0x6060 write
    auto sim = std::make_unique<SimBackend>(std::vector<SimSlaveModel>{model});
    Master m{make_cfg_switch(), std::move(sim)};
    m.init();
    m.configure();
    Options o = make_switch_opts();
    Telemetry tel;
    // THE profile swap -- the ONLY per-device code. Everything else is byte-identical to the A6 path.
    A6Control ctrl(o, tel, Cia402Mode::ProfilePosition, A6Control::make_m56s_profile(o));
    const bool confirmed = run_and_stop(m, ctrl, tel, fast_rc(), [](Telemetry& t) { return t.mode.load() == 3; }, 3000, /*settle_ms=*/20);
    CHECK(confirmed);                    // slow switch confirmed 0x6061=PV within the M56S T_switch
    CHECK(ctrl.switched_to_vel());
    CHECK_EQ(ctrl.confirmed_mode(), 3);  // PV(3)
    CHECK(!ctrl.mode_switch_failed());   // no give-up -- the profile's window was sized for the slow device
}

TEST("#47-P3b M56S: the T_switch knob is LOAD-BEARING -- the SAME slow device FAILS under a too-short window") {
    // Same slow-M56S sim (40-cycle lag), but a profile whose T_switch (20) is SHORTER than the lag ->
    // the confirm times out before 0x6061 echoes PV -> mode_switch_failed + SAFE. Proves the generic
    // policy adapts to the device by DATA ALONE: the one differing number decides success vs failure.
    SimSlaveModel model = make_model_switch();
    model.mode_switch_latency = 40;
    auto sim = std::make_unique<SimBackend>(std::vector<SimSlaveModel>{model});
    Master m{make_cfg_switch(), std::move(sim)};
    m.init();
    m.configure();
    Options o = make_switch_opts();
    ethercat::DeviceProfile too_short = A6Control::make_m56s_profile(o);
    too_short.mode_switch_settle_cycles = 20;  // < the 40-cycle device lag -> the switch cannot confirm in time
    Telemetry tel;
    A6Control ctrl(o, tel, Cia402Mode::ProfilePosition, too_short);
    run_and_stop(m, ctrl, tel, fast_rc(), [](Telemetry&) { return false; }, 800);
    CHECK(ctrl.mode_switch_failed());        // timed out before the slow 0x6061 echo -> failed
    CHECK(ctrl.switched_to_vel() == false);  // gave up: reverted to the confirmed (PP) mode
    CHECK_EQ(ctrl.confirmed_mode(), 1);      // still PP(1)
}

TEST("#47-P3b M56S: errors-on-unsupported-mode -> mode-switch FAILED + SAFE (the drive-error failure shape)") {
    // The OTHER §6-step-5 failure shape: the M56S REJECTS an unsupported mode (never applies it -> 0x6061
    // never echoes PV), distinct from the A6 SILENT-ignore. The generic policy catches BOTH the same way:
    // no confirm within T_switch -> mode_switch_failed -> give up, stay energized in the confirmed mode.
    SimSlaveModel model = make_model_switch();
    model.unsupported_mode = Cia402Mode::ProfileVelocity;  // M56S: PV is not supported here -> rejected
    auto sim = std::make_unique<SimBackend>(std::vector<SimSlaveModel>{model});
    Master m{make_cfg_switch(), std::move(sim)};
    m.init();
    m.configure();
    Options o = make_switch_opts();
    Telemetry tel;
    A6Control ctrl(o, tel, Cia402Mode::ProfilePosition, A6Control::make_m56s_profile(o));
    run_and_stop(m, ctrl, tel, fast_rc(), [](Telemetry&) { return false; }, 800);
    CHECK(ctrl.mode_switch_failed());        // rejected mode never confirmed -> failed (error shape)
    CHECK(ctrl.switched_to_vel() == false);  // gave up SAFE
    CHECK_EQ(ctrl.confirmed_mode(), 1);      // still PP(1) -- energized, no de-energize/lunge
    CHECK_EQ(tel.mode.load(), 1);            // 0x6061 held PP the whole time
}

TEST_MAIN()
