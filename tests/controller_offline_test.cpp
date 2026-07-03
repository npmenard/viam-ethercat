// Offline integration test: ServoController driving a SimBackend through the
// full RT loop -- no Viam SDK, no hardware. require_realtime=false so the RT
// thread runs SCHED_OTHER (CI has no CAP_SYS_NICE).

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "ethercat/backend.hpp"
#include "ethercat/errors.hpp"
#include "ethercat/sim_backend.hpp"
#include "test_harness.hpp"
#include "viam/lib/motion_profile.hpp"
#include "viam/lib/servo_config.hpp"
#include "viam/lib/servo_controller.hpp"

using ethercat::BusError;
using ethercat::ConfigError;
using ethercat::EcatBackend;
using ethercat::PdoEntry;
using ethercat::SimBackend;
using ethercat::SimSlaveModel;
using ethercat::servo::ControlMode;
using ethercat::servo::convert_sdo_monitor;
using ethercat::servo::SdoMonitor;
using ethercat::servo::SdoScaleKind;
using ethercat::servo::SdoValueType;
using ethercat::servo::ServoConfig;
using ethercat::servo::ServoController;

namespace {

constexpr double kCountsPerRev = 131072.0;

ServoConfig make_config(ControlMode mode, bool feedback = false) {
    ServoConfig c;
    c.ifname = "sim0";
    c.slave_id = 1;
    c.mode = mode;
    c.rxpdo.pdo_indices = {0x1600};
    // Authoritative A6 maps per mode: PP = ctrl + target position + profile velocity
    // (10 B); PV = ctrl + target velocity (6 B).
    // #47-P3b sub-step 5: 0x6060 (mode-of-operation, i8) is RxPDO-mapped so the runtime mode-switch
    // can write it cyclically (SDO in step() is illegal). Appended last so existing field offsets
    // are unchanged.
    if (mode == ControlMode::ProfileVelocity) {
        c.rxpdo.entries[0x1600] = {PdoEntry{0x6040, 0, 16}, PdoEntry{0x60FF, 0, 32}, PdoEntry{0x6060, 0, 8}};
    } else {
        c.rxpdo.entries[0x1600] = {PdoEntry{0x6040, 0, 16}, PdoEntry{0x607A, 0, 32}, PdoEntry{0x6081, 0, 32}, PdoEntry{0x6060, 0, 8}};
    }
    c.txpdo.pdo_indices = {0x1A00};
    // #16 feedback variant: add 0x603F (fault code) + 0x606C (velocity actual) to the
    // TxPDO so the controller resolves + reads them. status@0, actual@2, 603F@6, 606C@8.
    if (feedback) {
        c.txpdo.entries[0x1A00] = {PdoEntry{0x6041, 0, 16}, PdoEntry{0x6064, 0, 32}, PdoEntry{0x603F, 0, 16}, PdoEntry{0x606C, 0, 32}};
        c.fault_code_labels = {{0x8700, "Er74.1 / no SYNC0"}};
    } else {
        c.txpdo.entries[0x1A00] = {PdoEntry{0x6041, 0, 16}, PdoEntry{0x6064, 0, 32}};
    }
    c.max_motor_speed_rpm = 3000.0;
    c.motor_rated_current_amps = 2.5;
    c.gear_ratio = 1.0;
    c.counts_per_rev = kCountsPerRev;
    c.position_tolerance_counts = 20;
    c.velocity_threshold = 1'000'000'000;  // lenient: move-complete is essentially |dpos| <= tol
    c.target_loop_rate_hz = 1000;
    c.require_realtime = false;  // CI has no CAP_SYS_NICE
    c.command_queue_capacity = 64;
    c.handshake_timeout_cycles = 1000;
    c.quick_stop_decel = 500'000;  // #47-P3b: enable the policy's quick-stop configure (0x605A assert + 0x6085 write/readback)
    return c;
}

SimSlaveModel make_model(ControlMode mode, bool feedback = false) {
    SimSlaveModel m;
    m.input_bytes = 6;  // status@0, actual@2
    m.ctrlword_off = 0;
    m.statusword_off = 0;
    m.actual_off = 2;
    m.counts_per_step = 50'000;  // PP fallback speed (only used if 0x6081 is NOT mapped)
    // #16 feedback de-mask: drive the 0x603F/0x606C TxPDO offsets so the read path is
    // exercised offline (mirrors make_config(feedback): 603F@6, 606C@8, input 12 B).
    if (feedback) {
        m.input_bytes = 12;
        m.fault_code_off = 6;
        m.velocity_actual_off = 8;
    }
    if (mode == ControlMode::ProfileVelocity) {
        m.mode = ethercat::Cia402Mode::ProfileVelocity;
        m.output_bytes = 7;  // ctrl@0, target velocity@2, mode-of-op@6 (#47-P3b sub-step 5)
        m.velocity_off = 2;
        m.mode_of_op_off = 6;
    } else {
        m.mode = ethercat::Cia402Mode::ProfilePosition;
        m.output_bytes = 11;  // ctrl@0, target@2, profile velocity@6, mode-of-op@10 (#47-P3b sub-step 5)
        m.target_off = 2;
        m.profile_velocity_off = 6;  // de-masked: PP chases at the 0x6081 the RT loop writes
        m.mode_of_op_off = 10;
    }
    return m;
}

ServoController::BackendFactory sim_factory(ControlMode mode, SimBackend** out_ptr, bool feedback = false) {
    return [mode, out_ptr, feedback] {
        auto be = std::make_unique<SimBackend>(std::vector<SimSlaveModel>{make_model(mode, feedback)});
        if (out_ptr != nullptr) {
            *out_ptr = be.get();
        }
        return std::unique_ptr<EcatBackend>(std::move(be));
    };
}

template <class Pred>
bool wait_until(Pred pred, std::chrono::milliseconds timeout) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        if (pred()) {
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return pred();
}

}  // namespace

TEST("ServoController(PP): start() drives the lifecycle ladder to powered") {
    ServoController ctrl{make_config(ControlMode::ProfilePosition), sim_factory(ControlMode::ProfilePosition, nullptr)};
    ctrl.start();
    CHECK(wait_until([&] { return ctrl.is_powered(); }, std::chrono::milliseconds(500)));
    CHECK(!ctrl.is_disconnected());
}

TEST("ServoController(PP): go_to propagates + converges; position reads back") {
    ServoController ctrl{make_config(ControlMode::ProfilePosition), sim_factory(ControlMode::ProfilePosition, nullptr)};
    ctrl.start();
    CHECK(wait_until([&] { return ctrl.is_powered(); }, std::chrono::milliseconds(500)));

    ctrl.go_to(1000.0, 2.0);  // blocks until move-complete (or timeout)
    CHECK(!ctrl.is_moving());
    CHECK(std::abs(ctrl.position_revs() - 2.0) < 0.01);  // within tolerance, in revs
}

TEST("#59: reached/is_moving is noise-robust (position-delta) — omitting tolerance+velocity_threshold completes UNDER encoder noise") {
    // THE latent bug: position_tolerance_counts=0 AND velocity_threshold=0 fed the OLD predicate
    // (|actual-target|==0 && |vel|==0) -> under encoder noise a move NEVER completes + is_moving sticks
    // true. Fix: velocity_threshold=0 -> position-delta stability; position_tolerance_counts=0 -> DEFAULT
    // counts_per_rev/720 (=182 @ 131072). NON-VACUOUS: report_noise=60 makes 0x606C swing +/-120 at rest
    // (the old |vel|<=0 gate would hang -> go_to throws MoveStalled/timeout), while the position RANGE
    // (120) stays < 182 so the position-delta predicate correctly reports STOPPED.
    SimSlaveModel m = make_model(ControlMode::ProfilePosition, /*feedback=*/true);
    m.report_noise = 60;  // +/-60 counts encoder jitter -> 0x606C +/-120 nonzero at rest; range 120 < cpr/720 (182)
    SimBackend* sim = nullptr;
    ServoController::BackendFactory factory = [m, &sim] {
        auto be = std::make_unique<SimBackend>(std::vector<SimSlaveModel>{m});
        sim = be.get();
        return std::unique_ptr<EcatBackend>(std::move(be));
    };
    ServoConfig cfg = make_config(ControlMode::ProfilePosition, /*feedback=*/true);
    cfg.position_tolerance_counts = 0;  // OMIT -> validated() defaults to counts_per_rev/720 (0.5 deg)
    cfg.velocity_threshold = 0;         // OMIT -> position-delta method (kills the broken exact-|vel|<=0 gate)
    // #15: move_timeout_ms knob removed -- the RT no-progress watchdog (4x stall) bounds a stuck/regressed move.
    ServoController ctrl{cfg, factory};
    ctrl.start();
    CHECK(wait_until([&] { return ctrl.is_powered(); }, std::chrono::milliseconds(500)));
    ctrl.go_to(1000.0, 1.0);  // COMPLETES via position-delta despite the velocity noise (old predicate: hangs -> throws)
    CHECK(std::abs(ctrl.position_revs() - 1.0) < 0.01);
    CHECK(!ctrl.is_moving());  // at rest under noise -> position stable -> NOT moving (was stuck-true pre-#59)
    (void)sim;
}

TEST("#67: a frozen drive FAR from target must NOT report reached (the |actual-target|<=tol guard)") {
    // GAP (DA, run-proven): the move-complete predicate at servo_controller.cpp gates on
    //   at_target = |actual - target| <= tol  AND  stopped
    // Dead-code the position factor (at_target = stopped) and the FULL suite still passes --
    // a false-reached (reporting a move DONE while still far from target) goes uncaught. The
    // go_to tests assert only the FINAL position; the stall watchdog trips regardless of
    // at_target. Neither isolates "did NOT report reached while far." This test does.
    //
    // Frozen drive: profile_velocity_off=-1 => the sim does NOT read 0x6081 and falls back to
    // counts_per_step, which is 0 => the PP chase step is 0 => `actual` never leaves its start
    // while go_to commands a FAR target. WITH the guard, |Δ| >> tol forever, so the move NEVER
    // reports reached and go_to throws (stall / move-timeout). WITHOUT the guard, the frozen
    // (hence position-STABLE) drive reads at_target=true the instant the setpoint handshake
    // idles => completed_generation is set => go_to RETURNS SUCCESS -- the false-reached bug.
    // So CHECK_THROWS is the exact, sole distinguisher of the guard.
    //
    // DETERMINISTIC BY CONSTRUCTION (DA caveat: the naive thread+is_moving-poll shape aborts
    // NONDETERMINISTICALLY under the mutation -- a completion-timing UB). This test is
    // SINGLE-THREADED: it asserts on go_to's own OUTCOME (throw vs. return) on the calling
    // thread, leaving NO background poll thread joinable at teardown. move_timeout_ms bounds
    // BOTH the stall limit and the wait, so clean code throws within ~1.5 s while the mutant
    // returns within a few tens of cycles (handshake latency) -- a wide, race-free margin.
    SimSlaveModel m = make_model(ControlMode::ProfilePosition, /*feedback=*/true);
    m.profile_velocity_off = -1;  // don't read 0x6081 -> fall back to counts_per_step ...
    m.counts_per_step = 0;        // ... which is 0 -> the drive is FROZEN (never chases the target)
    SimBackend* sim = nullptr;
    ServoController::BackendFactory factory = [m, &sim] {
        auto be = std::make_unique<SimBackend>(std::vector<SimSlaveModel>{m});
        sim = be.get();
        return std::unique_ptr<EcatBackend>(std::move(be));
    };
    ServoConfig cfg = make_config(ControlMode::ProfilePosition, /*feedback=*/true);
    // #15: move_timeout_ms knob removed -- the RT no-progress watchdog (4x stall) bounds a stuck/regressed move.
    ServoController ctrl{cfg, factory};
    ctrl.start();
    CHECK(wait_until([&] { return ctrl.is_powered(); }, std::chrono::milliseconds(500)));
    // FAR target (5 rev = 655360 counts; tol = 20): the frozen drive stays put -> |Δ| >> tol.
    // Correct: NEVER report reached -> go_to throws. False-reached (dropped guard): go_to returns.
    CHECK_THROWS(ctrl.go_to(1000.0, 5.0), BusError);
    (void)sim;
}

TEST("ServoController: mode guards reject the wrong API with clear errors") {
    {
        ServoController pp{make_config(ControlMode::ProfilePosition), sim_factory(ControlMode::ProfilePosition, nullptr)};
        pp.start();
        CHECK(wait_until([&] { return pp.is_powered(); }, std::chrono::milliseconds(500)));
        CHECK_THROWS_MSG(pp.set_rpm(100.0), ConfigError, "Profile Velocity");
    }
    {
        ServoController pv{make_config(ControlMode::ProfileVelocity), sim_factory(ControlMode::ProfileVelocity, nullptr)};
        pv.start();
        CHECK(wait_until([&] { return pv.is_powered(); }, std::chrono::milliseconds(500)));
        CHECK_THROWS_MSG(pv.go_to(100.0, 1.0), ConfigError, "Profile Position");
    }
}

TEST("ServoController: a wrong-mode rejection is Tier-2 (stays powered, next command works)") {
    // The most common Tier-2 path: a bad API call (set_rpm in PP) must fail the
    // CALL but NOT fault/de-power the drive -- a subsequent valid command then
    // succeeds with NO fault_reset.
    ServoController ctrl{make_config(ControlMode::ProfilePosition), sim_factory(ControlMode::ProfilePosition, nullptr)};
    ctrl.start();
    CHECK(wait_until([&] { return ctrl.is_powered(); }, std::chrono::milliseconds(500)));

    CHECK_THROWS_MSG(ctrl.set_rpm(100.0), ConfigError, "Profile Velocity");  // wrong-mode reject
    CHECK(ctrl.is_powered());                                                // still energized -- NOT faulted
    CHECK(!ctrl.is_disconnected());

    ctrl.go_to(1000.0, 1.0);  // a valid command then works with no fault_reset
    CHECK(std::abs(ctrl.position_revs() - 1.0) < 0.01);
}

TEST("ServoController: reset_zero offsets the reported position") {
    ServoController ctrl{make_config(ControlMode::ProfilePosition), sim_factory(ControlMode::ProfilePosition, nullptr)};
    ctrl.start();
    CHECK(wait_until([&] { return ctrl.is_powered(); }, std::chrono::milliseconds(500)));
    ctrl.go_to(1000.0, 3.0);
    ctrl.set_zero();
    CHECK(std::abs(ctrl.position_revs()) < 0.01);  // now reads ~0 at the current actual
}

TEST("ServoController(PP): go_to is absolute in the ZEROED frame after reset_zero") {
    // Frame contract: get_position()/position_revs() report the zeroed frame, so
    // go_to must too -- go_to(X) after reset_zero must land at position_revs()==X.
    // (Regression: go_to used to target the raw frame, ignoring zero_offset.)
    ServoController ctrl{make_config(ControlMode::ProfilePosition), sim_factory(ControlMode::ProfilePosition, nullptr)};
    ctrl.start();
    CHECK(wait_until([&] { return ctrl.is_powered(); }, std::chrono::milliseconds(500)));

    ctrl.go_to(1000.0, 3.0);  // move to a NONZERO raw position
    ctrl.set_zero();          // zero HERE -> position_revs()==0 at raw 3 revs
    CHECK(std::abs(ctrl.position_revs()) < 0.01);

    ctrl.go_to(1000.0, 1.0);                             // absolute +1 rev in the zeroed frame
    CHECK(std::abs(ctrl.position_revs() - 1.0) < 0.01);  // lands at zeroed 1.0 (raw 4 revs)
}

TEST("ServoController(PP): the commanded rpm is written to profile velocity (0x6081)") {
    // Hardware gap (same class as 0x6060): if 0x6081 is mapped but the RT loop never
    // writes it, the move runs at the drive's DEFAULT speed and the rpm is ignored.
    // The shared PP map carries 0x6081; assert the device received the commanded value.
    SimBackend* sim = nullptr;
    // #47-P3b R1: the PP move speed (0x6081) is now ALSO clamped to the stop-window budget. Give this
    // test enough window that 600 rpm (=1.31e6 counts/s) stays UNDER budget, so it still verifies the
    // exact commanded value reaches 0x6081 (not the default). (The clamp itself is covered by the PV
    // guard test + the go_to-clamp test below.)
    ServoConfig cfg = make_config(ControlMode::ProfilePosition);
    cfg.controlled_stop_window_ms = 3000;  // budget = 500000 x (3.0 - 0.05) = 1.475e6 > 1.31e6 -> no clamp
    ServoController ctrl{cfg, sim_factory(ControlMode::ProfilePosition, &sim)};
    ctrl.start();
    CHECK(wait_until([&] { return ctrl.is_powered(); }, std::chrono::milliseconds(500)));

    ctrl.go_to(600.0, 1.0);  // 600 rpm
    CHECK(sim != nullptr);
    const std::int32_t expected = ethercat::servo::rpm_to_device_velocity(600.0, kCountsPerRev, 1.0);
    // received_profile_velocity() reads a NON-ATOMIC int the RT thread writes (sim_backend.hpp:
    // "call only AFTER the controller is stopped/joined"). stop() joins the RT thread first, so
    // the read is race-free (the last-written 0x6081 persists). The value reached the drive
    // during the move above; this just observes it without racing the writer.
    ctrl.stop();
    CHECK_EQ(sim->received_profile_velocity(1), std::abs(expected));  // rpm reached the drive, not 0
}

TEST("ServoController(PP): go_to move speed is clamped to the stop-window budget (#47-P3b R1)") {
    // The PP move speed (0x6081) is a per-velocity-setpoint hazard IDENTICAL to PV: a fast go_to
    // lifecycle-stopped mid-move would still be moving when close() de-energizes = torque-cut. So
    // go_to's rpm is clamped to the SAME stop-window budget as set_rpm (architect FLAG 2).
    SimBackend* sim = nullptr;
    ServoConfig cfg = make_config(ControlMode::ProfilePosition);
    cfg.quick_stop_decel = 100'000;
    cfg.controlled_stop_window_ms = 100;  // budget = 100000 x (0.100 - 0.050) = 5000 counts/s
    ServoController ctrl{cfg, sim_factory(ControlMode::ProfilePosition, &sim)};
    ctrl.start();
    CHECK(wait_until([&] { return ctrl.is_powered(); }, std::chrono::milliseconds(500)));
    ctrl.go_to(600.0, 0.02);  // 600 rpm = 1.31e6 counts/s requested, FAR above the 5000 budget; small move
    CHECK(sim != nullptr);
    ctrl.stop();
    CHECK_EQ(sim->received_profile_velocity(1), 5000);  // 0x6081 CLAMPED to budget, not the 1.31e6 request
}

TEST("ServoController(PP): go_for stays relative regardless of the zero") {
    // go_for is a RELATIVE move -- frame-agnostic. Guards against go_for routing
    // through the (now zero-offset-aware) go_to and double-shifting.
    ServoController ctrl{make_config(ControlMode::ProfilePosition), sim_factory(ControlMode::ProfilePosition, nullptr)};
    ctrl.start();
    CHECK(wait_until([&] { return ctrl.is_powered(); }, std::chrono::milliseconds(500)));

    ctrl.go_to(1000.0, 2.0);                             // raw 2 revs
    ctrl.set_zero();                                     // zeroed 0 at raw 2 revs
    ctrl.go_for(1000.0, 1.0);                            // relative +1 rev
    CHECK(std::abs(ctrl.position_revs() - 1.0) < 0.01);  // 0 + 1 = 1 (NOT double-shifted to 3)
}

TEST("ServoController: after stop() the fail-safe reports not-powered/disconnected") {
    ServoController ctrl{make_config(ControlMode::ProfilePosition), sim_factory(ControlMode::ProfilePosition, nullptr)};
    ctrl.start();
    CHECK(wait_until([&] { return ctrl.is_powered(); }, std::chrono::milliseconds(500)));
    ctrl.stop();
    CHECK(!ctrl.is_powered());
    CHECK(!ctrl.is_moving());
    CHECK(ctrl.is_disconnected());
}

TEST("ServoController: Stop (Halt) is sticky -- motor stays stopped, then re-commandable") {
    ServoController ctrl{make_config(ControlMode::ProfilePosition), sim_factory(ControlMode::ProfilePosition, nullptr)};
    ctrl.start();
    CHECK(wait_until([&] { return ctrl.is_powered(); }, std::chrono::milliseconds(500)));

    ctrl.halt();
    // Several cycles later it must STILL not be moving (Halt latched, not 1-shot).
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    CHECK(!ctrl.is_moving());
    // MOTION-stop is HOLD-ENERGIZED (spec §A R1 / #47-P3b R1): Halt holds via bit8, cw stays 0x0F,
    // the drive NEVER de-energizes (a de-energizing "stop" would fail this). Distinct from a
    // LIFECYCLE-stop (de-energize) and from disable() (operator de-energize).
    CHECK(ctrl.is_powered());
    // A new motion command clears Halt and moves again.
    ctrl.go_to(1000.0, 1.0);
    CHECK(std::abs(ctrl.position_revs() - 1.0) < 0.01);
}

TEST("ServoController(PV): the velocity guard clamps a command the quick-stop can't ramp down (#47-P3b R1)") {
    // A commanded velocity must be STOPPABLE within the controlled-stop teardown window: the guard
    // clamps set_rpm to what quick_stop_decel can null in (controlled_stop_window - margin). CLAMP
    // not reject -- the motor turns at the ceiling, observably below the request.
    // FEEDBACK mode: velocity_counts() reads the exact wire 0x606C (the sim's per-cycle advance =
    // the commanded device velocity), NOT the noisy delta-estimate -> a DETERMINISTIC read.
    ServoConfig cfg = make_config(ControlMode::ProfileVelocity, /*feedback=*/true);
    cfg.quick_stop_decel = 100'000;       // counts/s^2 (echoed back by the sim)
    cfg.controlled_stop_window_ms = 100;  // budget window; margin 50ms -> effective 50ms
    cfg.velocity_threshold = 1;           // is_moving = |vel| > 1
    // budget = 100000 * (0.100 - 0.050) = 5000 counts/s. set_rpm(60 rpm) = 131072 counts/s, FAR
    // above budget -> must clamp to ~5000, never the requested 131072.
    ServoController ctrl{cfg, sim_factory(ControlMode::ProfileVelocity, nullptr, /*feedback=*/true)};
    ctrl.start();
    CHECK(wait_until([&] { return ctrl.is_powered(); }, std::chrono::milliseconds(500)));
    ctrl.set_rpm(60.0);
    CHECK(wait_until([&] { return ctrl.is_moving(); }, std::chrono::milliseconds(300)));
    // The device velocity (0x606C) sits at the guard ceiling (~5000), NOT the requested 131072.
    CHECK(wait_until([&] { return std::abs(ctrl.velocity_counts()) > 100; }, std::chrono::milliseconds(300)));
    const std::int32_t v = std::abs(ctrl.velocity_counts());
    CHECK(v > 0);       // still moving -- clamped, not rejected
    CHECK(v <= 8'000);  // ~budget (5000) + slack; DEFINITELY below the 131072 unclamped request
}

TEST("ServoController: quick-stop OPT-OUT (no decel) -- configure skips the 0x605A/0x6085 SDO, stop coasts (#47-P3b R1)") {
    // The opt-out path: quick_stop_decel == 0 -> needs_quick_stop == false -> the policy's
    // quick-stop SDO setup (0x605A assert / 0x6085 write) is SKIPPED, and the stop is a P3a
    // disable-voltage coast (a known, predictable stop -- safe without a verified/sized decel).
    // The VEL guard is also inert (no budget to size against). This exercises the decel==0 branch
    // that make_config's decel>0 fixture otherwise never hits.
    ServoConfig cfg = make_config(ControlMode::ProfilePosition);
    cfg.quick_stop_decel = 0;  // opt out of the controlled ramp-stop
    ServoController ctrl{cfg, sim_factory(ControlMode::ProfilePosition, nullptr)};
    ctrl.start();  // configure must NOT throw despite no quick-stop SDO (needs_quick_stop=false)
    CHECK(wait_until([&] { return ctrl.is_powered(); }, std::chrono::milliseconds(500)));
    ctrl.go_to(1000.0, 1.0);
    CHECK(std::abs(ctrl.position_revs() - 1.0) < 0.01);
    ctrl.stop();                // LIFECYCLE-stop -> disable-voltage coast; must tear down cleanly (no hang/throw)
    CHECK(!ctrl.is_powered());  // de-energized after the stop
}

TEST("ServoController(PV): LIFECYCLE-stop is a RAMP-then-disable, not a torque-cut (#47-P3b R1, #53 landmark)") {
    // The controlled two-level stop must be a ramp-then-disable: on ctx.stopping the policy commands
    // CiA402 Quick-Stop, the drive stays ENERGIZED while |vel| ramps toward 0 (0x6085 decel), then
    // auto-disables at its own zero (0x605A==2). Assert entered_qsa (it Quick-Stopped, NOT a
    // straight torque-cut / disable-voltage) AND it left QuickStopActive with |vel| <= threshold
    // (de-energized only AFTER the ramp -- the #53 landmark). The VEL guard makes the command
    // rampable-in-window, so the ramp completes inside the teardown window.
    // NON-VACUOUS by construction (team-lead): drive the MAX-CLAMPED velocity (the largest the VEL
    // guard permits for this 0x6085 + window) with a ramp that needs ~the FULL window -- so a
    // 2-cycle teardown CANNOT complete the ramp (torque-cut) but the sized window CAN. budget =
    // 200000 x (0.100 - 0.050) = 10000 counts/s; the sim ramps at 150/cycle -> ~67 cycles to reach
    // rest (>> 2, < the 100-cycle sized window). set_rpm(3000) clamps to the 10000 ceiling.
    SimSlaveModel m = make_model(ControlMode::ProfileVelocity, /*feedback=*/true);
    m.quick_stop_decel_step = 500;  // ~20 cycles to ramp the 10000 max-clamped velocity to 0 (>>2, << the 100-cycle window)
    SimBackend* simp = nullptr;
    ServoController::BackendFactory factory = [m, &simp] {
        auto be = std::make_unique<SimBackend>(std::vector<SimSlaveModel>{m});
        simp = be.get();
        return std::unique_ptr<EcatBackend>(std::move(be));
    };
    ServoConfig cfg = make_config(ControlMode::ProfileVelocity, /*feedback=*/true);
    cfg.quick_stop_decel = 200'000;       // budget = 200000 * (0.100 - 0.050) = 10000 counts/s
    cfg.controlled_stop_window_ms = 100;  // teardown window = 100 cycles @1kHz (>> the ~67-cycle ramp)
    cfg.velocity_threshold = 2000;
    ServoController ctrl{cfg, factory};
    ctrl.start();
    CHECK(wait_until([&] { return ctrl.is_powered(); }, std::chrono::milliseconds(500)));
    ctrl.set_rpm(3000.0);  // >> ceiling -> clamped to the 10000 max the guard permits (needs the full window to ramp)
    CHECK(wait_until([&] { return ctrl.is_moving(); }, std::chrono::milliseconds(300)));
    ctrl.stop();  // LIFECYCLE-stop -> Quick-Stop (ramp, energized) -> de-energize at REST
    CHECK(simp != nullptr);
    // RESOLVER for the torque-cut MUST-FIX. entered_qsa: the stop engaged the controlled Quick-Stop
    // (energized decel ramp), NOT a straight disable-voltage. THE fix assertion: the ramp actually
    // REACHED REST before close() de-energized -- velocity_at_qsa_exit left its sentinel (the drive
    // hit SwitchOnDisabled) AND at |vel| <= threshold. With the OLD 2-cycle teardown this FAILS
    // (velocity_at_qsa_exit stuck at 0x7fffffff = close() cut torque mid-ramp); with the sized
    // teardown window + event-gate it PASSES. (The full cw-disposition landmark lands at sub-step 5
    // with M6's stop-sequence introspection.)
    CHECK(simp->entered_qsa(1));
    CHECK(simp->velocity_at_qsa_exit(1) != 0x7fffffff);                        // reached rest+SwitchOnDisabled BEFORE close (fix)
    CHECK(std::abs(simp->velocity_at_qsa_exit(1)) <= cfg.velocity_threshold);  // de-energized only AFTER the ramp
    CHECK(!ctrl.is_powered());                                                 // de-energized after the controlled stop
    // #47-P3b 5d cw-DISPOSITION landmark (DA's deferred R1 item): the last controlword the drive
    // consumed is the QUICK-STOP cw (0x000B = enable_operation with the quick-stop bit cleared) -- the
    // drive was under CONTROLLED Quick-Stop right through the de-energize (it auto-transitioned to
    // SwitchOnDisabled AT REST via 0x605A==2, the event-gate exiting the instant it disabled), NOT a
    // disable-voltage torque-cut mid-ramp. Controlled-QS-through-rest = (last cw 0x0B) + entered_qsa +
    // (velocity_at_qsa_exit <= thresh) together.
    CHECK_EQ(simp->received_controlword(1), std::uint16_t{0x000B});
}

TEST("ServoController(PV): M6 -- a switch-capable PV motion-hold LOCKS position via PP-at-current-counts (#47-P3b M6)") {
    // The interim R1 PV-hold held zero VELOCITY (bit8) -> under load the axis DRIFTS (PV has no position
    // loop). M6: when the map is switch-capable (0x6060 + 0x607A both RxPDO-mapped), a Halt of a PV move
    // runs the GENERIC mode-switch (ramp PV->0 -> write 0x6060=PP) then latches the AT-HALT position as
    // the PP setpoint, so the drive's POSITION loop LOCKS the shaft. Proven by: the drive's runtime mode
    // becomes PP, |vel|~0, the axis is HELD (position stable = no drift), and it stays ENERGIZED (R1).
    SimSlaveModel m = make_model(ControlMode::ProfileVelocity, /*feedback=*/true);
    m.output_bytes = 11;  // ctrl@0, target-velocity@2, target-position@6, mode-of-op@10 (switch-capable PV map)
    m.velocity_off = 2;
    m.target_off = 6;
    m.mode_of_op_off = 10;
    m.input_bytes = 13;       // feedback (sw@0,actual@2,fault@6,vel@8) + 0x6061 mode-display @12
    m.mode_display_off = 12;  // 0x6061 echo -- REQUIRED so the mode-switch Settle can CONFIRM the new mode
    SimBackend* simp = nullptr;
    ServoController::BackendFactory factory = [m, &simp] {
        auto be = std::make_unique<SimBackend>(std::vector<SimSlaveModel>{m});
        simp = be.get();
        return std::unique_ptr<EcatBackend>(std::move(be));
    };
    ServoConfig cfg = make_config(ControlMode::ProfileVelocity, /*feedback=*/true);
    // Switch-capable RxPDO: add 0x607A (PP target) so pv_hold_capable_ latches (else the interim bit8 hold).
    cfg.rxpdo.entries[0x1600] = {PdoEntry{0x6040, 0, 16}, PdoEntry{0x60FF, 0, 32}, PdoEntry{0x607A, 0, 32}, PdoEntry{0x6060, 0, 8}};
    // 0x6061 mode-display in the TxPDO -> the policy's Settle confirms 0x6061==PP (a real switch, not a hang).
    cfg.txpdo.entries[0x1A00] = {
        PdoEntry{0x6041, 0, 16}, PdoEntry{0x6064, 0, 32}, PdoEntry{0x603F, 0, 16}, PdoEntry{0x606C, 0, 32}, PdoEntry{0x6061, 0, 8}};
    cfg.quick_stop_decel = 100'000;
    cfg.controlled_stop_window_ms = 100;  // VEL budget = 100000*(0.100-0.050) = 5000 counts/s
    cfg.velocity_threshold = 50;
    ServoController ctrl{cfg, factory};
    ctrl.start();
    CHECK(wait_until([&] { return ctrl.is_powered(); }, std::chrono::milliseconds(500)));
    ctrl.set_rpm(1.0);  // ~2184 counts/s (< 5000 budget -> unclamped), well above the 50 threshold
    CHECK(wait_until([&] { return ctrl.is_moving(); }, std::chrono::milliseconds(300)));
    CHECK(simp != nullptr);
    // MOTION-stop (Halt) -> M6: the drive switches to PP-at-current-counts and LOCKS position. The
    // observation split is deliberate: is_powered/is_moving/velocity_counts/position_revs are CONTROLLER
    // atomics (race-free to poll during the run); simp->effective_mode() reads SIM-internal state, so it
    // is read ONLY after ctrl.stop() has JOINED the RT thread (mirrors the entered_qsa/received_* tests).
    ctrl.halt();
    CHECK(wait_until([&] { return std::abs(ctrl.velocity_counts()) <= cfg.velocity_threshold; },
                     std::chrono::milliseconds(500)));           // ramped to rest (StopFirst)
    std::this_thread::sleep_for(std::chrono::milliseconds(60));  // >> the ~10-cycle switch -> the PP hold is established
    CHECK(ctrl.is_powered());                                    // R1: the HOLD is ENERGIZED -- never de-energizes
    CHECK(!ctrl.is_moving());
    // HELD: position is STABLE (the PP position loop locks the shaft at REST -> zero drift, no back-jump).
    const double p0 = ctrl.position_revs();
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    CHECK(std::abs(ctrl.position_revs() - p0) < 1e-6);
    ctrl.stop();                                                              // JOIN the RT thread -> the sim accessor below is race-free
    CHECK(simp->effective_mode(1) == ethercat::Cia402Mode::ProfilePosition);  // the drive DID switch to PP (not bit8-PV)
}

TEST(
    "ServoController(PV): M6 -- a PV->PP hold-switch that can't confirm stays ENERGIZED in PV, no throw/de-energize (#47-P3b M6 failure)") {
    // Failure disposition (spec §A R1): if the hold-switch can't confirm the mode (0x6061 never echoes PP
    // -- the #45 silent-ignore shape), the module does NOT throw (it's an INTERNAL hold, not an operator
    // command) and does NOT de-energize -- it reverts to the interim PV-at-0 hold. Proven by: after the
    // settle window the drive is back in PV and still ENERGIZED (the load-bearing safety property).
    SimSlaveModel m = make_model(ControlMode::ProfileVelocity, /*feedback=*/true);
    m.output_bytes = 11;
    m.velocity_off = 2;
    m.target_off = 6;
    m.mode_of_op_off = 10;
    m.input_bytes = 13;
    m.mode_display_off = 12;
    m.mode_echo_forced = true;
    m.mode_echo_value = static_cast<std::int8_t>(ethercat::Cia402Mode::ProfileVelocity);  // 0x6061 ALWAYS says PV -> switch never confirms
    SimBackend* simp = nullptr;
    ServoController::BackendFactory factory = [m, &simp] {
        auto be = std::make_unique<SimBackend>(std::vector<SimSlaveModel>{m});
        simp = be.get();
        return std::unique_ptr<EcatBackend>(std::move(be));
    };
    ServoConfig cfg = make_config(ControlMode::ProfileVelocity, /*feedback=*/true);
    cfg.rxpdo.entries[0x1600] = {PdoEntry{0x6040, 0, 16}, PdoEntry{0x60FF, 0, 32}, PdoEntry{0x607A, 0, 32}, PdoEntry{0x6060, 0, 8}};
    cfg.txpdo.entries[0x1A00] = {
        PdoEntry{0x6041, 0, 16}, PdoEntry{0x6064, 0, 32}, PdoEntry{0x603F, 0, 16}, PdoEntry{0x606C, 0, 32}, PdoEntry{0x6061, 0, 8}};
    cfg.quick_stop_decel = 100'000;
    cfg.velocity_threshold = 50;
    ServoController ctrl{cfg, factory};
    ctrl.start();
    CHECK(wait_until([&] { return ctrl.is_powered(); }, std::chrono::milliseconds(500)));
    ctrl.set_rpm(1.0);
    CHECK(wait_until([&] { return ctrl.is_moving(); }, std::chrono::milliseconds(300)));
    ctrl.halt();  // attempts PV->PP; Settle never confirms (echo forced PV) -> mode_switch_failed -> revert
    // Wait past the settle-timeout window (default 200 cycles ~200ms @1kHz) for the failure + revert to
    // settle. The end-state is STABLE (reverted PV-at-0 hold), so this post-condition never flakes.
    std::this_thread::sleep_for(std::chrono::milliseconds(400));
    CHECK(ctrl.is_powered());  // THE safety property: stayed ENERGIZED through a failed switch (never de-energized)
    CHECK(std::abs(ctrl.velocity_counts()) <= cfg.velocity_threshold);  // reverted PV-at-0 -> at rest (atomic, race-free)
    ctrl.stop();                                                        // JOIN the RT thread -> the sim accessor below is race-free
    CHECK(simp != nullptr);
    CHECK(simp->effective_mode(1) == ethercat::Cia402Mode::ProfileVelocity);  // reverted to PV (interim bit8 hold), not stuck in PP
}

namespace {
// #47-P3c/#57: a switch-capable A6-shaped module config that maps BOTH 0x6060 (RxPDO) AND 0x6061 (TxPDO
// mode-display) -- the piece the standard make_model omits, which hid the module's enable-seed + #45 gate
// offline. 0x6061 @12 in the feedback TxPDO (status@0, actual@2, 603F@6, 606C@8, 6061@12 -> 13 B).
ServoConfig make_config_modegate() {
    ServoConfig c = make_config(ControlMode::ProfilePosition, /*feedback=*/true);
    c.txpdo.entries[0x1A00] = {
        PdoEntry{0x6041, 0, 16}, PdoEntry{0x6064, 0, 32}, PdoEntry{0x603F, 0, 16}, PdoEntry{0x606C, 0, 32}, PdoEntry{0x6061, 0, 8}};
    return c;
}
SimSlaveModel make_model_modegate() {
    SimSlaveModel m = make_model(ControlMode::ProfilePosition, /*feedback=*/true);
    m.input_bytes = 13;
    m.mode_display_off = 12;  // 0x6061 follows the PDO 0x6060 (faithful sim) unless mode_echo_forced
    return m;
}
}  // namespace

TEST("#47-P3c/#57: module 0x6060+0x6061-mapped enable ladder reaches OE with the mode seeded (non-vacuous vs the seed)") {
    // The module's OWN enable FSM seeds 0x6060 through the ladder; the faithful sim makes 0x6061 follow
    // the PDO mode, so the mode-echo gate sees 0x6061 == PP and allows OE. NON-VACUOUS vs the seed: with
    // the Enabling-branch seed disabled, wire 0x6060=0 -> 0x6061=0 != PP -> the gate refuses -> never
    // powered -> this FAILS. (Mirrors a6_control's P3c REGRESSION but through the MODULE's own ladder.)
    SimBackend* sim = nullptr;
    ServoController::BackendFactory factory = [&sim] {
        auto be = std::make_unique<SimBackend>(std::vector<SimSlaveModel>{make_model_modegate()});
        sim = be.get();
        return std::unique_ptr<EcatBackend>(std::move(be));
    };
    ServoController ctrl{make_config_modegate(), factory};
    ctrl.start();
    CHECK(wait_until([&] { return ctrl.is_powered(); }, std::chrono::milliseconds(500)));  // seed -> 0x6061=PP -> gate Passed -> OE
    (void)sim;
}

TEST("#47-P3c/#57: module mode-echo MISMATCH (0x6061 != commanded) REFUSES to energize (#45 fail-closed, last_error)") {
    // The #45 safety gate, now ACTIVE in the module's ladder (was structurally absent -- fsm_ was
    // mode-blind). The drive reports 0x6061 = CSP(8) != commanded PP(1) (silent-ignore shape) -> the gate
    // latches Failed -> de-energizes + surfaces RtError::ModeMismatch via last_error(). NON-VACUOUS vs the
    // gate: without it the module energizes into the wrong mode (is_powered true).
    SimSlaveModel m = make_model_modegate();
    m.mode_echo_forced = true;
    m.mode_echo_value = 8;  // CSP(8) != commanded PP(1): the drive "silently ignored" the mode-set
    SimBackend* sim = nullptr;
    ServoController::BackendFactory factory = [m, &sim] {
        auto be = std::make_unique<SimBackend>(std::vector<SimSlaveModel>{m});
        sim = be.get();
        return std::unique_ptr<EcatBackend>(std::move(be));
    };
    ServoController ctrl{make_config_modegate(), factory};
    ctrl.start();
    const bool powered = wait_until([&] { return ctrl.is_powered(); }, std::chrono::milliseconds(400));
    CHECK(!powered);                                                          // REFUSED: never energized into the wrong mode
    CHECK(ctrl.last_error().find("mode-of-operation") != std::string::npos);  // diagnosable: last_error names the mismatch
    (void)sim;
}

TEST("ServoController(PV): a displaced, stopped motor reports is_moving == false") {
    // Regression for the PP-predicate-in-PV bug: target_counts_ is never set in PV,
    // so the old position-tolerance predicate reported a stopped-but-displaced PV
    // motor as moving forever. PV is_moving must be velocity-based.
    ServoConfig cfg = make_config(ControlMode::ProfileVelocity);
    cfg.velocity_threshold = 1000;  // counts/s; PV is_moving = |velocity| > this
    ServoController ctrl{cfg, sim_factory(ControlMode::ProfileVelocity, nullptr)};
    ctrl.start();
    CHECK(wait_until([&] { return ctrl.is_powered(); }, std::chrono::milliseconds(500)));

    ctrl.set_rpm(60.0);  // integrate a nonzero velocity -> the motor turns
    CHECK(wait_until([&] { return ctrl.is_moving(); }, std::chrono::milliseconds(300)));
    CHECK(std::abs(ctrl.position_revs()) > 0.01);  // displaced from zero

    ctrl.set_rpm(0.0);  // stop commanding velocity -> actual stops advancing
    CHECK(wait_until([&] { return !ctrl.is_moving(); }, std::chrono::milliseconds(500)));
    CHECK(std::abs(ctrl.position_revs()) > 0.01);  // STILL displaced, but NOT moving
}

TEST("ServoController(PP): handshake timeout aborts the move PROMPTLY, then recovers") {
    // Drive withholds the PP set-point-acknowledge (bit12) -> the new-set-point
    // handshake times out. The move must abort promptly with the CORRECT reason
    // (not the slower stall watchdog), the drive must stay powered (a controller
    // move-error is NOT a drive fault), and a subsequent move must recover with a
    // clean last_error() (the latch cleared on the new move).
    ServoConfig cfg = make_config(ControlMode::ProfilePosition);
    cfg.handshake_timeout_cycles = 5;  // time out fast
    SimBackend* sim = nullptr;
    ServoController ctrl{cfg, sim_factory(ControlMode::ProfilePosition, &sim)};
    ctrl.start();
    CHECK(wait_until([&] { return ctrl.is_powered(); }, std::chrono::milliseconds(500)));
    CHECK(sim != nullptr);

    sim->suppress_setpoint_ack(1, true);
    CHECK_THROWS_MSG(ctrl.go_to(1000.0, 1.0), ethercat::Error, "acknowledge");
    CHECK(ctrl.is_powered());  // move-error must NOT de-power a healthy drive
    CHECK(ctrl.last_error().find("acknowledge") != std::string::npos);

    sim->suppress_setpoint_ack(1, false);  // recovery: ack again
    ctrl.go_to(1000.0, 1.0);               // fresh move adopts (clears the latch) + completes -- must NOT throw
    CHECK(std::abs(ctrl.position_revs() - 1.0) < 0.01);
    CHECK(ctrl.last_error().empty());  // stale handshake-timeout cleared
}

TEST("ServoController(PP): a no-progress move trips the stall watchdog; drive stays powered") {
    // A genuine no-progress stall (not a device fault): the drive stays enabled but
    // physically frozen (counts_per_step=0), so the move never reaches target. The
    // watchdog (move_timeout_ms=0 -> stall_threshold_cycles*4) must fail the move
    // via go_to throwing, WITHOUT de-powering the healthy drive.
    auto factory = [] {
        SimSlaveModel m = make_model(ControlMode::ProfilePosition);
        m.profile_velocity_off = -1;  // don't track 0x6081 -> use the fixed fallback speed...
        m.counts_per_step = 0;        // ...of 0 => physically frozen (drive enabled, no progress)
        return std::unique_ptr<EcatBackend>(std::make_unique<SimBackend>(std::vector<SimSlaveModel>{m}));
    };
    ServoController ctrl{make_config(ControlMode::ProfilePosition), factory};
    ctrl.start();
    CHECK(wait_until([&] { return ctrl.is_powered(); }, std::chrono::milliseconds(500)));
    CHECK_THROWS_MSG(ctrl.go_to(1000.0, 5.0), ethercat::Error, "stalled");
    CHECK(ctrl.is_powered());  // a stall fails the MOVE, not the drive
}

TEST("ServoController: fault inject -> not powered; fault_reset recovers") {
    SimBackend* sim = nullptr;
    ServoController ctrl{make_config(ControlMode::ProfilePosition), sim_factory(ControlMode::ProfilePosition, &sim)};
    ctrl.start();
    CHECK(wait_until([&] { return ctrl.is_powered(); }, std::chrono::milliseconds(500)));
    CHECK(sim != nullptr);

    sim->inject_fault(1);
    CHECK(wait_until([&] { return !ctrl.is_powered(); }, std::chrono::milliseconds(500)));
}

// ---- #16: TxPDO feedback (fault legibility + velocity) ----

TEST("ServoController(#16): a drive fault is legible -- last_error shows 0x603F + config gloss") {
    SimBackend* sim = nullptr;
    ServoController ctrl{make_config(ControlMode::ProfilePosition, /*feedback=*/true),
                         sim_factory(ControlMode::ProfilePosition, &sim, /*feedback=*/true)};
    ctrl.start();
    CHECK(wait_until([&] { return ctrl.is_powered(); }, std::chrono::milliseconds(500)));
    CHECK(sim != nullptr);

    sim->set_fault_code(1, 0x8700);  // Er74.1 (the missed-SYNC0 code this work illuminates)
    sim->inject_fault(1);
    CHECK(wait_until([&] { return !ctrl.is_powered(); }, std::chrono::milliseconds(500)));

    const std::string e = ctrl.last_error();
    CHECK(e.find("drive fault 0x8700") != std::string::npos);  // the raw code
    CHECK(e.find("Er74.1 / no SYNC0") != std::string::npos);   // the config-data gloss
}

TEST("ServoController(#16): compose-both -- a drive fault AND a WKC fault BOTH surface (Er74)") {
    // The regression test: a missed SYNC0 faults the drive (0x603F) AND drops WKC->0;
    // last_error() must report BOTH (root cause + symptom), never mask one.
    SimBackend* sim = nullptr;
    ServoController ctrl{make_config(ControlMode::ProfilePosition, true), sim_factory(ControlMode::ProfilePosition, &sim, true)};
    ctrl.start();
    CHECK(wait_until([&] { return ctrl.is_powered(); }, std::chrono::milliseconds(500)));

    sim->set_fault_code(1, 0x8700);
    sim->inject_fault(1);        // DRIVE tier (status bit3 + 0x603F)
    sim->force_short_wkc(true);  // BUS tier (sustained short WKC -> master latches fault)
    CHECK(wait_until(
        [&] {
            const std::string e = ctrl.last_error();
            return e.find("drive fault 0x8700") != std::string::npos && e.find("working-counter fault") != std::string::npos;
        },
        std::chrono::milliseconds(1000)));
}

TEST("ServoController(#16): 0x603F is live-read -- a code latched AFTER bit3 is still picked up") {
    SimBackend* sim = nullptr;
    ServoController ctrl{make_config(ControlMode::ProfilePosition, true), sim_factory(ControlMode::ProfilePosition, &sim, true)};
    ctrl.start();
    CHECK(wait_until([&] { return ctrl.is_powered(); }, std::chrono::milliseconds(500)));

    sim->inject_fault(1);  // bit3 set; fault_code defaults 0 -> "code pending"
    CHECK(wait_until([&] { return ctrl.last_error().find("code pending") != std::string::npos; }, std::chrono::milliseconds(500)));
    sim->set_fault_code(1, 0x8700);  // code latched a few cycles later -> live-read picks it up
    CHECK(wait_until([&] { return ctrl.last_error().find("0x8700") != std::string::npos; }, std::chrono::milliseconds(500)));
}

TEST("ServoController(#16): the drive-fault FLAG gates a stale 0x603F -- no phantom fault") {
    // The literal flag-gating test (inverse of the masking bug): a nonzero 0x603F on
    // the wire while bit3 is CLEAR (a stale code lingering after the fault cleared)
    // must NOT produce a drive tier -- last_error() reads drive_faulted (the flag)
    // before drive_fault_code (the payload), so a stale code is never reported as a
    // live fault. set_stale_fault_code forces 0x603F nonzero WITHOUT setting bit3.
    SimBackend* sim = nullptr;
    ServoController ctrl{make_config(ControlMode::ProfilePosition, true), sim_factory(ControlMode::ProfilePosition, &sim, true)};
    ctrl.start();
    CHECK(wait_until([&] { return ctrl.is_powered(); }, std::chrono::milliseconds(500)));

    sim->set_stale_fault_code(1, 0x8700);  // 0x603F = 0x8700 on the wire, but the drive is NOT faulted (bit3 = 0)
    // Give the controller several cycles to publish; the drive must stay powered (no
    // fault) and last_error() must NOT mention a drive fault despite the nonzero code.
    CHECK(!wait_until([&] { return ctrl.last_error().find("drive fault") != std::string::npos; }, std::chrono::milliseconds(200)));
    CHECK(ctrl.is_powered());
    CHECK(ctrl.last_error().find("drive fault") == std::string::npos);
}

TEST("ServoController(#16): velocity comes from the 0x606C wire value, not the estimate") {
    SimBackend* sim = nullptr;
    ServoController ctrl{make_config(ControlMode::ProfileVelocity, true), sim_factory(ControlMode::ProfileVelocity, &sim, true)};
    ctrl.start();
    CHECK(wait_until([&] { return ctrl.is_powered(); }, std::chrono::milliseconds(500)));

    // PV: the sim integrates s.actual += (0x60FF value) per cycle, so the per-cycle
    // delta it writes to 0x606C == the commanded device velocity. The estimate path
    // would instead report delta * loop_rate (1000x). Assert the WIRE value.
    const std::int32_t dev = ethercat::servo::rpm_to_device_velocity(60.0, kCountsPerRev, 1.0);
    ctrl.set_rpm(60.0);
    CHECK(wait_until([&] { return ctrl.velocity_counts() == dev; }, std::chrono::milliseconds(500)));
    CHECK(ctrl.velocity_counts() != dev * 1000);  // NOT the estimate (delta * rate)
}

TEST("ServoController(#16): live suppress_ack toggle vs the RT handshake read is race-free (TSan gate)") {
    // TSan regression gate for the SimBackend cross-thread test hooks. The existing
    // handshake-timeout test sets suppress_ack BEFORE go_to, so its accesses are
    // happens-before-ordered via the command queue -- TSan would NOT flag a plain-bool
    // suppress_ack there. This test instead hammers the hook from a background thread
    // with NO synchronization against the RT loop's read in step_device, so a future
    // non-atomic sibling (or a revert) trips TSan. Atomic -> clean. The assertion IS
    // "TSan observed no data race" (plus a clean lifecycle).
    SimBackend* sim = nullptr;
    ServoConfig cfg = make_config(ControlMode::ProfilePosition);
    cfg.handshake_timeout_cycles = 10;  // bound each handshake so go_for returns promptly
    ServoController ctrl{cfg, sim_factory(ControlMode::ProfilePosition, &sim)};
    ctrl.start();
    CHECK(wait_until([&] { return ctrl.is_powered(); }, std::chrono::milliseconds(500)));
    CHECK(sim != nullptr);

    std::atomic<bool> stop{false};
    std::thread toggler([&] {
        bool on = false;
        while (!stop.load(std::memory_order_relaxed)) {
            sim->suppress_setpoint_ack(1, on);  // unsynchronized vs the RT read in step_device
            on = !on;
        }
    });
    // Drive repeated PP handshakes (each reads suppress_ack on the bit4 rising edge)
    // while the toggler races it. Moves may complete or abort (handshake timeout) --
    // both fine; we only care that the concurrent access is clean.
    for (int i = 0; i < 4; ++i) {
        try {
            ctrl.go_for(1000.0, 0.1);
        } catch (const ethercat::Error&) {  // NOLINT(bugprone-empty-catch): timeout/abort is expected under the toggle
        }
    }
    stop.store(true, std::memory_order_relaxed);
    toggler.join();
    CHECK(ctrl.is_powered());  // a move-error never de-powers; the drive is healthy throughout
}

TEST("ServoController(#16): unmapped 0x603F/0x606C -> no OOB; velocity falls back to the estimate") {
    SimBackend* sim = nullptr;
    // Default config: NO 0x603F/0x606C in the TxPDO -> the optional guards must hold.
    ServoController ctrl{make_config(ControlMode::ProfileVelocity), sim_factory(ControlMode::ProfileVelocity, &sim, false)};
    ctrl.start();
    CHECK(wait_until([&] { return ctrl.is_powered(); }, std::chrono::milliseconds(500)));
    CHECK(ctrl.last_error().empty());  // healthy, no drive tier, no OOB

    // Velocity falls back to the instantaneous estimate (actual-delta * rate).
    ctrl.set_rpm(60.0);
    CHECK(wait_until([&] { return ctrl.velocity_counts() != 0; }, std::chrono::milliseconds(500)));
}

// --- #18: fault-reset Resetting sub-state (real-HW recovery) -------------------

// (1) Type-(a) reflect latency: ONE fault_reset recovers across the drive's clear
// delay. This is the regression guard -- the un-fixed FSM strands in Faulted here
// (Enabling bounces on the still-Fault drive, the one-shot reset is consumed).
TEST("ServoController(#18): type-(a) reflect latency -- ONE fault_reset recovers (regression)") {
    SimBackend* sim = nullptr;
    ServoController ctrl{make_config(ControlMode::ProfilePosition), sim_factory(ControlMode::ProfilePosition, &sim)};
    ctrl.start();
    CHECK(wait_until([&] { return ctrl.is_powered(); }, std::chrono::milliseconds(500)));
    CHECK(sim != nullptr);

    sim->set_fault_clear_delay(1, 30);  // accept the reset, reflect the clear after 30 cycles (< 200 window)
    sim->inject_fault(1);
    CHECK(wait_until([&] { return !ctrl.is_powered(); }, std::chrono::milliseconds(500)));  // faulted

    ctrl.request_fault_reset();  // ONE reset -> Resetting holds the intent across the latency
    CHECK(wait_until([&] { return ctrl.is_powered(); }, std::chrono::milliseconds(1500)));

    ctrl.go_to(600.0, 1.0);  // fully recovered: a subsequent command works
    CHECK(ctrl.is_powered());
}

// (2) Window boundary: a delay comfortably inside the window recovers; a delay far
// beyond it gives up (stays not-powered + the CTRL diagnostic surfaces).
TEST("ServoController(#18): recovers below the window, gives up above it") {
    {
        SimBackend* sim = nullptr;
        ServoController ctrl{make_config(ControlMode::ProfilePosition), sim_factory(ControlMode::ProfilePosition, &sim)};
        ctrl.start();
        CHECK(wait_until([&] { return ctrl.is_powered(); }, std::chrono::milliseconds(500)));
        sim->set_fault_clear_delay(1, 150);  // inside the 200-cycle window
        sim->inject_fault(1);
        CHECK(wait_until([&] { return !ctrl.is_powered(); }, std::chrono::milliseconds(500)));
        ctrl.request_fault_reset();
        CHECK(wait_until([&] { return ctrl.is_powered(); }, std::chrono::milliseconds(2000)));  // recovers
    }
    {
        SimBackend* sim = nullptr;
        ServoController ctrl{make_config(ControlMode::ProfilePosition), sim_factory(ControlMode::ProfilePosition, &sim)};
        ctrl.start();
        CHECK(wait_until([&] { return ctrl.is_powered(); }, std::chrono::milliseconds(500)));
        sim->set_fault_clear_delay(1, 100000);  // never reflects within the window
        sim->inject_fault(1);
        CHECK(wait_until([&] { return !ctrl.is_powered(); }, std::chrono::milliseconds(500)));
        ctrl.request_fault_reset();
        CHECK(wait_until([&] { return ctrl.last_error().find("fault-reset ineffective") != std::string::npos; },
                         std::chrono::milliseconds(2000)));  // gave up
        CHECK(!ctrl.is_powered());
    }
}

// (3) Type-(b) persistent cause: window expires -> give-up, and last_error composes
// BOTH tiers -- the live drive code (#16) + the CTRL reset verdict.
TEST("ServoController(#18): persistent-cause give-up -- last_error composes drive + CTRL tiers") {
    SimBackend* sim = nullptr;
    ServoConfig cfg = make_config(ControlMode::ProfilePosition, /*feedback=*/true);
    cfg.fault_code_labels = {{0x6320, "Er74.0 / cycle error"}};  // gloss for the compose
    ServoController ctrl{cfg, sim_factory(ControlMode::ProfilePosition, &sim, /*feedback=*/true)};
    ctrl.start();
    CHECK(wait_until([&] { return ctrl.is_powered(); }, std::chrono::milliseconds(500)));

    sim->set_fault_persistent(1, true);  // cause stays active -- no reset clears it
    sim->set_fault_code(1, 0x6320);
    sim->inject_fault(1);
    CHECK(wait_until([&] { return !ctrl.is_powered(); }, std::chrono::milliseconds(500)));

    ctrl.request_fault_reset();  // window expires -> give-up
    CHECK(wait_until([&] { return ctrl.last_error().find("fault-reset ineffective") != std::string::npos; },
                     std::chrono::milliseconds(2000)));
    const std::string err = ctrl.last_error();
    CHECK(err.find("0x6320") != std::string::npos);                   // #16 drive tier, STILL live
    CHECK(err.find("fault-reset ineffective") != std::string::npos);  // CTRL tier verdict
    CHECK(!ctrl.is_powered());
}

// (4) Type-(c) clear-then-refault: the drive clears MOMENTARILY on the reset edge then
// re-faults before the K-cycle confirm -> must NOT be mistaken for success -> the same
// give-up + compose-both as type-(b). Guards the debounce (clear_streak_).
TEST("ServoController(#18): clear-then-refault flicker -> give-up (not false recovery)") {
    SimBackend* sim = nullptr;
    ServoConfig cfg = make_config(ControlMode::ProfilePosition, /*feedback=*/true);
    cfg.fault_code_labels = {{0x6320, "Er74.0 / cycle error"}};
    ServoController ctrl{cfg, sim_factory(ControlMode::ProfilePosition, &sim, /*feedback=*/true)};
    ctrl.start();
    CHECK(wait_until([&] { return ctrl.is_powered(); }, std::chrono::milliseconds(500)));

    // Momentary clear holds only 1 cycle (< the 3-cycle confirm) -> the clear never
    // confirms; the re-edge re-clears -> repeating flicker, bounded by the window.
    sim->set_fault_clear_then_refault(1, 1);
    sim->set_fault_code(1, 0x6320);
    sim->inject_fault(1);
    CHECK(wait_until([&] { return !ctrl.is_powered(); }, std::chrono::milliseconds(500)));

    const std::uint64_t c0 = ctrl.loop_cycle();
    ctrl.request_fault_reset();
    // The give-up latches the CTRL verdict; the flickering drive settles back to Fault
    // after its final momentary clear (the live #16 drive tier is only present while
    // dev==Fault), so poll until BOTH tiers compose -- a flicker yields give-up, not
    // recovery. Generous wall-clock BACKSTOP only (real bound is the cycle delta below).
    CHECK(wait_until(
        [&] {
            const std::string e = ctrl.last_error();
            return e.find("fault-reset ineffective") != std::string::npos && e.find("0x6320") != std::string::npos;
        },
        std::chrono::milliseconds(5000)));
    CHECK(!ctrl.is_powered());
    // CYCLE-BASED BOUND (not wall-time): the give-up must land within ~1.5x the window.
    // A regression that decremented reset_cycles_remaining_ only on Fault cycles would let
    // the flicker (half the cycles are momentary clears) stretch the give-up to ~2x the
    // window -- this blows the cycle bound even though a loose wall-clock timeout might
    // still pass on a fast loop. (window=200 default; +K +enable-ladder +poll latency.)
    const std::uint64_t elapsed = ctrl.loop_cycle() - c0;
    CHECK(elapsed < cfg.fault_reset_window_cycles * 3 / 2);
}

// (4b) Sanity companion: a LARGE hold -- the clear holds past K, the drive reaches
// Operational (true recovery), THEN re-faults later. That later fault is a NEW episode
// -> generic Faulted with the live #16 code, NOT FaultResetFailed (which would wrongly
// blame the reset). Pins the new-fault-after-recovery boundary.
TEST("ServoController(#18): clear that HOLDS past K recovers, a later refault is a new fault") {
    SimBackend* sim = nullptr;
    ServoConfig cfg = make_config(ControlMode::ProfilePosition, /*feedback=*/true);
    cfg.fault_code_labels = {{0x6320, "Er74.0 / cycle error"}};
    ServoController ctrl{cfg, sim_factory(ControlMode::ProfilePosition, &sim, /*feedback=*/true)};
    ctrl.start();
    CHECK(wait_until([&] { return ctrl.is_powered(); }, std::chrono::milliseconds(500)));

    // Hold the clear far past K (3) so it CONFIRMS -> recovers to Operational...
    sim->set_fault_clear_then_refault(1, 100);
    sim->set_fault_code(1, 0x6320);
    sim->inject_fault(1);
    CHECK(wait_until([&] { return !ctrl.is_powered(); }, std::chrono::milliseconds(500)));
    ctrl.request_fault_reset();
    CHECK(wait_until([&] { return ctrl.is_powered(); }, std::chrono::milliseconds(2000)));  // confirmed recovery

    // ...then the type-c re-fault fires (~100 cycles after the clear) -> a NEW fault while
    // Operational -> Faulted with the live drive code, but NOT the FaultResetFailed verdict.
    CHECK(wait_until([&] { return !ctrl.is_powered(); }, std::chrono::milliseconds(2000)));
    const std::string err = ctrl.last_error();
    CHECK(err.find("0x6320") != std::string::npos);                   // live drive tier (new fault)
    CHECK(err.find("fault-reset ineffective") == std::string::npos);  // NOT a reset failure
}

// (5) Type-(b) then cause removed: a fresh reset recovers once the cause is gone.
TEST("ServoController(#18): once the persistent cause is removed, a reset recovers") {
    SimBackend* sim = nullptr;
    ServoController ctrl{make_config(ControlMode::ProfilePosition), sim_factory(ControlMode::ProfilePosition, &sim)};
    ctrl.start();
    CHECK(wait_until([&] { return ctrl.is_powered(); }, std::chrono::milliseconds(500)));

    sim->set_fault_persistent(1, true);
    sim->inject_fault(1);
    CHECK(wait_until([&] { return !ctrl.is_powered(); }, std::chrono::milliseconds(500)));
    ctrl.request_fault_reset();
    CHECK(wait_until([&] { return ctrl.last_error().find("fault-reset ineffective") != std::string::npos; },
                     std::chrono::milliseconds(2000)));  // gave up

    sim->set_fault_persistent(1, false);  // cause removed
    ctrl.request_fault_reset();           // a NEW reset now clears
    CHECK(wait_until([&] { return ctrl.is_powered(); }, std::chrono::milliseconds(2000)));
}

// (6) No-spin guarantee (DA's lane) -- CAPTURE THE EDGE STREAM, not just the end state.
// After a give-up: (i) zero further bit7 rising edges (no perpetual self-re-edge), and
// (ii) removing the cause WITHOUT a new reset does NOT auto-recover -- only an explicit
// reset does. Counting the edges proves the FSM isn't self-spinning Resetting.
TEST("ServoController(#18): post-give-up has zero bit7 edges + does not self-recover") {
    SimBackend* sim = nullptr;
    ServoController ctrl{make_config(ControlMode::ProfilePosition), sim_factory(ControlMode::ProfilePosition, &sim)};
    ctrl.start();
    CHECK(wait_until([&] { return ctrl.is_powered(); }, std::chrono::milliseconds(500)));

    sim->set_fault_persistent(1, true);
    sim->inject_fault(1);
    CHECK(wait_until([&] { return !ctrl.is_powered(); }, std::chrono::milliseconds(500)));
    ctrl.request_fault_reset();
    CHECK(wait_until([&] { return ctrl.last_error().find("fault-reset ineffective") != std::string::npos; },
                     std::chrono::milliseconds(2000)));  // gave up

    // Snapshot the cumulative bit7-edge count at give-up, then hold: a self-spinning FSM
    // would keep re-edging (count climbs). Remove the cause too -- a spinner would even
    // recover. The fixed FSM does NEITHER: edges flat AND stays faulted (no self-reset).
    const std::uint32_t edges_at_giveup = sim->fault_reset_edge_count(1);
    sim->set_fault_persistent(1, false);
    CHECK(!wait_until([&] { return ctrl.is_powered(); }, std::chrono::milliseconds(300)));  // no self-recovery
    CHECK_EQ(sim->fault_reset_edge_count(1), edges_at_giveup);                              // ZERO new edges post-give-up
    CHECK(ctrl.last_error().find("fault-reset ineffective") != std::string::npos);          // verdict still latched

    ctrl.request_fault_reset();  // only an EXPLICIT reset re-edges + recovers
    CHECK(wait_until([&] { return ctrl.is_powered(); }, std::chrono::milliseconds(2000)));
    CHECK(sim->fault_reset_edge_count(1) > edges_at_giveup);  // the explicit reset DID edge
}

// (6) Instant clear (the common case, delay=0 default): one reset recovers, unchanged.
TEST("ServoController(#18): instant clear (default) -- one reset recovers, unchanged") {
    SimBackend* sim = nullptr;
    ServoController ctrl{make_config(ControlMode::ProfilePosition), sim_factory(ControlMode::ProfilePosition, &sim)};
    ctrl.start();
    CHECK(wait_until([&] { return ctrl.is_powered(); }, std::chrono::milliseconds(500)));
    sim->inject_fault(1);  // no delay hook -> instant clear
    CHECK(wait_until([&] { return !ctrl.is_powered(); }, std::chrono::milliseconds(500)));
    ctrl.request_fault_reset();
    CHECK(wait_until([&] { return ctrl.is_powered(); }, std::chrono::milliseconds(1000)));
}

// (#39) The CONSUMER-side vendor fault-reset: executed once pre-RT-spawn when configured
// (the sim's SDO record shows the 0x2031 write), skipped entirely when absent. And the
// RT-phase bracket clears on stop(): a STOP -> START cycle re-runs the pre-spawn reset
// successfully -- if stop() left rt_active set, the restart's sdo_write would throw and
// start() would fail (the behavioral proof of the bracket's clear-after-join half).
TEST("#39: config-driven vendor fault-reset runs pre-spawn; absent = zero vendor traffic") {
    {  // present -> the 0x2031 write lands before the RT thread exists
        ServoConfig cfg = make_config(ControlMode::ProfilePosition);
        cfg.vendor_fault_reset = ethercat::SdoWrite{0x2031, 0x01, {std::byte{0x01}, std::byte{0x00}}};
        SimBackend* sim = nullptr;
        ServoController ctrl{cfg, sim_factory(ControlMode::ProfilePosition, &sim)};
        ctrl.start();
        CHECK(sim != nullptr);
        const std::vector<std::byte> rec = sim->recorded_sdo(1, 0x2031, 0x01);
        CHECK_EQ(rec.size(), std::size_t{2});
        CHECK(!rec.empty() && rec[0] == std::byte{0x01});
        ctrl.stop();
    }
    {  // absent -> no vendor object traffic at all
        SimBackend* sim = nullptr;
        ServoController ctrl{make_config(ControlMode::ProfilePosition), sim_factory(ControlMode::ProfilePosition, &sim)};
        ctrl.start();
        CHECK(sim != nullptr);
        CHECK(sim->recorded_sdo(1, 0x2031, 0x01).empty());
        ctrl.stop();
    }
}

TEST("#39: the RT-phase bracket clears after stop() -- a restart's pre-spawn reset succeeds") {
    ServoConfig cfg = make_config(ControlMode::ProfilePosition);
    cfg.vendor_fault_reset = ethercat::SdoWrite{0x2031, 0x01, {std::byte{0x01}, std::byte{0x00}}};
    SimBackend* sim = nullptr;
    ServoController ctrl{cfg, sim_factory(ControlMode::ProfilePosition, &sim)};
    ctrl.start();  // spawn: rt_active declared
    ctrl.stop();   // join: rt_active MUST clear (else the next pre-spawn SDO throws)
    // reconfigure (same config) = the restart path: its pre-spawn vendor reset must not
    // throw. A stale-true rt_active on a persisting Master is the hazard this catches --
    // the restart builds a fresh Master, and stop()'s explicit clear covers the
    // stop-then-external-SDO pattern; both paths land here green.
    ctrl.reconfigure(cfg);
    CHECK(sim != nullptr);
    CHECK_EQ(sim->recorded_sdo(1, 0x2031, 0x01).size(), std::size_t{2});  // the restart's reset landed
    ctrl.stop();
}

// #15: the pre-#15 RT-phase port-ownership gate is GONE. The Master's public SDO now runs the mailbox
// transfer directly on the caller's thread, concurrency-safe against a running RT PDO loop (SOEM v2's
// port is thread-safe). So a consumer-side SDO via master_for_sdo() succeeds BOTH while the RT phase is
// running AND after stop() -- no ConfigError refusal.
TEST("#15: consumer SDO works during the RT phase and after stop (no port-ownership gate)") {
    SimBackend* sim = nullptr;
    ServoController ctrl{make_config(ControlMode::ProfilePosition), sim_factory(ControlMode::ProfilePosition, &sim)};
    CHECK(ctrl.master_for_sdo() == nullptr);  // pre-first-start: no Master yet
    ctrl.start();
    CHECK(ctrl.master_for_sdo() != nullptr);
    const std::array<std::byte, 2> one{std::byte{0x01}, std::byte{0x00}};
    ctrl.master_for_sdo()->sdo_write(1, 0x2031, 0x01, one);  // #15: succeeds DURING the RT phase (was ConfigError)
    ctrl.stop();                                             // joined
    ctrl.master_for_sdo()->sdo_write(1, 0x2031, 0x01, one);  // and after stop
    CHECK(sim != nullptr);
    CHECK_EQ(sim->recorded_sdo(1, 0x2031, 0x01).size(), std::size_t{2});
}

TEST("ServoController(PP): R3 single-in-flight -- a 2nd blocking move is rejected; halt cancels the 1st (#47-P3b R3)") {
    // Keep the first move reliably IN-FLIGHT (the unpaced sim finishes a real move instantly): a
    // ZERO-speed go_to freezes the sim's chase (0x6081=0 -> never advances -> never at_target), and
    // a huge stall threshold disables the no-progress abort. The move parks until WE cancel it.
    ServoConfig cfg = make_config(ControlMode::ProfilePosition);
    cfg.stall_threshold_cycles = 1'000'000'000;
    ServoController ctrl{cfg, sim_factory(ControlMode::ProfilePosition, nullptr)};
    ctrl.start();
    CHECK(wait_until([&] { return ctrl.is_powered(); }, std::chrono::milliseconds(500)));

    std::string first_msg;
    std::thread mover([&] {
        try {
            ctrl.go_to(0.0, 100.0);  // 0 rpm -> frozen chase -> parks in-flight; we cancel it below
        } catch (const std::exception& e) {
            first_msg = e.what();
        }
    });

    CHECK(wait_until([&] { return ctrl.is_moving(); }, std::chrono::milliseconds(500)));  // 1st move is live
    // M7b: a 2nd blocking move CANNOT claim the slot while one is live -> "operation ongoing"
    // (deterministic: the frozen move stays parked). Both go_to AND go_for reject.
    CHECK_THROWS_MSG(ctrl.go_to(500.0, 1.0), BusError, "already in progress");
    CHECK_THROWS_MSG(ctrl.go_for(500.0, 1.0), BusError, "already in progress");

    ctrl.halt();  // R3 cancel -> the 1st move's waiter throws "motor stopped"
    mover.join();
    CHECK(first_msg.find("motor stopped") != std::string::npos);
    // (slot-reclaim after a terminal move is covered deterministically by the first-terminal-wins
    //  test's sequential go_to -> halt -> go_to -> go_to.)
}

TEST("ServoController(PP): R3 disable cancels an in-flight move -> waiter throws 'motor disabled' (#47-P3b R3)") {
    ServoConfig cfg = make_config(ControlMode::ProfilePosition);
    cfg.stall_threshold_cycles = 1'000'000'000;  // frozen 0-speed move parks in-flight (see the single-in-flight test)
    ServoController ctrl{cfg, sim_factory(ControlMode::ProfilePosition, nullptr)};
    ctrl.start();
    CHECK(wait_until([&] { return ctrl.is_powered(); }, std::chrono::milliseconds(500)));

    std::string msg;
    std::thread mover([&] {
        try {
            ctrl.go_to(0.0, 100.0);  // frozen -> parks in-flight
        } catch (const std::exception& e) {
            msg = e.what();
        }
    });
    CHECK(wait_until([&] { return ctrl.is_moving(); }, std::chrono::milliseconds(500)));
    ctrl.disable();  // operator de-energize cancels the move (S1)
    mover.join();
    CHECK(msg.find("motor disabled") != std::string::npos);
}

TEST("ServoController(PP): R3 first-terminal-wins -- cancel AFTER completion is a no-op (#47-P3b R3)") {
    // A halt issued once the move has already COMPLETED must NOT retro-fail it: the go_to already
    // returned success, and the next move still works (the completed gen's terminal state is
    // immutable). Also exercises slot-RECLAIM: sequential go_to -> halt -> go_to -> go_to.
    ServoController ctrl{make_config(ControlMode::ProfilePosition), sim_factory(ControlMode::ProfilePosition, nullptr)};
    ctrl.start();
    CHECK(wait_until([&] { return ctrl.is_powered(); }, std::chrono::milliseconds(500)));

    ctrl.go_to(1000.0, 1.0);  // completes (blocking) -> success
    CHECK(std::abs(ctrl.position_revs() - 1.0) < 0.01);
    ctrl.halt();              // cancel of an already-completed move: no-op (abort sees completed==gen)
    ctrl.go_to(1000.0, 2.0);  // slot reclaimed (terminal) -> a new move succeeds, no stuck "stopped" latch
    CHECK(std::abs(ctrl.position_revs() - 2.0) < 0.01);
}

TEST("#70: a PP move AFTER a SETTLED Stop() completes (bit4 gated past halt release)") {
    // WIRE-PROVEN (task #9): after Stop() the A6 holds Halt (bit8, cw 0x010F). A move issued once
    // the halt has SETTLED (a SEPARATE batch) clears halt AND raises the new-setpoint bit4 -- but if
    // both happen in ONE cycle (cw 0x010F->0x001F) the A6 ignores the coincident edge and the move
    // never acks -> "set-point acknowledge timed out". The policy's halt-settle gate delays bit4
    // until the drive has observed halt clear >= a full cycle. This test drives the SEPARATE-batch
    // (coincident-RELEASE) path -- distinct from R3's coalescing -- against the now-faithful sim.
    // BROKEN-BASELINE: revert the policy gate (kSetpointHaltSettleCycles handling) -> the coincident
    // 0x010F->0x001F edge is rejected by the sim and this go_to times out.
    ServoController ctrl{make_config(ControlMode::ProfilePosition, /*feedback=*/true),
                         sim_factory(ControlMode::ProfilePosition, nullptr, /*feedback=*/true)};
    ctrl.start();
    CHECK(wait_until([&] { return ctrl.is_powered(); }, std::chrono::milliseconds(500)));

    ctrl.go_to(1000.0, 1.0);
    CHECK(std::abs(ctrl.position_revs() - 1.0) < 0.02);

    // Stop() in its OWN batch: let it drain and HOLD Halt (bit8) on the wire for several cycles, so
    // the next go_to is a SEPARATE batch (the halt has SETTLED) -- the wire-proven release path, not
    // the coalescing R3 covers.
    const std::uint64_t c0 = ctrl.loop_cycle();
    ctrl.halt();
    CHECK(wait_until([&] { return ctrl.loop_cycle() > c0 + 15; }, std::chrono::milliseconds(500)));

    ctrl.go_to(1000.0, 2.0);  // move after a SETTLED halt -- must complete (bit4 raised only past halt release)
    CHECK(!ctrl.is_moving());
    CHECK(std::abs(ctrl.position_revs() - 2.0) < 0.02);
}

// --- #61: control_mode INTENT derives the map; switchable auto-routes GoTo->PP / SetRPM->PV ------------
namespace {
// A switchable module config with NO explicit rxpdo/txpdo -> the ctor (validated()) derives the superset.
ServoConfig make_config_switchable() {
    ServoConfig c = make_config(ControlMode::ProfilePosition);  // borrow the scalar fields
    c.mode = ControlMode::Switchable;
    c.rxpdo = ethercat::PdoMap{};  // clear -> apply_derived_pdo_maps() materializes the switchable superset
    c.txpdo = ethercat::PdoMap{};
    c.quick_stop_decel = 100'000;  // enable the policy's PV quick-stop configure
    c.velocity_threshold = 50;     // deterministic velocity gate (bypass position-delta for the jog check)
    return c;
}
// Sim model matching the DERIVED switchable byte layout: RxPDO {6040@0,6060@2,607A@3,6081@7,60FF@11}=15B,
// TxPDO {603F@0,6041@2,6061@4,6064@5,606C@9,6077@13}=15B.
SimSlaveModel make_model_switchable() {
    SimSlaveModel m;
    m.output_bytes = 15;
    m.ctrlword_off = 0;
    m.mode_of_op_off = 2;
    m.target_off = 3;
    m.profile_velocity_off = 7;
    m.velocity_off = 11;
    m.input_bytes = 15;
    m.fault_code_off = 0;
    m.statusword_off = 2;
    m.mode_display_off = 4;
    m.actual_off = 5;
    m.velocity_actual_off = 9;
    m.target_reached_always_set = true;  // A6 bit10 quirk (harmless; module never reads bit10)
    return m;
}
bool rx_has(const std::vector<PdoEntry>& v, std::uint16_t index) {
    for (const auto& e : v) {
        if (e.index == index) {
            return true;
        }
    }
    return false;
}
// #64: sim model matching the DERIVED PP map (no user rxpdo/txpdo) -- RxPDO {6040@0,607A@2,6081@6}=10B
// (PP is SDO-set mode, NO 0x6060), TxPDO {603F@0,6041@2,6061@4,6064@5,606C@9,6077@13}=15B.
SimSlaveModel make_model_derived_pp() {
    SimSlaveModel m;
    m.output_bytes = 10;
    m.ctrlword_off = 0;
    m.target_off = 2;
    m.profile_velocity_off = 6;
    m.input_bytes = 15;
    m.fault_code_off = 0;
    m.statusword_off = 2;
    m.mode_display_off = 4;
    m.actual_off = 5;
    m.velocity_actual_off = 9;
    m.target_reached_always_set = true;  // A6 bit10 quirk (module never reads bit10)
    return m;
}
}  // namespace

TEST("#61: control_mode=switchable with NO explicit map derives the 0x6060 superset RxPDO + full TxPDO") {
    ServoConfig c = make_config_switchable();
    c.apply_derived_pdo_maps();  // (also runs inside the ServoController ctor via validated())
    CHECK(c.rxpdo.entries.count(0x1600) == 1);
    const auto& rx = c.rxpdo.entries.at(0x1600);
    CHECK_EQ(rx.size(), std::size_t{5});
    CHECK(rx_has(rx, 0x6040) && rx_has(rx, 0x6060) && rx_has(rx, 0x607A) && rx_has(rx, 0x6081) && rx_has(rx, 0x60FF));
    CHECK(c.txpdo.entries.count(0x1A00) == 1);
    const auto& tx = c.txpdo.entries.at(0x1A00);
    CHECK(rx_has(tx, 0x6061));  // 0x6061 mapped -> mode-echo gate + switch-confirm work
    // PP-only derivation has no 0x6060 (SDO-set mode, no runtime switch):
    ServoConfig pp = make_config(ControlMode::ProfilePosition);
    pp.rxpdo = ethercat::PdoMap{};
    pp.mode = ControlMode::ProfilePosition;
    pp.apply_derived_pdo_maps();
    CHECK(!rx_has(pp.rxpdo.entries.at(0x1600), 0x6060));
}

TEST("#61: switchable -- go_to runs PP, then set_rpm switches the drive to PV (0x6061 confirms)") {
    SimBackend* sim = nullptr;
    ServoController::BackendFactory factory = [&sim] {
        auto be = std::make_unique<SimBackend>(std::vector<SimSlaveModel>{make_model_switchable()});
        sim = be.get();
        return std::unique_ptr<EcatBackend>(std::move(be));
    };
    ServoController ctrl{make_config_switchable(), factory};
    ctrl.start();
    CHECK(wait_until([&] { return ctrl.is_powered(); }, std::chrono::milliseconds(500)));  // enables in PP (intent default)
    ctrl.go_to(1000.0, 0.5);                                                               // PP move (accepted for switchable) completes
    CHECK(std::abs(ctrl.position_revs() - 0.5) < 0.02);
    ctrl.set_rpm(120.0);  // accepted for switchable -> §6 switch PP->PV, then jog
    CHECK(wait_until([&] { return ctrl.is_moving(); }, std::chrono::milliseconds(800)));  // PV jog started (switch confirmed)
    ctrl.stop();                                                                          // JOIN -> race-free sim read
    CHECK(sim->effective_mode(1) == ethercat::Cia402Mode::ProfileVelocity);               // the drive DID switch to PV
}

TEST("#61: switchable -- an unconfirmable switch reverts SAFE (energized, no throw, no retry storm)") {
    SimSlaveModel m = make_model_switchable();
    m.mode_echo_forced = true;
    m.mode_echo_value = 1;  // 0x6061 stuck at PP(1) -> a PP->PV switch can never confirm
    SimBackend* sim = nullptr;
    ServoController::BackendFactory factory = [m, &sim] {
        auto be = std::make_unique<SimBackend>(std::vector<SimSlaveModel>{m});
        sim = be.get();
        return std::unique_ptr<EcatBackend>(std::move(be));
    };
    ServoController ctrl{make_config_switchable(), factory};
    ctrl.start();
    CHECK(wait_until([&] { return ctrl.is_powered(); }, std::chrono::milliseconds(500)));
    ctrl.set_rpm(120.0);  // request PV; 0x6061 never echoes PV -> mode_switch_failed -> revert intent to PP
    std::this_thread::sleep_for(std::chrono::milliseconds(400));  // past T_switch; revert settled
    CHECK(ctrl.is_powered());  // SAFE: stayed ENERGIZED through the failed switch (no de-energize, no throw)
    CHECK(!ctrl.is_moving());  // reverted to PP at rest (not jogging in an unconfirmed PV)
    // NON-VACUITY (DA): the wrapper's switch_intent_ revert is the SOLE storm-limiter -- the policy
    // re-enters run_mode_switch_ every cycle current!=want and give-up does NOT latch `want`, so
    // WITHOUT the revert an unconfirmable switch re-arms forever (0x6060 oscillates PP<->PV each
    // settle window -> A6-wedge hazard). Prove the revert STOPS it: the sim's 0x6060-write transition
    // count must FREEZE after the single attempt. Rate-independent (no absolute bound): poll twice
    // across a full switch window and assert no growth. Dead-code the revert -> it climbs -> FAILS.
    // (Do NOT read effective_mode/switch_intent_ here: the former is masked by stop()'s intent reset,
    // the latter is a non-atomic live-RT read -- the sim counter is the race-free observable.)
    const std::uint32_t t1 = sim->mode_of_op_write_transitions(1);
    std::this_thread::sleep_for(std::chrono::milliseconds(600));  // > one settle window (200 cyc) at any sane rate
    const std::uint32_t t2 = sim->mode_of_op_write_transitions(1);
    CHECK_EQ(t2, t1);  // FROZEN: no re-arm storm (a non-reverting wrapper would keep transitioning)
}

TEST("#61: a fixed PP config still REJECTS set_rpm (switchable is opt-in)") {
    ServoController ctrl{make_config(ControlMode::ProfilePosition), sim_factory(ControlMode::ProfilePosition, nullptr)};
    ctrl.start();
    CHECK(wait_until([&] { return ctrl.is_powered(); }, std::chrono::milliseconds(500)));
    CHECK_THROWS_MSG(ctrl.set_rpm(60.0), ConfigError, "PP");  // cross-mode call rejected on a fixed PP config
}

TEST("#64: a MINIMAL config (no rxpdo/txpdo, no tolerances) derives the PP map + drives end-to-end") {
    // Mirrors etc/a6-minimal.example.json: only the required fields + empty maps. Proves the #61
    // default-driven path end-to-end -- derive -> validate -> field-resolve -> move -- with NO explicit
    // PDO map and NO tolerance knobs (the two things the minimal config omits).
    ServoConfig c;
    c.ifname = "sim";
    c.mode = ControlMode::ProfilePosition;
    c.max_motor_speed_rpm = 3000.0;
    c.counts_per_rev = kCountsPerRev;  // 131072
    c.motor_rated_current_amps = 2.5;
    c.require_realtime = false;  // offline/CI has no RT sched; the real minimal EXAMPLE keeps the true default
    // rxpdo/txpdo EMPTY; position_tolerance_counts/velocity_threshold 0 -> validated() defaults them.

    // (1) derivation materializes the standard PP map -- SDO-set mode, so NO 0x6060:
    ServoConfig d = c;
    d.apply_derived_pdo_maps();
    CHECK(d.rxpdo.entries.count(0x1600) == 1);
    const auto& rx = d.rxpdo.entries.at(0x1600);
    CHECK_EQ(rx.size(), std::size_t{3});
    CHECK(rx_has(rx, 0x6040) && rx_has(rx, 0x607A) && rx_has(rx, 0x6081) && !rx_has(rx, 0x6060));
    CHECK(d.txpdo.entries.count(0x1A00) == 1);

    // (2) end-to-end: the ctor's validated() derives the map + defaults the 0.5deg tolerance; the
    // controller brings a sim matching the DERIVED layout to OE (via the #45 0x6061 gate) and moves.
    ServoController::BackendFactory factory = [] {
        return std::unique_ptr<EcatBackend>(std::make_unique<SimBackend>(std::vector<SimSlaveModel>{make_model_derived_pp()}));
    };
    ServoController ctrl{c, factory};
    ctrl.start();
    CHECK(wait_until([&] { return ctrl.is_powered(); }, std::chrono::milliseconds(500)));  // derived map + gate -> OE
    ctrl.go_to(1000.0, 1.0);                                                               // field resolution + move on the DERIVED map
    CHECK(std::abs(ctrl.position_revs() - 1.0) < 0.02);
    CHECK(!ctrl.is_moving());
}

// ---------------------------------------------------------------------------
// #22 steady-state SDO: read a CoE object WHILE the RT loop runs (marshaled
// through the RT thread; the single-port-owner invariant holds by construction).
// ---------------------------------------------------------------------------

// A PP model carrying distinctive OD values for the three do_command objects, so a
// successful read proves the value threads end-to-end (not a coincidental default).
SimSlaveModel make_model_sdo() {
    SimSlaveModel m = make_model(ControlMode::ProfilePosition, /*feedback=*/true);
    m.dc_link_voltage_mv = 322'000;          // 0x6079 -> 322.0 V
    m.current_actual_permille = 400;         // 0x6078 -> 0.4 * rated
    m.supported_drive_modes = 0x0000'0185U;  // 0x6502 -> bits 0,2,7,8 = PP,PV,CSP,CSV
    return m;
}

TEST("#22: steady-state SDO read succeeds mid-run; values correct; RT loop keeps cycling") {
    SimBackend* sim = nullptr;
    ServoController::BackendFactory factory = [&sim] {
        auto be = std::make_unique<SimBackend>(std::vector<SimSlaveModel>{make_model_sdo()});
        sim = be.get();
        return std::unique_ptr<EcatBackend>(std::move(be));
    };
    ServoController ctrl{make_config(ControlMode::ProfilePosition, /*feedback=*/true), factory};
    ctrl.start();
    CHECK(wait_until([&] { return ctrl.is_powered(); }, std::chrono::milliseconds(500)));
    (void)sim;

    // Heartbeat BEFORE the read: a working servicer must NOT stall the RT loop -- process()
    // keeps flowing across the (deliberately blocking) mailbox cycle.
    const std::uint64_t cyc0 = ctrl.loop_cycle();

    std::array<std::byte, 4> buf{};
    const std::size_t n_v = ctrl.sdo_read(0x6079, 0, buf, std::chrono::milliseconds(200));
    CHECK_EQ(n_v, std::size_t{4});
    const double volts = ethercat::load_le<std::uint32_t>(std::span<const std::byte>(buf.data(), 4)) / 1000.0;
    CHECK(std::abs(volts - 322.0) < 0.001);

    std::array<std::byte, 2> cbuf{};
    const std::size_t n_c = ctrl.sdo_read(0x6078, 0, cbuf, std::chrono::milliseconds(200));
    CHECK_EQ(n_c, std::size_t{2});
    const std::int16_t permille = ethercat::load_le<std::int16_t>(std::span<const std::byte>(cbuf.data(), 2));
    const double amps = (permille / 1000.0) * ctrl.rated_current_amps();  // rated 2.5 A -> 0.4*2.5 = 1.0 A
    CHECK(std::abs(amps - 1.0) < 0.001);

    std::array<std::byte, 4> mbuf{};
    CHECK_EQ(ctrl.sdo_read(0x6502, 0, mbuf, std::chrono::milliseconds(200)), std::size_t{4});
    CHECK_EQ(ethercat::load_le<std::uint32_t>(std::span<const std::byte>(mbuf.data(), 4)), std::uint32_t{0x0000'0185U});

    // The RT loop advanced across the three reads -> PD never stalled (a dead servicer would
    // have timed the reads out above, so reaching here already proves servicing; this pins the
    // "loop keeps cycling" claim explicitly).
    CHECK(wait_until([&] { return ctrl.loop_cycle() > cyc0 + 2; }, std::chrono::milliseconds(500)));
    CHECK(!ctrl.is_disconnected());
}

TEST("#22: SDO reads succeed CONCURRENTLY with an in-flight move; the move still completes") {
    ServoController ctrl{
        make_config(ControlMode::ProfilePosition, /*feedback=*/true),
        [] { return std::unique_ptr<EcatBackend>(std::make_unique<SimBackend>(std::vector<SimSlaveModel>{make_model_sdo()})); }};
    ctrl.start();
    CHECK(wait_until([&] { return ctrl.is_powered(); }, std::chrono::milliseconds(500)));

    std::atomic<bool> mover_done{false};
    std::atomic<int> reads_ok{0};
    std::thread mover([&] {
        ctrl.go_to(300.0, 8.0);  // a multi-count move; runs while the reader hammers SDO reads
        mover_done.store(true, std::memory_order_release);
    });
    // Hammer SDO reads until the move finishes (or a generous cap) -- all must succeed, and the
    // RT loop must keep servicing PD (the move converges) WHILE the mailbox reads interleave.
    for (int i = 0; i < 200 && !mover_done.load(std::memory_order_acquire); ++i) {
        std::array<std::byte, 4> buf{};
        try {
            if (ctrl.sdo_read(0x6079, 0, buf, std::chrono::milliseconds(200)) == 4) {
                ++reads_ok;
            }
        } catch (const ethercat::Error&) {
            // A read racing the very end of the move / teardown may fail cleanly -- never hang.
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    mover.join();
    CHECK(mover_done.load(std::memory_order_acquire));
    CHECK(reads_ok.load() > 0);                          // at least one interleaved read succeeded mid-move
    CHECK(std::abs(ctrl.position_revs() - 8.0) < 0.05);  // the move converged despite the SDO interleave
}

TEST("#68: config-driven monitors read+convert end-to-end for BOTH default-standard and vendor-override") {
    // The FULL do_command path exercised without the SDK: controller.<monitor>() spec ->
    // sdo_read(spec) -> convert_sdo_monitor(spec). Two configs: the standard-CiA402 defaults
    // (sim OD serves 0x6079/0x6078) and a vendor override (sim OD serves 0x2040:07/:0D).
    const auto read_convert = [](ServoController& ctrl, const SdoMonitor& m) {
        std::array<std::byte, 8> buf{};
        const std::size_t n =
            ctrl.sdo_read(m.index, m.subindex, std::span<std::byte>(buf.data(), m.byte_width()), std::chrono::milliseconds(200));
        CHECK_EQ(n, m.byte_width());
        return convert_sdo_monitor(m, std::span<const std::byte>(buf.data(), n), ctrl.rated_current_amps());
    };

    // (1) DEFAULTS: standard CiA402 objects. Sim serves 0x6079=310000mV, 0x6078=400 permille.
    {
        SimSlaveModel model = make_model(ControlMode::ProfilePosition, /*feedback=*/true);
        model.dc_link_voltage_mv = 310'000;   // -> 310.0 V (÷1000)
        model.current_actual_permille = 400;  // -> 400 * 2.5 / 1000 = 1.0 A
        ServoController::BackendFactory f = [model] {
            return std::unique_ptr<EcatBackend>(std::make_unique<SimBackend>(std::vector<SimSlaveModel>{model}));
        };
        ServoConfig c = make_config(ControlMode::ProfilePosition, /*feedback=*/true);  // rated 2.5 A; monitors default
        ServoController ctrl{c, f};
        ctrl.start();
        CHECK(wait_until([&] { return ctrl.is_powered(); }, std::chrono::milliseconds(500)));
        CHECK(std::abs(read_convert(ctrl, ctrl.voltage_monitor()) - 310.0) < 1e-6);
        CHECK(std::abs(read_convert(ctrl, ctrl.current_monitor()) - 1.0) < 1e-6);
    }

    // (2) VENDOR OVERRIDE: point the monitors at 0x2040:07 (÷10 V) and 0x2040:0D (÷10 A), as the
    // A6 config does. Sim serves 0x2040:07=3154 (315.4V), 0x2040:0D=-55 (-5.5A).
    {
        SimSlaveModel model = make_model(ControlMode::ProfilePosition, /*feedback=*/true);
        model.vendor_bus_voltage_dV = 3154;   // -> 315.4 V (÷10)
        model.vendor_phase_current_dA = -55;  // -> -5.5 A  (÷10, signed)
        ServoController::BackendFactory f = [model] {
            return std::unique_ptr<EcatBackend>(std::make_unique<SimBackend>(std::vector<SimSlaveModel>{model}));
        };
        ServoConfig c = make_config(ControlMode::ProfilePosition, /*feedback=*/true);
        c.voltage_monitor = SdoMonitor{0x2040, 0x07, SdoValueType::U16, SdoScaleKind::Divisor, 10.0};
        c.current_monitor = SdoMonitor{0x2040, 0x0D, SdoValueType::I16, SdoScaleKind::Divisor, 10.0};
        ServoController ctrl{c, f};
        ctrl.start();
        CHECK(wait_until([&] { return ctrl.is_powered(); }, std::chrono::milliseconds(500)));
        CHECK(std::abs(read_convert(ctrl, ctrl.voltage_monitor()) - 315.4) < 1e-6);
        CHECK(std::abs(read_convert(ctrl, ctrl.current_monitor()) - (-5.5)) < 1e-6);
    }
}

TEST("#22/#15: an SDO read after stop() fails cleanly (not running) and does not hang") {
    ServoController ctrl{
        make_config(ControlMode::ProfilePosition, /*feedback=*/true),
        [] { return std::unique_ptr<EcatBackend>(std::make_unique<SimBackend>(std::vector<SimSlaveModel>{make_model_sdo()})); }};
    ctrl.start();
    CHECK(wait_until([&] { return ctrl.is_powered(); }, std::chrono::milliseconds(500)));
    ctrl.stop();  // joins the RT thread + drops the Runner (bus closed)

    // #15: after stop() the controller is not running (rt_runner_ reset), so sdo_read refuses PROMPTLY
    // (ConfigError) rather than reading a stale value off a closed bus. Bound the call to prove no hang.
    std::array<std::byte, 4> buf{};
    const auto t0 = std::chrono::steady_clock::now();
    CHECK_THROWS(ctrl.sdo_read(0x6079, 0, buf, std::chrono::milliseconds(500)), ethercat::Error);
    CHECK(std::chrono::steady_clock::now() - t0 < std::chrono::milliseconds(200));  // immediate, not a 500ms timeout
}

TEST("#22/#15: a reader looping SDO reads across stop() exits cleanly (not-running throw, no hang)") {
    ServoController ctrl{
        make_config(ControlMode::ProfilePosition, /*feedback=*/true),
        [] { return std::unique_ptr<EcatBackend>(std::make_unique<SimBackend>(std::vector<SimSlaveModel>{make_model_sdo()})); }};
    ctrl.start();
    CHECK(wait_until([&] { return ctrl.is_powered(); }, std::chrono::milliseconds(500)));

    // A reader looping SDO reads with a long timeout, while the main thread stops the
    // controller. The reader must never HANG past stop(): a request in flight when the
    // servicer window closes is failed + woken (not left to its full timeout). Bound the join.
    std::atomic<bool> reader_exited{false};
    std::thread reader([&] {
        for (int i = 0; i < 1000; ++i) {
            std::array<std::byte, 4> buf{};
            try {
                (void)ctrl.sdo_read(0x6079, 0, buf, std::chrono::milliseconds(2000));
            } catch (const ethercat::Error&) {
                break;  // window closed -> clean throw -> exit
            }
        }
        reader_exited.store(true, std::memory_order_release);
    });
    std::this_thread::sleep_for(std::chrono::milliseconds(20));  // let the reader get going
    ctrl.stop();
    // The reader must exit well within the 2s per-read timeout -- proving it was WOKEN by the
    // close, not left to time out.
    CHECK(wait_until([&] { return reader_exited.load(std::memory_order_acquire); }, std::chrono::milliseconds(500)));
    reader.join();
}

TEST_MAIN()
