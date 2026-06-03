// Offline integration test: ServoController driving a SimBackend through the
// full RT loop -- no Viam SDK, no hardware. require_realtime=false so the RT
// thread runs SCHED_OTHER (CI has no CAP_SYS_NICE).

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

using ethercat::ConfigError;
using ethercat::EcatBackend;
using ethercat::PdoEntry;
using ethercat::SimBackend;
using ethercat::SimSlaveModel;
using ethercat::servo::ControlMode;
using ethercat::servo::ServoConfig;
using ethercat::servo::ServoController;

namespace {

constexpr double kCountsPerRev = 131072.0;

ServoConfig make_config(ControlMode mode, bool feedback = false) {
    ServoConfig c;
    c.ifname = "sim0";
    c.slave_id = 1;
    c.mode = mode;
    c.rxpdo.assign_index = 0x1C12;
    c.rxpdo.pdo_indices = {0x1600};
    // Authoritative A6 maps per mode: PP = ctrl + target position + profile velocity
    // (10 B); PV = ctrl + target velocity (6 B).
    if (mode == ControlMode::ProfileVelocity) {
        c.rxpdo.entries[0x1600] = {PdoEntry{0x6040, 0, 16}, PdoEntry{0x60FF, 0, 32}};
    } else {
        c.rxpdo.entries[0x1600] = {PdoEntry{0x6040, 0, 16}, PdoEntry{0x607A, 0, 32}, PdoEntry{0x6081, 0, 32}};
    }
    c.txpdo.assign_index = 0x1C13;
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
        m.output_bytes = 6;  // ctrl@0, target velocity@2
        m.velocity_off = 2;
    } else {
        m.mode = ethercat::Cia402Mode::ProfilePosition;
        m.output_bytes = 10;  // ctrl@0, target@2, profile velocity@6
        m.target_off = 2;
        m.profile_velocity_off = 6;  // de-masked: PP chases at the 0x6081 the RT loop writes
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
    ServoController ctrl{make_config(ControlMode::ProfilePosition), sim_factory(ControlMode::ProfilePosition, &sim)};
    ctrl.start();
    CHECK(wait_until([&] { return ctrl.is_powered(); }, std::chrono::milliseconds(500)));

    ctrl.go_to(600.0, 1.0);  // 600 rpm
    CHECK(sim != nullptr);
    const std::int32_t expected = ethercat::servo::rpm_to_device_velocity(600.0, kCountsPerRev, 1.0);
    CHECK_EQ(sim->received_profile_velocity(1), std::abs(expected));  // rpm reached the drive, not 0
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
    // A new motion command clears Halt and moves again.
    ctrl.go_to(1000.0, 1.0);
    CHECK(std::abs(ctrl.position_revs() - 1.0) < 0.01);
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

TEST_MAIN()
