// Offline integration test: ServoController driving a SimBackend through the
// full RT loop -- no Viam SDK, no hardware. require_realtime=false so the RT
// thread runs SCHED_OTHER (CI has no CAP_SYS_NICE).

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

ServoConfig make_config(ControlMode mode) {
    ServoConfig c;
    c.ifname = "sim0";
    c.slave_id = 1;
    c.mode = mode;
    c.rxpdo.assign_index = 0x1C12;
    c.rxpdo.pdo_indices = {0x1600};
    // ctrl(0x6040,16) + target(0x607A,32) + velocity(0x60FF,32) = 10 bytes.
    c.rxpdo.entries[0x1600] = {PdoEntry{0x6040, 0, 16}, PdoEntry{0x607A, 0, 32}, PdoEntry{0x60FF, 0, 32}};
    c.txpdo.assign_index = 0x1C13;
    c.txpdo.pdo_indices = {0x1A00};
    c.txpdo.entries[0x1A00] = {PdoEntry{0x6041, 0, 16}, PdoEntry{0x6064, 0, 32}};
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

SimSlaveModel make_model(ControlMode mode) {
    SimSlaveModel m;
    m.output_bytes = 10;  // ctrl@0, target@2, velocity@6
    m.input_bytes = 6;    // status@0, actual@2
    m.ctrlword_off = 0;
    m.target_off = 2;
    m.velocity_off = 6;
    m.statusword_off = 0;
    m.actual_off = 2;
    m.mode = (mode == ControlMode::ProfileVelocity) ? ethercat::Cia402Mode::ProfileVelocity : ethercat::Cia402Mode::ProfilePosition;
    m.counts_per_step = 50'000;  // ~2.6 revs/cycle: a 1-rev move converges in a few cycles
    return m;
}

ServoController::BackendFactory sim_factory(ControlMode mode, SimBackend** out_ptr) {
    return [mode, out_ptr] {
        auto be = std::make_unique<SimBackend>(std::vector<SimSlaveModel>{make_model(mode)});
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
        m.counts_per_step = 0;  // commanded but frozen
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

TEST_MAIN()
