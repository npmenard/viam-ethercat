// Offline integration test: ServoController driving a SimBackend loopback STUB
// through the full RT loop -- no Viam SDK, no hardware. require_realtime=false so
// the RT thread runs SCHED_OTHER (CI has no CAP_SYS_NICE).
//
// SCOPE (post #17 sim demotion): this suite covers the LIFECYCLE + PLUMBING +
// CONCURRENCY the loopback stub can exercise -- the enable ladder to powered, PP
// move-complete, the zeroed-frame contract, Halt-hold, the single-in-flight move
// slot, the steady-state SDO servicer (mid-run / teardown / concurrent), the
// vendor-reset seam, and the WKC-fault latch. The DRIVE-FIDELITY behaviors that
// used to live here (fault-inject/clear FSM, quick-stop ramp shape, mode-switch,
// mode-echo mismatch, encoder noise, handshake-ack suppression) are RETIRED --
// they are drive behaviors, proven HW-first on the bench + the RDK campaign now.
// See docs/offline-test-retirement.md for the retired-case -> HW-check map.

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
#include "ethercat/cia402.hpp"
#include "ethercat/errors.hpp"
#include "ethercat/pdo_buffer.hpp"
#include "ethercat/sim_backend.hpp"
#include "test_harness.hpp"
#include "viam/lib/a6_servo_driver.hpp"
#include "viam/lib/motion_profile.hpp"
#include "viam/lib/servo_config.hpp"
#include "viam/lib/servo_controller.hpp"

using ethercat::BusError;
using ethercat::Cia402Mode;
using ethercat::EcatBackend;
using ethercat::PdoEntry;
using ethercat::SimBackend;
using ethercat::SimSlaveModel;
using ethercat::servo::A6ServoDriver;
using ethercat::servo::ServoConfig;
using ethercat::servo::ServoController;

namespace {

constexpr double kCountsPerRev = 131072.0;

// #18: the PDO map is a FIXED driver-defined superset -- the ServoController ctor sets it
// unconditionally, so a test config carries no map. This is the minimal scalar config.
ServoConfig make_config() {
    ServoConfig c;
    c.ifname = "sim0";
    c.slave_id = 1;
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
    c.quick_stop_decel = 500'000;  // exercise the quick-stop configure gate (0x605A assert + 0x6085 write/readback)
    return c;
}

// The driver's fixed superset PDO layout, as a loopback model:
//   RxPDO: 0x6040 cw@0, 0x6060 mode@2, 0x607A target@3, 0x6081 pvel@7, 0x60FF tvel@11 (15 B)
//   TxPDO: 0x603F@0, 0x6041 status@2, 0x6061 mdisp@4, 0x6064 actual@5, 0x606C velact@9, 0x6077@13 (15 B)
SimSlaveModel superset_model(Cia402Mode mode, std::int32_t counts_per_step) {
    SimSlaveModel m;
    m.output_bytes = 15;
    m.input_bytes = 15;
    m.ctrlword_off = 0;
    m.mode_of_op_off = 2;
    m.target_off = 3;
    m.velocity_off = 11;
    m.statusword_off = 2;
    m.mode_display_off = 4;
    m.actual_off = 5;
    m.velocity_actual_off = 9;
    m.mode = mode;
    m.counts_per_step = counts_per_step;
    return m;
}

ServoController::BackendFactory sim_factory(Cia402Mode mode, std::int32_t counts_per_step, SimBackend** out_ptr = nullptr) {
    return [mode, counts_per_step, out_ptr] {
        auto be = std::make_unique<SimBackend>(std::vector<SimSlaveModel>{superset_model(mode, counts_per_step)});
        if (out_ptr != nullptr) {
            *out_ptr = be.get();
        }
        return std::unique_ptr<EcatBackend>(std::move(be));
    };
}

ServoController::BackendFactory pp_factory(SimBackend** out_ptr = nullptr) {
    return sim_factory(Cia402Mode::ProfilePosition, 50'000, out_ptr);
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

bool rx_has(const std::vector<PdoEntry>& v, std::uint16_t index) {
    for (const auto& e : v) {
        if (e.index == index) {
            return true;
        }
    }
    return false;
}

}  // namespace

// --- lifecycle + move plumbing ---------------------------------------------

TEST("ServoController: start() drives the lifecycle ladder to powered") {
    ServoController ctrl{make_config(), pp_factory()};
    ctrl.start();
    CHECK(wait_until([&] { return ctrl.is_powered(); }, std::chrono::milliseconds(500)));
    CHECK(!ctrl.is_disconnected());
}

TEST("ServoController: go_to propagates + converges; position reads back") {
    ServoController ctrl{make_config(), pp_factory()};
    ctrl.start();
    CHECK(wait_until([&] { return ctrl.is_powered(); }, std::chrono::milliseconds(500)));

    ctrl.go_to(1000.0, 2.0);  // blocks until move-complete
    CHECK(!ctrl.is_moving());
    CHECK(std::abs(ctrl.position_revs() - 2.0) < 0.01);
}

TEST("ServoController: reset_zero offsets the reported position") {
    ServoController ctrl{make_config(), pp_factory()};
    ctrl.start();
    CHECK(wait_until([&] { return ctrl.is_powered(); }, std::chrono::milliseconds(500)));
    ctrl.go_to(1000.0, 3.0);
    ctrl.set_zero();
    CHECK(std::abs(ctrl.position_revs()) < 0.01);  // now reads ~0 at the current actual
}

TEST("ServoController: go_to is absolute in the ZEROED frame after reset_zero") {
    ServoController ctrl{make_config(), pp_factory()};
    ctrl.start();
    CHECK(wait_until([&] { return ctrl.is_powered(); }, std::chrono::milliseconds(500)));

    ctrl.go_to(1000.0, 3.0);  // move to a NONZERO raw position
    ctrl.set_zero();          // zero HERE -> position_revs()==0 at raw 3 revs
    CHECK(std::abs(ctrl.position_revs()) < 0.01);

    ctrl.go_to(1000.0, 1.0);                             // absolute +1 rev in the zeroed frame
    CHECK(std::abs(ctrl.position_revs() - 1.0) < 0.01);  // lands at zeroed 1.0 (raw 4 revs)
}

TEST("ServoController: go_for stays relative regardless of the zero") {
    ServoController ctrl{make_config(), pp_factory()};
    ctrl.start();
    CHECK(wait_until([&] { return ctrl.is_powered(); }, std::chrono::milliseconds(500)));

    ctrl.go_to(1000.0, 2.0);                             // raw 2 revs
    ctrl.set_zero();                                     // zeroed 0 at raw 2 revs
    ctrl.go_for(1000.0, 1.0);                            // relative +1 rev
    CHECK(std::abs(ctrl.position_revs() - 1.0) < 0.01);  // 0 + 1 = 1 (NOT double-shifted to 3)
}

TEST("ServoController: after stop() the fail-safe reports not-powered/disconnected") {
    ServoController ctrl{make_config(), pp_factory()};
    ctrl.start();
    CHECK(wait_until([&] { return ctrl.is_powered(); }, std::chrono::milliseconds(500)));
    ctrl.stop();
    CHECK(!ctrl.is_powered());
    CHECK(!ctrl.is_moving());
    CHECK(ctrl.is_disconnected());
}

TEST("ServoController: Stop (Halt) is sticky -- motor stays stopped, then re-commandable") {
    ServoController ctrl{make_config(), pp_factory()};
    ctrl.start();
    CHECK(wait_until([&] { return ctrl.is_powered(); }, std::chrono::milliseconds(500)));

    ctrl.halt();
    // Several cycles later it must STILL not be moving (Halt latched, not 1-shot).
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    CHECK(!ctrl.is_moving());
    // MOTION-stop is HOLD-ENERGIZED (spec §A R1): Halt holds, the drive NEVER de-energizes.
    CHECK(ctrl.is_powered());
    // A new motion command clears Halt and moves again.
    ctrl.go_to(1000.0, 1.0);
    CHECK(std::abs(ctrl.position_revs() - 1.0) < 0.01);
}

TEST("ServoController: quick-stop OPT-OUT (no decel) -- configure skips the 0x605A/0x6085 SDO, stop coasts") {
    // quick_stop_decel == 0 -> the policy's quick-stop SDO setup (0x605A assert / 0x6085 write) is
    // SKIPPED, and the stop is a disable-voltage coast. Exercises the decel==0 branch.
    ServoConfig cfg = make_config();
    cfg.quick_stop_decel = 0;
    ServoController ctrl{cfg, pp_factory()};
    ctrl.start();  // configure must NOT throw despite no quick-stop SDO (needs_quick_stop=false)
    CHECK(wait_until([&] { return ctrl.is_powered(); }, std::chrono::milliseconds(500)));
    ctrl.go_to(1000.0, 1.0);
    CHECK(std::abs(ctrl.position_revs() - 1.0) < 0.01);
    ctrl.stop();                // LIFECYCLE-stop -> disable-voltage coast; tear down cleanly
    CHECK(!ctrl.is_powered());  // de-energized after the stop
}

// --- velocity (PV) ----------------------------------------------------------

TEST("ServoController(PV): a displaced, stopped motor reports is_moving == false") {
    // PV is_moving must be velocity-based: a stopped-but-displaced motor is NOT moving.
    ServoConfig cfg = make_config();
    cfg.velocity_threshold = 1000;  // counts/s; PV is_moving = |velocity| > this
    ServoController ctrl{cfg, sim_factory(Cia402Mode::ProfileVelocity, 50'000)};
    ctrl.start();
    CHECK(wait_until([&] { return ctrl.is_powered(); }, std::chrono::milliseconds(500)));

    ctrl.set_rpm(60.0);  // integrate a nonzero velocity -> the motor turns
    CHECK(wait_until([&] { return ctrl.is_moving(); }, std::chrono::milliseconds(300)));
    CHECK(std::abs(ctrl.position_revs()) > 0.01);  // displaced from zero

    ctrl.set_rpm(0.0);  // stop commanding velocity -> actual stops advancing
    CHECK(wait_until([&] { return !ctrl.is_moving(); }, std::chrono::milliseconds(500)));
    CHECK(std::abs(ctrl.position_revs()) > 0.01);  // STILL displaced, but NOT moving
}

TEST("ServoController(PV): velocity comes from the 0x606C wire value, not the estimate") {
    ServoController ctrl{make_config(), sim_factory(Cia402Mode::ProfileVelocity, 50'000)};
    ctrl.start();
    CHECK(wait_until([&] { return ctrl.is_powered(); }, std::chrono::milliseconds(500)));

    // PV: the sim integrates s.actual += (0x60FF value) per cycle, so the per-cycle delta it
    // publishes to 0x606C == the commanded device velocity. The estimate path would report
    // delta * loop_rate (1000x). Assert the WIRE value.
    const std::int32_t dev = ethercat::servo::rpm_to_device_velocity(60.0, kCountsPerRev, 1.0);
    ctrl.set_rpm(60.0);
    CHECK(wait_until([&] { return ctrl.velocity_counts() == dev; }, std::chrono::milliseconds(500)));
    CHECK(ctrl.velocity_counts() != dev * 1000);  // NOT the estimate (delta * rate)
}

// --- vendor-reset seam + SDO plumbing (#15 / #39) ---------------------------

TEST("#15 item 2: the A6ServoDriver seam runs the vendor reset pre-spawn; the generic base emits none") {
    {  // A6 subclass -> the 0x2031 write lands before the RT thread exists
        SimBackend* sim = nullptr;
        A6ServoDriver ctrl{make_config(), pp_factory(&sim)};
        ctrl.start();
        CHECK(sim != nullptr);
        const std::vector<std::byte> rec = sim->recorded_sdo(1, 0x2031, 0x01);
        CHECK_EQ(rec.size(), std::size_t{2});
        CHECK(!rec.empty() && rec[0] == std::byte{0x01});
        ctrl.stop();
    }
    {  // generic base -> no vendor object traffic at all
        SimBackend* sim = nullptr;
        ServoController ctrl{make_config(), pp_factory(&sim)};
        ctrl.start();
        CHECK(sim != nullptr);
        CHECK(sim->recorded_sdo(1, 0x2031, 0x01).empty());
        ctrl.stop();
    }
}

TEST("#39: the RT-phase bracket clears after stop() -- a restart's pre-spawn reset succeeds") {
    SimBackend* sim = nullptr;
    A6ServoDriver ctrl{make_config(), pp_factory(&sim)};
    ctrl.start();  // spawn: rt_active declared
    ctrl.stop();   // join: rt_active MUST clear (else the next pre-spawn SDO throws)
    ctrl.reconfigure(make_config());
    CHECK(sim != nullptr);
    CHECK_EQ(sim->recorded_sdo(1, 0x2031, 0x01).size(), std::size_t{2});  // the restart's reset landed
    ctrl.stop();
}

TEST("#15: consumer SDO works during the RT phase and after stop (no port-ownership gate)") {
    SimBackend* sim = nullptr;
    ServoController ctrl{make_config(), pp_factory(&sim)};
    CHECK(ctrl.master_for_sdo() == nullptr);  // pre-first-start: no Master yet
    ctrl.start();
    CHECK(ctrl.master_for_sdo() != nullptr);
    const std::array<std::byte, 2> one{std::byte{0x01}, std::byte{0x00}};
    ctrl.master_for_sdo()->sdo_write(1, 0x2031, 0x01, one);  // succeeds DURING the RT phase
    ctrl.stop();                                             // joined
    ctrl.master_for_sdo()->sdo_write(1, 0x2031, 0x01, one);  // and after stop
    CHECK(sim != nullptr);
    CHECK_EQ(sim->recorded_sdo(1, 0x2031, 0x01).size(), std::size_t{2});
}

// --- single-in-flight move slot (#47-P3b R3) --------------------------------

TEST("ServoController(PP): R3 single-in-flight -- a 2nd blocking move is rejected; halt cancels the 1st") {
    // Keep the first move reliably IN-FLIGHT: a FROZEN drive (counts_per_step=0) never reaches target,
    // and a huge stall threshold disables the no-progress abort, so the move parks until WE cancel it.
    ServoConfig cfg = make_config();
    cfg.stall_threshold_cycles = 1'000'000'000;
    ServoController ctrl{cfg, sim_factory(Cia402Mode::ProfilePosition, /*counts_per_step=*/0)};
    ctrl.start();
    CHECK(wait_until([&] { return ctrl.is_powered(); }, std::chrono::milliseconds(500)));

    std::string first_msg;
    std::thread mover([&] {
        try {
            ctrl.go_to(1000.0, 100.0);  // frozen chase -> parks in-flight; we cancel it below
        } catch (const std::exception& e) {
            first_msg = e.what();
        }
    });

    CHECK(wait_until([&] { return ctrl.is_moving(); }, std::chrono::milliseconds(500)));  // 1st move is live
    // A 2nd blocking move CANNOT claim the slot while one is live -> "already in progress". Both reject.
    CHECK_THROWS_MSG(ctrl.go_to(500.0, 1.0), BusError, "already in progress");
    CHECK_THROWS_MSG(ctrl.go_for(500.0, 1.0), BusError, "already in progress");

    ctrl.halt();  // R3 cancel -> the 1st move's waiter throws "motor stopped"
    mover.join();
    CHECK(first_msg.find("motor stopped") != std::string::npos);
}

TEST("ServoController(PP): R3 disable cancels an in-flight move -> waiter throws 'motor disabled'") {
    ServoConfig cfg = make_config();
    cfg.stall_threshold_cycles = 1'000'000'000;
    ServoController ctrl{cfg, sim_factory(Cia402Mode::ProfilePosition, /*counts_per_step=*/0)};
    ctrl.start();
    CHECK(wait_until([&] { return ctrl.is_powered(); }, std::chrono::milliseconds(500)));

    std::string msg;
    std::thread mover([&] {
        try {
            ctrl.go_to(1000.0, 100.0);  // frozen -> parks in-flight
        } catch (const std::exception& e) {
            msg = e.what();
        }
    });
    CHECK(wait_until([&] { return ctrl.is_moving(); }, std::chrono::milliseconds(500)));
    ctrl.disable();  // operator de-energize cancels the move
    mover.join();
    CHECK(msg.find("motor disabled") != std::string::npos);
}

TEST("ServoController(PP): R3 first-terminal-wins -- cancel AFTER completion is a no-op") {
    ServoController ctrl{make_config(), pp_factory()};
    ctrl.start();
    CHECK(wait_until([&] { return ctrl.is_powered(); }, std::chrono::milliseconds(500)));

    ctrl.go_to(1000.0, 1.0);  // completes (blocking) -> success
    CHECK(std::abs(ctrl.position_revs() - 1.0) < 0.01);
    ctrl.halt();              // cancel of an already-completed move: no-op
    ctrl.go_to(1000.0, 2.0);  // slot reclaimed (terminal) -> a new move succeeds
    CHECK(std::abs(ctrl.position_revs() - 2.0) < 0.01);
}

// --- bus fault latch --------------------------------------------------------

TEST("ServoController: a sustained short WKC latches a bus fault (de-powers + last_error)") {
    // A sustained working-counter shortfall -> the Master latches a BusError; the controller
    // de-energizes and last_error() surfaces the working-counter tier.
    SimBackend* sim = nullptr;
    ServoController ctrl{make_config(), pp_factory(&sim)};
    ctrl.start();
    CHECK(wait_until([&] { return ctrl.is_powered(); }, std::chrono::milliseconds(500)));
    CHECK(sim != nullptr);

    sim->force_short_wkc(true);  // BUS tier: sustained short WKC -> master latches fault
    CHECK(
        wait_until([&] { return ctrl.last_error().find("working-counter fault") != std::string::npos; }, std::chrono::milliseconds(1000)));
    CHECK(!ctrl.is_powered());
}

// --- config: the fixed superset map (#18) -----------------------------------

TEST("#18: set_fixed_pdo_map materializes the 0x6060 superset RxPDO + full TxPDO") {
    ServoConfig c;
    c.set_fixed_pdo_map();
    CHECK(c.rxpdo.entries.count(0x1600) == 1);
    const auto& rx = c.rxpdo.entries.at(0x1600);
    CHECK_EQ(rx.size(), std::size_t{5});
    CHECK(rx_has(rx, 0x6040) && rx_has(rx, 0x6060) && rx_has(rx, 0x607A) && rx_has(rx, 0x6081) && rx_has(rx, 0x60FF));
    CHECK(c.txpdo.entries.count(0x1A00) == 1);
    const auto& tx = c.txpdo.entries.at(0x1A00);
    CHECK_EQ(tx.size(), std::size_t{6});
    CHECK(rx_has(tx, 0x603F) && rx_has(tx, 0x6041) && rx_has(tx, 0x6061) && rx_has(tx, 0x6064) && rx_has(tx, 0x606C) && rx_has(tx, 0x6077));
}

TEST("#64: a MINIMAL config (no tolerances) derives defaults + drives end-to-end") {
    // Only the required fields; position_tolerance_counts/velocity_threshold 0 -> validated() defaults
    // them, and the ctor sets the fixed superset map. Proves derive -> validate -> field-resolve -> move.
    ServoConfig c;
    c.ifname = "sim";
    c.max_motor_speed_rpm = 3000.0;
    c.counts_per_rev = kCountsPerRev;
    c.motor_rated_current_amps = 2.5;
    c.require_realtime = false;
    // position_tolerance_counts / velocity_threshold left 0 -> validated() defaults them.
    ServoController ctrl{c, pp_factory()};
    ctrl.start();
    CHECK(wait_until([&] { return ctrl.is_powered(); }, std::chrono::milliseconds(500)));  // fixed map + gate -> OE
    ctrl.go_to(1000.0, 1.0);
    CHECK(std::abs(ctrl.position_revs() - 1.0) < 0.02);
    CHECK(!ctrl.is_moving());
}

// --- steady-state SDO (#22): read a CoE object WHILE the RT loop runs --------

TEST("#22: SDO reads succeed CONCURRENTLY with an in-flight move; the move still completes") {
    ServoController ctrl{make_config(), pp_factory()};
    ctrl.start();
    CHECK(wait_until([&] { return ctrl.is_powered(); }, std::chrono::milliseconds(500)));

    std::atomic<bool> mover_done{false};
    std::atomic<int> reads_ok{0};
    std::thread mover([&] {
        ctrl.go_to(300.0, 8.0);  // a multi-count move; runs while the reader hammers SDO reads
        mover_done.store(true, std::memory_order_release);
    });
    // Hammer SDO reads until the move finishes -- all must succeed (the stub serves bytes), and the
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

TEST("#22/#15: an SDO read after stop() fails cleanly (not running) and does not hang") {
    ServoController ctrl{make_config(), pp_factory()};
    ctrl.start();
    CHECK(wait_until([&] { return ctrl.is_powered(); }, std::chrono::milliseconds(500)));
    ctrl.stop();  // joins the RT thread + drops the Runner (bus closed)

    // After stop() the controller is not running, so sdo_read refuses PROMPTLY (ConfigError)
    // rather than reading a stale value off a closed bus. Bound the call to prove no hang.
    std::array<std::byte, 4> buf{};
    const auto t0 = std::chrono::steady_clock::now();
    CHECK_THROWS(ctrl.sdo_read(0x6079, 0, buf, std::chrono::milliseconds(500)), ethercat::Error);
    CHECK(std::chrono::steady_clock::now() - t0 < std::chrono::milliseconds(200));  // immediate, not a 500ms timeout
}

TEST("#22/#15: a reader looping SDO reads across stop() exits cleanly (not-running throw, no hang)") {
    ServoController ctrl{make_config(), pp_factory()};
    ctrl.start();
    CHECK(wait_until([&] { return ctrl.is_powered(); }, std::chrono::milliseconds(500)));

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
    // The reader must exit well within the 2s per-read timeout -- proving it was WOKEN by the close.
    CHECK(wait_until([&] { return reader_exited.load(std::memory_order_acquire); }, std::chrono::milliseconds(500)));
    reader.join();
}

TEST_MAIN()
