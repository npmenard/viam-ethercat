// DA's gate: hammer the ServoController's non-RT API from N threads while the RT
// loop drains, then stop()/reconfigure() mid-load. Built BOTH normally and under
// -fsanitize=thread (the destroy-vs-pop / reset-vs-access races only surface
// under TSan). require_realtime=false (CI has no CAP_SYS_NICE).

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <thread>
#include <vector>

#include "ethercat/backend.hpp"
#include "ethercat/errors.hpp"
#include "ethercat/sim_backend.hpp"
#include "test_harness.hpp"
#include "viam/lib/servo_config.hpp"
#include "viam/lib/servo_controller.hpp"

using ethercat::EcatBackend;
using ethercat::PdoEntry;
using ethercat::SimBackend;
using ethercat::SimSlaveModel;
using ethercat::servo::ControlMode;
using ethercat::servo::ServoConfig;
using ethercat::servo::ServoController;

#if defined(__SANITIZE_THREAD__)
#define ECAT_TSAN 1
#elif defined(__has_feature)
#if __has_feature(thread_sanitizer)
#define ECAT_TSAN 1
#endif
#endif

namespace {

#ifdef ECAT_TSAN
constexpr int kReconfigures = 5;
constexpr auto kBurst = std::chrono::milliseconds(15);
#else
constexpr int kReconfigures = 20;
constexpr auto kBurst = std::chrono::milliseconds(8);
#endif

ServoConfig make_config() {
    ServoConfig c;
    c.ifname = "sim0";
    c.slave_id = 1;
    c.mode = ControlMode::ProfilePosition;
    c.rxpdo.pdo_indices = {0x1600};
    c.rxpdo.entries[0x1600] = {PdoEntry{0x6040, 0, 16}, PdoEntry{0x607A, 0, 32}};
    c.txpdo.pdo_indices = {0x1A00};
    c.txpdo.entries[0x1A00] = {PdoEntry{0x6041, 0, 16}, PdoEntry{0x6064, 0, 32}};
    c.max_motor_speed_rpm = 3000.0;
    c.motor_rated_current_amps = 2.5;
    c.gear_ratio = 1.0;
    c.counts_per_rev = 131072.0;
    c.position_tolerance_counts = 20;
    c.velocity_threshold = 1'000'000'000;
    c.target_loop_rate_hz = 500;  // gentler under TSan
    c.require_realtime = false;
    c.command_queue_capacity = 64;
    c.handshake_timeout_cycles = 1000;
    return c;
}

ServoController::BackendFactory sim_factory() {
    return [] {
        SimSlaveModel m;
        m.output_bytes = 6;
        m.input_bytes = 6;
        m.target_off = 2;
        m.actual_off = 2;
        m.counts_per_step = 50'000;
        return std::unique_ptr<EcatBackend>(std::make_unique<SimBackend>(std::vector<SimSlaveModel>{m}));
    };
}

// Hammer the non-RT API; swallow the expected lifecycle exceptions (the move is
// superseded/disconnected by stop/reconfigure, or the queue is briefly full).
void hammer(ServoController& ctrl, const std::atomic<bool>& stop) {
    while (!stop.load(std::memory_order_relaxed)) {
        try {
            ctrl.go_to(1000.0, 1.0);
        } catch (const ethercat::Error&) {
        }
        (void)ctrl.is_powered();
        (void)ctrl.is_moving();
        (void)ctrl.position_revs();
        (void)ctrl.last_error();  // exercise the lock-free fault-pair read (rt_error_/fault_wkc/expected_wkc) vs the RT publisher
        ctrl.halt();
    }
}

}  // namespace

TEST("teardown under load: reconfigure churn + parked go_to + concurrent API (TSan)") {
    ServoController ctrl{make_config(), sim_factory()};
    ctrl.start();

    std::atomic<bool> stop_hammer{false};
    std::vector<std::thread> threads;
    threads.reserve(4);
    for (int i = 0; i < 4; ++i) {
        threads.emplace_back([&] { hammer(ctrl, stop_hammer); });
    }

    // While the API is hammered + parked go_to waiters exist, churn reconfigure
    // (stop+join -> rebuild -> restart). This is the destroy-vs-pop / reset-vs-
    // access stress -- must be clean under TSan and never hang/UAF.
    for (int i = 0; i < kReconfigures; ++i) {
        std::this_thread::sleep_for(kBurst);
        ctrl.reconfigure(make_config());
    }

    stop_hammer.store(true, std::memory_order_relaxed);
    for (auto& t : threads) {
        t.join();
    }
    CHECK(true);  // reaching here without a hang / TSan race / UAF is the assertion
}

TEST("teardown under load: stop() while go_to is parked wakes it promptly") {
    ServoController ctrl{make_config(), sim_factory()};
    ctrl.start();

    std::atomic<bool> returned{false};
    std::thread mover([&] {
        try {
            ctrl.go_to(1.0, 100000.0);  // a huge move that won't complete before stop()
        } catch (const ethercat::Error&) {
        }
        returned.store(true, std::memory_order_release);
    });

    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    ctrl.stop();  // must wake the parked waiter within the wait_for slice

    // The parked go_to must return promptly (well under its 30s move timeout).
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (!returned.load(std::memory_order_acquire) && std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    CHECK(returned.load(std::memory_order_acquire));
    mover.join();
}

TEST("teardown: double stop() + dtor is idempotent (no double-join)") {
    ServoController ctrl{make_config(), sim_factory()};
    ctrl.start();
    ctrl.stop();
    ctrl.stop();  // idempotent -- must not std::terminate on a non-joinable thread
    CHECK(ctrl.is_disconnected());
    // dtor runs stop() a third time on scope exit.
}

TEST_MAIN()
