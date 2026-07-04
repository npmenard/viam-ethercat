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
using ethercat::SimBackend;
using ethercat::SimSlaveModel;
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

// #18: the PDO map is a FIXED driver-defined superset (set unconditionally by the
// ServoController ctor) -- the config carries no map. The sim model mirrors that superset
// layout (see superset_model below), so the Master's remapped offsets line up.
ServoConfig make_config() {
    ServoConfig c;
    c.ifname = "sim0";
    c.slave_id = 1;
    c.max_motor_speed_rpm = 3000.0;
    c.motor_rated_current_amps = 2.5;
    c.gear_ratio = 1.0;
    c.counts_per_rev = 131072.0;
    c.position_tolerance_counts = 20;
    c.target_loop_rate_hz = 500;  // gentler under TSan
    c.require_realtime = false;
    c.command_queue_capacity = 64;
    return c;
}

// The driver's fixed superset PDO layout, as a loopback model:
//   RxPDO: 0x6040 cw@0, 0x6060 mode@2, 0x607A target@3, 0x6081 pvel@7, 0x60FF tvel@11 (15 B)
//   TxPDO: 0x603F@0, 0x6041 status@2, 0x6061 mdisp@4, 0x6064 actual@5, 0x606C@9, 0x6077@13 (15 B)
SimSlaveModel superset_model(std::int32_t counts_per_step) {
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
    m.counts_per_step = counts_per_step;
    return m;
}

ServoController::BackendFactory sim_factory() {
    return [] { return std::unique_ptr<EcatBackend>(std::make_unique<SimBackend>(std::vector<SimSlaveModel>{superset_model(50'000)})); };
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
