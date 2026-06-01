// Module smoke test: drive a ServoMotor (the rdk:component:motor glue) over a
// SimBackend-backed ServoController -- no Viam server, no hardware. Exercises the
// Motor API surface directly on the object (like the UR module's test.cpp).
//
// Needs the Viam C++ SDK (ServoMotor derives from Motor), so it builds only in
// the ETHERCAT_BUILD_MODULE path (the Docker/CI image), not the bare-library path.

#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <thread>
#include <vector>

#include <viam/sdk/common/proto_value.hpp>

#include "ethercat/backend.hpp"
#include "ethercat/sim_backend.hpp"
#include "test_harness.hpp"
#include "viam/lib/servo_config.hpp"
#include "viam/lib/servo_controller.hpp"
#include "viam/module/servo_motor.hpp"

using ethercat::EcatBackend;
using ethercat::PdoEntry;
using ethercat::SimBackend;
using ethercat::SimSlaveModel;
using ethercat::servo::ControlMode;
using ethercat::servo::ServoConfig;
using ethercat::servo::ServoController;
using ethercat::servo::ServoMotor;
using viam::sdk::ProtoStruct;
using viam::sdk::ProtoValue;

namespace {

ServoConfig make_config() {
    ServoConfig c;
    c.ifname = "sim0";
    c.slave_id = 1;
    c.mode = ControlMode::ProfilePosition;
    c.rxpdo.assign_index = 0x1C12;
    c.rxpdo.pdo_indices = {0x1600};
    c.rxpdo.entries[0x1600] = {PdoEntry{0x6040, 0, 16}, PdoEntry{0x607A, 0, 32}, PdoEntry{0x60FF, 0, 32}};
    c.txpdo.assign_index = 0x1C13;
    c.txpdo.pdo_indices = {0x1A00};
    c.txpdo.entries[0x1A00] = {PdoEntry{0x6041, 0, 16}, PdoEntry{0x6064, 0, 32}};
    c.max_motor_speed_rpm = 3000.0;
    c.motor_rated_current_amps = 2.5;
    c.gear_ratio = 1.0;
    c.counts_per_rev = 131072.0;
    c.position_tolerance_counts = 20;
    c.velocity_threshold = 1'000'000'000;
    c.target_loop_rate_hz = 1000;
    c.require_realtime = false;  // CI has no CAP_SYS_NICE
    c.command_queue_capacity = 64;
    c.handshake_timeout_cycles = 1000;
    return c;
}

std::unique_ptr<ServoController> make_sim_controller() {
    auto factory = [] {
        SimSlaveModel m;
        m.output_bytes = 10;  // ctrl@0, target@2, velocity@6
        m.input_bytes = 6;    // status@0, actual@2
        m.ctrlword_off = 0;
        m.target_off = 2;
        m.velocity_off = 6;
        m.statusword_off = 0;
        m.actual_off = 2;
        m.mode = ethercat::Cia402Mode::ProfilePosition;
        m.counts_per_step = 50'000;
        return std::unique_ptr<EcatBackend>(std::make_unique<SimBackend>(std::vector<SimSlaveModel>{m}));
    };
    return std::make_unique<ServoController>(make_config(), factory);
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

TEST("ServoMotor: get_properties reports position_reporting") {
    ServoMotor motor{"test-motor", make_sim_controller()};
    CHECK(motor.get_properties(ProtoStruct{}).position_reporting);
}

TEST("ServoMotor: go_to moves, position reads back, is_moving settles, stop is safe") {
    ServoMotor motor{"test-motor", make_sim_controller()};
    CHECK(wait_until([&] { return motor.get_power_status(ProtoStruct{}).is_on; }, std::chrono::milliseconds(500)));

    motor.go_to(1000.0, 2.0, ProtoStruct{});  // blocks to completion
    CHECK(std::abs(motor.get_position(ProtoStruct{}) - 2.0) < 0.01);
    CHECK(!motor.is_moving());

    motor.stop(ProtoStruct{});
    CHECK(!motor.is_moving());
}

TEST("ServoMotor: set_power is rejected with a clear error") {
    ServoMotor motor{"test-motor", make_sim_controller()};
    CHECK_THROWS(motor.set_power(0.5, ProtoStruct{}), std::runtime_error);
}

TEST("ServoMotor: do_command status returns the diagnostic map") {
    ServoMotor motor{"test-motor", make_sim_controller()};
    CHECK(wait_until([&] { return motor.get_power_status(ProtoStruct{}).is_on; }, std::chrono::milliseconds(500)));

    ProtoStruct cmd;
    cmd.emplace("status", ProtoValue(true));
    const ProtoStruct result = motor.do_command(cmd);

    const auto it = result.find("status");
    CHECK(it != result.end());
    const ProtoStruct* const status = it->second.get<ProtoStruct>();
    CHECK(status != nullptr);
    CHECK(status->find("is_powered") != status->end());
    CHECK(status->find("position") != status->end());
    CHECK(status->find("last_error") != status->end());
}

TEST("ServoMotor: an unknown do_command throws") {
    ServoMotor motor{"test-motor", make_sim_controller()};
    ProtoStruct cmd;
    cmd.emplace("bogus", ProtoValue(true));
    CHECK_THROWS(motor.do_command(cmd), std::runtime_error);
}

TEST_MAIN()
