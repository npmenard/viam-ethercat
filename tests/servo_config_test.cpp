#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string>

#include "ethercat/errors.hpp"
#include "ethercat/pdo_buffer.hpp"  // store_le, to build raw test bytes
#include "test_harness.hpp"
#include "viam/lib/servo_config.hpp"

using ethercat::Error;
using ethercat::servo::ServoConfig;

namespace {

ServoConfig good_config() {
    ServoConfig c;
    c.ifname = "eth0";
    c.slave_id = 1;
    c.max_motor_speed_rpm = 3000.0;
    c.motor_rated_current_amps = 2.5;
    c.gear_ratio = 1.0;
    c.counts_per_rev = 131072.0;
    c.position_tolerance_counts = 10;
    c.target_loop_rate_hz = 1000;
    c.rt_priority = 80;
    c.command_queue_capacity = 64;
    c.use_distributed_clocks = true;  // the default control mode (csp) requires it
    return c;
}

}  // namespace

TEST("ServoConfig::validate accepts a good config") {
    good_config().validate();  // must not throw
}

TEST("control_mode csp requires distributed clocks; profile does not") {
    ServoConfig c = good_config();
    c.use_distributed_clocks = false;
    CHECK_THROWS_MSG(c.validate(), Error, "use_distributed_clocks");
    c.motion_mode = ethercat::servo::MotionModeKind::Profile;
    c.validate();  // must not throw
}

TEST("the RxPDO map follows the control mode; the TxPDO map does not") {
    ServoConfig c = good_config();
    c.set_fixed_pdo_map();
    const auto& cyclic = c.rxpdo.entries.at(0x1600);
    CHECK_EQ(cyclic.size(), std::size_t{2});  // 0x6040 + 0x607A, no mode byte
    CHECK_EQ(cyclic[1].index, std::uint16_t{0x607A});
    c.motion_mode = ethercat::servo::MotionModeKind::Profile;
    c.set_fixed_pdo_map();
    const auto& profile = c.rxpdo.entries.at(0x1600);
    CHECK_EQ(profile.size(), std::size_t{5});
    CHECK_EQ(profile[1].index, std::uint16_t{0x6060});
    CHECK_EQ(c.txpdo.entries.at(0x1A00).size(), std::size_t{6});
}

TEST("ServoConfig::validate rejects each invalid field with clear text") {
    {
        ServoConfig c = good_config();
        c.ifname.clear();
        CHECK_THROWS_MSG(c.validate(), Error, "ifname");
    }
    {
        ServoConfig c = good_config();
        c.max_motor_speed_rpm = -1.0;
        CHECK_THROWS_MSG(c.validate(), Error, "max_motor_speed_rpm");
    }
    {
        ServoConfig c = good_config();
        c.motor_rated_current_amps = 0.0;
        CHECK_THROWS_MSG(c.validate(), Error, "motor_rated_current_amps");
    }
    {
        ServoConfig c = good_config();
        c.gear_ratio = 0.0;
        CHECK_THROWS_MSG(c.validate(), Error, "gear_ratio");
    }
    {
        ServoConfig c = good_config();
        c.counts_per_rev = 0.0;
        CHECK_THROWS_MSG(c.validate(), Error, "counts_per_rev");
    }
    {
        ServoConfig c = good_config();
        c.position_tolerance_counts = -1;
        CHECK_THROWS_MSG(c.validate(), Error, "position_tolerance_counts");
    }
    {
        ServoConfig c = good_config();
        c.target_loop_rate_hz = 2000;
        CHECK_THROWS_MSG(c.validate(), Error, "target_loop_rate_hz");
    }
    {
        ServoConfig c = good_config();
        c.rt_priority = 0;
        CHECK_THROWS_MSG(c.validate(), Error, "rt_priority");
    }
    {
        ServoConfig c = good_config();
        c.command_queue_capacity = 0;
        CHECK_THROWS_MSG(c.validate(), Error, "command_queue_capacity");
    }
}

// The homing block is validated at config time, so a bad number is refused while nothing is moving
// rather than on the call that would drive the axis into a hard stop with it.
TEST("homing config rejects what the drive cannot be asked") {
    ServoConfig ok = good_config();
    ok.homing.method = -4;
    ok.homing.speed_rpm = 20.0;
    ok.validate();  // must not throw: the torque object is optional (limit-switch methods need none)
    {
        ServoConfig c = ok;
        c.homing.speed_rpm = 0.0;
        CHECK_THROWS_MSG(c.validate(), Error, "speed_rpm");
    }
    {
        ServoConfig c = ok;
        c.homing.torque_percent = 400.0;
        CHECK_THROWS_MSG(c.validate(), Error, "torque_percent");
    }
    {
        ServoConfig c = ok;
        c.homing.torque_percent = 50.0;  // a threshold with no object to write it to
        CHECK_THROWS_MSG(c.validate(), Error, "torque_object");
    }
    {
        ServoConfig c = ok;
        c.homing.timeout_s = 0.0;
        CHECK_THROWS_MSG(c.validate(), Error, "timeout_s");
    }
    {
        ServoConfig c = good_config();  // no method: the block is inert, not invalid
        c.homing.speed_rpm = 0.0;
        c.validate();
    }
}

TEST_MAIN()
