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

using ethercat::ConfigError;
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
    return c;
}

}  // namespace

TEST("ServoConfig::validate accepts a good config") {
    good_config().validate();  // must not throw
}

TEST("ServoConfig::validate rejects each invalid field with clear text") {
    {
        ServoConfig c = good_config();
        c.ifname.clear();
        CHECK_THROWS_MSG(c.validate(), ConfigError, "ifname");
    }
    {
        ServoConfig c = good_config();
        c.max_motor_speed_rpm = -1.0;
        CHECK_THROWS_MSG(c.validate(), ConfigError, "max_motor_speed_rpm");
    }
    {
        ServoConfig c = good_config();
        c.motor_rated_current_amps = 0.0;
        CHECK_THROWS_MSG(c.validate(), ConfigError, "motor_rated_current_amps");
    }
    {
        ServoConfig c = good_config();
        c.gear_ratio = 0.0;
        CHECK_THROWS_MSG(c.validate(), ConfigError, "gear_ratio");
    }
    {
        ServoConfig c = good_config();
        c.counts_per_rev = 0.0;
        CHECK_THROWS_MSG(c.validate(), ConfigError, "counts_per_rev");
    }
    {
        ServoConfig c = good_config();
        c.position_tolerance_counts = -1;
        CHECK_THROWS_MSG(c.validate(), ConfigError, "position_tolerance_counts");
    }
    {
        ServoConfig c = good_config();
        c.target_loop_rate_hz = 2000;
        CHECK_THROWS_MSG(c.validate(), ConfigError, "target_loop_rate_hz");
    }
    {
        ServoConfig c = good_config();
        c.rt_priority = 0;
        CHECK_THROWS_MSG(c.validate(), ConfigError, "rt_priority");
    }
    {
        ServoConfig c = good_config();
        c.command_queue_capacity = 0;
        CHECK_THROWS_MSG(c.validate(), ConfigError, "command_queue_capacity");
    }
}

TEST_MAIN()
