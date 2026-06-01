#include <cstdint>

#include "ethercat/errors.hpp"
#include "test_harness.hpp"
#include "viam/lib/motion_profile.hpp"
#include "viam/lib/servo_config.hpp"

using ethercat::ConfigError;
using ethercat::servo::ControlType;
using ethercat::servo::ServoConfig;

namespace mp = ethercat::servo;

namespace {

constexpr double kA6CountsPerRev = 131072.0;  // 2^17 absolute encoder

ServoConfig good_config() {
    ServoConfig c;
    c.ifname = "eth0";
    c.control_type = ControlType::ProfilePosition;
    c.max_motor_speed_rpm = 3000.0;
    c.peak_current_limit_amps = 5.0;
    c.rated_current_amps = 2.5;
    c.gear_ratio = 1.0;
    c.counts_per_rev = kA6CountsPerRev;
    c.target_loop_rate_hz = 1000;
    c.rt_priority = 80;
    return c;
}

}  // namespace

TEST("revs<->counts round-trip and gear ratio") {
    CHECK_EQ(mp::revs_to_counts(1.0, kA6CountsPerRev, 1.0), std::int32_t{131072});
    CHECK_EQ(mp::revs_to_counts(2.0, kA6CountsPerRev, 1.0), std::int32_t{262144});
    CHECK_EQ(mp::revs_to_counts(0.5, kA6CountsPerRev, 1.0), std::int32_t{65536});
    // gear ratio 10: 1 output rev = 10 motor revs.
    CHECK_EQ(mp::revs_to_counts(1.0, kA6CountsPerRev, 10.0), std::int32_t{1310720});
    // negative direction.
    CHECK_EQ(mp::revs_to_counts(-0.25, kA6CountsPerRev, 1.0), std::int32_t{-32768});
    // inverse.
    CHECK_EQ(mp::counts_to_revs(131072.0, kA6CountsPerRev, 1.0), 1.0);
    CHECK_EQ(mp::counts_to_revs(1310720.0, kA6CountsPerRev, 10.0), 1.0);
}

TEST("revs_to_counts saturates instead of overflowing int32") {
    CHECK_EQ(mp::revs_to_counts(1e9, kA6CountsPerRev, 1.0), std::int32_t{2147483647});
    CHECK_EQ(mp::revs_to_counts(-1e9, kA6CountsPerRev, 1.0), std::int32_t{-2147483648});
}

TEST("clamp_rpm bounds magnitude symmetrically") {
    CHECK_EQ(mp::clamp_rpm(3500.0, 3000.0), 3000.0);
    CHECK_EQ(mp::clamp_rpm(-3500.0, 3000.0), -3000.0);
    CHECK_EQ(mp::clamp_rpm(1500.0, 3000.0), 1500.0);
    CHECK_EQ(mp::clamp_rpm(1500.0, -1.0), 0.0);  // negative bound -> fully clamped
}

TEST("rpm<->device velocity via scale") {
    CHECK_EQ(mp::rpm_to_device_velocity(1000.0, 1.0), std::int32_t{1000});
    CHECK_EQ(mp::rpm_to_device_velocity(1000.0, 2.5), std::int32_t{2500});
    CHECK_EQ(mp::device_velocity_to_rpm(2500.0, 2.5), 1000.0);
}

TEST("amps -> torque per-mille (A6: rated current scaling, clamp 0..4000)") {
    CHECK_EQ(mp::amps_to_torque_permille(2.5, 2.5), std::uint16_t{1000});   // rated -> 1000 per-mille
    CHECK_EQ(mp::amps_to_torque_permille(5.0, 2.5), std::uint16_t{2000});   // 2x rated
    CHECK_EQ(mp::amps_to_torque_permille(20.0, 2.5), std::uint16_t{4000});  // clamps at 4000
    CHECK_EQ(mp::amps_to_torque_permille(0.0, 2.5), std::uint16_t{0});
    CHECK_EQ(mp::amps_to_torque_permille(5.0, 0.0), std::uint16_t{0});  // no rated current -> 0
}

TEST("parse_control_type accepts PP/PV (case-insensitive), rejects others") {
    CHECK_EQ(mp::parse_control_type("PP"), ControlType::ProfilePosition);
    CHECK_EQ(mp::parse_control_type("pv"), ControlType::ProfileVelocity);
    CHECK_THROWS_MSG(mp::parse_control_type("CSP"), ConfigError, "not valid");
    CHECK(std::string("PP") == mp::to_string(ControlType::ProfilePosition));
    CHECK(std::string("PV") == mp::to_string(ControlType::ProfileVelocity));
}

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
        c.peak_current_limit_amps = 5.0;
        c.rated_current_amps = 0.0;
        CHECK_THROWS_MSG(c.validate(), ConfigError, "rated_current_amps");
    }
}

TEST_MAIN()
