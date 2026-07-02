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
using ethercat::servo::ControlMode;
using ethercat::servo::convert_sdo_monitor;
using ethercat::servo::parse_control_mode;
using ethercat::servo::parse_sdo_value_type;
using ethercat::servo::SdoMonitor;
using ethercat::servo::SdoScaleKind;
using ethercat::servo::SdoValueType;
using ethercat::servo::ServoConfig;
using ethercat::servo::to_string;

namespace {

ServoConfig good_config() {
    ServoConfig c;
    c.ifname = "eth0";
    c.slave_id = 1;
    c.mode = ControlMode::ProfilePosition;
    c.max_motor_speed_rpm = 3000.0;
    c.peak_current_limit_amps = 5.0;
    c.motor_rated_current_amps = 2.5;
    c.gear_ratio = 1.0;
    c.counts_per_rev = 131072.0;
    c.position_tolerance_counts = 10;
    c.velocity_threshold = 5;
    c.target_loop_rate_hz = 1000;
    c.rt_priority = 80;
    c.command_queue_capacity = 64;
    return c;
}

}  // namespace

TEST("parse_control_mode accepts PP/PV (case-insensitive), rejects others") {
    CHECK_EQ(parse_control_mode("PP"), ControlMode::ProfilePosition);
    CHECK_EQ(parse_control_mode("pv"), ControlMode::ProfileVelocity);
    CHECK_THROWS_MSG(parse_control_mode("CSP"), ConfigError, "not valid");
    CHECK(std::string("PP") == to_string(ControlMode::ProfilePosition));
    CHECK(std::string("PV") == to_string(ControlMode::ProfileVelocity));
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

// ---------------------------------------------------------------------------
// #68 SdoMonitor: config-driven do_command SDO decode + scale.
// ---------------------------------------------------------------------------

TEST("#68: parse_sdo_value_type accepts the six widths (case-insensitive), rejects junk") {
    CHECK(parse_sdo_value_type("u8") == SdoValueType::U8);
    CHECK(parse_sdo_value_type("I8") == SdoValueType::I8);
    CHECK(parse_sdo_value_type("u16") == SdoValueType::U16);
    CHECK(parse_sdo_value_type("i16") == SdoValueType::I16);
    CHECK(parse_sdo_value_type("U32") == SdoValueType::U32);
    CHECK(parse_sdo_value_type("i32") == SdoValueType::I32);
    CHECK_THROWS_MSG(parse_sdo_value_type("u24"), ConfigError, "not valid");
    CHECK_THROWS_MSG(parse_sdo_value_type("float"), ConfigError, "u8/i8/u16/i16/u32/i32");
}

TEST("#68: convert_sdo_monitor decodes each type + scale exactly") {
    // Build raw little-endian bytes with the shared store_le (no hand-rolled packing).
    const auto le = [](auto v) {
        std::array<std::byte, 8> b{};
        ethercat::store_le<decltype(v)>(std::span<std::byte>(b.data(), sizeof(v)), v);
        return b;
    };

    // Standard voltage default: U32 mV / 1000 -> V.
    const SdoMonitor v_std{0x6079, 0x00, SdoValueType::U32, SdoScaleKind::Divisor, 1000.0};
    CHECK(std::abs(convert_sdo_monitor(v_std, le(std::uint32_t{310000}), 0.0) - 310.0) < 1e-9);

    // A6 vendor voltage: U16 dV / 10 -> V (the bench-read 3154 -> 315.4).
    const SdoMonitor v_a6{0x2040, 0x07, SdoValueType::U16, SdoScaleKind::Divisor, 10.0};
    CHECK(std::abs(convert_sdo_monitor(v_a6, le(std::uint16_t{3154}), 0.0) - 315.4) < 1e-9);

    // Standard current default: I16 per-mille * rated. 400 permille * 2.5 A / 1000 = 1.0 A.
    const SdoMonitor c_std{0x6078, 0x00, SdoValueType::I16, SdoScaleKind::RatedCurrentPermille, 1.0};
    CHECK(std::abs(convert_sdo_monitor(c_std, le(std::int16_t{400}), 2.5) - 1.0) < 1e-9);
    // SIGNED decode: -300 permille * 2.5 / 1000 = -0.75 A.
    CHECK(std::abs(convert_sdo_monitor(c_std, le(std::int16_t{-300}), 2.5) - (-0.75)) < 1e-9);

    // A6 vendor current: I16 dA / 10 -> A, signed. -55 -> -5.5 A; 0 -> 0.0 A.
    const SdoMonitor c_a6{0x2040, 0x0D, SdoValueType::I16, SdoScaleKind::Divisor, 10.0};
    CHECK(std::abs(convert_sdo_monitor(c_a6, le(std::int16_t{-55}), 0.0) - (-5.5)) < 1e-9);
    CHECK(std::abs(convert_sdo_monitor(c_a6, le(std::int16_t{0}), 0.0)) < 1e-9);

    CHECK_EQ(v_std.byte_width(), std::size_t{4});
    CHECK_EQ(c_a6.byte_width(), std::size_t{2});
}

TEST("#68: convert_sdo_monitor throws on a short buffer (drive returned too few bytes)") {
    const SdoMonitor v{0x6079, 0x00, SdoValueType::U32, SdoScaleKind::Divisor, 1000.0};
    std::array<std::byte, 2> too_short{};
    CHECK_THROWS_MSG(convert_sdo_monitor(v, std::span<const std::byte>(too_short), 0.0), ConfigError, "need 4");
}

TEST("#68: validate rejects a 0-index or 0-divisor sdo monitor") {
    {
        ServoConfig c = good_config();
        c.voltage_monitor.index = 0;
        CHECK_THROWS_MSG(c.validate(), ConfigError, "sdo_monitors.voltage");
    }
    {
        ServoConfig c = good_config();
        c.current_monitor.scale_kind = SdoScaleKind::Divisor;
        c.current_monitor.divisor = 0.0;
        CHECK_THROWS_MSG(c.validate(), ConfigError, "sdo_monitors.current");
    }
    // The shipped defaults pass validation.
    CHECK((good_config().validate(), true));
}

TEST_MAIN()
