#pragma once

// Validated configuration for one servo drive. Pure data + a validate() that
// throws ethercat::ConfigError with clear text. No SDK, no hardware -- the
// module parses the Viam attributes into this struct and validates before any
// hardware init (deferred-init pattern).

#include <cstdint>
#include <string>
#include <string_view>

namespace ethercat::servo {

// Viam motor control mode. PP = Profile Position (GoTo/GoFor); PV = Profile
// Velocity (SetRPM).
enum class ControlType : std::uint8_t {
    ProfilePosition,
    ProfileVelocity,
};

// Parse "PP"/"PV" (case-insensitive); throws ConfigError on anything else.
ControlType parse_control_type(std::string_view text);
const char* to_string(ControlType type) noexcept;

struct ServoConfig {
    std::string ifname;  // EtherCAT NIC
    ControlType control_type = ControlType::ProfilePosition;

    double max_motor_speed_rpm = 0.0;      // >= 0; the speed clamp
    double peak_current_limit_amps = 0.0;  // >= 0
    double rated_current_amps = 0.0;       // per-drive datum; required if a current limit is set
    double gear_ratio = 1.0;               // motor revs per output rev; != 0
    double counts_per_rev = 0.0;           // encoder counts per motor rev; > 0
    double velocity_scale = 1.0;           // rpm -> device velocity units

    std::uint32_t target_loop_rate_hz = 1000;  // 1..1000
    bool require_realtime = true;              // hard-fail if RT scheduling unavailable
    int rt_priority = 80;                      // SCHED_FIFO priority, 1..99

    // Throws ethercat::ConfigError (clear text) on any invalid field. Pure --
    // no I/O.
    void validate() const;
};

}  // namespace ethercat::servo
