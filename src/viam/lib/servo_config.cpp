#include "viam/lib/servo_config.hpp"

#include <string>

#include "ethercat/errors.hpp"

namespace ethercat::servo {

namespace {

bool iequals(std::string_view a, std::string_view b) noexcept {
    if (a.size() != b.size()) {
        return false;
    }
    for (std::size_t i = 0; i < a.size(); ++i) {
        const char ca = (a[i] >= 'a' && a[i] <= 'z') ? static_cast<char>(a[i] - 32) : a[i];
        const char cb = (b[i] >= 'a' && b[i] <= 'z') ? static_cast<char>(b[i] - 32) : b[i];
        if (ca != cb) {
            return false;
        }
    }
    return true;
}

}  // namespace

ControlMode parse_control_mode(std::string_view text) {
    if (iequals(text, "PP")) {
        return ControlMode::ProfilePosition;
    }
    if (iequals(text, "PV")) {
        return ControlMode::ProfileVelocity;
    }
    throw ConfigError("control_type '" + std::string(text) + "' is not valid (expected \"PP\" or \"PV\")");
}

const char* to_string(ControlMode mode) noexcept {
    switch (mode) {
        case ControlMode::ProfilePosition:
            return "PP";
        case ControlMode::ProfileVelocity:
            return "PV";
    }
    return "?";
}

void ServoConfig::validate() const {
    if (ifname.empty()) {
        throw ConfigError("servo config: 'ifname' (EtherCAT interface) must not be empty");
    }
    if (slave_id < 1) {
        throw ConfigError("servo config: 'slave_id' must be >= 1");
    }
    if (!(max_motor_speed_rpm >= 0.0)) {  // also rejects NaN
        throw ConfigError("servo config: 'max_motor_speed_rpm' must be >= 0");
    }
    if (peak_current_limit_amps < 0.0) {
        throw ConfigError("servo config: 'peak_current_limit_amps' must be >= 0");
    }
    if (!(motor_rated_current_amps > 0.0)) {
        throw ConfigError(
            "servo config: 'motor_rated_current_amps' must be > 0 "
            "(needed to convert a current limit to torque per-mille)");
    }
    if (gear_ratio == 0.0) {
        throw ConfigError("servo config: 'gear_ratio' must not be 0");
    }
    if (!(counts_per_rev > 0.0)) {
        throw ConfigError("servo config: 'counts_per_rev' must be > 0");
    }
    if (position_tolerance_counts < 0) {
        throw ConfigError("servo config: 'position_tolerance_counts' must be >= 0");
    }
    if (velocity_threshold < 0) {
        throw ConfigError("servo config: 'velocity_threshold' must be >= 0");
    }
    if (target_loop_rate_hz == 0 || target_loop_rate_hz > 1000) {
        throw ConfigError("servo config: 'target_loop_rate_hz' " + std::to_string(target_loop_rate_hz) + " out of range (1..1000)");
    }
    if (rt_priority < 1 || rt_priority > 99) {
        throw ConfigError("servo config: 'rt_priority' " + std::to_string(rt_priority) + " out of range (1..99)");
    }
    if (command_queue_capacity == 0) {
        throw ConfigError("servo config: 'command_queue_capacity' must be > 0");
    }
}

}  // namespace ethercat::servo
