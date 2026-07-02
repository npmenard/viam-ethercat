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
    if (iequals(text, "switchable") || iequals(text, "SW")) {
        return ControlMode::Switchable;
    }
    throw ConfigError("control_type '" + std::string(text) + "' is not valid (expected \"PP\", \"PV\", or \"switchable\")");
}

const char* to_string(ControlMode mode) noexcept {
    switch (mode) {
        case ControlMode::ProfilePosition:
            return "PP";
        case ControlMode::ProfileVelocity:
            return "PV";
        case ControlMode::Switchable:
            return "switchable";
    }
    return "?";
}

void ServoConfig::apply_derived_pdo_maps() {
    // #61: standard CiA402 objects only (#41) -- these ARE the standard, so they live in the library.
    constexpr std::uint16_t kCtrl = 0x6040, kMode = 0x6060, kTargetPos = 0x607A, kProfileVel = 0x6081, kTargetVel = 0x60FF;
    constexpr std::uint16_t kFault = 0x603F, kStatus = 0x6041, kModeDisp = 0x6061, kActualPos = 0x6064, kVelAct = 0x606C, kTorqueAct = 0x6077;
    const auto E = [](std::uint16_t index, std::uint8_t bits) { return ethercat::PdoEntry{index, 0, bits}; };

    if (rxpdo.pdo_indices.empty()) {  // absent -> derive; present -> advanced override, untouched
        std::vector<ethercat::PdoEntry> rx{E(kCtrl, 16)};
        if (mode == ControlMode::Switchable) {
            rx.push_back(E(kMode, 8));  // 0x6060 -> runtime §6 mode-switch enabled
        }
        if (mode != ControlMode::ProfileVelocity) {  // PP + switchable carry the position-move objects
            rx.push_back(E(kTargetPos, 32));
            rx.push_back(E(kProfileVel, 32));
        }
        if (mode != ControlMode::ProfilePosition) {  // PV + switchable carry the velocity object
            rx.push_back(E(kTargetVel, 32));
        }
        rxpdo.pdo_indices = {0x1600};
        rxpdo.entries[0x1600] = std::move(rx);
    }
    if (txpdo.pdo_indices.empty()) {  // one superset TxPDO for all modes (feedback is mode-independent)
        txpdo.pdo_indices = {0x1A00};
        txpdo.entries[0x1A00] = {E(kFault, 16), E(kStatus, 16), E(kModeDisp, 8), E(kActualPos, 32), E(kVelAct, 32), E(kTorqueAct, 16)};
    }
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
