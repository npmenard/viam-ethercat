#include "viam/lib/servo_config.hpp"

#include <string>

#include "ethercat/errors.hpp"
#include "ethercat/pdo_buffer.hpp"  // load_le

namespace ethercat::servo {

const char* to_string(ControlMode mode) noexcept {
    switch (mode) {
        case ControlMode::ProfilePosition:
            return "PP";
        case ControlMode::ProfileVelocity:
            return "PV";
    }
    return "?";
}

void ServoConfig::set_fixed_pdo_map() {
    // The one fixed driver-defined superset (standard CiA402 objects only). Always switch-capable
    // (0x6060 mapped) so any API call can ensure PP or PV at runtime. Unconditional: no per-mode
    // choice and no user override; overwrites whatever was there.
    constexpr std::uint16_t kCtrl = 0x6040;
    constexpr std::uint16_t kMode = 0x6060;
    constexpr std::uint16_t kTargetPos = 0x607A;
    constexpr std::uint16_t kProfileVel = 0x6081;
    constexpr std::uint16_t kTargetVel = 0x60FF;
    constexpr std::uint16_t kFault = 0x603F;
    constexpr std::uint16_t kStatus = 0x6041;
    constexpr std::uint16_t kModeDisp = 0x6061;
    constexpr std::uint16_t kActualPos = 0x6064;
    constexpr std::uint16_t kVelAct = 0x606C;
    constexpr std::uint16_t kTorqueAct = 0x6077;
    const auto E = [](std::uint16_t index, std::uint8_t bits) { return ethercat::PdoEntry{index, 0, bits}; };

    rxpdo.pdo_indices = {0x1600};
    rxpdo.entries[0x1600] = {E(kCtrl, 16), E(kMode, 8), E(kTargetPos, 32), E(kProfileVel, 32), E(kTargetVel, 32)};
    txpdo.pdo_indices = {0x1A00};
    txpdo.entries[0x1A00] = {E(kFault, 16), E(kStatus, 16), E(kModeDisp, 8), E(kActualPos, 32), E(kVelAct, 32), E(kTorqueAct, 16)};
}

void ServoConfig::validate() const {
    if (ifname.empty()) {
        throw Error("servo config: 'ifname' (EtherCAT interface) must not be empty");
    }
    if (slave_id < 1) {
        throw Error("servo config: 'slave_id' must be >= 1");
    }
    if (!(max_motor_speed_rpm >= 0.0)) {  // also rejects NaN
        throw Error("servo config: 'max_motor_speed_rpm' must be >= 0");
    }
    if (!(motor_rated_current_amps > 0.0)) {
        throw Error(
            "servo config: 'motor_rated_current_amps' must be > 0 "
            "(needed to convert a current limit to torque per-mille)");
    }
    if (gear_ratio == 0.0) {
        throw Error("servo config: 'gear_ratio' must not be 0");
    }
    if (!(counts_per_rev > 0.0)) {
        throw Error("servo config: 'counts_per_rev' must be > 0");
    }
    if (position_tolerance_counts < 0) {
        throw Error("servo config: 'position_tolerance_counts' must be >= 0");
    }
    if (target_loop_rate_hz == 0 || target_loop_rate_hz > 1000) {
        throw Error("servo config: 'target_loop_rate_hz' " + std::to_string(target_loop_rate_hz) + " out of range (1..1000)");
    }
    if (rt_priority < 1 || rt_priority > 99) {
        throw Error("servo config: 'rt_priority' " + std::to_string(rt_priority) + " out of range (1..99)");
    }
    if (command_queue_capacity == 0) {
        throw Error("servo config: 'command_queue_capacity' must be > 0");
    }
}

}  // namespace ethercat::servo
