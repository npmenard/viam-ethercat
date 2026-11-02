#include "viam/lib/servo_config.hpp"

#include <cmath>
#include <string>

#include "ethercat/errors.hpp"
#include "ethercat/pdo_buffer.hpp"  // load_le

namespace ethercat::servo {

const char* to_string(MotionModeKind mode) noexcept {
    return mode == MotionModeKind::CyclicPosition ? "csp" : "profile";
}

void ServoConfig::set_fixed_pdo_map() {
    // Driver-defined maps (standard CiA402 objects only), one per motion mode, no user override. The
    // profile map carries 0x6060 so PP/PV can switch at runtime; the cyclic map has no mode byte (the
    // mode is SDO-set once) and streams only the target position.
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
    if (motion_mode == MotionModeKind::CyclicPosition) {
        rxpdo.entries[0x1600] = {E(kCtrl, 16), E(kTargetPos, 32)};
    } else {
        rxpdo.entries[0x1600] = {E(kCtrl, 16), E(kMode, 8), E(kTargetPos, 32), E(kProfileVel, 32), E(kTargetVel, 32)};
    }
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
    if (motion_mode == MotionModeKind::CyclicPosition && !use_distributed_clocks) {
        throw Error("servo config: control_mode 'csp' requires 'use_distributed_clocks' = true (CSP runs synchronous to SYNC0)");
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
    if (max_accel_rpm_per_s < 0.0) {
        throw Error("servo config: 'max_accel_rpm_per_s' must be >= 0");
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
    // The homing block is checked here, not when `home` is called: a drive that would be driven into
    // a hard stop by a bad number must be refused at load, while nothing is moving.
    if (homing.method != 0) {
        if (!(homing.speed_rpm > 0.0)) {
            throw Error("servo config: 'homing.speed_rpm' must be > 0");
        }
        if (!(homing.accel_rpm_per_s >= 0.0)) {
            throw Error("servo config: 'homing.acceleration_rpm_per_s' must be >= 0");
        }
        // Above rated is legitimate (a stop is found by a brief overload); a multiple of it is a typo.
        if (homing.torque_percent < 0.0 || homing.torque_percent > 300.0 || std::isnan(homing.torque_percent)) {
            throw Error("servo config: 'homing.torque_percent' must be within 0..300");
        }
        if (homing.torque_percent > 0.0 && homing.torque.index == 0) {
            throw Error("servo config: 'homing.torque_percent' needs 'homing.torque_object' (the object is drive-specific)");
        }
        if (!(homing.timeout_s > 0.0)) {
            throw Error("servo config: 'homing.timeout_s' must be > 0");
        }
    }
}

}  // namespace ethercat::servo
