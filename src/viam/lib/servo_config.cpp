#include "viam/lib/servo_config.hpp"

#include <string>

#include "ethercat/errors.hpp"
#include "ethercat/pdo_buffer.hpp"  // load_le

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

SdoValueType parse_sdo_value_type(std::string_view text) {
    if (iequals(text, "u8")) {
        return SdoValueType::U8;
    }
    if (iequals(text, "i8")) {
        return SdoValueType::I8;
    }
    if (iequals(text, "u16")) {
        return SdoValueType::U16;
    }
    if (iequals(text, "i16")) {
        return SdoValueType::I16;
    }
    if (iequals(text, "u32")) {
        return SdoValueType::U32;
    }
    if (iequals(text, "i32")) {
        return SdoValueType::I32;
    }
    throw ConfigError("sdo_monitors: value type '" + std::string(text) + "' is not valid (expected u8/i8/u16/i16/u32/i32)");
}

const char* to_string(SdoValueType t) noexcept {
    switch (t) {
        case SdoValueType::U8:
            return "u8";
        case SdoValueType::I8:
            return "i8";
        case SdoValueType::U16:
            return "u16";
        case SdoValueType::I16:
            return "i16";
        case SdoValueType::U32:
            return "u32";
        case SdoValueType::I32:
            return "i32";
    }
    return "?";
}

std::size_t SdoMonitor::byte_width() const noexcept {
    switch (type) {
        case SdoValueType::U8:
        case SdoValueType::I8:
            return 1;
        case SdoValueType::U16:
        case SdoValueType::I16:
            return 2;
        case SdoValueType::U32:
        case SdoValueType::I32:
            return 4;
    }
    return 0;
}

double convert_sdo_monitor(const SdoMonitor& m, std::span<const std::byte> raw, double rated_current_amps) {
    const std::size_t width = m.byte_width();
    if (raw.size() < width) {
        throw ConfigError("convert_sdo_monitor: object 0x" + std::to_string(m.index) + " read " + std::to_string(raw.size()) +
                          " byte(s), need " + std::to_string(width) + " for type " + to_string(m.type));
    }
    // Decode the raw little-endian integer as the declared type into a double (exact for all
    // these widths). Uses the shared load_le -- no hand-rolled packing.
    double value = 0.0;
    switch (m.type) {
        case SdoValueType::U8:
            value = ethercat::load_le<std::uint8_t>(raw.first(1));
            break;
        case SdoValueType::I8:
            value = ethercat::load_le<std::int8_t>(raw.first(1));
            break;
        case SdoValueType::U16:
            value = ethercat::load_le<std::uint16_t>(raw.first(2));
            break;
        case SdoValueType::I16:
            value = ethercat::load_le<std::int16_t>(raw.first(2));
            break;
        case SdoValueType::U32:
            value = ethercat::load_le<std::uint32_t>(raw.first(4));
            break;
        case SdoValueType::I32:
            value = ethercat::load_le<std::int32_t>(raw.first(4));
            break;
    }
    if (m.scale_kind == SdoScaleKind::RatedCurrentPermille) {
        return (value / 1000.0) * rated_current_amps;
    }
    return value / m.divisor;  // divisor validated nonzero in ServoConfig::validate()
}

void ServoConfig::apply_derived_pdo_maps() {
    // #61: standard CiA402 objects only (#41) -- these ARE the standard, so they live in the library.
    constexpr std::uint16_t kCtrl = 0x6040, kMode = 0x6060, kTargetPos = 0x607A, kProfileVel = 0x6081, kTargetVel = 0x60FF;
    constexpr std::uint16_t kFault = 0x603F, kStatus = 0x6041, kModeDisp = 0x6061, kActualPos = 0x6064, kVelAct = 0x606C,
                            kTorqueAct = 0x6077;
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
    // #68 sdo_monitors: a 0 index is never a real object; a Divisor scale must be nonzero.
    const auto check_monitor = [](const SdoMonitor& m, const char* which) {
        if (m.index == 0) {
            throw ConfigError(std::string("servo config: sdo_monitors.") + which + " 'index' must be nonzero");
        }
        if (m.scale_kind == SdoScaleKind::Divisor && m.divisor == 0.0) {
            throw ConfigError(std::string("servo config: sdo_monitors.") + which + " 'scale' divisor must not be 0");
        }
    };
    check_monitor(voltage_monitor, "voltage");
    check_monitor(current_monitor, "current");
}

}  // namespace ethercat::servo
