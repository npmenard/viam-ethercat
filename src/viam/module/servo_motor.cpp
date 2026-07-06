#include "viam/module/servo_motor.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <viam/sdk/common/proto_value.hpp>

#include "ethercat/backend.hpp"
#include "ethercat/cia402.hpp"
#include "ethercat/errors.hpp"
#include "ethercat/pdo_mapping.hpp"
#include "viam/lib/a6_servo_driver.hpp"
#include "viam/lib/servo_config.hpp"

namespace ethercat::servo {

namespace {

// --- attribute helpers (numbers arrive as double; int is upcast by the SDK) ---

const ProtoValue* find_attr(const ProtoStruct& attrs, const std::string& key) {
    const auto it = attrs.find(key);
    return it == attrs.end() ? nullptr : &it->second;
}

template <class T>
std::optional<T> opt_attr(const ProtoStruct& attrs, const std::string& key) {
    const ProtoValue* const v = find_attr(attrs, key);
    if (v == nullptr) {
        return std::nullopt;
    }
    const T* const p = v->get<T>();
    if (p == nullptr) {
        throw Error("config attribute '" + key + "' has the wrong type");
    }
    return *p;
}

double req_num(const ProtoStruct& attrs, const std::string& key) {
    const auto v = opt_attr<double>(attrs, key);
    if (!v) {
        throw Error("required config attribute '" + key + "' is missing");
    }
    return *v;
}

double opt_num(const ProtoStruct& attrs, const std::string& key, double dflt) {
    return opt_attr<double>(attrs, key).value_or(dflt);
}

std::string req_str(const ProtoStruct& attrs, const std::string& key) {
    const auto v = opt_attr<std::string>(attrs, key);
    if (!v) {
        throw Error("required config attribute '" + key + "' is missing");
    }
    return *v;
}

ServoConfig config_from_attrs(const ProtoStruct& attrs) {
    ServoConfig c;
    c.ifname = req_str(attrs, "interface");
    c.slave_id = static_cast<std::uint16_t>(opt_num(attrs, "slave", 1.0));
    // No control_mode attribute: the driver is always switch-capable (each API call ensures its own
    // mode at runtime). No rxpdo/txpdo attributes either: the driver defines one fixed superset PDO
    // map (ServoConfig::set_fixed_pdo_map(), applied in validated()).

    c.max_motor_speed_rpm = req_num(attrs, "max_rpm");
    c.counts_per_rev = req_num(attrs, "counts_per_rev");
    c.motor_rated_current_amps = req_num(attrs, "motor_rated_current_amps");
    c.gear_ratio = opt_num(attrs, "gear_ratio", 1.0);

    c.position_tolerance_counts = static_cast<std::int32_t>(opt_num(attrs, "position_tolerance_counts", 0.0));

    c.target_loop_rate_hz = static_cast<std::uint32_t>(opt_num(attrs, "loop_rate_hz", 1000.0));
    c.require_realtime = opt_attr<bool>(attrs, "require_realtime").value_or(true);
    c.rt_priority = static_cast<int>(opt_num(attrs, "rt_priority", 80.0));
    c.use_distributed_clocks = opt_attr<bool>(attrs, "use_distributed_clocks").value_or(false);
    // Optional drive datum: the SYNC0 cycle granularity the drive accepts (e.g. 250000 ns). When set,
    // the Master validates loop rate vs granularity at config time (clear text) instead of the drive
    // rejecting the cycle cryptically at OP entry.
    c.sync_cycle_granularity_ns = static_cast<std::uint32_t>(opt_num(attrs, "sync_cycle_granularity_ns", 0.0));
    // The A6 "no-sync" 0x603F code, the vendor fault-reset SDO, and the 0x603F gloss are not config
    // attributes; they live in the A6ServoDriver subclass (the viam:ethercat:a6-servo model). A generic
    // config carries none of them; the generic base drives standard CiA402 only.

    c.max_consecutive_wkc_errors = static_cast<int>(opt_num(attrs, "max_consecutive_wkc_errors", 5.0));
    c.command_queue_capacity = static_cast<std::size_t>(opt_num(attrs, "command_queue_capacity", 64.0));
    // handshake_timeout_cycles is an internal policy constant, not a config attribute.

    c.validate();  // throws Error (clear text) on any invalid field
    return c;
}

// Build the controller for a model. `Controller` is the generic ServoController (viam:ethercat:servo)
// or the A6ServoDriver subclass (viam:ethercat:a6-servo); both share the same config parser and ctors
// (A6 inherits them) and differ only in the three device seams. Returns a base-typed unique_ptr so
// ServoMotor stays subclass-agnostic (reconfigure() rebuilds the master in place, preserving the type).
template <class Controller>
std::unique_ptr<ServoController> build_controller(const ResourceConfig& cfg) {
    return std::make_unique<Controller>(config_from_attrs(cfg.attributes()));
}

bool command_flag(const ProtoStruct& command, const char* key) {
    const auto it = command.find(key);
    if (it == command.end()) {
        return false;
    }
    const bool* const b = it->second.get<bool>();
    return b != nullptr && *b;
}

// --- do_command SDO reads: always the standard CiA402 objects (no config, no override) -----
// get_motor_voltage=0x6079 (U32 mV -> V /1000), get_motor_current=0x6078 (I16 per-mille of the
// motor's rated current -> A), get_motor_drive_modes=0x6502 (U32 bitmask). A drive that lacks an
// object (the A6 aborts 0x6079/0x6078) reports value 0 plus the abort text under a "<key>_diag" key
// at the do_command site, so moves are unaffected.
constexpr std::uint16_t kDcLinkVoltage = 0x6079;        // U32, milliVolts
constexpr std::uint16_t kCurrentActual = 0x6078;        // I16, per-mille of the motor's rated current
constexpr std::uint16_t kSupportedDriveModes = 0x6502;  // U32 bitmask of supported modes
constexpr std::chrono::milliseconds kSdoTimeout{200};

// Read a standard little-endian integer object via the controller's direct SDO. Throws
// ethercat::Error on abort/timeout/short-read; the caller maps that to value 0 plus a "<key>_diag" key.
template <typename T>
T read_std_sdo(ServoController& ctrl, std::uint16_t index) {
    std::array<std::byte, sizeof(T)> buf{};
    const std::size_t n = ctrl.sdo_read(index, 0, std::span<std::byte>(buf.data(), sizeof(T)), kSdoTimeout);
    if (n < sizeof(T)) {
        throw ethercat::SdoError("standard SDO object " + std::to_string(index) + " short read: " + std::to_string(n) + " of " +
                                 std::to_string(sizeof(T)) + " byte(s)");
    }
    return ethercat::load_le<T>(std::span<const std::byte>(buf.data(), sizeof(T)));
}

// Decode the 0x6502 supported-drive-modes bitmask into CiA402 mode-name strings.
std::vector<ProtoValue> decode_drive_modes(std::uint32_t bits) {
    struct Bit {
        unsigned bit;
        const char* name;
    };
    static constexpr std::array<Bit, 9> kModeBits{{
        {0, "PP"},
        {1, "VL"},
        {2, "PV"},
        {3, "TQ"},
        {5, "HM"},
        {6, "IP"},
        {7, "CSP"},
        {8, "CSV"},
        {9, "CST"},
    }};
    std::vector<ProtoValue> modes;
    for (const Bit& b : kModeBits) {
        if ((bits & (1U << b.bit)) != 0U) {
            modes.emplace_back(std::string(b.name));
        }
    }
    return modes;
}

}  // namespace

ServoConfig parse_servo_config(const ProtoStruct& attributes) {
    return config_from_attrs(attributes);
}

const ModelFamily& ServoMotor::model_family() {
    // TEST ONLY (registry dry run): family namespace matches the personal test org so the
    // registry-delivered module binds configured models. REVERT to "viam" with meta.json.in.
    static const auto family = ModelFamily{"dcec8531-c6bb-41ee-b1b0-35f9767830a6", "ethercat"};
    return family;
}

Model ServoMotor::model() {
    return {model_family(), "servo"};
}

Model ServoMotor::a6_model() {
    return {model_family(), "a6-servo"};
}

std::vector<std::shared_ptr<ModelRegistration>> ServoMotor::create_model_registrations() {
    // Both models are rdk:component:motor with the same validator (one config parser). They differ
    // only in the controller subclass the factory builds: the generic base vs the A6ServoDriver seams.
    return {
        std::make_shared<ModelRegistration>(
            API::get<Motor>(),
            model(),  // viam:ethercat:servo -- generic standard-CiA402 driver
            [](const auto& /*deps*/, const auto& cfg) {
                return std::make_shared<ServoMotor>(cfg.name(), build_controller<ServoController>(cfg));
            },
            [](const auto& cfg) { return ServoMotor::validate(cfg); }),
        std::make_shared<ModelRegistration>(
            API::get<Motor>(),
            a6_model(),  // viam:ethercat:a6-servo -- A6ServoDriver subclass
            [](const auto& /*deps*/, const auto& cfg) {
                return std::make_shared<ServoMotor>(cfg.name(), build_controller<A6ServoDriver>(cfg));
            },
            [](const auto& cfg) { return ServoMotor::validate(cfg); }),
    };
}

std::vector<std::string> ServoMotor::validate(const ResourceConfig& cfg) {
    (void)parse_servo_config(cfg.attributes());  // throws Error on any problem
    return {};                                   // a motor has no dependencies
}

ServoMotor::ServoMotor(const Dependencies& /*deps*/, const ResourceConfig& cfg)
    : Motor(cfg.name()), controller_(build_controller<ServoController>(cfg)) {
    controller_->start();
}

ServoMotor::ServoMotor(std::string name, std::unique_ptr<ServoController> controller)
    : Motor(std::move(name)), controller_(std::move(controller)) {
    if (!controller_) {
        throw Error("ServoMotor: null controller");
    }
    controller_->start();
}

ServoMotor::~ServoMotor() {
    if (controller_) {
        controller_->stop();  // idempotent; joins the RT thread
    }
}

void ServoMotor::reconfigure(const Dependencies& /*deps*/, const ResourceConfig& cfg) {
    // Validate before mutate: parse and validate the new config (throws) before any teardown. The
    // controller owns the stop->join->rebuild->restart lifecycle.
    ServoConfig sc = parse_servo_config(cfg.attributes());
    controller_->reconfigure(std::move(sc));
}

void ServoMotor::set_power(double /*power_pct*/, const ProtoStruct& /*extra*/) {
    throw std::runtime_error(
        "set_power is unsupported: this servo has no open-loop torque/power mode; use go_to/go_for (PP) or set_rpm (PV)");
}

void ServoMotor::set_rpm(double rpm, const ProtoStruct& /*extra*/) {
    controller_->set_rpm(rpm);  // PV only; the controller throws a clear error in PP
}

void ServoMotor::go_for(double rpm, double revolutions, const ProtoStruct& /*extra*/) {
    // RDK convention: distance = |revolutions|, direction = sign(rpm)*sign(revolutions)
    // (both-negative = forward), speed = |rpm|. The controller takes signed revs
    // (direction) + magnitude rpm (profile velocity is always the magnitude).
    const double signed_revs = (rpm < 0.0 ? -1.0 : 1.0) * revolutions;
    controller_->go_for(std::abs(rpm), signed_revs);  // PP relative move / PV timed run
}

void ServoMotor::go_to(double rpm, double position_revolutions, const ProtoStruct& /*extra*/) {
    // Absolute target (from the reset_zero zero); direction is inherent, speed = |rpm|.
    controller_->go_to(std::abs(rpm), position_revolutions);  // PP only; controller throws in PV / on stall / timeout
}

void ServoMotor::reset_zero_position(double offset, const ProtoStruct& /*extra*/) {
    controller_->set_zero(offset);  // make the current position read `offset` revs
}

Motor::position ServoMotor::get_position(const ProtoStruct& /*extra*/) {
    return controller_->position_revs();
}

Motor::properties ServoMotor::get_properties(const ProtoStruct& /*extra*/) {
    return properties{/*position_reporting=*/true};
}

Motor::power_status ServoMotor::get_power_status(const ProtoStruct& /*extra*/) {
    const bool on = controller_->is_powered();
    return power_status{/*is_on=*/on, /*power_pct=*/on ? 1.0 : 0.0};  // no torque feedback, so 1.0/0.0
}

bool ServoMotor::is_moving() {
    return controller_->is_moving();  // fail-safe: stale/disconnected -> false
}

void ServoMotor::stop(const ProtoStruct& /*extra*/) {
    controller_->halt();  // Stop == Halt (sticky); the drive stays enabled
}

ProtoStruct ServoMotor::do_command(const ProtoStruct& command) {
    ProtoStruct result;
    if (command_flag(command, "fault_reset")) {
        controller_->request_fault_reset();
        result.emplace("fault_reset", ProtoValue(true));
    }
    if (command_flag(command, "enable")) {
        controller_->enable();
        result.emplace("enable", ProtoValue(true));
    }
    if (command_flag(command, "disable")) {
        controller_->disable();
        result.emplace("disable", ProtoValue(true));
    }
    if (command_flag(command, "status")) {
        ProtoStruct status;
        status.emplace("is_powered", ProtoValue(controller_->is_powered()));
        status.emplace("is_moving", ProtoValue(controller_->is_moving()));
        status.emplace("position", ProtoValue(controller_->position_revs()));
        status.emplace("is_disconnected", ProtoValue(controller_->is_disconnected()));
        status.emplace("last_error", ProtoValue(controller_->last_error()));
        result.emplace("status", ProtoValue(std::move(status)));
    }
    // Standard-CiA402 SDO reads (no config, no override). On success, the converted value goes under
    // the clear key. On SDO abort/failure (e.g. the A6 does not implement 0x6079/0x6078), the value is
    // reported as 0 and the abort text lands under a secondary "<key>_diag" key (not a *_error key), so
    // moves are unaffected.
    if (command_flag(command, "get_motor_voltage")) {
        try {
            const auto mv = read_std_sdo<std::uint32_t>(*controller_, kDcLinkVoltage);
            result.emplace("voltage_volts", ProtoValue(static_cast<double>(mv) / 1000.0));
        } catch (const ethercat::Error& e) {
            result.emplace("voltage_volts", ProtoValue(0.0));
            result.emplace("voltage_volts_diag", ProtoValue(std::string("0x6079 unavailable (reporting 0): ") + e.what()));
        }
    }
    if (command_flag(command, "get_motor_current_actual_value")) {
        try {
            const auto permille = read_std_sdo<std::int16_t>(*controller_, kCurrentActual);
            result.emplace("current_amps", ProtoValue((static_cast<double>(permille) / 1000.0) * controller_->rated_current_amps()));
        } catch (const ethercat::Error& e) {
            result.emplace("current_amps", ProtoValue(0.0));
            result.emplace("current_amps_diag", ProtoValue(std::string("0x6078 unavailable (reporting 0): ") + e.what()));
        }
    }
    if (command_flag(command, "get_motor_drive_modes")) {
        try {
            const auto bits = read_std_sdo<std::uint32_t>(*controller_, kSupportedDriveModes);
            result.emplace("drive_modes", ProtoValue(decode_drive_modes(bits)));
        } catch (const ethercat::Error& e) {
            result.emplace("drive_modes", ProtoValue(std::vector<ProtoValue>{}));
            result.emplace("drive_modes_diag", ProtoValue(std::string("0x6502 unavailable (reporting none): ") + e.what()));
        }
    }
    if (result.empty()) {
        throw std::runtime_error(
            "unknown do_command; supported keys (each a bool): fault_reset, enable, disable, status; "
            "get_motor_voltage, get_motor_current_actual_value, get_motor_drive_modes (converted SDO reads)");
    }
    return result;
}

std::vector<GeometryConfig> ServoMotor::get_geometries(const ProtoStruct& /*extra*/) {
    return {};  // a bare motor has no geometry
}

}  // namespace ethercat::servo
