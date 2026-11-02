#include "viam/module/servo_motor.hpp"
#include "viam/module/proto_attrs.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <viam/sdk/common/proto_value.hpp>
#include <viam/sdk/log/logging.hpp>

#include "ethercat/cia402.hpp"
#include "ethercat/errors.hpp"
#include "ethercat/log.hpp"
#include "ethercat/pdo_mapping.hpp"
#include "ethercat/util.hpp"
#include "viam/lib/a6_servo_driver.hpp"
#include "viam/lib/sdo_codec.hpp"
#include "viam/lib/servo_config.hpp"

namespace ethercat::servo {

using attrs::find_attr;
using attrs::nested;
using attrs::opt_attr;
using attrs::opt_num;
using attrs::req_num;

namespace {

namespace codec = ethercat::servo::sdo_codec;

struct SdoRequest {
    std::uint16_t index = 0;
    std::uint8_t sub = 0;
    const codec::TypeInfo* type = nullptr;
    codec::Input value;
};

codec::Input input_of(const ProtoValue& v, const char* key) {
    if (const double* d = v.get<double>()) {
        return codec::Input{*d, std::nullopt};
    }
    if (const std::string* t = v.get<std::string>()) {
        return codec::Input{std::nullopt, *t};
    }
    throw Error(std::string("'") + key + "' must be a number or a string");
}

std::uint32_t small_uint(const ProtoValue& v, const char* key, const char* type) {
    const codec::TypeInfo& t = *codec::find_type(type);
    return static_cast<std::uint32_t>(*t.decode(t.encode(input_of(v, key))).number);
}

// The {index, sub?, type?} triple every config-named drive object uses (the encoder mode object,
// the homing torque object). Writes through to the config fields, leaving `sub`/`type` at their
// defaults when absent. `where` names the attribute in the error text.
void object_ref(const ProtoStruct& o, const char* where, std::uint16_t& index, std::uint8_t& sub, std::string& type) {
    const ProtoValue* const i = find_attr(o, "index");
    if (i == nullptr) {
        throw Error(std::string("config attribute '") + where + "' needs an 'index'");
    }
    index = static_cast<std::uint16_t>(small_uint(*i, "index", "u16"));
    if (const ProtoValue* const s = find_attr(o, "sub")) {
        sub = static_cast<std::uint8_t>(small_uint(*s, "sub", "u8"));
    }
    type = opt_attr<std::string>(o, "type").value_or(type);
    if (codec::find_type(type) == nullptr) {
        throw Error(std::string("config attribute '") + where + ".type' is not a known SDO type: " + type);
    }
}

// Upload buffer for a raw read. Every CoE scalar fits in 8 bytes; the rest is headroom for the
// string objects a drive exposes (name, version), which are the only variable-width ones here.
constexpr std::size_t kSdoReadBytes = 64;

std::string hex4(std::uint16_t v) {
    char out[8];
    (void)std::snprintf(out, sizeof out, "0x%04X", v);
    return out;
}

std::string req_str(const ProtoStruct& attrs, const std::string& key) {
    const auto v = opt_attr<std::string>(attrs, key);
    if (!v) {
        throw Error("required config attribute '" + key + "' is missing");
    }
    return *v;
}

// Forwards the library log to the SDK's resource logger, so lines carry the component name and honor
// its app-configured log level instead of arriving as raw stderr (which the RDK files as errors).
class SdkLogSink final : public ethercat::log::Sink {
   public:
    explicit SdkLogSink(const viam::sdk::Resource& resource) noexcept : resource_(resource) {}
    void write(ethercat::log::Level level, std::string_view tag, std::string_view message, ethercat::log::Origin origin) noexcept override {
        try {
            // VIAM_RESOURCE_LOG stamps its own __FILE__/__LINE__; stamp the library's instead.
            BOOST_LOG_SEV(viam::sdk::log_detail::logger_access::logger(resource_), to_sdk_level(level))
                << boost::log::add_value(viam::sdk::attr_file_type{}, viam::sdk::log_detail::trim_filename(origin.file))
                << boost::log::add_value(viam::sdk::attr_line_type{}, origin.line) << tag << ": " << message;
        } catch (...) {
            // A logging failure must never propagate into the RT path; keep the line visible on stderr.
            (void)std::fprintf(stderr,
                               "[ethercat] %s %.*s: %.*s (SDK logger unavailable)\n",
                               ethercat::log::to_string(level),
                               static_cast<int>(tag.size()),
                               tag.data(),
                               static_cast<int>(message.size()),
                               message.data());
        }
    }

   private:
    static viam::sdk::log_level to_sdk_level(ethercat::log::Level level) noexcept {
        switch (level) {
            case ethercat::log::Level::Debug:
                return viam::sdk::log_level::debug;
            case ethercat::log::Level::Info:
                return viam::sdk::log_level::info;
            case ethercat::log::Level::Warn:
                return viam::sdk::log_level::warn;
            case ethercat::log::Level::Error:
                return viam::sdk::log_level::error;
        }
        return viam::sdk::log_level::info;
    }

    const viam::sdk::Resource& resource_;
};

ethercat::log::Level to_library_level(viam::sdk::log_level lvl) noexcept {
    using viam::sdk::log_level;
    switch (lvl) {
        case log_level::trace:
        case log_level::debug:
            return ethercat::log::Level::Debug;
        case log_level::info:
            return ethercat::log::Level::Info;
        case log_level::warn:
            return ethercat::log::Level::Warn;
        case log_level::error:
        case log_level::fatal:
            return ethercat::log::Level::Error;
    }
    return ethercat::log::Level::Info;
}

ServoConfig config_from_attrs(const ProtoStruct& attrs, ethercat::servo::MotionModeKind default_mode) {
    ServoConfig c;
    c.ifname = req_str(attrs, "interface");
    c.slave_id = static_cast<std::uint16_t>(opt_num(attrs, "slave", 1.0));
    // "csp" (default for the generic model) or "profile" (PP/PV). The driver derives the PDO map from
    // it and selects it on the drive by SDO at start. No rxpdo/txpdo attributes: the maps are driver-defined.
    c.motion_mode = default_mode;
    if (const auto mode = opt_attr<std::string>(attrs, "control_mode")) {
        if (*mode == "csp") {
            c.motion_mode = ethercat::servo::MotionModeKind::CyclicPosition;
        } else if (*mode == "profile") {
            c.motion_mode = ethercat::servo::MotionModeKind::Profile;
        } else {
            throw Error("config attribute 'control_mode' must be \"csp\" or \"profile\" (got '" + *mode + "')");
        }
    }

    c.max_motor_speed_rpm = req_num(attrs, "max_rpm");
    // Ramp acceleration of the master-side trajectory in CSP; 0/absent = max_rpm per second. Ignored in PP.
    c.max_accel_rpm_per_s = opt_num(attrs, "max_acceleration_rpm_per_s", 0.0);
    c.counts_per_rev = req_num(attrs, "counts_per_rev");
    c.motor_rated_current_amps = req_num(attrs, "motor_rated_current_amps");
    c.gear_ratio = opt_num(attrs, "gear_ratio", 1.0);
    // Absolute-encoder declaration: which object says the encoder retains position, and which 0x603F
    // codes mean the retained position is gone. All drive-specific, so all from config.
    if (const ProtoValue* const v = find_attr(attrs, "absolute_encoder")) {
        const ProtoStruct* const o = v->get<ProtoStruct>();
        const ProtoValue* const equals = o ? find_attr(*o, "equals") : nullptr;
        const double* const eq = equals == nullptr ? nullptr : equals->get<double>();
        if (eq == nullptr) {
            throw Error("config attribute 'absolute_encoder' expects {index, sub?, type?, equals, invalid_codes?}");
        }
        ServoConfig::AbsoluteEncoder& enc = c.absolute_encoder;
        object_ref(*o, "absolute_encoder", enc.index, enc.sub, enc.type);
        enc.equals = *eq;
        if (const ProtoValue* const codes = find_attr(*o, "invalid_codes")) {
            const std::vector<ProtoValue>* const list = codes->get<std::vector<ProtoValue>>();
            if (list == nullptr) {
                throw Error("config attribute 'absolute_encoder.invalid_codes' must be a list");
            }
            for (const ProtoValue& code : *list) {
                enc.invalid_codes.push_back(static_cast<std::uint16_t>(small_uint(code, "invalid_codes", "u16")));
            }
        }
    }
    // Everything one homing run needs, in one block: the 0x6098 method is drive-specific (CiA402
    // leaves the hard-stop methods to the manufacturer) and so is the torque object, which only the
    // current-sensing methods use. Ranges are checked by ServoConfig::validate().
    if (const ProtoValue* const v = find_attr(attrs, "homing")) {
        const ProtoStruct* const o = v->get<ProtoStruct>();
        if (o == nullptr) {
            throw Error(
                "config attribute 'homing' expects {method, speed_rpm, acceleration_rpm_per_s?, torque_object?, torque_percent?, "
                "timeout_s?}");
        }
        ServoConfig::Homing& h = c.homing;
        const double method = req_num(*o, "method");
        if (method < -128.0 || method > 127.0 || method != std::trunc(method) || method == 0.0) {
            throw Error("config attribute 'homing.method' must be a non-zero 0x6098 method (int8)");
        }
        h.method = static_cast<std::int8_t>(method);
        h.speed_rpm = req_num(*o, "speed_rpm");
        h.accel_rpm_per_s = opt_num(*o, "acceleration_rpm_per_s", 0.0);
        h.torque_percent = opt_num(*o, "torque_percent", 0.0);
        h.timeout_s = opt_num(*o, "timeout_s", 120.0);
        if (const ProtoStruct* const t = nested(*o, "torque_object")) {
            object_ref(*t, "homing.torque_object", h.torque.index, h.torque.sub, h.torque.type);
            h.torque.units_per_percent = opt_num(*t, "units_per_percent", 10.0);
        }
    }
    // 0x603F code the drive reports while its emergency-stop input is engaged (e.g. "0xFF3A").
    if (const ProtoValue* const v = find_attr(attrs, "emergency_stop_fault_code")) {
        unsigned long code = 0;
        if (const double* const d = v->get<double>()) {
            code = static_cast<unsigned long>(*d);
        } else if (const std::string* const t = v->get<std::string>()) {
            try {
                code = std::stoul(*t, nullptr, 0);  // "0xFF3A" or decimal
            } catch (const std::exception&) {
                throw Error("config attribute 'emergency_stop_fault_code' is not a number: " + *t);
            }
        }
        if (code > 0xFFFF) {
            throw Error("config attribute 'emergency_stop_fault_code' must fit in 16 bits");
        }
        c.estop_fault_code = static_cast<std::uint16_t>(code);
    }
    // Deceleration written to 0x6085 (the drive's quick stop, also its E-stop input ramp); 0/absent = not configured.
    const double qs_rpm_s = opt_num(attrs, "quick_stop_decel_rpm_per_s", 0.0);
    if (!(qs_rpm_s >= 0.0)) {
        throw Error("config attribute 'quick_stop_decel_rpm_per_s' must be >= 0");
    }
    c.quick_stop_decel = static_cast<std::uint32_t>(ethercat::rpm_to_device_velocity(qs_rpm_s, c.counts_per_rev, c.gear_ratio));

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
// ServoMotor stays subclass-agnostic.
template <class Controller>
std::unique_ptr<ServoController> build_controller(const ResourceConfig& cfg) {
    return std::make_unique<Controller>(config_from_attrs(cfg.attributes(), Controller::kDefaultMotionMode));
}

// --- raw SDO verbs. Reads reach the whole dictionary: reading an object changes nothing, and a
// drive-side setting the driver does not model (a stop ramp, an input function) can only be
// diagnosed by reading it. Writes stay within the manufacturer range (0x2000-0x5FFF) plus 0x1010
// (store parameters), because the driver owns the standard objects and a write behind its back
// would fight it.
SdoRequest parse_sdo_request(const ProtoValue& v, const char* verb, bool with_value) {
    const ProtoStruct* const req = v.get<ProtoStruct>();
    const ProtoValue* const index = req ? find_attr(*req, "index") : nullptr;
    const auto type_name = req ? opt_attr<std::string>(*req, "type") : std::nullopt;
    const codec::TypeInfo* const type = type_name ? codec::find_type(*type_name) : nullptr;
    if (index == nullptr || type == nullptr) {
        throw Error(std::string(verb) + " expects {index, sub?, type: u8|i8|u16|i16|u32|i32|string|bytes" +
                    (with_value ? ", value}" : "}"));
    }
    SdoRequest r;
    r.index = static_cast<std::uint16_t>(small_uint(*index, "index", "u16"));
    if (const ProtoValue* const sub = find_attr(*req, "sub")) {
        r.sub = static_cast<std::uint8_t>(small_uint(*sub, "sub", "u8"));
    }
    r.type = type;
    if (with_value) {
        if ((r.index < 0x2000 || r.index > 0x5FFF) && r.index != 0x1010) {
            throw Error(std::string(verb) + ": only manufacturer-specific objects (0x2000-0x5FFF) and 0x1010 are writable");
        }
        const ProtoValue* const val = find_attr(*req, "value");
        if (val == nullptr) {
            throw Error("sdo_write: 'value' is required");
        }
        r.value = input_of(*val, "value");
    }
    return r;
}

ProtoStruct sdo_reply(const codec::Decoded& d, const std::vector<std::byte>& raw) {
    ProtoStruct out;
    out.emplace("value", d.number ? ProtoValue(*d.number) : ProtoValue(d.text));
    out.emplace("raw", ProtoValue(ethercat::log::hex_bytes(raw)));
    out.emplace("ok", ProtoValue(true));
    return out;
}

ProtoStruct sdo_refusal(const ethercat::SdoError& e) {
    char code[16];
    (void)std::snprintf(code, sizeof code, "0x%08X", e.abort_code());
    ProtoStruct out;
    out.emplace("ok", ProtoValue(false));
    out.emplace("abort_code", ProtoValue(std::string(code)));
    out.emplace("error", ProtoValue(std::string(e.what())));
    return out;
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

ServoConfig parse_servo_config(const ProtoStruct& attributes, ethercat::servo::MotionModeKind default_mode) {
    return config_from_attrs(attributes, default_mode);
}

const ModelFamily& ServoMotor::model_family() {
    static const auto family = ModelFamily{"viam", "ethercat"};
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
            [](const auto& cfg) { return ServoMotor::validate(cfg, ServoController::kDefaultMotionMode); }),
        std::make_shared<ModelRegistration>(
            API::get<Motor>(),
            a6_model(),  // viam:ethercat:a6-servo -- A6ServoDriver subclass
            [](const auto& /*deps*/, const auto& cfg) {
                return std::make_shared<ServoMotor>(cfg.name(), build_controller<A6ServoDriver>(cfg));
            },
            [](const auto& cfg) { return ServoMotor::validate(cfg, A6ServoDriver::kDefaultMotionMode); }),
    };
}

std::vector<std::string> ServoMotor::validate(const ResourceConfig& cfg, ethercat::servo::MotionModeKind default_mode) {
    (void)parse_servo_config(cfg.attributes(), default_mode);  // throws Error on any problem
    return {};                                                 // a motor has no dependencies
}

ServoMotor::ServoMotor(const Dependencies& /*deps*/, const ResourceConfig& cfg)
    : Motor(cfg.name()), log_sink_(std::make_shared<SdkLogSink>(*this)), controller_(build_controller<ServoController>(cfg)) {
    install_logging(cfg.get_log_level());
    controller_->start();
}

ServoMotor::ServoMotor(std::string name, std::unique_ptr<ServoController> controller)
    : Motor(std::move(name)), log_sink_(std::make_shared<SdkLogSink>(*this)), controller_(std::move(controller)) {
    if (!controller_) {
        throw Error("ServoMotor: null controller");
    }
    install_logging(viam::sdk::log_level::info);
    controller_->start();
}

ServoMotor::~ServoMotor() {
    if (controller_) {
        controller_->stop();  // idempotent; joins the RT thread
    }
    ethercat::log::clear_sink(log_sink_.get());  // after the RT thread is gone: nothing of ours logs any more
}

// The library log is process-wide; the most recently configured motor owns it (one motor per NIC
// is the expected deployment). Its level follows the component's app-configured log level.
void ServoMotor::install_logging(viam::sdk::log_level level) {
    ethercat::log::set_level(to_library_level(level));
    ethercat::log::set_sink(log_sink_);
}

// The `is_abs_enc_valid` reply: is the drive's absolute encoder set to retain position, and has it
// kept it? The mode object is a drive setting, so it is read once and cached; the lost-reference
// verdict is the live 0x603F against the configured codes, so it costs nothing to re-check. Never
// throws: "cannot tell" is `valid: false` with a reason, the same instruction to the caller -- home it.
ProtoStruct ServoMotor::absolute_encoder_status() {
    const ServoConfig::AbsoluteEncoder& enc = controller_->absolute_encoder();
    ProtoStruct out;
    out.emplace("valid", ProtoValue(false));
    if (enc.index == 0) {
        out.emplace("reason", ProtoValue(std::string("no 'absolute_encoder' config attribute on this motor")));
        return out;
    }
    // A bus drop may have taken the drive's control power with it, and an encoder whose battery is
    // flat loses its count while dark, so the cached mode is forgotten -- and asked again on the
    // first call after the bus is back, not now, when the drive cannot answer.
    if (controller_->is_disconnected()) {
        abs_enc_mode_known_.store(false, std::memory_order_relaxed);
        out.emplace("reason", ProtoValue(std::string("the bus is disconnected, so the drive cannot be asked")));
        return out;
    }
    std::string reason;
    double mode = abs_enc_mode_.load(std::memory_order_relaxed);
    if (!abs_enc_mode_known_.load(std::memory_order_acquire)) {
        try {
            std::vector<std::byte> buf(kSdoReadBytes);
            buf.resize(std::min(controller_->sdo_read(enc.index, enc.sub, buf), buf.size()));
            mode = codec::find_type(enc.type)->decode(buf).number.value_or(0.0);
            abs_enc_mode_.store(mode, std::memory_order_relaxed);
            abs_enc_mode_known_.store(true, std::memory_order_release);
        } catch (const std::exception& e) {
            reason = std::string("reading the encoder mode object failed: ") + e.what();
        }
    }
    const bool configured = reason.empty() && mode == enc.equals;
    const std::uint16_t code = controller_->drive_fault_code();
    const bool lost = std::find(enc.invalid_codes.begin(), enc.invalid_codes.end(), code) != enc.invalid_codes.end();
    if (lost) {
        reason = "the drive reports " + hex4(code) + ", which this config lists as a lost reference";
    } else if (reason.empty() && !configured) {
        reason = "the encoder mode object does not read the configured value";
    }
    out.insert_or_assign("valid", ProtoValue(configured && !lost));
    out.emplace("configured", ProtoValue(configured));
    out.emplace("mode", ProtoValue(mode));
    out.emplace("fault_code", ProtoValue(hex4(code)));
    out.emplace("warning", ProtoValue(controller_->drive_warning()));
    if (!reason.empty()) {
        out.emplace("reason", ProtoValue(reason));
    }
    return out;
}

// The drive's state in one struct, for the API's get_status and the `status` do_command alike.
ProtoStruct ServoMotor::get_status() {
    ProtoStruct status;
    status.emplace("is_powered", ProtoValue(controller_->is_powered()));
    status.emplace("is_moving", ProtoValue(controller_->is_moving()));
    status.emplace("position", ProtoValue(controller_->position_revs()));
    status.emplace("is_disconnected", ProtoValue(controller_->is_disconnected()));
    status.emplace("last_error", ProtoValue(controller_->last_error()));
    status.emplace("motion_mode", ProtoValue(std::string(controller_->motion_mode_name())));  // "PP" or "CSP"
    status.emplace("emergency_stop", ProtoValue(controller_->emergency_stopped()));
    status.emplace("warning", ProtoValue(controller_->drive_warning()));  // statusword bit 7
    status.emplace("fault_code", ProtoValue(hex4(controller_->drive_fault_code())));
    // Homed means the axis reference is known: a run succeeded this session, or the configured
    // absolute encoder kept its count across the restart.
    const bool ran = controller_->homed();
    const bool absolute = !ran && attrs::flag(absolute_encoder_status(), "valid");
    const char* by = "none";
    if (ran || absolute) {
        by = ran ? "home_run" : "absolute_encoder";
    }
    status.emplace("homed", ProtoValue(ran || absolute));
    status.emplace("homed_by", ProtoValue(std::string(by)));
    status.emplace("homing", ProtoValue(std::string(to_string(controller_->homing_state()))));
    if (const std::string err = controller_->homing_error(); !err.empty()) {
        status.emplace("homing_error", ProtoValue(err));
    }
    return status;
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
    if (attrs::flag(command, "fault_reset")) {
        controller_->request_fault_reset();
        result.emplace("fault_reset", ProtoValue(true));
    }
    if (attrs::flag(command, "enable")) {
        controller_->enable();
        result.emplace("enable", ProtoValue(true));
    }
    if (attrs::flag(command, "disable")) {
        controller_->disable();
        result.emplace("disable", ProtoValue(true));
    }
    // Identity of the driver behind this motor, for a component that needs its contract (the
    // gantry checks it at config time rather than discovering a missing verb mid-home).
    if (attrs::flag(command, "describe")) {
        ProtoStruct d;
        d.emplace("driver", ProtoValue(std::string("viam:ethercat")));
        d.emplace("contract", ProtoValue(1.0));
        d.emplace("homing", ProtoValue(true));
        result.emplace("describe", ProtoValue(std::move(d)));
    }
    if (attrs::flag(command, "status")) {
        result.emplace("status", ProtoValue(get_status()));
    }
    // Standard-CiA402 SDO reads (no config, no override). On success, the converted value goes under
    // the clear key. On SDO abort/failure (e.g. the A6 does not implement 0x6079/0x6078), the value is
    // reported as 0 and the abort text lands under a secondary "<key>_diag" key (not a *_error key), so
    // moves are unaffected.
    if (attrs::flag(command, "get_motor_voltage")) {
        try {
            const auto mv = read_std_sdo<std::uint32_t>(*controller_, kDcLinkVoltage);
            result.emplace("voltage_volts", ProtoValue(static_cast<double>(mv) / 1000.0));
        } catch (const ethercat::Error& e) {
            result.emplace("voltage_volts", ProtoValue(0.0));
            result.emplace("voltage_volts_diag", ProtoValue(std::string("0x6079 unavailable (reporting 0): ") + e.what()));
        }
    }
    if (attrs::flag(command, "get_motor_current_actual_value")) {
        try {
            const auto permille = read_std_sdo<std::int16_t>(*controller_, kCurrentActual);
            result.emplace("current_amps", ProtoValue((static_cast<double>(permille) / 1000.0) * controller_->rated_current_amps()));
        } catch (const ethercat::Error& e) {
            result.emplace("current_amps", ProtoValue(0.0));
            result.emplace("current_amps_diag", ProtoValue(std::string("0x6078 unavailable (reporting 0): ") + e.what()));
        }
    }
    if (attrs::flag(command, "get_motor_drive_modes")) {
        try {
            const auto bits = read_std_sdo<std::uint32_t>(*controller_, kSupportedDriveModes);
            result.emplace("drive_modes", ProtoValue(decode_drive_modes(bits)));
        } catch (const ethercat::Error& e) {
            result.emplace("drive_modes", ProtoValue(std::vector<ProtoValue>{}));
            result.emplace("drive_modes_diag", ProtoValue(std::string("0x6502 unavailable (reporting none): ") + e.what()));
        }
    }
    // Raw typed SDO access. A drive refusal is data ({ok:false, abort_code, error}); a bad request or
    // a dead mailbox is a gRPC error.
    // Is the drive's absolute encoder configured to retain position, and has it kept it? Reads the
    // declared mode object and the live 0x603F. Reported as data (never an exception) so a caller
    // can branch on it: `valid` false with a reason means home the axis.
    if (attrs::flag(command, "is_abs_enc_valid")) {
        result.emplace("is_abs_enc_valid", ProtoValue(absolute_encoder_status()));
    }
    // Homing (CiA402 mode 6), every parameter from the motor's `homing` config. Blocks for the run
    // and reports the drive's verdict; a homing failure is a result, not an exception, so a caller
    // can retry with a different threshold. `counts` is the axis reference: the actual position at
    // the moment the drive attained home, with whatever back-off its method performed already in it.
    if (attrs::flag(command, "home")) {
        ProtoStruct reply;
        const bool homed = controller_->home();
        reply.emplace("homed", ProtoValue(homed));
        reply.emplace("counts", ProtoValue(static_cast<double>(controller_->home_position_counts())));
        reply.emplace("revolutions", ProtoValue(controller_->home_position_revs()));
        if (const std::string err = controller_->homing_error(); !err.empty()) {
            reply.emplace("error", ProtoValue(err));
        }
        result.emplace("home", ProtoValue(std::move(reply)));
    }
    if (const ProtoValue* const rq = find_attr(command, "sdo_read")) {
        const SdoRequest r = parse_sdo_request(*rq, "sdo_read", false);
        try {
            // Always read into a buffer wider than the requested type and check the width the drive
            // actually returned. Sizing the buffer to the type instead makes a width mismatch look
            // like a transport failure: a drive may answer a short request with nothing at all.
            std::vector<std::byte> buf(kSdoReadBytes);
            buf.resize(std::min(controller_->sdo_read(r.index, r.sub, buf), buf.size()));
            if (r.type->width > 0 && buf.size() != r.type->width) {
                throw Error("sdo_read: object " + hex4(r.index) + ":" + std::to_string(r.sub) + " is " + std::to_string(buf.size()) +
                            " bytes, type '" + std::string(r.type->name) + "' is " + std::to_string(r.type->width));
            }
            result.emplace("sdo_read", ProtoValue(sdo_reply(r.type->decode(buf), buf)));
        } catch (const ethercat::SdoError& e) {
            result.emplace("sdo_read", ProtoValue(sdo_refusal(e)));
        }
    }
    if (const ProtoValue* const rq = find_attr(command, "sdo_write")) {
        const SdoRequest r = parse_sdo_request(*rq, "sdo_write", true);
        if (controller_->is_moving()) {
            throw std::runtime_error("sdo_write: the motor is moving; stop it first");
        }
        const auto bytes = r.type->encode(r.value);
        ETHERCAT_LOG_INFO("servo", "raw SDO write 0x{:04X}:{} <- [{}]", r.index, r.sub, ethercat::log::hex_bytes(bytes));
        try {
            controller_->sdo_write(r.index, r.sub, bytes);
            result.emplace("sdo_write", ProtoValue(sdo_reply(r.type->decode(bytes), bytes)));
        } catch (const ethercat::SdoError& e) {
            result.emplace("sdo_write", ProtoValue(sdo_refusal(e)));
        }
    }
    if (result.empty()) {
        throw std::runtime_error(
            "unknown do_command; supported keys (each a bool): describe, fault_reset, enable, disable, status, home, "
            "is_abs_enc_valid; "
            "get_motor_voltage, get_motor_current_actual_value, get_motor_drive_modes (converted SDO reads); "
            "sdo_read {index, sub, type} (any object) / sdo_write {index, sub, type, value} (0x2000-0x5FFF and 0x1010)");
    }
    return result;
}

std::vector<GeometryConfig> ServoMotor::get_geometries(const ProtoStruct& /*extra*/) {
    return {};  // a bare motor has no geometry
}

}  // namespace ethercat::servo
