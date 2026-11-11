#include "viam/module/gantry_axis.hpp"

#include <cmath>
#include <fstream>
#include <iterator>
#include <stdexcept>
#include <utility>

#include <viam/sdk/log/logging.hpp>
#include <viam/sdk/resource/resource.hpp>

#include "ethercat/errors.hpp"
#include "viam/module/proto_attrs.hpp"
#include "viam/module/servo_motor.hpp"

namespace ethercat::servo {

using attrs::find_attr;
using attrs::flag;
using attrs::nested;
using attrs::opt_num;

namespace {

// The whole file, unparsed: the RDK builds the frame-system model from it. Only what would otherwise
// surface as an opaque planner error later is checked here.
std::vector<unsigned char> read_kinematics_file(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    std::vector<unsigned char> bytes((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    if (!in || bytes.empty()) {
        throw Error("config attribute 'kinematics_file': '" + path + "' is missing or empty");
    }
    return bytes;
}

}  // namespace

Model GantryAxis::model() {
    return {ServoMotor::model_family(), "gantry"};
}

GantryAxis::Attributes GantryAxis::parse(const ResourceConfig& cfg) {
    const ProtoStruct& a = cfg.attributes();
    Attributes out;
    const ProtoValue* const motor = find_attr(a, "motor");
    const std::string* const name = motor == nullptr ? nullptr : motor->get<std::string>();
    if (name == nullptr || name->empty()) {
        throw Error("required config attribute 'motor' is missing");
    }
    out.motor = *name;
    out.length_mm = opt_num(a, "length_mm", 0.0);
    out.mm_per_rev = opt_num(a, "mm_per_rev", 0.0);
    if (!(out.length_mm > 0.0)) {
        throw Error("config attribute 'length_mm' must be positive");
    }
    if (!(out.mm_per_rev > 0.0)) {
        throw Error("config attribute 'mm_per_rev' must be positive");
    }
    out.mm_per_sec = opt_num(a, "gantry_mm_per_sec", 0.0);
    if (out.mm_per_sec < 0.0) {
        throw Error("config attribute 'gantry_mm_per_sec' must be >= 0");
    }
    const ProtoValue* const dir = find_attr(a, "travel_direction");
    if (dir != nullptr) {
        const std::string* const d = dir->get<std::string>();
        if (d == nullptr || (*d != "negative" && *d != "positive")) {
            throw Error("config attribute 'travel_direction' must be \"negative\" or \"positive\"");
        }
        out.travel_positive = *d == "positive";
    }
    if (const auto path = attrs::opt_attr<std::string>(a, "kinematics_file")) {
        out.kinematics = read_kinematics_file(*path);
    }
    return out;
}

std::vector<std::string> GantryAxis::validate(const ResourceConfig& cfg) {
    return {parse(cfg).motor};  // the axis motor is this component's only dependency
}

std::shared_ptr<ModelRegistration> GantryAxis::create_model_registration() {
    return std::make_shared<ModelRegistration>(
        API::get<Gantry>(),
        model(),
        [](const Dependencies& deps, const ResourceConfig& cfg) { return std::make_shared<GantryAxis>(deps, cfg); },
        [](const ResourceConfig& cfg) { return GantryAxis::validate(cfg); });
}

GantryAxis::GantryAxis(const Dependencies& deps, const ResourceConfig& cfg) : Gantry(cfg.name()) {
    configure(deps, cfg);
}

void GantryAxis::configure(const Dependencies& deps, const ResourceConfig& cfg) {
    attrs_ = parse(cfg);
    motor_.reset();
    for (const auto& [dep_name, resource] : deps) {
        if (dep_name.short_name() == attrs_.motor) {
            motor_ = std::dynamic_pointer_cast<Motor>(resource);
        }
    }
    if (!motor_) {
        throw Error("gantry: dependency '" + attrs_.motor + "' is missing or is not a motor");
    }
    // The axis needs this module's homing contract, not just any motor. `describe` is the cheapest
    // proof: a motor that does not answer it cannot home, so say so at config time.
    ProtoStruct probe;
    probe.emplace("describe", ProtoValue(true));
    const ProtoStruct* d = nullptr;
    ProtoStruct reply;
    try {
        reply = motor_->do_command(probe);
        d = nested(reply, "describe");
    } catch (const std::exception& e) {
        throw Error("gantry: motor '" + attrs_.motor + "' does not answer the describe do_command (" + e.what() +
                    "); this gantry needs a viam:ethercat servo");
    }
    const ProtoValue* const driver = d == nullptr ? nullptr : find_attr(*d, "driver");
    const std::string* const driver_name = driver == nullptr ? nullptr : driver->get<std::string>();
    if (driver_name == nullptr || *driver_name != "viam:ethercat" || !flag(*d, "homing")) {
        throw Error("gantry: motor '" + attrs_.motor + "' is not a viam:ethercat servo with homing support");
    }
}

ProtoStruct GantryAxis::motor_status() const {
    ProtoStruct cmd;
    cmd.emplace("status", ProtoValue(true));
    const ProtoStruct reply = motor_->do_command(cmd);
    const ProtoStruct* const status = nested(reply, "status");
    if (status == nullptr) {
        throw Error("gantry: the motor returned no status");
    }
    return *status;
}

bool GantryAxis::homed() const {
    return flag(motor_status(), "homed");
}

double GantryAxis::home_revs(const ProtoStruct& motor_status) {
    return opt_num(motor_status, "home_revolutions", 0.0);
}

std::vector<double> GantryAxis::get_position(const ProtoStruct& extra) {
    const ProtoStruct status = motor_status();
    if (!flag(status, "homed")) {
        throw Error("gantry: position is unknown until the axis is homed -- call home()");
    }
    return {revs_to_mm(home_revs(status), motor_->get_position(extra))};
}

std::vector<double> GantryAxis::get_lengths(const ProtoStruct& /*extra*/) {
    return {attrs_.length_mm};
}

void GantryAxis::move_to_position(const std::vector<movement_coordinate>& coordinates, const ProtoStruct& extra) {
    if (coordinates.size() != 1) {
        throw Error("gantry: this axis takes exactly one coordinate, got " + std::to_string(coordinates.size()));
    }
    const ProtoStruct status = motor_status();
    if (!flag(status, "homed")) {
        throw Error("gantry: home the axis before moving it -- call home()");
    }
    const movement_coordinate& c = coordinates.front();
    if (!std::isfinite(c.position_mm) || c.position_mm < 0.0 || c.position_mm > attrs_.length_mm) {
        throw Error("gantry: " + std::to_string(c.position_mm) + " mm is outside the axis (0 .. " + std::to_string(attrs_.length_mm) +
                    " mm)");
    }
    const double speed = c.speed_mm_per_sec > 0.0 ? c.speed_mm_per_sec : attrs_.mm_per_sec;
    if (!(speed > 0.0)) {
        throw Error("gantry: no speed given and no 'gantry_mm_per_sec' configured");
    }
    // The motor takes a signed revolution target and an unsigned speed; the sign of the target
    // carries the direction, so the speed is a magnitude in rpm.
    motor_->go_to(speed / attrs_.mm_per_rev * 60.0, mm_to_revs(home_revs(status), c.position_mm), extra);
}

// The motor owns the procedure, so the axis only asks for it: the position the drive ended the run
// at is this axis's zero, whatever back-off the homing method performed, and the motor reports it
// in its status from then on.
bool GantryAxis::home(const ProtoStruct& /*extra*/) {
    ProtoStruct cmd;
    cmd.emplace("home", ProtoValue(true));
    const ProtoStruct reply = motor_->do_command(cmd);
    const ProtoStruct* const home = nested(reply, "home");
    return home != nullptr && flag(*home, "homed");
}

// The axis state in one struct: its own millimetre view plus the motor's, so one call explains
// both why a move was refused and what the drive is doing.
ProtoStruct GantryAxis::get_status() {
    ProtoStruct status = motor_status();
    status.emplace("length_mm", ProtoValue(attrs_.length_mm));
    if (flag(status, "homed")) {
        status.emplace("position_mm", ProtoValue(revs_to_mm(home_revs(status), motor_->get_position({}))));
    }
    return status;
}

bool GantryAxis::is_moving() {
    return motor_->is_moving();
}

void GantryAxis::stop(const ProtoStruct& extra) {
    // The motor's own stop is a halt that cannot fail, so the only thing that can throw here is the
    // RPC to reach it: the motor is gone or going. That is what happens on every shutdown, where the
    // RDK stops this axis as it removes it and removes the motor in the same sweep. A motor that is
    // already gone has nothing left to stop (its own close halts the drive), and letting the error
    // out would fail the whole removal, which the RDK reports as a fatal shutdown.
    try {
        motor_->stop(extra);
    } catch (const std::exception& e) {
        VIAM_RESOURCE_LOG(warn) << "stop: motor '" << attrs_.motor << "' is unreachable, nothing left to stop (" << e.what() << ")";
    }
}

ProtoStruct GantryAxis::do_command(const ProtoStruct& command) {
    // No verbs of its own: the axis forwards to the motor, so the drive-level commands (status,
    // fault_reset, raw SDO, ...) stay reachable through the component the user configured.
    return motor_->do_command(command);
}

std::vector<GeometryConfig> GantryAxis::get_geometries(const ProtoStruct& /*extra*/) {
    return {};
}

KinematicsData GantryAxis::get_kinematics(const ProtoStruct& /*extra*/) {
    if (attrs_.kinematics.empty()) {
        return KinematicsDataUnspecified{};
    }
    return KinematicsDataSVA{attrs_.kinematics};
}

}  // namespace ethercat::servo
