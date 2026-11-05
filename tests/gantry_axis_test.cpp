// viam:ethercat:gantry without a drive: attribute parsing (and the kinematics file it reads on the
// way), plus the axis's behaviour against a fake motor where the drive does not matter.

#include <filesystem>
#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <viam/sdk/common/proto_value.hpp>
#include <viam/sdk/components/motor.hpp>
#include <viam/sdk/config/resource.hpp>
#include <viam/sdk/resource/resource.hpp>
#include <viam/sdk/resource/resource_api.hpp>

#include "ethercat/errors.hpp"
#include "test_harness.hpp"
#include "viam/module/gantry_axis.hpp"

using ethercat::Error;
using ethercat::servo::GantryAxis;
using viam::sdk::API;
using viam::sdk::Dependencies;
using viam::sdk::GeometryConfig;
using viam::sdk::Motor;
using viam::sdk::Name;
using viam::sdk::ProtoStruct;
using viam::sdk::ProtoValue;
using viam::sdk::ResourceConfig;

namespace {

const char* const kSva = R"({"name":"x-axis","kinematic_param_type":"SVA","links":[],"joints":[]})";

// A valid config, plus whatever `attrs` adds.
ResourceConfig config(ProtoStruct attrs = {}) {
    attrs.emplace("motor", ProtoValue(std::string("x-motor")));
    attrs.emplace("length_mm", ProtoValue(500.0));
    attrs.emplace("mm_per_rev", ProtoValue(40.0));
    return ResourceConfig("gantry", "x-axis", "rdk", std::move(attrs), "rdk:component:gantry", GantryAxis::model());
}

GantryAxis::Attributes parse_with_file(ProtoValue kinematics_file) {
    ProtoStruct a;
    a.emplace("kinematics_file", std::move(kinematics_file));
    return GantryAxis::parse(config(std::move(a)));
}

// Overwritten by every test that uses it, so a leftover from an earlier run is never read.
std::string temp_file(const std::string& contents) {
    const std::string path = (std::filesystem::temp_directory_path() / "gantry_axis_test.json").string();
    std::ofstream(path, std::ios::binary) << contents;
    return path;
}

// The motor as the gantry sees it over RPC: it passes the describe probe, and its stop() either
// works or fails the way a call to a motor the RDK has already removed does.
class FakeMotor final : public Motor {
   public:
    FakeMotor() : Motor("x-motor") {}

    int stops = 0;
    bool unreachable = false;

    void stop(const ProtoStruct& /*extra*/) override {
        ++stops;
        if (unreachable) {
            throw std::runtime_error("Unavailable: motor is being removed");
        }
    }
    ProtoStruct do_command(const ProtoStruct& /*command*/) override {
        ProtoStruct describe;
        describe.emplace("driver", ProtoValue(std::string("viam:ethercat")));
        describe.emplace("homing", ProtoValue(true));
        ProtoStruct reply;
        reply.emplace("describe", ProtoValue(describe));
        return reply;
    }
    void set_power(double /*power_pct*/, const ProtoStruct& /*extra*/) override {}
    void go_for(double /*rpm*/, double /*revolutions*/, const ProtoStruct& /*extra*/) override {}
    void go_to(double /*rpm*/, double /*position_revolutions*/, const ProtoStruct& /*extra*/) override {}
    void set_rpm(double /*rpm*/, const ProtoStruct& /*extra*/) override {}
    void reset_zero_position(double /*offset*/, const ProtoStruct& /*extra*/) override {}
    position get_position(const ProtoStruct& /*extra*/) override {
        return 0.0;
    }
    properties get_properties(const ProtoStruct& /*extra*/) override {
        return {};
    }
    power_status get_power_status(const ProtoStruct& /*extra*/) override {
        return {};
    }
    bool is_moving() override {
        return false;
    }
    ProtoStruct get_status() override {
        return {};
    }
    std::vector<GeometryConfig> get_geometries(const ProtoStruct& /*extra*/) override {
        return {};
    }
};

Dependencies deps_with(const std::shared_ptr<FakeMotor>& motor) {
    Dependencies deps;
    deps.emplace(Name(API::from_string("rdk:component:motor"), "", "x-motor"), motor);
    return deps;
}

}  // namespace

TEST("no kinematics_file: the axis has no kinematics") {
    const GantryAxis::Attributes attrs = GantryAxis::parse(config());
    CHECK_EQ(attrs.motor, std::string("x-motor"));
    CHECK(attrs.kinematics.empty());
}

TEST("kinematics_file: the file's bytes are kept verbatim") {
    const std::vector<unsigned char> k = parse_with_file(ProtoValue(temp_file(kSva))).kinematics;
    CHECK_EQ(std::string(k.begin(), k.end()), std::string(kSva));
}

TEST("kinematics_file: a missing or empty file is a config error, not a silent fixed frame") {
    CHECK_THROWS_MSG(parse_with_file(ProtoValue(std::string("/nonexistent/gantry_axis_test.json"))), Error, "missing or empty");
    CHECK_THROWS_MSG(parse_with_file(ProtoValue(temp_file(""))), Error, "missing or empty");
    CHECK_THROWS_MSG(parse_with_file(ProtoValue(std::string())), Error, "missing or empty");
}

TEST("kinematics_file: must be a string") {
    CHECK_THROWS_MSG(parse_with_file(ProtoValue(42.0)), Error, "wrong type");
}

TEST("validate names the motor as the only dependency") {
    const std::vector<std::string> deps = GantryAxis::validate(config());
    CHECK_EQ(deps.size(), std::size_t{1});
    CHECK_EQ(deps[0], std::string("x-motor"));
}

TEST("stop forwards to the motor") {
    const auto motor = std::make_shared<FakeMotor>();
    GantryAxis axis(deps_with(motor), config());
    axis.stop({});
    CHECK_EQ(motor->stops, 1);
}

TEST("stop survives a motor that is already gone, so removal on shutdown cannot fail") {
    const auto motor = std::make_shared<FakeMotor>();
    GantryAxis axis(deps_with(motor), config());
    motor->unreachable = true;
    axis.stop({});
    CHECK_EQ(motor->stops, 1);
}

TEST_MAIN()
