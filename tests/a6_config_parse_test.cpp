// Validates the shipped example configs parse through the REAL module parser
// (parse_servo_config) and ServoConfig::validate() -- offline, no hardware. Reads
// the actual JSON files, converts JSON -> google::protobuf::Struct -> ProtoStruct
// (the same shape viam-server hands the module), then asserts they parse + validate.
//
// Needs the Viam SDK (ProtoStruct) + protobuf JSON, so it builds in the
// ETHERCAT_BUILD_MODULE path. The file paths come from -D compile definitions.

#include <fstream>
#include <sstream>
#include <string>

#include <google/protobuf/struct.pb.h>
#include <google/protobuf/util/json_util.h>

#include <viam/sdk/common/proto_convert.hpp>
#include <viam/sdk/common/proto_value.hpp>

#include "ethercat/errors.hpp"
#include "test_harness.hpp"
#include "viam/lib/servo_config.hpp"
#include "viam/module/servo_motor.hpp"

using ethercat::servo::ControlMode;
using ethercat::servo::parse_servo_config;
using ethercat::servo::ServoConfig;
using viam::sdk::ProtoStruct;
using viam::sdk::ProtoValue;

namespace {

// Read a JSON config file and return its top-level "attributes" object as a
// ProtoStruct -- exactly what cfg.attributes() yields inside the module.
ProtoStruct load_attributes(const std::string& path) {
    std::ifstream in(path);
    CHECK(in.good());
    std::stringstream buf;
    buf << in.rdbuf();

    google::protobuf::Struct doc;
    const auto status = google::protobuf::util::JsonStringToMessage(buf.str(), &doc);
    CHECK(status.ok());

    const ProtoStruct top = viam::sdk::from_proto(doc);
    const auto it = top.find("attributes");
    CHECK(it != top.end());
    const ProtoStruct* const attrs = it->second.get<ProtoStruct>();
    CHECK(attrs != nullptr);
    return *attrs;
}

}  // namespace

TEST("a6-hardware.example.json: PP config parses through the real parser + validate()") {
    const ProtoStruct attrs = load_attributes(A6_HW_CONFIG_PATH);
    const ServoConfig c = parse_servo_config(attrs);  // throws ConfigError on any invalid field
    CHECK(c.mode == ControlMode::ProfilePosition);
    CHECK(c.counts_per_rev == 131072.0);  // 2^17
    CHECK(c.require_realtime);
    CHECK(c.rxpdo.entries.at(0x1600).size() == 3);  // controlword + target position + profile velocity
    CHECK(c.txpdo.entries.at(0x1A00).size() == 6);  // fault, status, mode-display, pos, vel, torque
    // #39: the consumer-side vendor fault-reset rides in the example (A6 0x2031:01 = 1, U16 LE).
    CHECK(c.vendor_fault_reset.has_value());
    CHECK(c.vendor_fault_reset->index == 0x2031);
    CHECK(c.vendor_fault_reset->subindex == 0x01);
    CHECK(c.vendor_fault_reset->data.size() == 2);
    CHECK(c.vendor_fault_reset->data[0] == std::byte{0x01});
    CHECK(c.vendor_fault_reset->data[1] == std::byte{0x00});
}

TEST("#39: the obsolete 'fault_reset' config key is REJECTED, never silently ignored") {
    // A deployed config silently losing its reset would be a silent behavior change --
    // the parser must fail loudly with the migration path in the message.
    ProtoStruct attrs = load_attributes(A6_HW_CONFIG_PATH);
    attrs["fault_reset"] = viam::sdk::ProtoValue(true);  // the pre-#39 key, any shape
    CHECK_THROWS(parse_servo_config(attrs), ethercat::ConfigError);
}

TEST("a6-hardware.example.json: PV variant parses + validates") {
    ProtoStruct attrs = load_attributes(A6_HW_CONFIG_PATH);
    const auto pv_it = attrs.find("_pv_variant");
    CHECK(pv_it != attrs.end());
    const ProtoStruct* const pv = pv_it->second.get<ProtoStruct>();
    CHECK(pv != nullptr);
    // Apply the documented PV overrides (control_mode + rxpdo) and re-parse.
    attrs["control_mode"] = pv->at("control_mode");
    attrs["rxpdo"] = pv->at("rxpdo");
    const ServoConfig c = parse_servo_config(attrs);
    CHECK(c.mode == ControlMode::ProfileVelocity);
    CHECK(c.rxpdo.entries.at(0x1600).size() == 2);  // controlword + target velocity
}

TEST("a6-servo.example.json: sim config parses + validates") {
    const ProtoStruct attrs = load_attributes(A6_SIM_CONFIG_PATH);
    const ServoConfig c = parse_servo_config(attrs);
    CHECK(c.mode == ControlMode::ProfilePosition);
    CHECK(c.counts_per_rev == 131072.0);
}

TEST_MAIN()
