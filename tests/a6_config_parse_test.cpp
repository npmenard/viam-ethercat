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
    // #15 item 2: the A6 no-sync code, vendor fault-reset, and 0x603F gloss are NO LONGER config
    // attributes -- they moved to the A6ServoDriver subclass (the viam:ethercat:a6-servo model).
    // The example carries none of them; the parser produces a generic ServoConfig.
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

TEST("a6-minimal.example.json: no rxpdo/txpdo -> parses, derives the PP map (#61/#64), validates") {
    // The #64 minimal-path proof: the shipped minimal config carries NO rxpdo/txpdo and NO
    // move-complete tolerances -- it parses, and apply_derived_pdo_maps() materializes the
    // standard CiA402 PP map (#61) from control_mode alone, then validate() passes.
    ProtoStruct attrs = load_attributes(A6_MINIMAL_CONFIG_PATH);
    ServoConfig c = parse_servo_config(attrs);
    CHECK(c.mode == ControlMode::ProfilePosition);
    CHECK(c.counts_per_rev == 131072.0);
    CHECK(c.motor_rated_current_amps == 2.5);
    // The A6 hardware bits ride in the minimal config (never library constants).
    CHECK(c.use_distributed_clocks);
    CHECK(c.sync_cycle_granularity_ns == 250000);
    // As shipped: the maps are EMPTY (derived, not spelled out).
    CHECK(c.rxpdo.entries.empty());
    CHECK(c.txpdo.entries.empty());

    c.apply_derived_pdo_maps();

    // Derived PP RxPDO 0x1600: controlword + target-pos + profile-vel (NO 0x6060 -- PP is fixed).
    const auto& rx = c.rxpdo.entries.at(0x1600);
    CHECK(rx.size() == 3);
    CHECK(rx.at(0).index == 0x6040);  // controlword
    CHECK(rx.at(1).index == 0x607A);  // target position
    CHECK(rx.at(2).index == 0x6081);  // profile velocity
    for (const auto& e : rx) {
        CHECK(e.index != 0x6060);  // mode-of-operation is NOT mapped for fixed PP
    }
    // Derived TxPDO 0x1A00: fault + status + mode-display + pos + vel + torque.
    const auto& tx = c.txpdo.entries.at(0x1A00);
    CHECK(tx.size() == 6);
    CHECK(tx.at(0).index == 0x603F);
    CHECK(tx.at(2).index == 0x6061);  // mode-display -- the #45/#57 mode-echo source
    // SM assign-index derived from direction (0x1600->0x1C12, 0x1A00->0x1C13).
    CHECK(c.rxpdo.assign_index(ethercat::PdoDirection::Rx) == 0x1C12);
    CHECK(c.txpdo.assign_index(ethercat::PdoDirection::Tx) == 0x1C13);

    c.require_realtime = false;  // CI has no RT scheduling; the shipped config keeps true.
    c.validate();                // throws ConfigError on any invalid field
}

TEST_MAIN()
