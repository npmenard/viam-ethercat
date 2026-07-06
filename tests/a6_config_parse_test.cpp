// Validates the shipped example configs parse through the REAL module parser
// (parse_servo_config) and ServoConfig::validate() -- offline, no hardware. Reads
// the actual JSON files, converts JSON -> google::protobuf::Struct -> ProtoStruct
// (the same shape viam-server hands the module), then asserts they parse + validate.
//
// #18: there is no control_mode + no rxpdo/txpdo in a config -- the driver is always
// switch-capable and sets the FIXED superset PDO map itself (in the ServoController
// ctor's validated(), not the parser). So parse_servo_config yields EMPTY rxpdo/txpdo;
// the map materializes at construction. (set_fixed_pdo_map is covered by
// controller_offline_test.)
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

using ethercat::servo::parse_servo_config;
using ethercat::servo::ServoConfig;
using viam::sdk::ProtoStruct;

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

TEST("a6-hardware.example.json parses through the real parser + validate()") {
    const ProtoStruct attrs = load_attributes(A6_HW_CONFIG_PATH);
    const ServoConfig c = parse_servo_config(attrs);  // throws Error on any invalid field
    CHECK(c.counts_per_rev == 131072.0);              // 2^17
    CHECK(c.require_realtime);
    CHECK(c.use_distributed_clocks);
    CHECK(c.sync_cycle_granularity_ns == 250000);
    CHECK(c.gear_ratio == 1.0);
    // #18: no per-mode map + no control_mode -- the parser produces EMPTY maps; the fixed
    // superset is set at construction. #15 item 2: the A6 no-sync code / vendor reset / gloss
    // are code (the a6-servo subclass), not config attributes.
    CHECK(c.rxpdo.entries.empty());
    CHECK(c.txpdo.entries.empty());
}

TEST("a6-minimal.example.json: only required fields + A6 hw bits -> parses + validates") {
    // The minimal-path proof: the shipped minimal config carries NO PDO map and NO move-complete
    // tolerances -- it parses, validate() passes, and the A6 hardware bits ride in the config.
    ProtoStruct attrs = load_attributes(A6_MINIMAL_CONFIG_PATH);
    const ServoConfig c = parse_servo_config(attrs);
    CHECK(c.counts_per_rev == 131072.0);
    CHECK(c.motor_rated_current_amps == 2.5);
    CHECK(c.use_distributed_clocks);  // A6 hardware bit, carried in config
    CHECK(c.sync_cycle_granularity_ns == 250000);
    CHECK(c.rxpdo.entries.empty());  // no map in the config (fixed superset set at construction)
    CHECK(c.txpdo.entries.empty());

    // The fixed superset the driver applies at construction: controlword + 0x6060 + target-pos +
    // profile-vel + target-vel (RxPDO); the full 6-object TxPDO incl. 0x6061 mode-display.
    ServoConfig d = c;
    d.set_fixed_pdo_map();
    CHECK(d.rxpdo.entries.at(0x1600).size() == 5);
    CHECK(d.txpdo.entries.at(0x1A00).size() == 6);
}

TEST_MAIN()
