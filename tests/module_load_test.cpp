// Module-LOAD smoke: exercise the full Viam SDK boundary that servo_motor_test
// (which uses the direct test ctor) SKIPS -- model registration (the registered
// construct + validate functors actually INVOKED), config validate (+ negatives),
// the CONFIG-DRIVEN production ctor (ServoMotor{deps, ResourceConfig} -> parse ->
// wants_simulation -> SimBackend), the Motor API over that resource, Reconfigure
// (PP->PV contract flip), and clean shutdown -- all offline against SimBackend.
// The deferred Phase-6 done-criterion (#12): prove the module LOADS + runs through
// the gRPC-resource lifecycle a viam-server would drive, without a real server.
//
// Needs the Viam C++ SDK, so it builds only in the ETHERCAT_BUILD_MODULE path (the
// Docker/CI image). The sim config path comes from a -D compile definition.

#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <google/protobuf/struct.pb.h>
#include <google/protobuf/util/json_util.h>

#include <viam/sdk/common/proto_convert.hpp>
#include <viam/sdk/common/proto_value.hpp>
#include <viam/sdk/components/motor.hpp>
#include <viam/sdk/config/resource.hpp>
#include <viam/sdk/registry/registry.hpp>
#include <viam/sdk/resource/resource.hpp>

#include "ethercat/backend.hpp"
#include "ethercat/errors.hpp"
#include "ethercat/sim_backend.hpp"
#include "test_harness.hpp"
#include "viam/lib/servo_config.hpp"
#include "viam/lib/servo_controller.hpp"
#include "viam/module/servo_motor.hpp"

using ethercat::ConfigError;
using ethercat::EcatBackend;
using ethercat::PdoEntry;
using ethercat::SimBackend;
using ethercat::SimSlaveModel;
using ethercat::servo::ControlMode;
using ethercat::servo::ServoConfig;
using ethercat::servo::ServoController;
using ethercat::servo::ServoMotor;
using viam::sdk::Dependencies;
using viam::sdk::Motor;
using viam::sdk::ProtoStruct;
using viam::sdk::ProtoValue;
using viam::sdk::ResourceConfig;

namespace {

// Load etc/a6-servo.example.json's "attributes" object as a ProtoStruct -- exactly
// what viam-server hands the module's constructor (JSON -> protobuf -> ProtoStruct).
ProtoStruct load_sim_attrs() {
    std::ifstream in(A6_SIM_CONFIG_PATH);
    CHECK(in.good());
    std::stringstream buf;
    buf << in.rdbuf();
    google::protobuf::Struct doc;
    CHECK(google::protobuf::util::JsonStringToMessage(buf.str(), &doc).ok());
    const ProtoStruct top = viam::sdk::from_proto(doc);
    const auto it = top.find("attributes");
    CHECK(it != top.end());
    const ProtoStruct* const attrs = it->second.get<ProtoStruct>();
    CHECK(attrs != nullptr);
    return *attrs;
}

// Wrap attributes into a ResourceConfig the way the SDK would for our model. The
// ServoMotor ctor reads only cfg.name() + cfg.attributes(), so the metadata
// (type/namespace/api) just needs to be well-formed.
ResourceConfig make_resource_config(ProtoStruct attrs, const std::string& name) {
    return ResourceConfig("motor", name, "rdk", std::move(attrs), "rdk:component:motor", ServoMotor::model());
}

template <class Pred>
bool wait_until(Pred pred, std::chrono::milliseconds timeout) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        if (pred()) {
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return pred();
}

// do_command("status") -> the nested status map (is_powered/is_moving/position/
// is_disconnected/last_error).
ProtoStruct status_of(ServoMotor& motor) {
    ProtoStruct cmd;
    cmd.emplace("status", ProtoValue(true));
    const ProtoStruct out = motor.do_command(cmd);
    const auto it = out.find("status");
    CHECK(it != out.end());
    const ProtoStruct* const s = it->second.get<ProtoStruct>();
    CHECK(s != nullptr);
    return *s;
}

bool status_powered(ServoMotor& motor) {
    const ProtoStruct s = status_of(motor);
    const auto it = s.find("is_powered");
    const bool* const b = it != s.end() ? it->second.get<bool>() : nullptr;
    return b != nullptr && *b;
}

std::string status_last_error(ServoMotor& motor) {
    const ProtoStruct s = status_of(motor);
    const auto it = s.find("last_error");
    const std::string* const e = it != s.end() ? it->second.get<std::string>() : nullptr;
    return e != nullptr ? *e : std::string{"<missing>"};
}

}  // namespace

// ---- Part A: the config-driven module-load path (what servo_motor_test skips) ----

TEST("module-load: the model registers and the registered construct/validate functors WORK") {
    const auto regs = ServoMotor::create_model_registrations();
    CHECK(!regs.empty());
    CHECK(ServoMotor::model().to_string() == "viam:ethercat:servo");

    // Identity isn't enough -- the point of #12 is that the SDK can actually FIND and
    // CONSTRUCT through this registration. Invoke the registered functors (what the
    // SDK's resource manager calls when a viam-server loads the module).
    const auto& reg = regs.front();
    const ResourceConfig cfg = make_resource_config(load_sim_attrs(), "registered");
    CHECK(reg->validate(cfg).empty());  // registered validator -> no deps

    const std::shared_ptr<viam::sdk::Resource> res = reg->construct_resource(Dependencies{}, cfg);
    const std::shared_ptr<Motor> motor = std::dynamic_pointer_cast<Motor>(res);
    CHECK(motor != nullptr);                   // constructed a Motor through the registration
    (void)motor->get_position(ProtoStruct{});  // and it responds to the Motor API
}

TEST("module-load: validate() accepts the sim config and rejects malformed ones with clear text") {
    CHECK(ServoMotor::validate(make_resource_config(load_sim_attrs(), "ok")).empty());

    // Negatives -- Viam surfaces validate() text to the user, so the message must name
    // the offending field. (a) missing required map, (b) missing required scalar,
    // (c) a proto TYPE mismatch (control_mode as a number).
    ProtoStruct no_tx = load_sim_attrs();
    no_tx.erase("txpdo");
    CHECK_THROWS_MSG(ServoMotor::validate(make_resource_config(std::move(no_tx), "no-tx")), ConfigError, "txpdo");

    ProtoStruct no_cpr = load_sim_attrs();
    no_cpr.erase("counts_per_rev");
    CHECK_THROWS_MSG(ServoMotor::validate(make_resource_config(std::move(no_cpr), "no-cpr")), ConfigError, "counts_per_rev");

    ProtoStruct bad_mode = load_sim_attrs();
    bad_mode["control_mode"] = ProtoValue(static_cast<double>(1));  // number where a string is required
    CHECK_THROWS_MSG(ServoMotor::validate(make_resource_config(std::move(bad_mode), "bad-mode")), ConfigError, "control_mode");
}

TEST("module-load: simulate defaults FALSE -- a config without it routes to real hardware") {
    // The single most important negative: if `simulate` ever became the default,
    // production would silently run fake hardware. Omit it (and use a non-"sim"
    // interface so the ifname trigger doesn't apply) -> wants_simulation()==false ->
    // SoemBackend -> start() can't open the bogus NIC -> the ctor THROWS. If this ever
    // STOPS throwing, sim has become the default and that's the regression.
    ProtoStruct attrs = load_sim_attrs();
    attrs.erase("simulate");
    attrs["interface"] = ProtoValue(std::string("ethercat-no-such-nic-9999"));
    CHECK_THROWS(ServoMotor(Dependencies{}, make_resource_config(std::move(attrs), "real-hw")), std::exception);
}

TEST("module-load: config-driven construct -> SimBackend -> Motor API (require_realtime=false)") {
    // The actual load path: ServoMotor{deps, cfg} parses the attributes, sees
    // simulate=true, builds a SimBackend-backed controller, and starts the RT loop
    // BEST-EFFORT (require_realtime=false -- CI has no SCHED_FIFO; construction must
    // NOT throw and the loop must still cycle).
    ServoMotor motor{Dependencies{}, make_resource_config(load_sim_attrs(), "sim-motor")};
    CHECK(wait_until([&] { return status_powered(motor); }, std::chrono::milliseconds(1000)));

    // Config ROUND-TRIP: go_to a known revs and read it back. It converges + reads
    // back ~2.0 only if the controller was built from the PARSED kinematics (a
    // silently-defaulted counts_per_rev=0 would fail validate() before we got here;
    // any other parse drop breaks the move). The loop CYCLING is itself the
    // require_realtime=false best-effort proof.
    motor.go_to(1000.0, 2.0, ProtoStruct{});  // blocks to completion
    CHECK(std::abs(motor.get_position(ProtoStruct{}) - 2.0) < 0.01);
    CHECK(!motor.is_moving());
    motor.stop(ProtoStruct{});
    CHECK(!motor.is_moving());

    CHECK(status_last_error(motor).find("drive fault") == std::string::npos);  // healthy -> no fault tier
}

TEST("module-load: a Tier-2 API error leaves the resource operational + next command works") {
    // set_power is unsupported -> a clear error to the caller, but it must NOT
    // de-power/fault the drive; a subsequent valid command then succeeds with NO
    // fault_reset (the two-tier fault-model ruling, end-to-end through the Motor API).
    ServoMotor motor{Dependencies{}, make_resource_config(load_sim_attrs(), "tier2-motor")};
    CHECK(wait_until([&] { return status_powered(motor); }, std::chrono::milliseconds(1000)));

    CHECK_THROWS(motor.set_power(0.5, ProtoStruct{}), std::runtime_error);  // Tier-2 reject
    CHECK(status_powered(motor));                                           // still energized -- NOT faulted

    motor.go_to(1000.0, 1.0, ProtoStruct{});  // a valid command works without fault_reset
    CHECK(std::abs(motor.get_position(ProtoStruct{}) - 1.0) < 0.01);
}

TEST("module-load: is_moving uses the |delta| completion predicate (go_for moves then settles)") {
    ServoMotor motor{Dependencies{}, make_resource_config(load_sim_attrs(), "moving-motor")};
    CHECK(wait_until([&] { return status_powered(motor); }, std::chrono::milliseconds(1000)));

    std::thread mover([&] { motor.go_for(500.0, 5.0, ProtoStruct{}); });  // relative move in flight
    CHECK(wait_until([&] { return motor.is_moving(); }, std::chrono::milliseconds(500)));
    mover.join();
    CHECK(!motor.is_moving());  // settled via |target-actual|<=tol, never statusword bit10
}

TEST("module-load: reconfigure PP -> PV FLIPS the API contract (re-resolves the mode)") {
    ServoMotor motor{Dependencies{}, make_resource_config(load_sim_attrs(), "recfg-motor")};
    CHECK(wait_until([&] { return status_powered(motor); }, std::chrono::milliseconds(1000)));
    motor.go_to(1000.0, 1.0, ProtoStruct{});  // PP: go_to is accepted
    CHECK(std::abs(motor.get_position(ProtoStruct{}) - 1.0) < 0.01);

    // Reconfigure to Profile-Velocity -> stop the old controller, rebuild a fresh
    // SimBackend-backed one in PV mode (the sim RxPDO maps 0x60FF). The mode must
    // actually flip: go_to now REJECTS (PP-only) -- proving reconfigure re-resolved
    // the control mode, not just restarted on the stale resolution.
    ProtoStruct pv = load_sim_attrs();
    pv["control_mode"] = ProtoValue(std::string("PV"));
    motor.reconfigure(Dependencies{}, make_resource_config(std::move(pv), "recfg-motor"));
    CHECK(wait_until([&] { return status_powered(motor); }, std::chrono::milliseconds(1000)));  // usable after rebuild

    CHECK_THROWS(motor.go_to(1000.0, 2.0, ProtoStruct{}), ConfigError);  // PP-only -> rejected in PV
}

TEST("module-load: API-after-stop is fail-safe (not powered, last_error doesn't crash)") {
    ServoMotor motor{Dependencies{}, make_resource_config(load_sim_attrs(), "stop-motor")};
    CHECK(wait_until([&] { return status_powered(motor); }, std::chrono::milliseconds(1000)));
    motor.stop(ProtoStruct{});  // Halt (sticky): the motor stays stopped
    CHECK(!motor.is_moving());
    (void)status_last_error(motor);           // must not crash post-stop (stopping_ fail-safe)
    (void)motor.get_position(ProtoStruct{});  // accessors stay safe
}

TEST("module-load: do_command power keys (fault_reset/enable/disable) return without throwing") {
    ServoMotor motor{Dependencies{}, make_resource_config(load_sim_attrs(), "docmd-motor")};
    CHECK(wait_until([&] { return status_powered(motor); }, std::chrono::milliseconds(1000)));
    for (const char* key : {"fault_reset", "disable", "enable"}) {
        ProtoStruct cmd;
        cmd.emplace(key, ProtoValue(true));
        const ProtoStruct out = motor.do_command(cmd);
        CHECK(out.find(key) != out.end());  // each acknowledged in the reply
    }
}

TEST("module-load: clean shutdown joins the RT thread promptly (no hang/leak)") {
    const auto t0 = std::chrono::steady_clock::now();
    {
        ServoMotor motor{Dependencies{}, make_resource_config(load_sim_attrs(), "shutdown-motor")};
        CHECK(wait_until([&] { return status_powered(motor); }, std::chrono::milliseconds(1000)));
    }  // dtor -> controller stop() -> RT thread join
    CHECK(std::chrono::steady_clock::now() - t0 < std::chrono::seconds(5));
}

// ---- Part B: #16 fault legibility reaches the gRPC consumer (do_command) ----
// The config-driven ctor hides the SimBackend, so to drive a fault we use the TEST
// ctor with a captured backend pointer -- but the assertion is still on the real
// Motor API (do_command return), proving #16's compose-all last_error() reaches a
// gRPC consumer end-to-end through the module boundary. (Per team-lead: no
// production test-seam; fault-injection through the full config path would be a
// future sim-only config knob, not #12.)

namespace {

ServoConfig fault_config() {
    ServoConfig c;
    c.ifname = "sim0";
    c.slave_id = 1;
    c.mode = ControlMode::ProfilePosition;
    c.rxpdo.pdo_indices = {0x1600};
    c.rxpdo.entries[0x1600] = {PdoEntry{0x6040, 0, 16}, PdoEntry{0x607A, 0, 32}, PdoEntry{0x6081, 0, 32}};
    c.txpdo.pdo_indices = {0x1A00};
    c.txpdo.entries[0x1A00] = {PdoEntry{0x6041, 0, 16}, PdoEntry{0x6064, 0, 32}, PdoEntry{0x603F, 0, 16}};
    c.fault_code_labels = {{0x8700, "Er74.1 / no SYNC0"}};
    c.max_motor_speed_rpm = 3000.0;
    c.motor_rated_current_amps = 2.5;
    c.gear_ratio = 1.0;
    c.counts_per_rev = 131072.0;
    c.position_tolerance_counts = 20;
    c.velocity_threshold = 1'000'000'000;
    c.target_loop_rate_hz = 1000;
    c.require_realtime = false;
    c.command_queue_capacity = 64;
    c.handshake_timeout_cycles = 1000;
    return c;
}

std::unique_ptr<ServoController> fault_controller(SimBackend** out) {
    auto factory = [out] {
        SimSlaveModel m;
        m.output_bytes = 10;  // ctrl@0, target@2, profile-vel@6
        m.input_bytes = 8;    // status@0, actual@2, 0x603F@6
        m.ctrlword_off = 0;
        m.target_off = 2;
        m.profile_velocity_off = 6;
        m.statusword_off = 0;
        m.actual_off = 2;
        m.fault_code_off = 6;
        m.mode = ethercat::Cia402Mode::ProfilePosition;
        m.counts_per_step = 50'000;
        auto be = std::make_unique<SimBackend>(std::vector<SimSlaveModel>{m});
        if (out != nullptr) {
            *out = be.get();
        }
        return std::unique_ptr<EcatBackend>(std::move(be));
    };
    return std::make_unique<ServoController>(fault_config(), factory);
}

}  // namespace

TEST("module-load: a drive fault is legible through do_command(status).last_error") {
    SimBackend* sim = nullptr;
    ServoMotor motor{"fault-motor", fault_controller(&sim)};
    CHECK(wait_until([&] { return status_powered(motor); }, std::chrono::milliseconds(1000)));
    CHECK(sim != nullptr);

    sim->set_fault_code(1, 0x8700);  // Er74.1 (the missed-SYNC0 code)
    sim->inject_fault(1);
    CHECK(wait_until([&] { return !status_powered(motor); }, std::chrono::milliseconds(1000)));

    // The #16 compose-all legibility reaches the gRPC return verbatim.
    const std::string err = status_last_error(motor);
    CHECK(err.find("drive fault 0x8700") != std::string::npos);
    CHECK(err.find("Er74.1 / no SYNC0") != std::string::npos);
}

TEST_MAIN()
