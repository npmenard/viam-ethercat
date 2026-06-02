#pragma once

// ServoMotor -- the Viam rdk:component:motor implementation over a ServoController.
//
// This is THIN SDK glue: every Motor method forwards to the ServoController (the
// RT/concurrency engine from Phase 5), mapping the generic motor API onto CiA402
// Profile-Position / Profile-Velocity. ServoMotor adds NO locking of its own --
// all API-vs-Reconfigure serialization lives in the controller's api_mutex_.
//
// A6 specifics never appear here: the PDO map, counts/rev, gear ratio, etc. are
// parsed from the Viam resource config into a ServoConfig (config data, not code).

#include <memory>
#include <string>
#include <vector>

#include <viam/sdk/components/motor.hpp>
#include <viam/sdk/config/resource.hpp>
#include <viam/sdk/registry/registry.hpp>
#include <viam/sdk/resource/reconfigurable.hpp>

#include "viam/lib/servo_controller.hpp"

namespace ethercat::servo {

using namespace viam::sdk;

// Parse a Viam resource config's attributes (the ProtoStruct from cfg.attributes())
// into a validated ServoConfig. Throws ethercat::ConfigError (clear text) on any
// missing/wrong-typed/invalid field. Exposed for offline config-file validation.
ServoConfig parse_servo_config(const ProtoStruct& attributes);

class ServoMotor final : public Motor, public Reconfigurable {
   public:
    // Model identity: viam:ethercat:servo (generic -- A6 is config data, not a model).
    static const ModelFamily& model_family();
    static Model model();
    static std::vector<std::shared_ptr<ModelRegistration>> create_model_registrations();

    // Static validator for ModelRegistration: parse+validate the config, throwing
    // a clear ConfigError on any problem. A motor has no dependencies -> returns {}.
    static std::vector<std::string> validate(const ResourceConfig& cfg);

    // Production constructor: parse cfg -> ServoConfig -> build + start a
    // ServoController (SoemBackend, or a SimBackend when `simulate`/ifname=="sim").
    ServoMotor(const Dependencies& deps, const ResourceConfig& cfg);

    // Test seam: inject an already-built controller (e.g. SimBackend-backed). Starts it.
    ServoMotor(std::string name, std::unique_ptr<ServoController> controller);

    ~ServoMotor() override;

    ServoMotor(const ServoMotor&) = delete;
    ServoMotor& operator=(const ServoMotor&) = delete;
    ServoMotor(ServoMotor&&) = delete;
    ServoMotor& operator=(ServoMotor&&) = delete;

    void reconfigure(const Dependencies& deps, const ResourceConfig& cfg) override;

    // --- rdk:component:motor ---
    void set_power(double power_pct, const ProtoStruct& extra) override;
    void set_rpm(double rpm, const ProtoStruct& extra) override;
    void go_for(double rpm, double revolutions, const ProtoStruct& extra) override;
    void go_to(double rpm, double position_revolutions, const ProtoStruct& extra) override;
    void reset_zero_position(double offset, const ProtoStruct& extra) override;
    position get_position(const ProtoStruct& extra) override;
    properties get_properties(const ProtoStruct& extra) override;
    power_status get_power_status(const ProtoStruct& extra) override;
    bool is_moving() override;
    void stop(const ProtoStruct& extra) override;
    ProtoStruct do_command(const ProtoStruct& command) override;
    std::vector<GeometryConfig> get_geometries(const ProtoStruct& extra) override;

   private:
    std::unique_ptr<ServoController> controller_;
};

}  // namespace ethercat::servo
