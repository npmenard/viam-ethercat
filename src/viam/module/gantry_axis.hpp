#pragma once

// viam:ethercat:gantry -- one linear axis driven by a viam:ethercat servo motor.
//
// The gantry owns millimetres: the travel length, the conversion to motor revolutions, and the
// bound that keeps a move inside the axis. Everything below that (the cycle loop, the CiA402 mode
// switch, the homing procedure) belongs to the motor, which this component drives through the
// standard motor API plus the module's own do_command contract. It therefore depends on a motor
// component and never touches the EtherCAT bus itself.

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <viam/sdk/common/kinematics.hpp>
#include <viam/sdk/components/gantry.hpp>
#include <viam/sdk/components/motor.hpp>
#include <viam/sdk/config/resource.hpp>
#include <viam/sdk/registry/registry.hpp>
#include <viam/sdk/resource/resource.hpp>

namespace ethercat::servo {

using namespace viam::sdk;

class GantryAxis final : public Gantry {
   public:
    static Model model();
    static std::shared_ptr<ModelRegistration> create_model_registration();
    // Validator: parses the attributes and returns the motor as this axis's dependency.
    static std::vector<std::string> validate(const ResourceConfig& cfg);

    // Attributes. Nothing about homing: the motor owns the whole procedure and answers `home` with
    // the position it ended at, which is this axis's zero.
    struct Attributes {
        std::string motor;
        double length_mm = 0.0;
        double mm_per_rev = 0.0;
        double mm_per_sec = 0.0;  // default move speed; 0 = a speed is required per move
        bool travel_positive = true;
        // Contents of `kinematics_file`, served verbatim as SVA. Empty = fixed frame, as without a file.
        std::vector<unsigned char> kinematics;
    };
    // Public so the parser is testable without a motor to depend on.
    static Attributes parse(const ResourceConfig& cfg);

    GantryAxis(const Dependencies& deps, const ResourceConfig& cfg);

    GantryAxis(const GantryAxis&) = delete;
    GantryAxis& operator=(const GantryAxis&) = delete;
    GantryAxis(GantryAxis&&) = delete;
    GantryAxis& operator=(GantryAxis&&) = delete;
    ~GantryAxis() override = default;

    // --- rdk:component:gantry (one axis, so every vector has one entry) ---
    std::vector<double> get_position(const ProtoStruct& extra) override;
    void move_to_position(const std::vector<movement_coordinate>& coordinates, const ProtoStruct& extra) override;
    bool home(const ProtoStruct& extra) override;
    std::vector<double> get_lengths(const ProtoStruct& extra) override;
    bool is_moving() override;
    ProtoStruct get_status() override;
    void stop(const ProtoStruct& extra) override;
    ProtoStruct do_command(const ProtoStruct& command) override;
    std::vector<GeometryConfig> get_geometries(const ProtoStruct& extra) override;
    KinematicsData get_kinematics(const ProtoStruct& extra) override;

   private:
    // Resolve the attributes and the motor dependency. Called from the constructor: the SDK has no
    // reconfigure hook, so a config change builds a new component.
    void configure(const Dependencies& deps, const ResourceConfig& cfg);

    // Which way motor revolutions run as the axis travels away from its home end.
    double sign() const noexcept {
        return attrs_.travel_positive ? 1.0 : -1.0;
    }
    // Millimetres are measured from the home reference, so every conversion goes through it.
    double mm_to_revs(double mm) const noexcept {
        return home_revs_ + (sign() * mm / attrs_.mm_per_rev);
    }
    double revs_to_mm(double revs) const noexcept {
        return sign() * (revs - home_revs_) * attrs_.mm_per_rev;
    }
    // The motor's `status`, which also proves the dependency is one of ours.
    ProtoStruct motor_status() const;
    // The motor's verdict: a homing run succeeded this session, or its absolute encoder kept the
    // reference across the restart.
    bool homed() const;

    Attributes attrs_;
    std::shared_ptr<Motor> motor_;
    // Motor position of this axis's zero, captured from the last home. 0 until then: a drive whose
    // absolute encoder retained the reference reports position in the frame its last home set.
    double home_revs_ = 0.0;
};

}  // namespace ethercat::servo
