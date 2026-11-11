#pragma once

// YaskawaServoDriver -- the Yaskawa Sigma-X (SGDXS) specialization of the generic CiA402 driver.
//
// The drive scales every reference object through manufacturer "user unit" ratios (2701h-2704h).
// Position and velocity default to 64/1, so a generic master writing 0x607A in encoder counts
// commands 64x the intended distance. This subclass sets the ratios so the drive speaks counts.

#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "ethercat/runner.hpp"  // ConfigContext
#include "viam/lib/servo_controller.hpp"

namespace ethercat::servo {

// One manufacturer user-unit object: the drive scales a reference by numerator/denominator, held in
// subindexes 1 and 2. A ratio outside the drive's documented range raises A20h (parameter setting
// error) at bring-up, so the values are pinned by test rather than discovered on the machine.
// What one acceleration unit (0x6085, 0x609A) is worth in the counts the rest of the driver speaks.
// 1 unit = (2703h ratio) x 10^4 increments/s^2, and a count is (2701h ratio) increments, so the two
// ratios decide this together; a test holds all three in step.
inline constexpr double kAccelUnitCps2 = 10000.0;

// The drive's hardwired safety inputs, as seen on 0x60FD. Which CN1 pin lands on which bit, and
// which level means safe, is wiring -- so both are declared, never inferred: CiA402 leaves bits
// 16-31 manufacturer-defined and says nothing about polarity, and guessing either is a safety bug.
//
// Which 0x60FD bit is which pin, and which level is safe, is wiring -- declared, never inferred
// (CiA402 fixes only bits 0-2 and leaves 16-31 manufacturer-defined).
struct SafetyInputs {
    struct Tier {
        std::vector<std::uint8_t> require_high;  // motion permitted only while every one of these reads 1
        std::vector<std::uint8_t> require_low;   // ... and every one of these reads 0
        bool declared() const noexcept {
            return !require_high.empty() || !require_low.empty();
        }
    };
    Tier stop;        // hold CiA402 Shutdown: torque is already gone, there is nothing to ramp
    Tier quick_stop;  // hold CiA402 quick-stop: the drive ramps at 0x6085 while it still has power
    bool declared() const noexcept {
        return stop.declared() || quick_stop.declared();
    }
    void validate() const;  // throws Error on a bit outside 0-31 or a bit in both lists of one tier
};

// True when a tier's declared inputs forbid motion: any require_high bit reads 0, or any require_low
// bit reads 1. An undeclared tier never engages. Pure, so the whole decision is testable off-bus.
bool safety_engaged(const SafetyInputs::Tier& tier, std::uint32_t digital_inputs) noexcept;

// The TxPDO entry a declared interlock needs, for the model's builder to add to extra_txpdo.

class YaskawaServoDriver final : public ServoController {
   public:
    using ServoController::ServoController;
    YaskawaServoDriver(ServoConfig config, SafetyInputs safety) : ServoController(std::move(config)), safety_(std::move(safety)) {}
    // The drive runs CSP synchronous to SYNC0, and validate() refuses CSP without distributed clocks.
    // Profile, not CSP: the drive bounds velocity with 0x6081, so a bad target is a wrong move
    // rather than a loop-gain-scaled lunge. CSP stays available through control_mode.
    static constexpr MotionModeKind kDefaultMotionMode = MotionModeKind::Profile;

   protected:
    void configure_device(ConfigContext& cfg) override;
    void resolve_device_fields(Master& master, std::uint16_t slave) override;
    Inhibit inhibit(const CycleContext& ctx) const noexcept override;
    double accel_units_per_cps2() const noexcept override {
        return 1.0 / kAccelUnitCps2;
    }
    std::string fault_description(std::uint16_t code) const override;

   private:
    SafetyInputs safety_;
    std::optional<FieldLocation> f_digital_inputs_;
};

}  // namespace ethercat::servo
