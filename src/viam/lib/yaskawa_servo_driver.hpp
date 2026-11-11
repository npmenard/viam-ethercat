#pragma once

// YaskawaServoDriver -- the Yaskawa Sigma-X (SGDXS) specialization of the generic CiA402 driver.
//
// The drive scales every reference object through manufacturer "user unit" ratios (2701h-2704h).
// Position and velocity default to 64/1, so a generic master writing 0x607A in encoder counts
// commands 64x the intended distance. This subclass sets the ratios so the drive speaks counts.

#include <cstdint>
#include <string>

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

class YaskawaServoDriver final : public ServoController {
   public:
    using ServoController::ServoController;
    // The drive runs CSP synchronous to SYNC0, and validate() refuses CSP without distributed clocks.
    // Profile, not CSP: the drive bounds velocity with 0x6081, so a bad target is a wrong move
    // rather than a loop-gain-scaled lunge. CSP stays available through control_mode.
    static constexpr MotionModeKind kDefaultMotionMode = MotionModeKind::Profile;

   protected:
    void configure_device(ConfigContext& cfg) override;
    std::string fault_description(std::uint16_t code) const override;
};

}  // namespace ethercat::servo
