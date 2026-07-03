#pragma once

// A6ServoDriver -- the ANCTL AS715N (A6-EC) specialization of the generic ServoController
// (#15 item 2). The base ServoController IS the generic CiA402 servo driver; this subclass
// overrides EXACTLY the three seams that carry A6 vendor knowledge -- knowledge that USED to
// ride in the hardware JSON as config (sync_fault_code / vendor_fault_reset / fault_code_labels).
// Specialization by subclass is now the sanctioned extension mechanism (it reverses the old
// "drive specifics live only as config data" rule).
//
// The three A6 residuals, each a one-line override:
//   1. vendor fault-reset: write u16 1 to 0x2031:01 -- the A6's reset is a vendor SDO, NOT the
//      standard CiA402 controlword bit7 (CLAUDE.md lesson 4).
//   2. the DC "no-SYNC0" bring-up fault code: 0x8700 (Er74.1) -- the 0x603F value the drive
//      reports while SYNC0 has not yet established (the bring-up gate reads it).
//   3. the 0x603F gloss for that code -> "Er74.1 / no SYNC0" in last_error().
//
// The A6 model (viam:ethercat:a6-servo) is a TEST VEHICLE: correctness bar = moves work + the
// RDK campaign tests pass. Everything else (PDO map, limits, kinematics) stays per-machine config.
//
// HW-VERIFY (not modeled in sim): that writing 0x2031:01=1 actually CLEARS the A6's fault, and
// that 0x8700 is the exact code the drive parks at pre-sync, are drive behaviors proven on the
// bench, not offline. What IS offline-testable (and tested): the seam EMITS the 0x2031 write and
// reports the 0x8700 gloss, where the generic base does neither.

#include <cstdint>
#include <optional>
#include <string>

#include "ethercat/pdo_mapping.hpp"  // ethercat::SdoWrite
#include "viam/lib/servo_controller.hpp"

namespace ethercat::servo {

class A6ServoDriver final : public ServoController {
   public:
    // Inherit the generic constructors verbatim -- the A6 adds no config, only behavior.
    using ServoController::ServoController;

   protected:
    std::optional<ethercat::SdoWrite> vendor_fault_reset_sdo() const override {
        // A6 vendor fault reset: write u16 1 to 0x2031:01 (little-endian), NOT CiA402 bit7.
        return ethercat::SdoWrite{kFaultResetIndex, kFaultResetSub, {std::byte{0x01}, std::byte{0x00}}};
    }
    std::optional<std::uint16_t> sync_fault_code() const noexcept override {
        return kNoSyncCode;  // Er74.1 "no SYNC0"
    }
    std::string fault_description(std::uint16_t code) const override {
        return code == kNoSyncCode ? std::string{"Er74.1 / no SYNC0"} : std::string{};
    }

   private:
    static constexpr std::uint16_t kFaultResetIndex = 0x2031;
    static constexpr std::uint8_t kFaultResetSub = 0x01;
    static constexpr std::uint16_t kNoSyncCode = 0x8700;
};

}  // namespace ethercat::servo
