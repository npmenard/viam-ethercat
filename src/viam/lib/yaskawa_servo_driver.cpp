#include "viam/lib/yaskawa_servo_driver.hpp"

#include <array>
#include <chrono>
#include <cstddef>
#include <thread>

#include "ethercat/errors.hpp"
#include "ethercat/pdo_buffer.hpp"  // load_le / store_le

namespace ethercat::servo {
namespace {

// One manufacturer user-unit ratio: 1 reference unit = numerator/denominator of the drive's own
// unit for that quantity (2701h position, 2702h velocity, 2703h acceleration, 2704h torque).
struct UserUnit {
    std::uint16_t index;
    std::uint32_t numerator;
    std::uint32_t denominator;
};

constexpr std::uint8_t kNumeratorSub = 1;
constexpr std::uint8_t kDenominatorSub = 2;

// The drive's own defaults, asserted not assumed: a count is 64 increments (2701h/2702h), one
// acceleration unit is 10^4 counts/s^2 (2703h), torque is per-mille of rated (2704h).
constexpr std::array<UserUnit, 4> kUserUnits{{
    {0x2701, 64, 1},
    {0x2702, 64, 1},
    {0x2703, 64, 1},
    {0x2704, 1, 10},
}};

// What the bench cannot check, the compiler can. The drive refuses a ratio outside 1/256..1048576
// with an A20h at bring-up, 0x607A and 0x60FF must measure in the same count, and the acceleration
// scale follows from the acceleration and position ratios together.
constexpr double ratio(std::size_t i) {
    return static_cast<double>(kUserUnits[i].numerator) / static_cast<double>(kUserUnits[i].denominator);
}
static_assert(ratio(0) == ratio(1), "0x607A and 0x60FF must measure in the same count");
static_assert(ratio(3) == 0.1, "torque is per-mille of rated, as the CiA402 torque objects define it");
static_assert(ratio(2) * 1e4 / ratio(0) == kAccelUnitCps2, "kAccelUnitCps2 must follow from 2703h and 2701h");
constexpr bool in_range(std::size_t i) {
    return ratio(i) >= 1.0 / 256.0 && ratio(i) <= 1048576.0;  // outside, the drive raises A20h at bring-up
}
static_assert(in_range(0) && in_range(1) && in_range(2) && in_range(3));

// Writing 2701h-2704h only stores them; 2700h = 1 makes the new set live in-session (the drive
// clears it when the set is enabled).
constexpr std::uint16_t kUserParamConfig = 0x2700;
constexpr auto kConfigPoll = std::chrono::milliseconds(10);
constexpr int kConfigPolls = 200;  // 2 s; enabling a parameter set is a sub-second operation

// Set one subindex, skipping the write when the drive already holds the value: these are
// EEPROM-backed, so rewriting a correct ratio on every bring-up wears the drive for nothing.
void correct(ConfigContext& cfg, std::uint16_t index, std::uint8_t sub, std::uint32_t want) {
    std::array<std::byte, 4> buf{};
    cfg.sdo_read(index, sub, buf);
    if (load_le<std::uint32_t>(buf) == want) {
        return;
    }
    store_le<std::uint32_t>(buf, want);
    cfg.sdo_write(index, sub, buf);
}

}  // namespace

void YaskawaServoDriver::configure_device(ConfigContext& cfg) {
    for (const UserUnit& u : kUserUnits) {
        correct(cfg, u.index, kNumeratorSub, u.numerator);
        correct(cfg, u.index, kDenominatorSub, u.denominator);
    }
    // Unconditional: the stored ratios read back correct even when the latched ones are stale, so
    // this cannot be gated on a change. Enabling also re-initialises the drive's position data.
    std::array<std::byte, 4> buf{};
    store_le<std::uint32_t>(buf, 1);
    cfg.sdo_write(kUserParamConfig, 0, buf);
    for (int i = 0; i < kConfigPolls; ++i) {
        std::this_thread::sleep_for(kConfigPoll);
        cfg.sdo_read(kUserParamConfig, 0, buf);
        if (load_le<std::uint32_t>(buf) == 0) {
            return;  // the drive cleared it: the ratios above are the units it now serves
        }
    }
    throw Error(
        "the drive did not clear 2700h (user parameter configuration) -- the reference units in "
        "2701h-2704h were stored but not enabled, so every position, velocity and acceleration "
        "command would be scaled by the units it latched at power-on");
}

std::string YaskawaServoDriver::fault_description(std::uint16_t code) const {
    // Only codes seen on a real drive; the base returns "" (bare hex) for anything else.
    if (code == 0x0A12) {
        return "A.12 / EtherCAT output data synchronization error -- the drive missed SYNC0-aligned "
               "process data (see 1C32h:11 for the miss count)";
    }
    return {};
}

}  // namespace ethercat::servo
