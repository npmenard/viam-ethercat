// The 0x60FD safety interlock, modelled on the machine it was written for: the two overtravel limit
// switches read HIGH when the carriage hits an end, and the safety relay's early contact reads HIGH
// while it is safe (wired normally-closed, so a broken wire reads unsafe). Both polarities appear
// here on purpose -- getting one backwards is the failure this code exists to prevent, and it cannot
// be caught by inspection.

#include <cstdint>

#include "ethercat/errors.hpp"
#include "test_harness.hpp"
#include "viam/lib/yaskawa_servo_driver.hpp"

using ethercat::Error;
using ethercat::servo::safety_engaged;
using ethercat::servo::SafetyInputs;

namespace {

constexpr std::uint8_t kLimitNeg = 24;      // CN1 overtravel, reads 1 when hit
constexpr std::uint8_t kLimitPos = 25;      // CN1 overtravel, reads 1 when hit
constexpr std::uint8_t kEarlyContact = 19;  // SI3 from the safety relay, reads 1 while safe

constexpr std::uint32_t bit(std::uint8_t b) {
    return std::uint32_t{1} << b;
}

// The machine's declaration: limits stop outright, the early contact asks for a ramp.
SafetyInputs machine() {
    SafetyInputs in;
    in.stop.require_low = {kLimitNeg, kLimitPos};
    in.quick_stop.require_high = {kEarlyContact};
    return in;
}

// Every declared input reading its safe level: limits clear, early contact closed.
constexpr std::uint32_t kAllSafe = bit(kEarlyContact);

}  // namespace

TEST("an undeclared interlock never engages, whatever the drive reports") {
    const SafetyInputs none;
    CHECK(!none.declared());
    CHECK(!safety_engaged(none.stop, 0x00000000));
    CHECK(!safety_engaged(none.stop, 0xFFFFFFFF));
    CHECK(!safety_engaged(none.quick_stop, 0xFFFFFFFF));
}

TEST("each tier engages on its own input, and only its own") {
    const SafetyInputs in = machine();
    CHECK(!safety_engaged(in.stop, kAllSafe));
    CHECK(!safety_engaged(in.quick_stop, kAllSafe));

    // A limit switch hit: the stop tier engages; the early contact is still closed, so quick-stop does not.
    CHECK(safety_engaged(in.stop, kAllSafe | bit(kLimitPos)));
    CHECK(!safety_engaged(in.quick_stop, kAllSafe | bit(kLimitPos)));

    // The relay's early contact opens: quick-stop engages while the limits are still clear.
    CHECK(safety_engaged(in.quick_stop, 0));
    CHECK(!safety_engaged(in.stop, 0));
}

TEST("a bit required both high and low is refused at config time") {
    // Unsatisfiable, so it would hold the drive down forever; better to fail the config.
    SafetyInputs in;
    in.stop.require_high = {kEarlyContact};
    in.stop.require_low = {kEarlyContact};
    CHECK_THROWS_MSG(in.validate(), Error, "both high and low");

    SafetyInputs oob;
    oob.quick_stop.require_high = {32};  // 0x60FD is 32 bits wide
    CHECK_THROWS_MSG(oob.validate(), Error, "outside");
}

TEST_MAIN()
