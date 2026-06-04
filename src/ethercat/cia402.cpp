#include "ethercat/cia402.hpp"

namespace ethercat {

const char* to_string(Cia402State state) noexcept {
    switch (state) {
        case Cia402State::NotReadyToSwitchOn:
            return "NotReadyToSwitchOn";
        case Cia402State::SwitchOnDisabled:
            return "SwitchOnDisabled";
        case Cia402State::ReadyToSwitchOn:
            return "ReadyToSwitchOn";
        case Cia402State::SwitchedOn:
            return "SwitchedOn";
        case Cia402State::OperationEnabled:
            return "OperationEnabled";
        case Cia402State::QuickStopActive:
            return "QuickStopActive";
        case Cia402State::FaultReactionActive:
            return "FaultReactionActive";
        case Cia402State::Fault:
            return "Fault";
    }
    return "Unknown";
}

const char* to_string(Cia402Mode mode) noexcept {
    switch (mode) {
        case Cia402Mode::None:
            return "None";
        case Cia402Mode::ProfilePosition:
            return "ProfilePosition";
        case Cia402Mode::ProfileVelocity:
            return "ProfileVelocity";
        case Cia402Mode::CyclicSyncPosition:
            return "CyclicSyncPosition";
    }
    return "Unknown";
}

Cia402State Status::decode() const noexcept {
    // Standard DS402 decode. Masks 0x4F (bits 0..3,6) and 0x6F (bits 0..3,5,6)
    // isolate the state-defining bits; the patterns below are mutually
    // exclusive, so order only matters for ill-formed words (handled by the
    // defensive fall-through to Fault).
    const unsigned s = raw;
    if ((s & 0x4FU) == 0x00U) {
        return Cia402State::NotReadyToSwitchOn;
    }
    if ((s & 0x4FU) == 0x40U) {
        return Cia402State::SwitchOnDisabled;
    }
    if ((s & 0x6FU) == 0x21U) {
        return Cia402State::ReadyToSwitchOn;
    }
    if ((s & 0x6FU) == 0x23U) {
        return Cia402State::SwitchedOn;
    }
    if ((s & 0x6FU) == 0x27U) {
        return Cia402State::OperationEnabled;
    }
    if ((s & 0x6FU) == 0x07U) {
        return Cia402State::QuickStopActive;
    }
    if ((s & 0x4FU) == 0x0FU) {
        return Cia402State::FaultReactionActive;
    }
    if ((s & 0x4FU) == 0x08U) {
        return Cia402State::Fault;
    }
    // Unrecognized statusword: treat as Fault (never report operational).
    return Cia402State::Fault;
}

// step() is intentionally a (const) member rather than static: Cia402Fsm is the
// extensible policy seam, and future versions may carry per-drive config (e.g.
// quick-stop option codes) that step() consults. Keep it instance-callable.
// NOLINTNEXTLINE(readability-convert-member-functions-to-static)
std::uint16_t Cia402Fsm::step(Status current, Cia402State goal) const noexcept {
    const Cia402State state = current.decode();

    // Fault handling takes priority over the goal. In Fault, request a reset
    // LEVEL (bit7) -- the driver turns this into the rising edge. While the
    // drive runs its own fault reaction, we can only wait it out with voltage
    // disabled until it settles into Fault.
    if (state == Cia402State::Fault) {
        return ControlWord::fault_reset();
    }
    if (state == Cia402State::FaultReactionActive) {
        return ControlWord::disable_voltage();
    }

    switch (goal) {
        case Cia402State::SwitchOnDisabled:
            // Full power-off is reachable from any active state in one step.
            return ControlWord::disable_voltage();

        case Cia402State::ReadyToSwitchOn:
            return ControlWord::shutdown();

        case Cia402State::QuickStopActive:
            return ControlWord::quick_stop();

        case Cia402State::SwitchedOn:
            if (state == Cia402State::SwitchOnDisabled) {
                return ControlWord::shutdown();  // climb one rung toward SwitchedOn
            }
            return ControlWord::switch_on();  // from ReadyToSwitchOn up, or drop from OperationEnabled

        case Cia402State::NotReadyToSwitchOn:
        case Cia402State::OperationEnabled:
        case Cia402State::FaultReactionActive:
        case Cia402State::Fault:
        default:
            // Goal is OperationEnabled (the common case) or anything else: climb
            // the standard enable ladder one transition per call.
            switch (state) {
                case Cia402State::SwitchOnDisabled:
                    return ControlWord::shutdown();  // -> ReadyToSwitchOn
                case Cia402State::ReadyToSwitchOn:
                    return ControlWord::switch_on();  // -> SwitchedOn
                // SwitchedOn advances to OperationEnabled; OperationEnabled holds; and
                // QuickStopActive resumes (DS402 transition 16) -- all the same level.
                case Cia402State::SwitchedOn:
                case Cia402State::OperationEnabled:
                case Cia402State::QuickStopActive:
                    return ControlWord::enable_operation();
                case Cia402State::NotReadyToSwitchOn:
                case Cia402State::FaultReactionActive:
                case Cia402State::Fault:
                default:
                    return ControlWord::disable_voltage();
            }
    }
}

}  // namespace ethercat
