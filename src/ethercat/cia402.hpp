#pragma once

#include <cstdint>

namespace ethercat {

// CiA402 modes of operation (object 0x6060, display 0x6061). Values match the DS402
// wire encoding. The enable ladder (0x06 -> 0x07 -> 0x0F via Cia402Fsm::step) is
// mode-agnostic: it only advances the DS402 state machine, so every mode reaches
// OperationEnabled the same way. Only post-enable streaming differs (PP uses the bit-4
// new-set-point handshake; CSP streams 0x607A every cycle with the controlword held at 0x0F).
enum class Cia402Mode : std::int8_t {
    None = 0,
    ProfilePosition = 1,
    ProfileVelocity = 3,
    CyclicSyncPosition = 8,  // CSP: master streams target position (0x607A) every cycle
};

// The eight canonical CiA402 drive states (the DS402 state machine).
enum class Cia402State : std::uint8_t {
    NotReadyToSwitchOn,
    SwitchOnDisabled,
    ReadyToSwitchOn,
    SwitchedOn,
    OperationEnabled,
    QuickStopActive,
    FaultReactionActive,
    Fault,
};

// Human-readable name for logs/errors. Defined in cia402.cpp.
const char* to_string(Cia402State state) noexcept;
const char* to_string(Cia402Mode mode) noexcept;

// Decoded view of the CiA402 statusword (object 0x6041). `raw` is the 16-bit
// word read from the TxPDO; the accessors interpret it. Decoding the drive
// state uses the standard DS402 mask table (see decode()).
struct Status {
    std::uint16_t raw = 0;

    // --- Standard statusword bits ---
    bool ready_to_switch_on() const noexcept {
        return bit(0);
    }
    bool switched_on() const noexcept {
        return bit(1);
    }
    bool operation_enabled() const noexcept {
        return bit(2);
    }
    bool fault() const noexcept {
        return bit(3);
    }
    bool voltage_enabled() const noexcept {
        return bit(4);
    }
    bool quick_stop() const noexcept {
        return bit(5);
    }
    bool switch_on_disabled() const noexcept {
        return bit(6);
    }
    bool warning() const noexcept {
        return bit(7);
    }
    bool remote() const noexcept {
        return bit(9);
    }
    bool internal_limit_active() const noexcept {
        return bit(11);
    }

    // bit 10 "target reached". Some drives tie bit 10 permanently high and cannot
    // signal move-completion there;
    bool target_reached() const noexcept {
        return bit(10);
    }

    // Profile-Position handshake bits.
    bool setpoint_acknowledged() const noexcept {
        return bit(12);
    }  // drive latched a new target
    bool following_error() const noexcept {
        return bit(13);
    }  // position deviation

    // Decode the drive state from `raw` per the DS402 mask table.
    Cia402State decode() const noexcept;

   private:
    bool bit(unsigned n) const noexcept {
        return (raw & static_cast<std::uint16_t>(1U << n)) != 0;
    }
};

// Controlword (object 0x6040) encoders. Each returns an absolute controlword
// LEVEL to write -- NOT a delta. Edge-triggered semantics (fault-reset bit7,
// new-set-point bit4) are produced by the driver toggling these levels across
// cycles; this type only encodes the levels.
struct ControlWord {
    static constexpr std::uint16_t kNewSetpointBit = 0x0010;  // bit4 (PP)
    static constexpr std::uint16_t kRelativeBit = 0x0040;     // bit6 (PP: relative target)
    static constexpr std::uint16_t kFaultResetBit = 0x0080;   // bit7
    static constexpr std::uint16_t kHaltBit = 0x0100;         // bit8

    static constexpr std::uint16_t shutdown() noexcept {
        return 0x0006;
    }
    static constexpr std::uint16_t switch_on() noexcept {
        return 0x0007;
    }
    static constexpr std::uint16_t enable_operation() noexcept {
        return 0x000F;
    }
    static constexpr std::uint16_t disable_voltage() noexcept {
        return 0x0000;
    }
    static constexpr std::uint16_t quick_stop() noexcept {
        return 0x0002;
    }

    // Fault reset as a LEVEL (bit7 set). The rising edge that actually clears
    // the fault is the driver's responsibility (clear bit7 one cycle, set it
    // the next); step() returns this level while in Fault.
    static constexpr std::uint16_t fault_reset() noexcept {
        return kFaultResetBit;
    }

    // Set/clear the Halt bit (bit8) on a base controlword. Stop semantics in the
    // module map to asserting Halt.
    static constexpr std::uint16_t with_halt(std::uint16_t base, bool halt) noexcept {
        return halt ? static_cast<std::uint16_t>(base | kHaltBit) : static_cast<std::uint16_t>(base & ~kHaltBit);
    }

    // PP mode: set/clear the new-set-point bit (bit4) on a base controlword.
    // with_new_setpoint(enable_operation(), true) == 0x001F (the 0x0F->0x1F edge).
    static constexpr std::uint16_t with_new_setpoint(std::uint16_t base, bool new_setpoint) noexcept {
        return new_setpoint ? static_cast<std::uint16_t>(base | kNewSetpointBit) : static_cast<std::uint16_t>(base & ~kNewSetpointBit);
    }
};

// Stateless goal-seeking CiA402 helper. Maps statusword plus goal to one controlword toward the
// goal, re-planned fresh each cycle from the current decoded state (drive auto-transitions are
// absorbed, not fought). The caller loops step() per RT cycle toward OperationEnabled; a fault
// yields a reset level (bit 7) the caller edges. Pure statusword-in, controlword-out: noexcept,
// no stored state, no I/O.
class Cia402Fsm {
   public:
    // step() is a const member rather than static so Cia402Fsm stays an extensible policy seam: a
    // future version may carry per-drive config (e.g. quick-stop option codes) that step() consults.
    // NOLINTNEXTLINE(readability-convert-member-functions-to-static)
    std::uint16_t step(Status current, Cia402State goal) const noexcept;
};

}  // namespace ethercat
