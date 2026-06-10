#pragma once

#include <cstdint>

#include "ethercat/expected.hpp"

namespace ethercat {

// CiA402 "modes of operation" (object 0x6060 / display 0x6061). Values match the
// DS402 wire encoding. The enable ladder (0x06->0x07->0x0F via Cia402Fsm::step) is
// MODE-AGNOSTIC -- it only advances the DS402 state machine, so every mode reaches
// OperationEnabled the same way; only post-enable streaming differs (PP = bit4
// new-set-point handshake; CSP = stream 0x607A every cycle with cw held at 0x0F).
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

    // bit10 "target reached". A6-EC QUIRK: this drive ties bit10 permanently
    // high, so it is USELESS for move-completion there -- use position deviation
    // (following_error / bit13) or actual-vs-target instead. Exposed only for
    // completeness on conformant drives.
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

// Errors returned by the goal-walking FSM's INTENT calls (#38 §9). update()/get_cw()
// never error. Returned via Expected (the C++20 std::expected stand-in, expected.hpp) at the intent call (immediate).
enum class FsmError : std::uint8_t {
    FaultActive,     // goal refused: a fault is present -- reset + re-issue the goal
    InvalidGoal,     // QSA/Fault/NotReady/FRA are not set_state goals (quick_stop is an intent)
    WrongMode,       // set_point outside ProfilePosition
    NotOperational,  // set_point outside OperationEnabled
    Busy,            // a bit-4 handshake is already in flight
};
const char* to_string(FsmError e) noexcept;

// Phases of the PP bit-4 new-set-point handshake (#38 §7), advanced in update(),
// observable via setpoint_phase(). bit4 (+bit6 if relative) is emitted during
// Arm/AwaitAck; cleared from Drop on.
enum class SetpointPhase : std::uint8_t { Idle, Arm, AwaitAck, Drop, AwaitAckClear };

// --- #38 pure routing core (unit-tested exhaustively; the shell composes over it) ---

// Per-state SUSTAIN controlword (#38 §3): the deterministic level that HOLDS the given
// state (emitted for HOLD-at-goal and WAIT-for-auto-transition -- history-independent).
// B1 (DA-corrected, bench-proven): SwitchOnDisabled sustains with 0x00, NOT 0x06 --
// 0x06 IS the T2 trigger from SOD (a6_validate observed "0x06 -> ReadyToSwitchOn"), so
// a 0x06 hold would ping-pong SOD<->RTSO at loop rate. 0x00 is genuinely inert in SOD.
constexpr std::uint16_t sustain_cw(Cia402State s) noexcept {
    switch (s) {
        case Cia402State::NotReadyToSwitchOn:
            return 0x0000;  // inert pre-init
        case Cia402State::SwitchOnDisabled:
            return 0x0000;  // B1: 0x06 would FIRE T2
        case Cia402State::ReadyToSwitchOn:
            return 0x0006;  // level that holds RTSO
        case Cia402State::SwitchedOn:
            return 0x0007;  // level that holds SwitchedOn
        case Cia402State::OperationEnabled:
            return 0x000F;  // sustains energization
        case Cia402State::QuickStopActive:
            return 0x0002;  // sustains the quick stop (A6 default auto-T12s out regardless)
        case Cia402State::FaultReactionActive:
        case Cia402State::Fault:
            return 0x0006;  // inert during/after the fault path; lands SOD-stable after T15
    }
    return 0x0000;
}

// The #38 §3 routing table: ONE legal DS402 transition from `current` toward `goal`,
// re-planned fresh every cycle from the CURRENT decoded state (§4 -- the walker stores
// only the goal, never a path, so drive auto-transitions T1/T12/T13/T14 are absorbed,
// not fought). WAIT cells emit the CURRENT state's sustain cw; HOLD cells the goal's.
// Returns FsmError::InvalidGoal for the non-goal states (QSA/Fault/NotReady/FRA --
// quick_stop is a transition INTENT, not a goal) and FsmError::FaultActive where the
// table refuses (Fault row; FRA x OperationEnabled).
// T16 (QSA->OE, 0x0F) is present per the CiA402 standard (valid for drives/configs
// with 0x605A in 5..7); on the A6 DEFAULT (0x605A=2) the drive auto-T12s out of QSA
// before it can apply -- the re-planner simply routes onward from SOD, so the
// unreachable edge is a non-event (no special-casing).
// QSA->RTSO/SwitchedOn route 0x00 (T12 -- land SOD, re-walk), NOT WAIT: WAIT would
// emit QSA's sustain 0x02 which actively SUSTAINS the stop, making those goals
// unreachable on a holding drive. Any goal present during QSA was issued AFTER
// quick_stop() cancelled the previous one = a fresh deliberate override -- honor it.
constexpr Expected<std::uint16_t, FsmError> next_cw(Cia402State current, Cia402State goal) noexcept {
    switch (goal) {
        case Cia402State::SwitchOnDisabled:
        case Cia402State::ReadyToSwitchOn:
        case Cia402State::SwitchedOn:
        case Cia402State::OperationEnabled:
            break;  // the four valid goals
        case Cia402State::NotReadyToSwitchOn:
        case Cia402State::QuickStopActive:
        case Cia402State::FaultReactionActive:
        case Cia402State::Fault:
            return Unexpected(FsmError::InvalidGoal);
    }
    switch (current) {
        case Cia402State::NotReadyToSwitchOn:
            return sustain_cw(current);  // WAIT: T1 is auto
        case Cia402State::SwitchOnDisabled:
            if (goal == Cia402State::SwitchOnDisabled) {
                return sustain_cw(goal);  // HOLD 0x00 (B1)
            }
            return std::uint16_t{0x0006};  // T2 toward everything above
        case Cia402State::ReadyToSwitchOn:
            switch (goal) {
                case Cia402State::SwitchOnDisabled:
                    return std::uint16_t{0x0000};  // T7
                case Cia402State::ReadyToSwitchOn:
                    return sustain_cw(goal);  // HOLD 0x06
                default:
                    return std::uint16_t{0x0007};  // T3 toward SwitchedOn/OE
            }
        case Cia402State::SwitchedOn:
            switch (goal) {
                case Cia402State::SwitchOnDisabled:
                    return std::uint16_t{0x0000};  // T10
                case Cia402State::ReadyToSwitchOn:
                    return std::uint16_t{0x0006};  // T6
                case Cia402State::SwitchedOn:
                    return sustain_cw(goal);  // HOLD 0x07
                default:
                    return std::uint16_t{0x000F};  // T4 -- ENERGIZES
            }
        case Cia402State::OperationEnabled:
            switch (goal) {
                case Cia402State::SwitchOnDisabled:
                    return std::uint16_t{0x0000};  // T9
                case Cia402State::ReadyToSwitchOn:
                    return std::uint16_t{0x0006};  // T8
                case Cia402State::SwitchedOn:
                    return std::uint16_t{0x0007};  // T5
                default:
                    return sustain_cw(goal);  // HOLD 0x0F
            }
        case Cia402State::QuickStopActive:
            if (goal == Cia402State::OperationEnabled) {
                return std::uint16_t{0x000F};  // T16 (where 0x605A holds QSA; see note above)
            }
            return std::uint16_t{0x0000};  // T12: land SOD, re-walk (incl. goal RTSO/SO -- the override)
        case Cia402State::FaultReactionActive:
            if (goal == Cia402State::OperationEnabled) {
                return Unexpected(FsmError::FaultActive);
            }
            return sustain_cw(current);  // WAIT: T14 is auto; 0x06 is inert here
        case Cia402State::Fault:
            return Unexpected(FsmError::FaultActive);  // reset is the shell's job (§5)
    }
    return Unexpected(FsmError::FaultActive);
}

// Goal-walking CiA402 FSM (#38): statusword in -> intents -> controlword out.
//
// Pure core + stateful shell: the routing above is the exhaustively-tested core;
// this class owns goal / fault-reset edge / bit-4 handshake phase / halt level /
// mode and COMPOSES get_cw() from them (§8 order).
//
// CYCLE CONTRACT (§1): call update(sw) exactly ONCE per cycle, BEFORE any get_cw()
// for that cycle. All edge machinery advances in update() only; get_cw() is a pure
// read (idempotent within a cycle). Intents (set_state/set_point/halt/quick_stop/
// fault.reset) may be called between updates and take effect at the next get_cw().
//
// INITIAL GOAL = ADOPT-CURRENT (D3): the FIRST update() adopts the decoded state as
// the goal. Consequence: constructing the FSM while the drive is already
// OperationEnabled SUSTAINS 0x0F -- energization is maintained, not initiated (an
// FSM swap-in must not de-energize a running axis). A non-goal first state
// (QSA/Fault/FRA/NotReady) adopts HOLD-current (sustain) until a set_state arrives.
//
// FAULT (§5): decoding Fault/FaultReactionActive CANCELS the active goal -- a reset
// must NOT auto-re-energize; the caller re-issues enable() deliberately. fault.reset()
// arms ONE update-cycle of clean cw=0x80 (the bench-proven #18 form; the A6 masks all
// other refs while bit7=1 anyway); the next cycle returns to base with bit7=0 -- that
// FALLING edge re-arms the drive's rising-edge detector. Retry/give-up = caller policy.
//
// RT-resident: every method noexcept, no allocation, no exceptions (Expected, the std::expected stand-in).
// No timeouts inside (caller policy via cycles_in_state() + remote() + setpoint_phase()).
// Never touches 0x6060 (set_mode is intent-validation only). No vendor-fault knowledge
// (Er74-class clears are consumer policy, #39/#22): after an ineffective reset pulse,
// fault.active() stays true and cycles_in_state() grows -- the caller's bounded retry
// sees it honestly. No port I/O, no Master dependency -- pure sw-in/cw-out.
class Cia402Fsm {
   public:
    Cia402Fsm() noexcept = default;
    // Copy/move re-seat the fault interface's back-pointer (see FaultControl).
    Cia402Fsm(const Cia402Fsm& o) noexcept : st_(o.st_) {}
    Cia402Fsm& operator=(const Cia402Fsm& o) noexcept {
        st_ = o.st_;
        return *this;
    }
    Cia402Fsm(Cia402Fsm&& o) noexcept : st_(o.st_) {}
    Cia402Fsm& operator=(Cia402Fsm&& o) noexcept {
        st_ = o.st_;
        return *this;
    }
    ~Cia402Fsm() = default;

    // THE cycle boundary: decode + adopt/cancel goal + advance edges. Once per cycle.
    void update(std::uint16_t statusword) noexcept {
        st_.last_sw = statusword;
        const Cia402State s = Status{statusword}.decode();
        st_.reset_pulse = false;  // the one-cycle 0x80 pulse ends here (falling edge this cycle)
        if (!st_.adopted) {
            st_.adopted = true;
            st_.has_goal = is_goal(s);  // D3 adopt-current (a non-goal first state -> hold-current)
            st_.goal = st_.has_goal ? s : Cia402State::SwitchOnDisabled;
            st_.cycles_in_state = 1;
        } else {
            st_.cycles_in_state = (s == st_.state) ? st_.cycles_in_state + 1 : 1;
        }
        if (s == Cia402State::Fault || s == Cia402State::FaultReactionActive) {
            st_.has_goal = false;  // §5: fault cancels the goal -- no implicit re-energize
        }
        // B3 (#38 gate): an UNCOMMANDED QuickStopActive ENTRY also cancels the goal. A
        // drive can enter QSA externally (an estop input wired to its quick-stop
        // function); without this, goal=OE would survive the stop, re-walk T2/T3/T4 from
        // the auto-T12 SOD landing, and RE-ENERGIZE the instant the estop releases -- the
        // exact §5 hazard class (no re-energize without a fresh caller decision).
        // ENTRY-EDGE form: our own quick_stop() already cancelled at intent time (this is
        // then a no-op), and a goal issued DURING QSA (set_state while decoded QSA) stays
        // honored -- it is a fresh, deliberate post-stop caller decision (the T12-override).
        if (s == Cia402State::QuickStopActive && st_.state != Cia402State::QuickStopActive) {
            st_.has_goal = false;
        }
        if (s != Cia402State::OperationEnabled) {
            st_.sp_phase = SetpointPhase::Idle;  // B2: any exit from OE aborts the handshake
            st_.qs_intent = false;               // quick-stop intent consumed-or-dropped (minor-b)
        } else {
            switch (st_.sp_phase) {
                case SetpointPhase::Arm:
                    st_.sp_phase = SetpointPhase::AwaitAck;  // bit4 went out during the Arm cycle
                    break;
                case SetpointPhase::AwaitAck:
                    if (Status{statusword}.setpoint_acknowledged()) {
                        st_.sp_phase = SetpointPhase::Drop;
                    }
                    break;
                case SetpointPhase::Drop:
                    st_.sp_phase = SetpointPhase::AwaitAckClear;  // bit4 cleared from the Drop cycle
                    break;
                case SetpointPhase::AwaitAckClear:
                    if (!Status{statusword}.setpoint_acknowledged()) {
                        st_.sp_phase = SetpointPhase::Idle;
                    }
                    break;
                case SetpointPhase::Idle:
                    break;
            }
        }
        st_.state = s;
    }

    Cia402State state() const noexcept {
        return st_.state;
    }
    bool operation_enabled() const noexcept {
        return st_.state == Cia402State::OperationEnabled;
    }
    // sw bit9: the drive honors controlword writes. bit9==0 => cw is being IGNORED
    // (drive local/offline) -- the state won't advance and cycles_in_state() grows;
    // this lets the caller's timeout diagnostic say WHY (§6). No FSM-internal timeout.
    bool remote() const noexcept {
        return (st_.last_sw & 0x0200U) != 0;
    }
    std::uint32_t cycles_in_state() const noexcept {
        return st_.cycles_in_state;
    }
    SetpointPhase setpoint_phase() const noexcept {
        return st_.sp_phase;
    }

    // Goal-driven walk toward one of the four stable goals. QSA/Fault/NotReady/FRA
    // are not goals (InvalidGoal -- quick_stop is an intent). Refused while a fault
    // is present (FaultActive): reset + re-issue once the drive clears (§5).
    Expected<void, FsmError> set_state(Cia402State goal) noexcept {
        if (!is_goal(goal)) {
            return Unexpected(FsmError::InvalidGoal);
        }
        if (st_.state == Cia402State::Fault || st_.state == Cia402State::FaultReactionActive) {
            return Unexpected(FsmError::FaultActive);
        }
        st_.has_goal = true;
        st_.goal = goal;
        return {};
    }
    Expected<void, FsmError> enable() noexcept {
        return set_state(Cia402State::OperationEnabled);
    }
    Expected<void, FsmError> disable() noexcept {
        return set_state(Cia402State::SwitchOnDisabled);
    }

    // PP bit-4 new-set-point handshake (§7). bit6 (relative) rides with it if asked.
    Expected<void, FsmError> set_point(bool relative = false) noexcept {
        if (st_.mode != Cia402Mode::ProfilePosition) {
            return Unexpected(FsmError::WrongMode);
        }
        if (st_.state != Cia402State::OperationEnabled) {
            return Unexpected(FsmError::NotOperational);
        }
        if (st_.sp_phase != SetpointPhase::Idle) {
            return Unexpected(FsmError::Busy);
        }
        st_.sp_phase = SetpointPhase::Arm;
        st_.sp_relative = relative;
        return {};
    }

    // Sticky Halt level (bit8); composes in all modes (standard). Suppressed only
    // while a quick-stop override or a reset pulse is emitting.
    void halt(bool h) noexcept {
        st_.halt = h;
    }

    // Quick-stop INTENT (not a goal): while the drive is OperationEnabled, force
    // cw=0x02 and CANCEL the active goal (it's an abort, not a detour). Outside OE
    // this is a NO-OP (minor-b: a stale stop firing minutes later on a fresh enable
    // would be a hazard); the intent is consumed when the drive leaves OE.
    void quick_stop() noexcept {
        if (st_.state == Cia402State::OperationEnabled) {
            st_.qs_intent = true;
            st_.has_goal = false;  // an abort, not a detour
        }
    }

    // Intent VALIDATION only (gates set_point); never writes 0x6060 (configure-time,
    // library-side).
    void set_mode(Cia402Mode m) noexcept {
        st_.mode = m;
    }

    // Compose this cycle's controlword (§8 order). Pure/idempotent within a cycle.
    std::uint16_t get_cw() const noexcept {
        if (st_.reset_pulse) {
            return ControlWord::kFaultResetBit;  // clean 0x80 (#18; the A6 masks other refs anyway)
        }
        if (st_.qs_intent && st_.state == Cia402State::OperationEnabled) {
            return 0x0002;  // quick stop IS the priority; halt/bit4 suppressed
        }
        std::uint16_t cw = base_cw();
        if (st_.mode == Cia402Mode::ProfilePosition && st_.state == Cia402State::OperationEnabled &&
            (st_.sp_phase == SetpointPhase::Arm || st_.sp_phase == SetpointPhase::AwaitAck)) {
            cw |= ControlWord::kNewSetpointBit;
            if (st_.sp_relative) {
                cw |= ControlWord::kRelativeBit;
            }
        }
        if (st_.halt) {
            cw |= ControlWord::kHaltBit;
        }
        return cw;
    }

    // FSM-owned fault interface (the locked API's `fault` member): active() reflects
    // the last update; reset() arms ONE cycle of clean 0x80 (no-op if no fault).
    class FaultControl {
       public:
        bool active() const noexcept {
            return self_->st_.state == Cia402State::Fault || self_->st_.state == Cia402State::FaultReactionActive;
        }
        void reset() noexcept {
            if (active()) {
                self_->st_.reset_pulse = true;  // one pulse per reset() call
            }
        }

       private:
        friend class Cia402Fsm;
        explicit FaultControl(Cia402Fsm* self) noexcept : self_(self) {}
        Cia402Fsm* self_;
    };
    FaultControl fault{this};

    // LEGACY (pre-#38) pure one-shot helper -- consumers migrate onto the goal API in
    // #41, then this goes. Stateless/const; does not interact with the shell state.
    std::uint16_t step(Status current, Cia402State goal) const noexcept;

   private:
    static constexpr bool is_goal(Cia402State s) noexcept {
        return s == Cia402State::SwitchOnDisabled || s == Cia402State::ReadyToSwitchOn || s == Cia402State::SwitchedOn ||
               s == Cia402State::OperationEnabled;
    }
    std::uint16_t base_cw() const noexcept {
        if (!st_.has_goal) {
            return sustain_cw(st_.state);  // no/cancelled goal = HOLD-current (genuinely inert at SOD, B1)
        }
        const auto r = next_cw(st_.state, st_.goal);
        // The shell never carries a goal through fault states (update() cancels it),
        // so routing errors are unreachable here -- fall back to the inert sustain.
        return r ? *r : sustain_cw(st_.state);
    }

    // All mutable state in one plain struct so copy/move re-seat `fault` trivially.
    struct State {
        Cia402State state = Cia402State::NotReadyToSwitchOn;
        bool has_goal = false;                             // false = no active walk (hold-current)
        Cia402State goal = Cia402State::SwitchOnDisabled;  // valid only while has_goal (always initialized -- POD, no optional)
        std::uint16_t last_sw = 0;
        std::uint32_t cycles_in_state = 0;
        Cia402Mode mode = Cia402Mode::None;
        SetpointPhase sp_phase = SetpointPhase::Idle;
        bool sp_relative = false;
        bool halt = false;
        bool qs_intent = false;
        bool reset_pulse = false;
        bool adopted = false;
    };
    State st_;
};

}  // namespace ethercat
