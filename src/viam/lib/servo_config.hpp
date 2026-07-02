#pragma once

// Validated configuration for one servo drive. Pure data + a validate() that
// throws ethercat::ConfigError with clear text. No SDK, no hardware -- the
// module parses the Viam attributes into this struct and validates before any
// hardware init (deferred-init pattern). The A6 PDO map lives here as CONFIG
// DATA (never hardcoded in generic code).

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "ethercat/pdo_mapping.hpp"

namespace ethercat::servo {

// #68: a do_command converted-SDO read target. Describes ONE CoE object read while operating
// (via the #22 marshaled steady-state SDO) plus how to convert its raw integer to a reported
// double. CONFIG DATA, never a hardcoded vendor constant: the DEFAULTS are the standard CiA402
// objects (a generic drive works with zero config), and a vendor drive overrides index/
// subindex/type/scale from the hardware JSON "sdo_monitors" block. (The A6 does NOT implement
// the standard 0x6079/0x6078 and exposes bus voltage / phase current only via vendor object
// 0x2040 -- so its config points these at 0x2040:07 / 0x2040:0D, keeping this core vendor-free.)
enum class SdoValueType : std::uint8_t { U8, I8, U16, I16, U32, I32 };
// Parse "u8"/"i8"/"u16"/"i16"/"u32"/"i32" (case-insensitive); throws ConfigError otherwise.
SdoValueType parse_sdo_value_type(std::string_view text);
const char* to_string(SdoValueType t) noexcept;

// How the raw integer becomes the reported double. Either a fixed DIVISOR (value = raw/divisor
// -- 1000 for CiA402 mV->V, 10 for the A6's 0.1 V / 0.1 A) or the CiA402 RATED-CURRENT per-mille
// (value = (raw/1000) * motor_rated_current_amps).
enum class SdoScaleKind : std::uint8_t { Divisor, RatedCurrentPermille };

struct SdoMonitor {
    std::uint16_t index = 0;
    std::uint8_t subindex = 0;
    SdoValueType type = SdoValueType::U32;
    SdoScaleKind scale_kind = SdoScaleKind::Divisor;
    double divisor = 1.0;  // used ONLY when scale_kind == Divisor (value = raw / divisor)
    // Bytes to read = sizeof the value type (1/2/4).
    std::size_t byte_width() const noexcept;
};

// Decode `raw` (little-endian, at least m.byte_width() bytes) as m.type, then apply m's scale.
// rated_current_amps is used ONLY by RatedCurrentPermille. Throws ConfigError if raw is short.
double convert_sdo_monitor(const SdoMonitor& m, std::span<const std::byte> raw, double rated_current_amps);

// Viam motor control mode (INTENT; the driver derives the CiA402 PDO map, #61).
// PP = Profile Position (GoTo/GoFor); PV = Profile Velocity (SetRPM); Switchable =
// BOTH -- the superset map incl. 0x6060 so GoTo auto-selects PP and SetRPM auto-
// selects PV via the runtime §6 mode-switch.
enum class ControlMode : std::uint8_t {
    ProfilePosition,
    ProfileVelocity,
    Switchable,
};

// Parse "PP"/"PV"/"switchable" (case-insensitive); throws ConfigError on anything else.
ControlMode parse_control_mode(std::string_view text);
const char* to_string(ControlMode mode) noexcept;

struct ServoConfig {
    // --- identity / bus ---
    std::string ifname;          // EtherCAT NIC
    std::uint16_t slave_id = 1;  // 1-based ring position
    ethercat::PdoMap rxpdo;      // command map (0x1C12) -- A6 specifics are config data
    ethercat::PdoMap txpdo;      // feedback map (0x1C13)

    // --- motor mode + limits ---
    ControlMode mode = ControlMode::ProfilePosition;
    double max_motor_speed_rpm = 0.0;       // >= 0; the speed clamp
    double peak_current_limit_amps = 0.0;   // >= 0
    double motor_rated_current_amps = 0.0;  // > 0 (A6 datum: amps -> torque per-mille)
    double gear_ratio = 1.0;                // motor revs per output rev; != 0
    double counts_per_rev = 0.0;            // encoder counts per motor rev; > 0 (A6 = 131072)

    // --- #68 do_command converted-SDO read targets (optional "sdo_monitors" JSON block) ---
    // DEFAULTS = the standard CiA402 objects, so a generic drive works with zero config. A
    // vendor drive overrides them (the A6 lacks 0x6079/0x6078; it uses 0x2040:07 ÷10 V and
    // 0x2040:0D ÷10 A). get_motor_drive_modes (0x6502) is standard + fixed, NOT in this block.
    SdoMonitor voltage_monitor{0x6079, 0x00, SdoValueType::U32, SdoScaleKind::Divisor, 1000.0};            // DC-link mV -> V
    SdoMonitor current_monitor{0x6078, 0x00, SdoValueType::I16, SdoScaleKind::RatedCurrentPermille, 1.0};  // per-mille * rated -> A

    // --- move-complete predicate (noise-robust position-delta, #59) ---
    // reached/is_moving = |actual-target| <= position_tolerance_counts AND the position is STABLE (its
    // range over the last N cycles <= position_tolerance_counts). Both fields OPTIONAL:
    std::int32_t position_tolerance_counts = 0;  // >= 0; 0 => DEFAULT counts_per_rev/720 (0.5 deg), set in validated()
    std::int32_t velocity_threshold = 0;  // >= 0; 0 => use the position-delta stability method; >0 => honor a velocity gate (override)

    // --- RT ---
    std::uint32_t target_loop_rate_hz = 1000;  // 1..1000
    bool require_realtime = true;              // hard-fail if RT scheduling unavailable
    int rt_priority = 80;                      // SCHED_FIFO priority, 1..99
    // Enable Distributed-Clock SYNC0. REQUIRED by drives that support only DC sync
    // (the A6-EC faults out of OP -- Er74.1 "no sync signal", WKC->0 -- without it).
    // The SYNC0 cycle = 1e9 / target_loop_rate_hz; that period MUST be a value the
    // drive accepts (A6: an integer multiple of 250 us -> use 1000/500/250 Hz).
    bool use_distributed_clocks = false;
    // OPTIONAL SYNC0 cycle granularity the drive accepts, in ns (#44; CONFIG DATA from
    // the hardware JSON -- A6: 250000). When set (and DC is on), the Master validates the
    // loop rate against it AT CONFIG TIME with clear text + nearest valid rates, instead
    // of the drive rejecting the cycle cryptically at OP entry (A6 Er74.0). 0 = none.
    std::uint32_t sync_cycle_granularity_ns = 0;
    // OPTIONAL drive "no-sync" fault code (#TODO-4: CONFIG DATA, never a hardcoded
    // constant in the generic servo core). The 0x603F value a DC drive reports while
    // SYNC0 has not yet established -- the A6's is 0x8700 (Er74.1 "no SYNC0"). The
    // bring-up gate feeds (mapped 0x603F == this) to Master::bringup_step as the
    // drive-sync-faulted signal, keeping Master AND the generic servo core free of any
    // vendor code. nullopt ⇒ no sync-fault detection (the gate signal is always false --
    // a generic drive with no such code). Lives in the hardware JSON ("sync_fault_code"),
    // like sync_cycle_granularity_ns (#44) and vendor_fault_reset (#39).
    std::optional<std::uint16_t> sync_fault_code;
    // OPTIONAL vendor fault-reset SDO (#39: CONSUMER-side policy -- the library's
    // configure() carries zero vendor knowledge now). Executed ONCE by ServoController
    // at start()/reconfigure() AFTER Master::configure(), BEFORE the RT thread spawns
    // (single port owner -> a plain blocking Master::sdo_write; best-effort, logged).
    // The A6's reset is a vendor SDO write 1 to 0x2031:01, NOT CiA402 controlword bit7
    // (CLAUDE.md) -- that datum lives in the hardware JSON ("vendor_fault_reset"), never
    // in code. Present ⇒ clear a latent fault at bring-up; absent ⇒ no vendor reset (a
    // generic CiA402 drive uses the bit7 path the controller already drives).
    // Steady-state operator reset (RT running) is #22's queue, not this.
    std::optional<ethercat::SdoWrite> vendor_fault_reset;

    // --- health / boundary ---
    int max_consecutive_wkc_errors = 5;            // WKC latch threshold (passed to Master)
    std::uint64_t stall_threshold_cycles = 10;     // cycle-stall -> stale
    std::size_t command_queue_capacity = 64;       // > 0
    std::uint32_t handshake_timeout_cycles = 100;  // PP bit12 ack timeout
    // Quick-stop deceleration (0x6085, counts/s^2) for the R1 controlled stop. 0 = quick-stop not
    // configured -> the policy's quick-stop SDO setup (0x605A assert + 0x6085 write/readback) is
    // SKIPPED (the drive falls back to disable-voltage on stop). >0 -> configured + asserted at
    // bring-up. Standard CiA402 tunable; from the hardware JSON (never a hardcoded device value).
    std::uint32_t quick_stop_decel = 0;
    // Controlled-stop WINDOW (ms): the single source of truth for the LIFECYCLE-stop (#47-P3b R1).
    // It sizes the RT teardown window (teardown_cycles = window x loop_rate) so a Quick-Stop ramp
    // COMPLETES before close()->INIT de-energizes (no torque-cut at speed), AND the velocity guard
    // is DERIVED from it: a commanded velocity (0x60FF set_rpm AND 0x6081 go_to) is clamped to what
    // quick_stop_decel can ramp to 0 within this window minus a watchdog margin. One knob -> the
    // window and the budget can never disagree. Only active when quick_stop_decel > 0 (else the
    // stop is a disable-voltage coast, instant, and the guard is inert). Distinct from the
    // mode-switch ramp_stop_timeout (§6, a queue-liveness bound, not a de-energize deadline).
    std::uint32_t controlled_stop_window_ms = 1000;
    // Fault-reset recovery window: cycles to hold the reset intent waiting for the drive
    // to reflect Fault->Switch-On-Disabled before giving up (type-(b) persistent cause).
    // Must exceed the drive's real clear-reflect latency; a too-large N only delays the
    // give-up diagnostic, never breaks correctness. Tuned to the A6 at first light (#18).
    std::uint32_t fault_reset_window_cycles = 200;  // 200ms @ 1kHz -- generous default
    // Consecutive dev!=Fault cycles required to confirm the clear STUCK before declaring
    // reset success (type-(c) clear-then-refault debounce, #18). A refault within this
    // window counts as reset-ineffective, not a new fault. Small: outlast a flicker, but
    // don't delay genuine recovery. (Risky direction is too-SMALL fault_reset_window_cycles
    // -- below the drive's clear-reflect latency it false-fails; keep it >= that latency.)
    std::uint32_t fault_reset_clear_confirm_cycles = 3;
    std::uint32_t move_timeout_ms = 0;  // 0 = no-progress watchdog only

    // --- diagnostics ---
    // OPTIONAL gloss for the 0x603F drive error code -> human label, surfaced by
    // last_error() (e.g. 0x8700 -> "Er74.1 / no SYNC0"). CONFIG DATA, never a
    // hardcoded A6 table: populated from the hardware JSON. A code not in this list
    // glosses to bare hex, so the line is never wrong, just less descriptive. Small
    // (a handful of codes); looked up on the cold last_error() path only.
    std::vector<std::pair<std::uint16_t, std::string>> fault_code_labels;

    // Throws ethercat::ConfigError (clear text) on any invalid field. Pure --
    // no I/O.
    void validate() const;

    // #61: fill an EMPTY rxpdo/txpdo from the standard CiA402 map derived from `mode` (PP/PV/switchable).
    // A user-supplied map (non-empty) is left verbatim (advanced override). Standard objects only (#41):
    //   PP  RxPDO 0x1600: 0x6040,0x607A,0x6081     PV RxPDO: 0x6040,0x60FF
    //   switchable RxPDO: 0x6040,0x6060,0x607A,0x6081,0x60FF  (superset; runtime §6 switch enabled)
    //   TxPDO 0x1A00 (all): 0x603F,0x6041,0x6061,0x6064,0x606C,0x6077
    // Assign 0x1600->0x1C12 / 0x1A00->0x1C13 (derived from direction). Idempotent.
    void apply_derived_pdo_maps();
};

}  // namespace ethercat::servo
