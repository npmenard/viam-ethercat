#pragma once

// Validated configuration for one servo drive. Pure data + a validate() that
// throws ethercat::ConfigError with clear text. No SDK, no hardware -- the
// module parses the Viam attributes into this struct and validates before any
// hardware init (deferred-init pattern). The A6 PDO map lives here as CONFIG
// DATA (never hardcoded in generic code).

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "ethercat/pdo_mapping.hpp"

namespace ethercat::servo {

// Viam motor control mode. PP = Profile Position (GoTo/GoFor); PV = Profile
// Velocity (SetRPM).
enum class ControlMode : std::uint8_t {
    ProfilePosition,
    ProfileVelocity,
};

// Parse "PP"/"PV" (case-insensitive); throws ConfigError on anything else.
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

    // --- move-complete predicate ---
    std::int32_t position_tolerance_counts = 0;  // >= 0
    std::int32_t velocity_threshold = 0;         // >= 0 (device velocity units)

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
};

}  // namespace ethercat::servo
