#pragma once

// Validated configuration for one servo drive. Pure data + a validate() that
// throws ethercat::Error with clear text. No SDK, no hardware -- the
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

// The CiA402 command INTENT for a single motor API call (#18: always-switchable). PP = Profile
// Position (GoTo/GoFor); PV = Profile Velocity (SetRPM). This is NOT a config choice -- the driver
// is ALWAYS switch-capable and each API call ensures its own required mode at runtime (the driver
// orchestrates the switch). Used internally as the per-command mode; never parsed from config.
enum class ControlMode : std::uint8_t {
    ProfilePosition,
    ProfileVelocity,
};

const char* to_string(ControlMode mode) noexcept;

struct ServoConfig {
    // --- identity / bus ---
    std::string ifname;          // EtherCAT NIC
    std::uint16_t slave_id = 1;  // 1-based ring position
    // #18: the RxPDO/TxPDO map is NO LONGER config -- it is a FIXED driver-defined superset
    // (set_fixed_pdo_map()), never user-supplied. These fields hold that built map (fed to the
    // Master's SDO remap path); they are populated by the driver, not parsed from the machine config.
    ethercat::PdoMap rxpdo;  // command map (0x1C12): cw + 0x6060 + 0x607A + 0x6081 + 0x60FF
    ethercat::PdoMap txpdo;  // feedback map (0x1C13): 0x603F,0x6041,0x6061,0x6064,0x606C,0x6077

    // --- motor limits ---
    double max_motor_speed_rpm = 0.0;       // >= 0; the speed clamp
    double motor_rated_current_amps = 0.0;  // > 0 (A6 datum; do_command's per-mille current readback)
    double gear_ratio = 1.0;                // motor revs per output rev; != 0
    double counts_per_rev = 0.0;            // encoder counts per motor rev; > 0 (A6 = 131072)

    // #15: do_command SDO reads target the fixed STANDARD CiA402 objects (0x6079/0x6078/0x6502) in
    // the module handler -- no config, no override (see servo_motor.cpp read_std_sdo).
    // --- move-complete predicate (noise-robust position-delta, #59) ---
    // reached/is_moving = |actual-target| <= position_tolerance_counts AND the position is STABLE (its
    // range over the last N cycles <= position_tolerance_counts). #19: the velocity_threshold override
    // is gone -- is-moving/at-rest is ALWAYS the position-stability heuristic (PP and PV alike).
    std::int32_t position_tolerance_counts = 0;  // >= 0; 0 => DEFAULT counts_per_rev/720 (0.5 deg), set in validated()

    // --- RT ---
    std::uint32_t target_loop_rate_hz = 1000;  // 1..1000
    bool require_realtime = true;              // hard-fail if RT scheduling unavailable
    int rt_priority = 80;                      // SCHED_FIFO priority, 1..99
    // #15: op_await_timeout_ms removed as a config knob -- MasterConfig's fixed 30s default applies.
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
    // #15 item 2: the drive "no-sync" 0x603F code, the vendor fault-reset SDO, and the
    // 0x603F->label gloss are NO LONGER config data. They are DEVICE knowledge, now carried
    // by a ServoController subclass (A6ServoDriver) via its overridable seams -- the sanctioned
    // extension mechanism (reverses the old "drive specifics live only as config" rule). The
    // generic base drives STANDARD CiA402 only. PDO map / limits / kinematics stay per-machine
    // config below.

    // --- health / boundary ---
    int max_consecutive_wkc_errors = 5;       // WKC latch threshold (passed to Master)
    std::size_t command_queue_capacity = 64;  // > 0
    // #17: the no-progress (stuck-move) watchdog is GONE -- a stuck move parks until the client stops it
    // / the drive faults / the RT loop exits (client-owned cancellation). stall_threshold_cycles removed.
    // #17: the PP new-setpoint ack timeout is no longer a config knob -- it is an internal Cia402Policy
    // constant (kHandshakeTimeoutCycles). The 4-phase handshake is always bounded by it.
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
    // #17: the blocking go_to/go_for wait has NO timeout at all -- a long move must not be killed by a
    // clock, and a stuck move parks until the client stops it / the drive faults / the RT loop exits.

    // Throws ethercat::Error (clear text) on any invalid field. Pure --
    // no I/O.
    void validate() const;

    // #18: set the FIXED driver-defined superset PDO map (unconditionally -- there is no per-mode
    // choice and no user-supplied map). Standard CiA402 objects only (#41), always switch-capable:
    //   RxPDO 0x1600: 0x6040 cw, 0x6060 mode, 0x607A target-pos, 0x6081 profile-vel, 0x60FF target-vel
    //   TxPDO 0x1A00: 0x603F fault, 0x6041 status, 0x6061 mode-disp, 0x6064 actual-pos, 0x606C vel, 0x6077 torque
    // Assign 0x1600->0x1C12 / 0x1A00->0x1C13 (derived from direction). Idempotent.
    void set_fixed_pdo_map();
};

}  // namespace ethercat::servo
