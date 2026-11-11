#pragma once

// Validated configuration for one servo drive. Pure data plus a validate() that throws
// ethercat::Error with clear text. No SDK, no hardware: the module parses the Viam attributes
// into this struct and validates before any hardware init.

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

// Which CiA402 mode family the driver runs (the "control_mode" attribute). CyclicPosition: CSP, the
// master streams 0x607A from its own trajectory. Profile: PP for go_to/go_for and PV for set_rpm,
// switched at runtime; the drive generates the trajectory.
enum class MotionModeKind : std::uint8_t {
    CyclicPosition,
    Profile,
};

const char* to_string(MotionModeKind mode) noexcept;

struct ServoConfig {
    // --- identity / bus ---
    std::string ifname;          // EtherCAT NIC
    std::uint16_t slave_id = 1;  // 1-based ring position
    MotionModeKind motion_mode = MotionModeKind::CyclicPosition;
    // The PDO maps are driver-defined per motion_mode (set_fixed_pdo_map()), never user-supplied.
    ethercat::PdoMap rxpdo;  // command map (0x1C12): cw + 0x607A (cyclic) | cw + 0x6060 + 0x607A + 0x6081 + 0x60FF (profile)
    ethercat::PdoMap txpdo;  // feedback map (0x1C13): 0x603F,0x6041,0x6061,0x6064,0x606C,0x6077
    // Feedback a device model needs beyond the fixed map, appended by set_fixed_pdo_map(). Set by
    // the model's own builder, not from user config: the map stays driver-defined.
    std::vector<ethercat::PdoEntry> extra_txpdo;

    // --- motor limits ---
    double max_motor_speed_rpm = 0.0;  // >= 0; the speed clamp
    // Ramp acceleration of the master-side trajectory (CSP), rpm/s; 0 = max_motor_speed_rpm per
    // second. Ignored in profile mode (the drive's 0x6083/0x6084 apply). >= 0.
    double max_accel_rpm_per_s = 0.0;
    double motor_rated_current_amps = 0.0;  // > 0 (nameplate datum; scales per-mille current readbacks)
    double gear_ratio = 1.0;                // motor revs per output rev; != 0
    double counts_per_rev = 0.0;            // encoder counts per motor rev; > 0 (e.g. 2^17 = 131072)

    // do_command SDO reads target the fixed standard CiA402 objects (0x6079/0x6078/0x6502) in the
    // module handler (see servo_motor.cpp read_std_sdo).
    // --- move-complete predicate (noise-robust position-delta) ---
    // reached/is_moving = |actual-target| <= position_tolerance_counts and the position is stable (its
    // range over the last N cycles <= position_tolerance_counts). is-moving/at-rest is always the
    // position-stability heuristic (PP and PV alike).
    std::int32_t position_tolerance_counts = 0;  // >= 0; 0 defaults to counts_per_rev/720 (0.5 deg), set in validated()

    // --- RT ---
    std::uint32_t target_loop_rate_hz = 1000;  // 1..1000
    bool require_realtime = true;              // hard-fail if RT scheduling unavailable
    int rt_priority = 80;                      // SCHED_FIFO priority, 1..99
    // op_await_timeout_ms is not a config knob; MasterConfig's fixed 30s default applies.
    // Enable Distributed-Clock SYNC0. Required by drives that support only DC sync (a DC-only
    // drive refuses free-run OP, or sync-faults out of OP with WKC -> 0). The SYNC0 cycle =
    // 1e9 / target_loop_rate_hz; that period must be a value the drive accepts (some drives allow
    // only integer multiples of a base tick -- see sync_cycle_granularity_ns).
    bool use_distributed_clocks = false;
    // Optional SYNC0 cycle granularity the drive accepts, in ns (config data; e.g. 250000 for a
    // 250 us base tick). When set (and DC is on), the Master validates the loop rate against it at
    // config time with clear text and nearest valid rates, instead of the drive rejecting the
    // cycle cryptically at OP entry. 0 = none.
    std::uint32_t sync_cycle_granularity_ns = 0;
    // The drive "no-sync" 0x603F code, the vendor fault-reset SDO, and the 0x603F->label gloss are
    // not config data. They are device knowledge, carried by a ServoController subclass via its
    // overridable seams. The generic base drives standard CiA402 only. PDO map, limits, and
    // kinematics stay per-machine config below.

    // --- health / boundary ---
    int max_consecutive_wkc_errors = 5;       // WKC latch threshold (passed to Master)
    std::size_t command_queue_capacity = 64;  // > 0
    // The PP new-set-point ack timeout is an internal Cia402Sequencer constant (kHandshakeTimeoutCycles),
    // not a config knob; the four-phase handshake is always bounded by it.
    // Quick-stop deceleration (0x6085, counts/s^2) for the controlled stop. 0 = quick-stop not
    // configured, so the sequencer's quick-stop SDO setup (0x605A assert, 0x6085 write/readback) is
    // skipped and the drive falls back to disable-voltage on stop. >0 = configured and asserted at
    // bring-up. Standard CiA402 tunable; from config, never a hardcoded device value.
    std::uint32_t quick_stop_decel = 0;
    // Everything the drive needs for one homing run. CiA402 standardises the objects but not the
    // method numbers (the negative ones are reserved for the manufacturer) nor any torque limit, so
    // the drive-specific values come from config and are refused, not guessed. The torque block is
    // only for the methods that detect the end by motor current: a limit-switch method (1, 2, 17,
    // 18, ...) finds a switch wired ahead of time and needs none.
    struct Homing {
        std::int8_t method = 0;        // 0x6098; 0 = not configured, so `home` is refused
        double speed_rpm = 0.0;        // 0x6099:01 approach speed
        double accel_rpm_per_s = 0.0;  // 0x609A; 0 leaves the drive's value
        double torque_percent = 0.0;   // contact threshold; 0 = do not clamp
        double timeout_s = 120.0;      // the run fails if the drive has not attained home by then
        // Object that bounds motor current during the run. No CiA402 object standardises it, so it
        // comes from config: index 0 = not configured. `type` is an sdo_codec type name.
        struct TorqueObject {
            std::uint16_t index = 0;
            std::uint8_t sub = 0;
            std::string type = "u16";
            double units_per_percent = 10.0;  // object units for one percent of rated current
        };
        TorqueObject torque;
    };
    Homing homing;
    // Optional absolute-encoder declaration. `index/sub/type` name the manufacturer-specific object
    // that holds the encoder mode and `equals` the value meaning "retains position across power
    // cycles"; `invalid_codes` are the 0x603F values that mean the retained position is gone (these
    // are warnings on some drives, so they never raise the fault bit). index 0 = not declared.
    struct AbsoluteEncoder {
        std::uint16_t index = 0;
        std::uint8_t sub = 0;
        std::string type = "u16";
        double equals = 0.0;
        std::vector<std::uint16_t> invalid_codes;
    };
    AbsoluteEncoder absolute_encoder;
    // 0x603F value the drive reports while its emergency-stop input is engaged (drive-specific, so it
    // comes from config). 0 = not configured. While it is present the motion API refuses with
    // "emergency stop engaged" and no fault-reset edges are presented.
    std::uint16_t estop_fault_code = 0;
    // Controlled-stop window (ms): the single source of truth for the lifecycle-stop. It sizes the
    // RT teardown window (teardown_cycles = window x loop_rate) so a quick-stop ramp completes before
    // close()->INIT de-energizes (no torque-cut at speed), and the velocity guard is derived from it:
    // a commanded velocity (0x60FF set_rpm and 0x6081 go_to) is clamped to what quick_stop_decel can
    // ramp to 0 within this window minus a watchdog margin, so the window and the budget cannot
    // disagree. Only active when quick_stop_decel > 0 (else the stop is an instant disable-voltage
    // coast and the guard is inert).
    std::uint32_t controlled_stop_window_ms = 1000;
    // Period (cycles) between CiA402 fault-reset edges while the drive stays in Fault. Must exceed the
    // drive's clear latency so a reset that is working is not re-pulsed.
    std::uint32_t fault_reset_window_cycles = 200;  // 200 ms @ 1 kHz
    // Consecutive non-Fault cycles before a clear counts (a clear-then-refault flicker is not recovery).
    std::uint32_t fault_reset_clear_confirm_cycles = 3;
    // The blocking go_to/go_for wait has no timeout: a long move must not be killed by a clock, and a
    // stuck move parks until the client stops it, the drive faults, or the RT loop exits.

    // Throws ethercat::Error (clear text) on any invalid field. Pure --
    // no I/O.
    void validate() const;

    // Set the fixed driver-defined superset PDO map (unconditional: no per-mode choice, no
    // user-supplied map). Standard CiA402 objects only, always switch-capable:
    //   RxPDO 0x1600: 0x6040 cw, 0x6060 mode, 0x607A target-pos, 0x6081 profile-vel, 0x60FF target-vel
    //   TxPDO 0x1A00: 0x603F fault, 0x6041 status, 0x6061 mode-disp, 0x6064 actual-pos, 0x606C vel, 0x6077 torque
    // Assign 0x1600->0x1C12 / 0x1A00->0x1C13 (derived from direction). Idempotent.
    void set_fixed_pdo_map();
};

}  // namespace ethercat::servo
