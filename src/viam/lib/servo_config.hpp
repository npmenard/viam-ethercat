#pragma once

// Validated configuration for one servo drive. Pure data + a validate() that
// throws ethercat::ConfigError with clear text. No SDK, no hardware -- the
// module parses the Viam attributes into this struct and validates before any
// hardware init (deferred-init pattern). The A6 PDO map lives here as CONFIG
// DATA (never hardcoded in generic code).

#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>

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

    // --- health / boundary ---
    int max_consecutive_wkc_errors = 5;            // WKC latch threshold (passed to Master)
    std::uint64_t stall_threshold_cycles = 10;     // cycle-stall -> stale
    std::size_t command_queue_capacity = 64;       // > 0
    std::uint32_t handshake_timeout_cycles = 100;  // PP bit12 ack timeout
    std::uint32_t move_timeout_ms = 0;             // 0 = no-progress watchdog only

    // Throws ethercat::ConfigError (clear text) on any invalid field. Pure --
    // no I/O.
    void validate() const;
};

}  // namespace ethercat::servo
