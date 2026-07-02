"""Scenario machine-configs for the RDK validation campaign (docs/viam-driven-validation.md).

All configs target the REAL A6 on enp86s0. Attribute values come from the
bench-proven /home/viam/a6-robot.trixie.json, with the campaign deltas:
  - control_mode "switchable" (tests use SetRPM + GoFor + GoTo on one motor);
  - NO rxpdo/txpdo blocks (exercises the #61 derived superset map);
  - NO position_tolerance_counts / velocity_threshold (exercises the #59
    0.5-degree default + position-stability reach);
  - move_timeout_ms raised to 120 s (Test6 drives a 2000-rev move that would
    trip a 10 s watchdog before its mid-move DoCommand checks finish).

Scenario variants for the "What to test" list:
  valid            - the known-good switchable config
  wrong            - fails config validation (bogus control_mode) -> error in logs
  wrong_interface  - parses, but the NIC doesn't exist -> runtime init error in logs
  dc_validation    - loop_rate_hz 300 -> cycle 3.333 ms, NOT a multiple of the
                     declared 250 us SYNC0 granularity -> rejected at config time
  dc_drive         - passes OUR validation but the DRIVE rejects it on the wire:
                     use_distributed_clocks=false -> A6 refuses free-run
                     (AL 0x0027 "Freerun not supported").
                     SAFE variant: no repeated Er74 OP faults, so no drive wedge.
  dc_drive_sync0   - alternate on-wire variant: sync_cycle_granularity_ns
                     deliberately mis-declared as 125 us + loop_rate_hz 8000 ->
                     our validation passes, drive sees a 125 us SYNC0 it doesn't
                     support. WEDGE RISK (repeated Er74 can require a control
                     power cycle) -- run ONCE, only if dc_drive isn't accepted.
"""

MODULE_PATH = "/home/viam/ethercat-servo-module/bin/ethercat-servo"
NIC = "enp86s0"


def _modules() -> list:
    return [{"type": "local", "name": "ethercat-servo", "executable_path": MODULE_PATH}]


def _base_attributes() -> dict:
    return {
        "interface": NIC,
        "slave": 1,
        "control_mode": "switchable",
        "max_rpm": 3000.0,
        "motor_rated_current_amps": 2.5,
        "peak_current_amps": 7.5,
        "counts_per_rev": 131072.0,
        "gear_ratio": 1.0,
        "require_realtime": True,
        "rt_priority": 80,
        "loop_rate_hz": 1000,
        "use_distributed_clocks": True,
        "sync_cycle_granularity_ns": 250000,
        "sync_fault_code": 34560,
        "vendor_fault_reset": {"index": 8241, "subindex": 1, "value": 1, "value_bytes": 2},
        "move_timeout_ms": 120000,
        "handshake_timeout_cycles": 1000,
        "stall_threshold_cycles": 2000,
        "command_queue_capacity": 64,
        "max_consecutive_wkc_errors": 5,
        "fault_code_labels": [{"code": 34560, "label": "Er74.1 / no SYNC0"}],
    }


def _machine(attributes: dict) -> dict:
    return {
        "modules": _modules(),
        "components": [
            {
                "name": "servo",
                "api": "rdk:component:motor",
                "model": "viam:ethercat:servo",
                "attributes": attributes,
            }
        ],
    }


def empty() -> dict:
    return {"components": []}


def valid() -> dict:
    return _machine(_base_attributes())


def wrong() -> dict:
    a = _base_attributes()
    a["control_mode"] = "torque"  # unsupported -> config parse/validation error in logs
    return _machine(a)


def wrong_interface() -> dict:
    a = _base_attributes()
    a["interface"] = "enp99s0"  # parses fine; NIC open fails at init -> runtime error in logs
    return _machine(a)


def dc_validation() -> dict:
    a = _base_attributes()
    a["loop_rate_hz"] = 300  # 3.333 ms cycle, not an integer multiple of 250 us -> config-time reject
    return _machine(a)


def dc_drive() -> dict:
    a = _base_attributes()
    a["use_distributed_clocks"] = False  # A6: free-run unsupported -> AL 0x0027 on the wire (safe, no wedge)
    return _machine(a)


def dc_drive_sync0() -> dict:
    a = _base_attributes()
    a["sync_cycle_granularity_ns"] = 125000  # lie: drive granularity is 250 us
    a["loop_rate_hz"] = 8000  # 125 us cycle passes OUR check, drive rejects SYNC0 -- WEDGE RISK, run once
    return _machine(a)


SCENARIOS = {
    "empty": empty,
    "valid": valid,
    "wrong": wrong,
    "wrong_interface": wrong_interface,
    "dc_validation": dc_validation,
    "dc_drive": dc_drive,
    "dc_drive_sync0": dc_drive_sync0,
}


def build(name: str) -> dict:
    if name not in SCENARIOS:
        raise SystemExit(f"unknown scenario '{name}'; choose from: {', '.join(SCENARIOS)}")
    return SCENARIOS[name]()
