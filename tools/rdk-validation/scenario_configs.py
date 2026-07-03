"""Scenario machine-configs for the RDK validation campaign (docs/viam-driven-validation.md).

All configs target the REAL A6 on enp86s0. Attribute values come from the
bench-proven /home/viam/a6-robot.trixie.json, with the campaign deltas:
  - control_mode "switchable" (tests use SetRPM + GoFor + GoTo on one motor);
  - NO rxpdo/txpdo blocks (exercises the #61 derived superset map);
  - NO position_tolerance_counts / velocity_threshold (exercises the #59
    0.5-degree default + position-stability reach);
  - #15: move_timeout_ms REMOVED -- the blocking-API wait now has a fixed 10min backstop and the
    RT no-progress watchdog is the real stuck-move safety, so Test6's long 2000-rev move just works.

Scenario variants for the "What to test" list:
  valid            - the known-good switchable config
  wrong            - fails config validation (bogus control_mode) -> error in logs
  wrong_interface  - parses, but the NIC doesn't exist -> runtime init error in logs
  dc_validation    - loop_rate_hz 300 -> cycle 3.333 ms, NOT a multiple of the
                     declared 250 us SYNC0 granularity -> rejected at COMPONENT
                     CONSTRUCTION (Master ctor ConfigError in the logs; validate()
                     itself passes -- 300 is in range; no bus contact either way)
  dc_drive         - passes OUR validation but the DRIVE rejects it on the wire:
                     use_distributed_clocks=false -> A6 refuses free-run
                     (AL 0x0027 "Freerun not supported").
                     SAFE variant: no repeated Er74 OP faults, so no drive wedge.
  dc_drive_cycle   - alternate on-wire variant: loop_rate_hz 700 with the
                     granularity declaration OMITTED -> passes our validation
                     (no #44 check without the declaration), drive gets a
                     1,428,571 ns SYNC0 cycle that is not a 250 us multiple ->
                     rejects on the wire (Er74.0 cycle error). WEDGE RISK
                     (repeated Er74 can require a control power cycle) -- run
                     ONCE, only if dc_drive isn't accepted. (A sub-ms SYNC0 is
                     inexpressible: loop_rate_hz is hard-capped at 1000.)
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
        "handshake_timeout_cycles": 1000,
        "stall_threshold_cycles": 2000,
        "command_queue_capacity": 64,
        "max_consecutive_wkc_errors": 5,
        # #15 item 2: sync_fault_code, vendor_fault_reset, and fault_code_labels are NO LONGER config
        # attributes -- the a6-servo model (an A6ServoDriver subclass) carries them in code. #15 item 1:
        # sdo_monitors is gone too; Test 6's DoCommands (get_motor_voltage/current) read the STANDARD
        # CiA402 objects (0x6079/0x6078), which the A6 aborts -> value 0 + a "<key>_diag" note (a 0V
        # reading is valid; the A6 is a test vehicle), so Test 6 passes on 0-values.
    }


def _machine(attributes: dict) -> dict:
    return {
        "modules": _modules(),
        "components": [
            {
                "name": "servo",
                "api": "rdk:component:motor",
                "model": "viam:ethercat:a6-servo",
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


def dc_drive_cycle() -> dict:
    a = _base_attributes()
    del a["sync_cycle_granularity_ns"]  # no declaration -> the #44 config check cannot catch it
    a["loop_rate_hz"] = 700  # 1,428,571 ns cycle: passes validation, NOT a 250 us multiple -> drive Er74.0 -- WEDGE RISK, run once
    return _machine(a)


SCENARIOS = {
    "empty": empty,
    "valid": valid,
    "wrong": wrong,
    "wrong_interface": wrong_interface,
    "dc_validation": dc_validation,
    "dc_drive": dc_drive,
    "dc_drive_cycle": dc_drive_cycle,
}


def build(name: str) -> dict:
    if name not in SCENARIOS:
        raise SystemExit(f"unknown scenario '{name}'; choose from: {', '.join(SCENARIOS)}")
    return SCENARIOS[name]()
