# viam-ethercat

A SOEM-based EtherCAT master library and a Viam `rdk:component:motor` module for
CiA402 servo drives. The module runs a phase-locked 1 kHz real-time loop with
Distributed-Clocks (SYNC0) support and exposes the standard Viam motor API
(`go_to`, `go_for`, `set_rpm`, `stop`, …) in output revolutions.

Two motor models ship in one module:

| Model | Use for |
|---|---|
| `viam:ethercat:servo` | any conformant CiA402 EtherCAT servo drive |
| `viam:ethercat:a6-servo` | ANCTL A6-EC series drives (adds the A6's vendor fault-reset, sync-fault code, and position-stability move-completion, since the A6 ties statusword bit 10 high) |
| `viam:ethercat:gantry` | one linear axis driven by one of the motors above (`rdk:component:gantry`) |

## Requirements

- Linux with a dedicated NIC wired to the EtherCAT segment.
- **Realtime scheduling.** The module needs SCHED_FIFO for its cycle thread and
  fails at load with an actionable error if the host does not allow it: run
  viam-server as root or grant it realtime permission (systemd
  `LimitRTPRIO=99`, or an `rtprio` entry in `/etc/security/limits.d`). A
  PREEMPT_RT kernel is strongly recommended. `CAP_NET_RAW` is required for the
  raw EtherCAT socket. To evaluate without realtime (development only), set
  `"require_realtime": false`.

## Configuration and usage

Add a motor component with one of the models above.

### Attributes

| Name | Type | Inclusion | Default | Description |
|------|------|-----------|---------|-------------|
| `interface` | string | **Required** | — | EtherCAT NIC name (e.g. `enp3s0`). |
| `max_rpm` | float | **Required** | — | Motor speed clamp (nameplate rated speed). |
| `counts_per_rev` | float | **Required** | — | Encoder counts per motor revolution. |
| `motor_rated_current_amps` | float | **Required** | — | Nameplate rated current. |
| `slave` | int | Optional | `1` | 1-based position of the drive on the bus. |
| `control_mode` | string | Optional | `csp` (`profile` for `a6-servo`) | `csp`: cyclic synchronous position, master-side trajectory, needs `use_distributed_clocks`. `profile`: PP for `go_to`/`go_for`, PV for `set_rpm`, the drive generates the trajectory. |
| `gear_ratio` | float | Optional | `1.0` | Motor revolutions per output revolution. |
| `position_tolerance_counts` | int | Optional | `counts_per_rev`/720 | "Reached/stopped" stability band. |
| `loop_rate_hz` | int | Optional | `1000` | Cycle rate (1..1000). |
| `use_distributed_clocks` | bool | Optional | `false` | DC/SYNC0 sync; required by DC-only drives. |
| `sync_cycle_granularity_ns` | int | Optional | `0` | The drive's SYNC0 cycle granularity; a bad `loop_rate_hz` is then rejected at config time. |
| `require_realtime` | bool | Optional | `true` | Hard-fail at load without SCHED_FIFO. |
| `rt_priority` | int | Optional | `80` | SCHED_FIFO priority (1..99). |
| `max_acceleration_rpm_per_s` | float | Optional | `max_rpm` per second | Ramp acceleration of the master-side trajectory in `csp`; ignored in `profile`. |
| `quick_stop_decel_rpm_per_s` | float | Optional | `0` (not configured) | Deceleration written to the drive's 0x6085: its quick stop, its E-stop input ramp, and in `profile` mode the module's own `stop()`; in `profile` mode the drive must hold 0x605A = 2. |
| `emergency_stop_fault_code` | string or int | Optional | none | 0x603F value the drive reports while its emergency-stop input is engaged (e.g. `"0xFF3A"`). While present, `status` reports `emergency_stop: true`, motion calls fail with "emergency stop engaged", and no automatic fault-reset edges are sent (the `fault_reset` do_command still works, for a drive that latches the stop). |
| `absolute_encoder` | object | Optional | none | Declares the drive object that says the encoder retains position, and the 0x603F codes that mean it lost it: `{"index": "0x2A9C", "sub": 4, "type": "u16", "equals": 2, "invalid_codes": ["0xFF3C", "0xFF3D", "0xFF0F"]}`. Read by the `is_abs_enc_valid` verb and by `homed`. |
| `homing` | object | Optional | none | Everything one homing run needs; without it `home` is refused. See [Homing](#homing). |

### Control mode

`control_mode` picks the CiA402 mode family and, with it, the driver-defined PDO map:

- `csp` (default): cyclic synchronous position. The driver runs a trapezoidal trajectory on the
  master and streams the target position every cycle; `max_acceleration_rpm_per_s` sets the ramp.
  Requires `use_distributed_clocks: true`.
- `profile`: profile position for `go_to`/`go_for` and profile velocity for `set_rpm`, switched at
  runtime; the drive generates the trajectory. Default for the `a6-servo` model.

The mode is selected on the drive by SDO at start. A drive that does not implement it fails to
start with an error naming the refused mode; the running mode is reported as `motion_mode` in the
`status` do_command. There is no PDO-map configuration: the driver defines one CiA402 map per
control mode.

### Generic CiA402 servo

```json
{
  "name": "servo",
  "api": "rdk:component:motor",
  "model": "viam:ethercat:servo",
  "attributes": {
    "interface": "enp3s0",
    "slave": 1,
    "max_rpm": 3000.0,
    "counts_per_rev": 8388608.0,
    "motor_rated_current_amps": 2.5,
    "gear_ratio": 1.0
  }
}
```

### ANCTL A6-EC servo

The A6 supports only DC sync (it refuses free-run) and accepts SYNC0 cycles in
250 µs multiples, so `use_distributed_clocks` and the granularity declaration
matter:

```json
{
  "name": "servo",
  "api": "rdk:component:motor",
  "model": "viam:ethercat:a6-servo",
  "attributes": {
    "interface": "enp3s0",
    "slave": 1,
    "max_rpm": 3000.0,
    "counts_per_rev": 131072.0,
    "motor_rated_current_amps": 2.5,
    "gear_ratio": 1.0,
    "use_distributed_clocks": true,
    "sync_cycle_granularity_ns": 250000,
    "loop_rate_hz": 1000
  }
}
```

### Homing

One `homing` object holds everything a run needs, so `home` takes no arguments and a dependent
component (the gantry) configures nothing about it:

```json
"homing": {
  "method": -1,
  "speed_rpm": 20,
  "acceleration_rpm_per_s": 100,
  "torque_object": {"index": "0x6072", "sub": 0, "type": "u16", "units_per_percent": 10},
  "torque_percent": 20,
  "timeout_s": 120
}
```

| Name | Type | Inclusion | Default | Description |
|------|------|-----------|---------|-------------|
| `method` | int | **Required** | — | The 0x6098 homing method. CiA402 leaves the negative methods to the manufacturer, so they differ per drive and cannot be defaulted: the A6-EC homes into a hard stop with -1 (reverse) and -2 (forward), an Applied Motion M56S with -4 and -3. The drive is asked for it and the value is read back, so a method it does not support fails the `home` call before anything moves. |
| `speed_rpm` | float | **Required** | — | Approach speed, written to 0x6099:01. |
| `acceleration_rpm_per_s` | float | Optional | `0` (the drive's own) | Written to 0x609A. |
| `torque_object` | object | Optional | none | Drive object that bounds motor current during the run, and so sets the contact threshold for a hard-stop method: `{"index", "sub", "type", "units_per_percent"}`. The type and scale are the drive's own -- read the object first, a wrong width is refused. A limit-switch method (1, 2, 17, 18, …) needs none. |
| `torque_percent` | float | Optional | `0` (the drive's own) | Contact threshold in percent of rated current; needs `torque_object`. The object is saved before the run and written back after it, so a drive whose only torque limit is the global one is not left clamped. |
| `timeout_s` | float | Optional | `120` | The run fails if the drive has not attained home by then. |

`{"home": true}` blocks for the run and replies
`{"homed": true, "counts": 131749, "revolutions": 1.005}`, or `{"homed": false, "error": "..."}`
when the drive reports a homing error or the run times out. `counts` is the actual position at the
cycle the drive attained home -- the axis reference, with whatever back-off the method performed
already in it, so nothing downstream has to model the back-off. `status` carries `homed`,
`homed_by`, `homing`, the same `home_counts`/`home_revolutions`, and after a failure `homing_error`.

`homed` is true once a run has succeeded, or when the `absolute_encoder` object says the drive kept
its reference across the restart (`homed_by` says which). The encoder-mode read is cached until the
bus drops, since a drive that lost control power may have lost a battery-backed count with it.

### Behavior notes

- `stop()` is a CiA402 Halt: the motor ramps to standstill and **holds position
  energized**. Use the `disable` do_command to de-energize (coast).
- A drive fault is recovered the standard CiA402 way: the module presents fault-reset edges
  (controlword bit 7) every `fault_reset_window_cycles` until the drive leaves Fault, then re-enables.
  `last_error` names the fault code meanwhile; the `fault_reset` do_command requests an edge at once.
- On bus loss (cable pull, NIC down) the module stays alive: reads return
  fail-safe values, motion calls return a clear error, and the next motion call
  after the link returns rebuilds the connection automatically.
- Log verbosity follows the component's log level in the machine config
  (`"log_configuration": {"level": "debug"}` on the motor). `debug` traces every SDO (object,
  payload, decoded abort) and AL state request during bring-up; `info` prints one line per
  bring-up phase and one per fault-code change.
- `{"is_abs_enc_valid": true}` answers whether the drive's absolute encoder is set to retain position
  and still holds it: `{"valid": true, "configured": true, "mode": 2, "fault_code": "0x0000", "warning": false}`, or
  `valid: false` with a `reason`. Needs the `absolute_encoder` attribute; without it the answer is
  false with that as the reason. The codes it treats as a lost reference are warnings on some drives,
  so they never raise the fault bit and are read from the live 0x603F instead.
- Raw SDO access for commissioning: `{"sdo_read": {"index": "0x6085", "sub": 0, "type": "u32"}}`,
  `{"sdo_write": {"index": "0x2A9C", "sub": 4, "type": "u16", "value": 2}}`. **Reads** reach any
  object, so a drive-side setting this module does not model can be inspected; **writes** are limited
  to manufacturer-specific objects (0x2000-0x5FFF) and 0x1010 (store parameters), since the driver
  owns the standard objects. Types `u8 i8 u16 i16 u32 i32 string bytes`; values are numbers or
  `"0x.."` / hex-byte strings. A drive refusal returns
  `{"ok": false, "abort_code": "0x06090030", "error": "..."}`; writes are refused while the motor moves.
- `do_command` verbs: `status`, `fault_reset`, `enable`, `disable`, `home`, `is_abs_enc_valid`,
  `get_motor_voltage`, `get_motor_current_actual_value`,
  `get_motor_drive_modes`.

## Single-axis gantry

`viam:ethercat:gantry` turns one of the motors above into a linear axis: it owns the travel
length, the millimetre scale and the bound that keeps a move inside the axis, and drives the
motor through the standard motor API. It depends on the motor component and never opens the bus
itself. It refuses to build if its dependency is not a `viam:ethercat` servo, since it needs this
module's homing contract.

Homing belongs to the motor: the axis asks for it and keeps the answer. The position the drive
ended the run at is 0 mm, whatever back-off the homing method performed, so there is nothing to
subtract from `length_mm` and nothing about homing to configure here. `get_position` and
`move_to_position` are refused until the motor reports `homed` -- which is also true when its
absolute encoder kept the reference across a restart, in which case 0 mm is the drive's retained
zero and no run is needed.

### Attributes

| Name | Type | Inclusion | Default | Description |
|------|------|-----------|---------|-------------|
| `motor` | string | **Required** | — | Name of the axis motor (also list it in `depends_on`). |
| `length_mm` | float | **Required** | — | Travel length of the axis, measured from the home reference. |
| `mm_per_rev` | float | **Required** | — | Linear travel per motor revolution. |
| `gantry_mm_per_sec` | float | Optional | `0` | Default speed when a move does not give one. |
| `travel_direction` | string | Optional | `positive` | Which way motor revolutions run as the axis travels away from its home end. Set it to `negative` when homing moves the motor in the positive direction. |

```json
{
  "name": "x-axis",
  "api": "rdk:component:gantry",
  "model": "viam:ethercat:gantry",
  "attributes": {
    "motor": "x-motor",
    "length_mm": 500,
    "mm_per_rev": 40,
    "gantry_mm_per_sec": 50,
    "travel_direction": "positive"
  },
  "depends_on": ["x-motor"]
}
```

## Third-party code

The CSP trajectory generator is built on a vendored, lightly modified copy of WPILib's
`TrapezoidProfile` (BSD 3-Clause), kept apart from project code in `src/third_party/wpilib/`
with its own license file. See `THIRD_PARTY_NOTICES.md` for that and for the build-time
dependencies (SOEM, Viam C++ SDK).

## Building

Local development build (CMake + FetchContent, clang or gcc ≥ 13):

```sh
make build          # library + module
make test           # offline test suites
```

Release packaging (Conan, linux/amd64 — what CI ships):

```sh
./bin/setup.sh      # one-time: builds the C++ SDK + dependencies
./bin/build.sh      # produces module.tar.gz
```
