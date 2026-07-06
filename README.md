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

Add a motor component with one of the models above. Attributes:

| Attribute | Required | Default | Meaning |
|---|---|---|---|
| `interface` | yes | — | EtherCAT NIC name (e.g. `enp3s0`) |
| `max_rpm` | yes | — | motor speed clamp (nameplate rated speed) |
| `counts_per_rev` | yes | — | encoder counts per motor revolution |
| `motor_rated_current_amps` | yes | — | nameplate rated current |
| `slave` | no | 1 | 1-based position of the drive on the bus |
| `gear_ratio` | no | 1.0 | motor revolutions per output revolution |
| `position_tolerance_counts` | no | counts_per_rev/720 | "reached/stopped" stability band |
| `loop_rate_hz` | no | 1000 | cycle rate (1..1000) |
| `use_distributed_clocks` | no | false | DC/SYNC0 sync; required by DC-only drives |
| `sync_cycle_granularity_ns` | no | 0 | drive's SYNC0 cycle granularity; a bad `loop_rate_hz` is then rejected at config time |
| `require_realtime` | no | true | hard-fail at load without SCHED_FIFO |
| `rt_priority` | no | 80 | SCHED_FIFO priority (1..99) |

There is no PDO-map or control-mode configuration: the driver defines one fixed
CiA402 map and switches between profile-position and profile-velocity at
runtime per API call.

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

### Behavior notes

- `stop()` is a CiA402 Halt: the motor ramps to standstill and **holds position
  energized**. Use the `disable` do_command to de-energize (coast).
- On bus loss (cable pull, NIC down) the module stays alive: reads return
  fail-safe values, motion calls return a clear error, and the next motion call
  after the link returns rebuilds the connection automatically.
- `do_command` verbs: `status`, `fault_reset`, `enable`, `disable`,
  `get_motor_voltage`, `get_motor_current_actual_value`,
  `get_motor_drive_modes`.

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
