# Offline test retirement (#17 item 12 — sim-fidelity demotion)

The offline `SimBackend` was a drive-fidelity simulator (statusword ladder, PP
set-point-ack, quick-stop deceleration ramp, fault-inject/clear FSM, mode-echo,
mode-switch latency, encoder noise). It is now a **loopback STUB** — a pipe, not a
drive — whose only charter is letting the `Master` / `ServoController` / `Runner`
CYCLE for the concurrency + plumbing suites. Every behavior that asserted on
*drive fidelity* is therefore retired from the offline suite and is proven
**HW-first**: on the bench (`a6_validate`) and in the RDK campaign
(`tools/rdk-validation/rdk_tests.py`), or deleted along with a removed feature.

The stub still models the generic-CiA402 minimum the surviving suites need: the
enable ladder to OperationEnabled, the PP set-point-ack handshake (bit4→bit12),
a position loopback + PV integrator, a conformant mode echo (0x6060→0x6061), a
conformant velocity feedback (0x606C = per-cycle delta), and a conformant
quick-stop configure gate (0x605A reads 2, 0x6085 echoes the write). None of that
is drive fidelity — it is the standard, which any conformant drive satisfies.

This table is the audit trail: every retired test/case → the HW check that now
owns that behavior, or "deleted with the feature".

## `sim_backend_test.cpp` — RETIRED WHOLE (removed from CMake)

Its entire subject was the sim's own drive modeling (statusword ladder, PP ack,
quick-stop, fault reflect, mode-echo). The stub has no drive behavior to unit-test.
→ **Deleted with the feature** (the fidelity simulator). Generic-CiA402 loopback is
exercised transitively by every controller/master suite.

## `a6_control_test.cpp` — kept T1 (CLI), retired T2–T16

| Retired case | Now owned by |
|---|---|
| DA-B mode-echo refuse (0x6061 != commanded) | `a6_validate` mode-echo gate on HW; campaign |
| PV Quick-Stop ramp-then-disable, both 0x605A regimes | `a6_validate --move-vel` bench (#53 landmark) |
| Configure refusals: 0x605A==2, 0x6085 readback, VEL guard | bench configure on the real A6 |
| P3c 0x6060-seed reaches OE; runtime PP↔PV mode-switch confirm/fail | driver mode-ensure (slice 2) on bench; campaign switchable path |
| M56S slow-device mode-switch (T_switch confirm / timeout) | **Deleted with the mode-switch FSM** (#17/#18) |
| PP rising-edge handshake + reached predicate (T10) | `controller_offline` go_to + `servo_motor_test` on the stub; drive-side ack timing on bench |

## `controller_offline_test.cpp` — rewritten (lifecycle/plumbing/concurrency kept)

| Retired case | Now owned by |
|---|---|
| #59 reached/is_moving under encoder noise (`report_noise`) | A6ServoDriver position-stability heuristic (#59, kept) + bench move |
| #67 frozen-far-from-target false-reached guard | driver is-moving/reached (slice 3) + bench |
| mode guards reject wrong API (set_rpm in PP / go_to in PV) | **Deleted with the feature** — always-switchable, no mode-reject (#18) |
| commanded rpm → 0x6081; go_to speed clamp (`received_profile_velocity`) | bench (`a6_validate --move-pos` writes 0x6081) |
| PV velocity guard clamps to stop-window budget | bench (velocity clamp under real decel) |
| PV LIFECYCLE-stop is ramp-then-disable (quick-stop ramp shape) | `a6_validate --move-vel` bench (#53 landmark) |
| M6 PV-hold locks position via PP-at-counts; unconfirmable-switch revert | **Deleted with the mode-switch FSM** (#17/#18); driver mode-ensure on bench |
| module enable-ladder mode-seed + mode-echo MISMATCH refuse | implicit (every powered stub test maps 0x6061 + passes the gate); mismatch refuse on bench |
| PP handshake-timeout abort (`suppress_setpoint_ack`) | bench (drive withholds the ack) |
| no-progress stall watchdog trips the move | **Deleted with the feature** (#17 item 9 — watchdog removed in slice 4) |
| #16 fault inject / A6 gloss / compose drive+WKC / live-read / stale-flag gate | bench + campaign fault handling |
| #16 velocity from the 0x606C wire | **KEPT** (adapted — the stub publishes 0x606C) |
| #16 unmapped 0x603F/0x606C fallback-to-estimate | **Deleted** — the fixed superset always maps both (#18), so "unmapped" is unreachable |
| live suppress_ack vs RT handshake TSan gate | **Deleted with the hook** (`suppress_setpoint_ack`) |
| #18 fault-reset Resetting FSM: reflect-latency / window / persistent / clear-then-refault / no-spin / instant (9 cases) | bench fault-reset + slice-5 bring-up retry; the in-loop bit7 reset is HW-proven |
| #70 PP move after a settled Stop (coincident halt-release / bit4 edge) | bench (task #9, wire-proven) |
| #61 switchable go_to→PP then set_rpm→PV; unconfirmable-switch revert-safe | driver mode-ensure (slice 2) on bench; **mode-switch FSM deleted** |
| #61/#64 derived-map + minimal-config end-to-end | **KEPT** (adapted to `set_fixed_pdo_map` + a stub move) |
| #22 steady-state SDO values (322 V / 1 A / 0x0185) | campaign do_command reads on the real A6; the CONCURRENCY plumbing is **KEPT** |

## `master_test.cpp` — one assertion dropped

The "full RT loop reaches OperationEnabled" case dropped its `target_reached()`
(statusword bit10) assertions — bit10-always-set was the A6 quirk (dropped model
field). The WKC-holds + reach-OP plumbing is kept. Bit10 semantics are a drive
behavior → bench.

## `module_load_test.cpp` — Part B retired

The "#16 fault legible through do_command" case drove a fault via the dropped
`inject_fault`/`set_fault_code` hooks. → bench + campaign fault handling. Part A
(config-driven construct → SimBackend → Motor API → Reconfigure → shutdown) is
kept whole.
