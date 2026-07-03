# RDK Validation Campaign — Report

_Campaign of `docs/viam-driven-validation.md`, executed 2026-07-02/03 against the real
ANCTL AS715N (A6-EC) on `enp86s0` through viam-server + the Viam motor API (the product
path). Machine: `ethercat-test` (part `59e04139-…`). Final build under test: master
`7c9b204`. Shaft-clear blanket go-ahead applied throughout; every drive-behavior claim
below is wire-verified (tcpdump) or client-observed on the real drive._

## Verdict

The module passes the full test sequence (Tests 1–6, 23 steps) repeatedly and
recovers from every config-error scenario. The campaign found **four real bugs**
(#70, #71, #72, #73): three were fixed, gate-signed, and re-verified on hardware
during the campaign; the fourth is a **proven upstream RDK regression** with a
drafted fix. CPU usage is ~2.9% during active testing at a 1 kHz RT loop.

## Scenario outcomes

| # | Scenario (from the test doc) | Result |
|---|---|---|
| 1 | empty → valid config, Tests 1–6 | **PASS 23/23** (after finding + fixing #70) |
| 2 | empty → wrong config, error in logs | **PASS** — `control_type 'torque' is not valid (expected "PP", "PV", or "switchable")` |
| 3 | DC sync0 error, both variants | **PASS** — validation-level reject with actionable text ("Nearest valid rates: 250 Hz / 400 Hz", no bus contact); drive-level reject (free-run) surfaced as `AL 0x0027 (freerun not supported — set use_distributed_clocks=true)` in ~6 s, no drive wedge, recovery via config-cycle verified. Found #71 (was silent) — fixed twice over (see below). |
| 4 | valid → tests → reconfigure valid → tests | **PASS 23/23 + 23/23** on the final build, zero tripwire hits. The one historical failure here became #72 (see below). |
| 5 | valid → tests → wrong → error → fix → tests | **PASS except the fix step** — bug #73 (RDK regression) bricks the recovery until the module process restarts; post-restart pass 23/23. Documented hazard + workaround below. |

Test 5.1 characterization (per user decision): a client-side asyncio cancel of a
blocking `GoTo` **does not stop the motor** — the cancel never reaches the module
(no cancellation hook in the module SDK contract); the motor completed +100 revs
after the cancel. `Stop()` is the correct interrupt (verified: fails the in-flight
GoTo with "motor stopped", motor halts).

Test 6 (mid-move SDO DoCommands): `voltage_volts ≈ 312–318` (plausible rectified
220 VAC), `current_amps ≈ 0.2`, `drive_modes = [PP, PV, TQ, HM, CSP, CSV, CST]`,
motor still moving afterward — the marshaled steady-state SDO path proven on the
product path. (Note: the A6 lacks the standard 0x6079/0x6078 objects; the campaign
build read vendor 0x2040:07/:0D via config. Per subsequent user ruling, task #15
moves do_commands to standard-objects-always with 0-value fallback.)

## Bugs found (all described before fixing, per the campaign rule)

### #70 — move-after-Stop never acknowledges (FIXED, HW-verified)
`stop()` holds the CiA402 Halt bit (correct); the next `go_to` cleared Halt and
raised the new-set-point bit **in the same PDO cycle**. The A6 ignores a bit-4 edge
coincident with Halt release → every post-stop move timed out, forever. Wire-proven
(healthy handshake acks in ~2 ms; post-stop handshake never acks). Fix: bit4 rises
only after the written controlword has had Halt clear ≥2 cycles. The fix also
exposed and fixed a sibling: Stop+GoTo coalesced into one RT command batch ran the
move *under* Halt (fixed precedence) — now the latest command wins the halt
disposition while an in-flight move is still cancelled unconditionally.
Re-verified: scenario-1 re-run 23/23 including the exact previously-failing step.

### #71 — free-run bring-up failure was silent (FIXED twice, HW-accepted)
With `use_distributed_clocks=false` the DC-only A6 never energizes, but the module
surfaced nothing (empty `last_error`, no stderr) indefinitely. Two-layer root cause:
(1) the give-up path never read the EtherCAT AL status code (and mis-attributed to
the configured Er74.1 when it reported at all); (2) deeper — the give-up never
*fired*, because under free-run the A6 **zombie-PDOs**: it answers cyclic exchange
with a full working counter while its inputs read dead zero, so the WKC-only OP
gate confirmed OP (the #25 backlog prediction, exactly). Fix: OP-confirm now
requires a plausible statusword (`drive_present` hook) + the AL code is latched
(last non-zero seen during the await; the live register reads 0 by give-up time)
and surfaced with actionable text. Accepted on the bench: silent → `drive refused
OP: AL 0x0027 (freerun not supported — set use_distributed_clocks=true…)` in ~6 s,
correct attribution, clean recovery.

### #72 — one-off ~26 ms RT stall dropped DC sync (CLOSED: environmental)
A single in-place reconfigure was followed 153 s later by an instant drop from OP
(Er74.1, WKC collapse). Differential wire analysis refuted the initial
"reconfigure leaves DC marginal / drifts" theory: the hold was pristine until a
single **~26 ms gap in the master's outbound frames** (the RT thread stalled); a
clean control run carried 2× the clock drift and held 16 min. Three controlled
reconfigure re-runs all held. Directed reproduction (CPU storm; then SDO polling +
root-privileged FIFO test threads, 1490 in-window polls) failed to reproduce; the
audit found `ulimit -r 0` also weakens both candidate mechanisms for the original
event — **mechanism unidentified** (leading unfalsified candidate: firmware SMI).
Shipped anyway: the one real RT-path inversion removed (SDO servicing is now
try-lock — the RT thread can never block), a permanent tripwire (`[ethercat] RT
cycle overrun Xms` one-shot stderr), mlockall verification, a reconfigure/soak
harness (`a6_validate --cycle`), and an "RT determinism under host load" section in
`docs/deployment-capabilities.md`. Not reconfigure-specific: any host stall >~drive
sync tolerance mid-OP would do this; the drive tolerates *planned* gaps (240 ms
re-bring-up) fine.

### #73 — valid→invalid→valid config cycle bricks the servo (PROVEN RDK REGRESSION, open)
When a new config fails module validation, RDK errors the resource node **without
telling the module** (no Close/RemoveResource) — the old instance stays alive and
**energized** while unreachable by clients. The corrected config then arrives as an
Add, which collides with the never-removed instance: `Attempted to add resource
that already existed` retries forever; only a module-process restart recovers.
Proven end-to-end with a ~70-line **Go** module on a local-config viam-server
(reproduces identically — not a C++-SDK issue), with the orphan's `Close()` never
called (its own log proves it). Introduced by **rdk PR #3386 / commit `2525d53c`**
(2024-01-02, first shipped v0.18.0) — a regression: the prior code used
`GraphNode.UnsafeResource()` (the documented errored-node reconfiguration accessor)
and correctly closed/removed the old instance; #3386's new `closeAndUnsetResource`
switched to `Resource()`, which early-returns on an errored node. Carried ~2.5
years / 168 minor releases. Suggested fix: use `UnsafeResource()` in
`closeAndUnsetResource`. **Operational workaround until fixed: after a
valid→invalid→valid cycle, restart the module process (or viam-server); be aware
the axis holds torque while bricked.** Module-side safety net (de-energize the
orphan on a colliding Add) is queued. Upstream issue text drafted — filing decision
with the user.

## Also observed
- viam-server's cloud-config stream stalled once (~50 min uptime; pushes stopped
  applying; local gRPC still fine); a server restart recovered. RDK-side; not chased.
- The module warns `quick_stop_decel not set → STOP is an uncontrolled
  disable-voltage coast` — consider setting 0x6085 in production configs.
- Module CPU: ~2.9% avg during active testing (1 kHz loop + gRPC); the RT loop
  sleeps on absolute `clock_nanosleep` deadlines.

## Artifacts
- Harness: `tools/rdk-validation/` (scenario configs, Tests 1–6 runner, server/CPU
  wrappers). Wire captures + decoders + the #73 Go repro in the session scratchpad.
- Builds under test progressed with the fixes: `3969715` → `6ce0e4a` (#70) →
  `058e7b3` (#71) → `7c9b204` (final, incl. #72 closure package).
