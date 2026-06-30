# #47-P3 / #37 — Viam module → Runner + SlaveControl convergence

**Status:** DRAFT — **Revision 4** (DeviceProfile shrunk per your §1 pushback; §10 confirmed). **DA-ready.** No code written.
**Authors:** architect (design), team-lead (relay). **Reviewer:** _user._

> Comment with `> COMMENT: ...` or `<!-- ... -->` anywhere; I fold notes into the next revision.

### Thesis (proven, not asserted): a correct GENERIC CiA402 driver needs almost NO per-device code.
The A6 "profile" is essentially **`{ fault_reset_mechanism = vendor-sdo 0x2031:01, counts_per_rev = 131072 }`** +
standard CiA402 tunables. Everything else — stop-disposition, bounded recovery, the in-flight slot, mode-switch,
is_moving, units — is **generic behavior**. A new CiA402 drive runs by supplying ~those two facts.

### What changed across revisions (your comments → where)
- generic / report raw 0x603F → **§1** (profile shrank to ~1 knob; 4 knobs became generic behavior).
- DC0 / freerun → **§7**; config params → **§5**; counts→Viam unit → **§4**; on_configured → **§9 gloss**.
- set_power=fraction-of-max-vel, per-command PP↔PV → **§6 (locked)**; RT-spawn-fail → **§8 Degraded-alive**.
- always-energized/hold → **§A R1 two-level stop**; auto-recover → **§A R2 bounded**; single-in-flight → **§3**.
- §10 confirmed: PV-hold=PP-at-current-counts ✓, fault-retry cap=3 ✓, ready-for-DA ✓.

---

## §0 Goal

Retire `ServoController`'s hand-rolled RT loop + lifecycle FSM; drive the module through the library
`Runner` + a module `SlaveControl`, on a **generic CiA402 policy** (A6 = a tiny bit of config data). The
Viam API and the three boundary primitives stay; the RT engine, CiA402 sequencing, and lifecycle (§A) change.

---

## §1 Genericity — the device profile collapses to ~one knob  *(your §1 pushback, applied)*

**Former "profile knobs" → now GENERIC behavior (no device knob):**

| concern | rev-3 knob (DROPPED) | rev-4 generic behavior |
|---|---|---|
| sync-fault / bring-up | `sync_fault_code` (0x8700) | reach+hold OP within `bringup_timeout` (WKC-good) = SUCCESS — a transient `0x603F` (Er74.1) clears itself at OP (CLAUDE.md "don't gate OP on Er74.1"). FAILURE = didn't reach OP → report **raw 0x603F**. `sync_faulted()` is INFORMATIONAL, not a coded gate. |
| is_moving / reached | `bit10_target_reached_reliable` | ALWAYS internal: `\|target−actual\| ≤ position_tolerance && \|vel\| ≤ zero_vel_threshold`. NEVER trust statusword bit10 on any device (already the controller rule). |
| DC vs free-run | `dc_required` | attempt the configured regime; if the drive AL-rejects free-run (A6: `0x0027`) bring-up fails → **Degraded-alive (§8)** surfaces it. Don't pre-declare — let the drive reject. |
| mode set | `mode_echo_required` | ALWAYS echo-check `0x6061 == commanded`; a compliant drive always passes, a silent-ignorer (A6 #45) is caught. No knob — always verify. |

**Moved to GENERIC standard-CiA402 config** (you're right — these are STANDARD objects, not A6 quirks):
`quick_stop_decel` (`0x6085`) + `quick_stop_option` (`0x605A`) — generic optional, sensible defaults; used ONLY
by the DisableAtRest/quick-stop path. (With R1 the module HOLDS on motion-stop, so quick-stop is just the
lifecycle/emergency de-energize — central for the bench, rarely bites the module.)

**KEPT — the ONE genuine device-specific (evidence, so it's not mistaken for over-engineering):**
> **`fault_reset_mechanism` { cia402-bit7 | vendor-sdo }** — you asked "fault reset can be controlword, right?"
> For most drives, yes (bit7). **For the A6, NO:** it requires write-1-to-vendor-`0x2031:01`; **controlword bit7
> SILENTLY fails to clear an A6 fault** (CLAUDE.md lesson 4; #18 Enabling→Faulted race; #22). A silent fail here
> makes R2 auto-recovery loop-and-latch on every fault. So: generic **DEFAULT = bit7**; **A6 OVERRIDE =
> vendor-sdo `0x2031:01`**. Without it R2 cannot recover the A6 — the single legitimate fault-path device knob.

Generic code NEVER names "Er74.1"/"0x8700"; faults are reported as **raw `0x603F` hex** (label = optional gloss table).

---

## §2 The RT ↔ non-RT boundary — no API call touches the RT loop or the Master

```
  NON-RT  (gRPC / Viam API threads)              │   RT THREAD (Runner-owned, SCHED_FIFO)
  ─────────────────────────────────────────      │   ─────────────────────────────────────
  go_to go_for set_rpm set_power halt enable      │   Runner:  process() → step() → pace()
  disable fault_reset set_zero                    │   the ONLY toucher of Master / the port
        │  [unit convert: revs/rpm → counts]      │            ▲                  │
        │  submit (lock-free, never blocks)       │   drain ───┘            publish│
        ▼                                         │   ┌──────────────┐             ▼
   [ CommandQueue ]  ─────────────────────────►   │   │ step() runs   │   [ ControllerState ]
   (MPSC, lock-free; Cmd carries MODE)            │   │ GENERIC CiA402 │   atomics (pos/vel/flags/err)
                                                  │   │ policy        │   + [ PdoCache seqlock ] snapshot
  position() is_moving() is_powered()             │   └──────────────┘             │
  last_error() velocity()                         │                                │
        ▲  [unit convert: counts → revs/rpm]      │                                │
        └───────────── read-only, fail-safe ◄─────┼────────────────────────────────┘
  ══ only CommandQueue (→) and ControllerState/PdoCache (◄) cross; neither enters the RT loop ══
  ══ unit conversion (revs/rpm ↔ counts) is NON-RT, in the wrapper; the policy sees PURE COUNTS ══
```

### step() — drain → policy → publish  (RT, every cycle, `noexcept`)
```cpp
void ModuleControl::step(CycleContext& ctx) noexcept {
  Cmd c;
  while (cq_.try_pop(c)) latest_.apply(c);        // PV-setpoint coalesces; blocking moves use the slot (§3)
  if (c.mode != cur_mode_) switch_mode(c.mode);   // per-command PP↔PV: switch 0x6060 + re-echo-check
  lifecycle_.step(latest_, ctx);                  // §A: hold / move / recovering / faulted-latched + stop-disposition
  state_.publish(pos, vel, lifecycle_.flags(), lifecycle_.completed_gen());  // RT→non-RT
}
```

---

## §3 Command-submit, completion, and SINGLE-IN-FLIGHT moves *(R3)*

At most **one active motion intent.** Blocking moves (`go_to`/`go_for`, have a completion future) take an
**exclusive slot**; PV setpoints (`set_rpm`/`set_power`, continuous) are latest-wins among themselves but yield to the slot.

```cpp
future<bool> ServoMotor::go_to(rpm, pos_revs) {          // NON-RT (gRPC)
  gen = state_.claim_motion_slot();                       // THROWS "operation ongoing" if a motion is active
  cq_.submit({GoTo, PP, to_counts(pos_revs), to_cps(rpm), gen});
  return completion_future(gen);
}
```

**Exclusion matrix:**

| incoming ↓ | none active | blocking move active | PV setpoint active (≠0) |
|---|---|---|---|
| `go_to` / `go_for` | ACCEPT (claim slot) | THROW "operation ongoing" | THROW "operation ongoing" |
| `set_rpm` / `set_power` | ACCEPT (setpoint) | THROW "operation ongoing" | ACCEPT (latest-wins) |
| `stop` / `halt` | ACCEPT → HOLD | CANCEL → waiter throws "motor stopped" → HOLD | ACCEPT → HOLD |
| `enable` / `disable` / `fault_reset` | ACCEPT (control) | ACCEPT (control) | ACCEPT |

- One motion intent at a time; `stop`/`halt`/RPC-cancel cancel the in-flight move → waiter throws **"motor stopped"** → HOLD (§A R1).
- Replaces drain+coalesce for blocking moves (the slot is reject-or-cancel); PV-setpoint latest-wins survives.

**Completion notification:** RT store-release `completed_gen`/`failed_gen`/`stopped_gen` + notify; NON-RT waiter
keyed by gen classifies: done→true | failed→throw(raw `0x603F`) | stopped→throw "motor stopped" | timeout→throw.

---

## §4 Units — conversion is NON-RT (wrapper); the policy is PURE COUNTS

Viam API = **revolutions** + **RPM**. `counts = revs × counts_per_rev × gear_ratio` (`motion_profile.hpp`).
Conversion lives in the non-RT wrapper (diagram boxes); the policy + `CycleContext` see only counts.

> **`counts_per_rev`** = encoder resolution, position counts per ONE motor revolution (A6 absolute encoder =
> 131072 = 2¹⁷). The datum that converts raw counts ↔ revolutions/rpm for the Viam API. Per-motor fact → config, never code.

---

## §5 Config parameters (regrouped post-shrink)

- **bus / RT (generic):** `ifname`, `target_loop_rate_hz`, `require_realtime`, `rt_priority`, `use_distributed_clocks`, `sync0_cycle_ns`
- **motion / units (per-motor data):** `counts_per_rev`, `gear_ratio`, `max_motor_speed_rpm`
- **standard CiA402 tunables (generic, defaulted):** `quick_stop_decel` (0x6085), `quick_stop_option` (0x605A), `position_tolerance`, `zero_vel_threshold`
- **recovery tuning (generic app-policy):** `fault_recovery_max_attempts` (=3), `fault_recovery_backoff_ms`
- **device-specific (the residual ONE):** `fault_reset_mechanism` (default bit7; A6 = vendor-sdo)
- **optional:** `peak_current_limit`/`rated_current` (torque-permille `0x6072` basis is device-specific; absent by default)

_(`control_type` is gone — mode is per-command, §6.)_

---

## §6 API → mode, is_moving  *(set_power + per-command LOCKED)*

- **`go_to`/`go_for` → PP; `set_rpm` → PV; `set_power(p)` → PV velocity = `p × max_motor_speed_rpm`** (fraction-of-max-vel; velocity-scaled, NOT torque).
- **Per-command mode:** `Cmd` carries the mode; `step()` switches `0x6060` + re-echo-checks on change. _(HW-verify live A6 PP↔PV switch — §10 P3c.)_
- **`is_moving`:** ALWAYS internal — `|target−actual| ≤ position_tolerance && |vel| ≤ zero_vel_threshold`. No statusword-bit10 dependence on any device.

---

## §7 DC vs free-run

Runner treats DC-vs-free-run as a **config fact** (`step()` byte-identical). Free-run supported via config
(`use_distributed_clocks` + `sync0_cycle_ns`). No `dc_required` knob: attempt the configured regime; if the
drive rejects free-run (A6: `AL 0x0027`) bring-up fails → **Degraded-alive (§8)** surfaces the error on API calls.

---

## §8 RT-thread-creation failure → driver stays ALIVE, APIs throw

Wrapper catches all start-time failures (on_configured refusal | realtime setup | spawn | drive AL-reject) →
**Degraded-but-alive**: motion APIs throw "RT unavailable: {reason}"; accessors fail-safe; `reconfigure()`
re-attempts. `require_realtime=true` → no SCHED_FIFO is a Degraded reason; `false` → best-effort. **Never crashes the module process.**

---

## §A Lifecycle — always-energized HOLD (R1) + bounded auto-recovery (R2)

### R1 — "always energized / never un-energized": TWO levels of stop
- **MOTION-stop** (Halt / RPC-cancel / idle between moves) → **HOLD-ENERGIZED.** `step()` ramps vel→0 (PV: then
  **switch to PP at current counts** — confirmed, robust position-hold, no drift) or holds last target (PP);
  `cw` stays `0x0F` at OperationEnabled; the drive NEVER de-energizes. The R1 steady-state contract.
- **LIFECYCLE-stop** (resource remove / reconfigure / process-exit) → **DE-ENERGIZE at rest.** ONLY here the
  Runner tears down; `step()`-during-`ctx.stopping()` ramps to 0 first so close() never cuts at speed.

```cpp
enum class StopDisposition { HoldEnergized,   // module motion-stop: ramp→0 (PV→PP-hold), cw stays 0x0F
                             DisableAtRest };  // bench Ctrl-C + module lifecycle-teardown: ramp→0 then de-energize
```

### R2 — bounded auto-re-energize on fault (honors the A6-wedge lesson)
On fault: **(a) SURFACE** — in-flight waiter throws; later calls throw until resolved; `last_error` = raw `0x603F`.
**AND (b) RT loop AUTO-recovers, BOUNDED** (CLAUDE.md: repeated Er74 WEDGES the A6 → power-cycle; NEVER hammer):
```
OPERATIONAL (moving | holding)
   │ fault (sw fault bit | WKC latch | 0x603F≠0)   → in-flight waiter THROWS
   ▼
RECOVERING(attempt k = 1..N=3)
   1. bus WEDGED? (NIC NO-CARRIER / re-enum fail) → FAULTED_LATCHED NOW    [never hammer a wedge]
   2. profile.fault_reset()          // bit7 (default) OR vendor 0x2031:01 (A6) — the §1 device knob
   3. await fault-clear (bounded timeout)
   4. re-enable → OperationEnabled → resume HOLD (R1)         → OPERATIONAL
   5. else backoff(k) ≈ 200·2^k ms and retry
   budget N=3 exhausted → FAULTED_LATCHED
   ▼
FAULTED_LATCHED   // STOP auto-retry. motion APIs throw "faulted 0x{603F} — reset required";
                  // is_powered=false; cleared ONLY by explicit fault_reset() API or reconfigure()
```

> **DA pressure-test targets:** (i) does ONE reset+re-enable settle the A6, or does recovery need the
> confirmed-PRE-OP SAFE-OP settle (CLAUDE.md lesson 1)? If so a retry is a full bounce — budget for it.
> (ii) is NIC NO-CARRIER detectable mid-run (fast wedge-guard)? If not, N+backoff is the only bound.
> (iii) #18 Enabling→Faulted race — recovery must not re-trip it. (iv) the §1 `fault_reset_mechanism` evidence
> (bit7 silently fails the A6) — confirm vendor-sdo is wired for the A6 profile.

---

## §9 Hook mapping (generic mechanism → SlaveControl hook; API plumbing in the wrapper)

> `on_configured(ConfigContext&)`: Runner calls it ONCE, NON-RT, **before** spawning the RT thread — the home
> for configure-time SDO reads/writes + fail-closed validation. Throwing aborts `start()` with nothing spawned.

| piece | hook |
|---|---|
| field resolve, quick-stop SDO setup (0x6085/0x605A if used) | `on_configured` (throwing, pre-spawn) |
| seed FSM, capture enable-position, **always echo-check 0x6061==commanded** | `on_operational` |
| drain, mode-switch, generic policy, in-flight slot, R1 hold/stop-disposition, R2 recovery, publish | `step` |
| bring-up: report **raw 0x603F** (informational, not a coded gate) | `sync_faulted` |
| StopReason → two-tier fault + cancel completion gens | `on_stop` |
| **Viam API, units, CommandQueue, completion futures, in-flight slot, Degraded-state, reconfigure** | **module wrapper (non-RT)** |

---

## §10 Phasing & status

**Phasing (each independently testable):**
- **P3a — Runner adoption, behavior-PRESERVING.** Hand-rolled loop → `Runner` + module `SlaveControl`
  reproducing today's behavior; deletes the hand-rolled RT loop + unbounded join; adds Degraded-alive (§8).
- **P3b — generic policy + A6 profile + R1/R2/R3 lifecycle.** Build the generic policy + the (tiny) profile;
  module `SlaveControl` AND a6_validate `A6Control` both wrap it. Gate: policy sim tests + module API tests + HW bench.
- **P3c — runtime PP↔PV mode-switch HW-verify** (§6): the one behavior new vs the fixed-mode proven path; user-gated bench.

**Confirmed (rev 4):** PV-hold = PP-at-current-counts ✓ · fault-retry cap = 3 + exp backoff ✓ · **Ready for DA: YES.**

**Next:** DA pressure-tests R1 (hold-vs-disable), R2 (bounded-retry/wedge + the fault-reset-mechanism evidence +
the SAFE-OP-settle question), R3 (exclusion matrix), and the bring-up "reach-OP, don't-gate-on-Er74.1" → then cpp-expert starts P3a.
