# #47-P3 / #37 — Viam module → Runner + SlaveControl convergence

**Status:** DRAFT — **Revision 5** (folds DA's design-gate: 7 must / 5 should / 3 nits). Pending DA **re-gate** of the R2 redesign. No code written.
**Authors:** architect (design), team-lead (relay). **Reviewer:** _user._

> Comment with `> COMMENT: ...` / `<!-- ... -->` anywhere.

### Thesis (proven): a correct GENERIC CiA402 driver needs almost NO per-device code.
The A6 "profile" ≈ **`{ fault_reset_mechanism = vendor-sdo 0x2031:01, counts_per_rev = 131072 }`** + standard CiA402
tunables. Everything else is generic behavior. `fault_reset_mechanism` also selects fault-recovery **location** (§A R2).

### Revision history (your comments → where)
- generic / raw 0x603F / profile-shrink → **§1**; DC-freerun → **§7**; config → **§5**; units → **§4**.
- set_power=fraction-of-max-vel, per-command PP↔PV → **§6**; RT-spawn-fail → **§8 Degraded-alive**.
- always-energized → **§A R1**; auto-recover → **§A R2**; single-in-flight → **§3**.
- **rev 5 (DA gate):** R2 redesign (recovery location-by-mechanism + fault-class split, M1/M2/M3); R1 honesty + reconfigure-load-drop + PV→PP no-lunge (M4/M5/M6); R3 completion races (M7); S1–S5; N1–N3.

---

## §0 Goal

Retire `ServoController`'s hand-rolled RT loop + lifecycle FSM; drive the module through the library `Runner` +
a module `SlaveControl`, on a **generic CiA402 policy** (A6 = tiny config). The Viam API + the three boundary
primitives stay; the RT engine, CiA402 sequencing, and lifecycle (§A) change.

---

## §1 Genericity — profile collapses to ~one knob

Former "profile knobs" → now generic behavior: `sync_fault_code` → bring-up reports **raw 0x603F** on failure;
`bit10` → **always** `|target−actual|≤tol && |vel|≤zvel`; `dc_required` → let the drive **AL-reject** free-run → Degraded;
`mode_echo_required` → **always** echo-check `0x6061==commanded`. `0x6085`/`0x605A` are **standard CiA402** tunables.

**The ONE genuine device-specific:** `fault_reset_mechanism { cia402-bit7 | vendor-sdo }`. For the A6, bit7 SILENTLY
fails — it requires vendor `0x2031:01` (CLAUDE.md L4; #18/#22/#39). Per §A R2 this knob ALSO selects recovery
**location** (in-RT vs non-RT). Generic code never names "Er74.1"; faults report as raw `0x603F` hex.

---

## §2 The RT ↔ non-RT boundary — no API call touches the RT loop or the Master

```
  NON-RT  (gRPC / Viam API threads)              │   RT THREAD (Runner-owned, SCHED_FIFO)
  go_to go_for set_rpm set_power halt enable      │   Runner:  process() → step() → pace()
  disable fault_reset set_zero                    │   the ONLY toucher of Master / the port
        │  [unit convert: revs/rpm → counts]      │            ▲                  │
        │  submit (lock-free, never blocks)       │   drain ───┘            publish│
        ▼                                         │   ┌──────────────┐             ▼
   [ CommandQueue ]  ─────────────────────────►   │   │ step()        │   [ ControllerState ]
   (MPSC, lock-free; Cmd carries MODE)            │   │ GENERIC CiA402 │   atomics (pos/vel/flags/err/gen)
   ┌───────────────────┐  watches latched-fault   │   │ policy        │   + [ PdoCache seqlock ]
   │ recovery SUPERVISOR│◄─ flag (§A R2 class-B) ──│   └──────────────┘             │
   │ bounded Runner.start()                        │                                │
   position() is_moving() is_powered()  ◄──────────┼────────────────────────────────┘
        ▲  [unit convert: counts → revs/rpm]  read-only, fail-safe on staleness
  ══ CommandQueue (→) + ControllerState/PdoCache (◄) cross; the supervisor is NON-RT (restarts the Runner) ══
```

`step()` (RT, `noexcept`): `drain+coalesce → if mode change switch_mode → lifecycle_.step(latest_, ctx) → publish`.
**Only a controlword (PDO) edge is legal in step(); SDO + EtherCAT-state-recovery are NON-RT** (runner.hpp:102).

---

## §3 Command-submit, completion, SINGLE-IN-FLIGHT + cancellation *(R3, M7/S1/S2)*

One active motion intent. Blocking moves (`go_to`/`go_for`) take an exclusive slot; PV setpoints (`set_rpm`/`set_power`) are latest-wins but yield to the slot.

**Exclusion matrix:**

| incoming ↓ | none active | blocking move active | PV setpoint active (≠0) |
|---|---|---|---|
| `go_to` / `go_for` | ACCEPT (claim slot) | THROW "operation ongoing" | THROW "operation ongoing" |
| `set_rpm` / `set_power` | ACCEPT | THROW "operation ongoing" (set_rpm(0) too — **`halt()` is the stop verb**, S2) | ACCEPT (latest-wins) |
| `stop` / `halt` | ACCEPT → HOLD | CANCEL → waiter throws "motor stopped" → HOLD | ACCEPT → HOLD |
| `disable` | ACCEPT → de-energize | **CANCEL → waiter throws "motor disabled"** → de-energize (operator override, S1) | ACCEPT → de-energize |
| `enable` / `fault_reset` | ACCEPT (control) | ACCEPT | ACCEPT |

**Completion + slot races (M7 — get these right):**
- **(a) lost wakeup:** the gen-keyed waiter uses a **predicate loop** — `while(!terminal(gen)) cv.wait(...)` — re-checking completed/failed/stopped_gen AFTER arming, so a notify between check-and-sleep isn't lost.
- **(b) slot reclaim:** `claim_motion_slot()` **reclaims a slot whose active gen is already TERMINAL** (else a 2nd go_to spuriously throws "operation ongoing" on a finished move). Claim reclaims-if-terminal; the waiter does not own release.
- **(c) terminal precedence:** RT sets gen N's terminal state **write-once / first-terminal-wins** (immutable); cancel of an already-completed gen = no-op. Classify: done→true | failed→throw(raw 0x603F) | stopped→throw "motor stopped" | disabled→throw "motor disabled" | timeout→throw.

---

## §4 Units — conversion is NON-RT (wrapper); the policy is PURE COUNTS

`counts = revs × counts_per_rev × gear_ratio` in the non-RT wrapper; the policy sees only counts.
**`counts_per_rev`** = encoder resolution (counts per motor rev; A6 = 131072 = 2¹⁷) — the datum that converts raw counts ↔ revs/rpm for the Viam API. Per-motor config, never code.

---

## §5 Config (regrouped)

- **bus / RT:** `ifname`, `target_loop_rate_hz`, `require_realtime`, `rt_priority`, `use_distributed_clocks`, `sync0_cycle_ns`
- **motion / units (per-motor):** `counts_per_rev`, `gear_ratio`, `max_motor_speed_rpm`
- **standard CiA402 tunables:** `quick_stop_decel` (0x6085), `quick_stop_option` (0x605A), `position_tolerance`, `zero_vel_threshold`
- **recovery (generic app-policy):** `fault_recovery_max_attempts` (=3), `fault_recovery_backoff_ms`
- **device-specific (the residual ONE):** `fault_reset_mechanism` (default bit7; A6 = vendor-sdo) — also selects recovery location (§A R2)
- **optional:** `peak_current_limit`/`rated_current` (torque-permille, absent by default)

_Live-apply vs respawn (M5):_ tolerances / max_speed / gear / recovery-N — **live-apply** (atomic swap, no teardown, load HELD). Bus/drive params (ifname / dc / sync0 / 0x6085·0x605A SDO / PDO map) — **respawn** (teardown → **a documented LOAD-DROP window**; brake/support first on a load axis).

---

## §6 API → mode, is_moving

- `go_to`/`go_for` → PP; `set_rpm` → PV; `set_power(p)` → PV velocity = `p × max_motor_speed_rpm` (velocity-scaled, NOT torque).
- Per-command mode: `Cmd` carries mode; `step()` switches `0x6060` + re-echo-checks on change. (HW-verify A6 PP↔PV — §10 P3c.)
- `is_moving` = ALWAYS internal `|target−actual| ≤ position_tolerance && |vel| ≤ zero_vel_threshold`.

---

## §7 DC vs free-run

Runner treats DC-vs-free-run as config (`step()` byte-identical). No `dc_required` knob: attempt the configured
regime; a drive that rejects free-run (A6: `AL 0x0027`) fails bring-up → **Degraded** carrying **the AL code + an
operator hint** (S5: "0x0027 = free-run not supported — set use_distributed_clocks").

---

## §8 RT-thread / bring-up failure → driver stays ALIVE, APIs throw

Wrapper catches all start-time failures (on_configured refusal | realtime setup | spawn | drive AL-reject) →
**Degraded-but-alive**: motion APIs throw "{reason}" (AL-rejects carry code + hint, S5); accessors fail-safe;
`reconfigure()` re-attempts. Never crashes the module process.

---

## §A Lifecycle — energized-HOLD (R1) + bounded recovery (R2)  *(redesigned per DA)*

### R1 — "always energized EXCEPT during a fault" (M4: the honest contract)
A fault is an INVOLUNTARY de-energize (the drive drops its own torque); R2's job is re-energize-or-latch. Otherwise, two levels of stop:
- **MOTION-stop** (Halt / cancel / idle) → **HOLD-ENERGIZED**: ramp vel→0; PV then **switches to PP-at-current-counts** (M6, below); `cw` stays `0x0F`; never de-energizes.
- **LIFECYCLE-stop** (resource remove / respawn-reconfigure / exit) → de-energize via Runner teardown.
- **`disable()`** → explicit **operator** de-energize (its own disposition; overrides the hold contract, S1).

**M6 — ordered PV→PP hold-switch (no lunge):** ramp vel→0 in PV → set `0x6060=PP` → **seed `0x607A`=ACTUAL counts** →
bit4 rising edge → hold. **Failure disposition:** mode-echo or bit4-ack fails mid-switch → **STAY in PV-at-0 hold**
(accept small drift), do NOT de-energize.

### R2 — recovery LOCATION by mechanism × fault CLASS (M1/M2/M3)
**Rule: only a controlword (PDO) edge is legal in step(); SDO reset + EtherCAT-state recovery are NON-RT.**

| | CLASS A: fault, drive HOLDS OP (WKC good) | CLASS B: drive LEFT OP (state-loss / Er74-OP-entry) |
|---|---|---|
| `bit7` | **IN-LOOP** (cw bit7 is a PDO field) via the #18 machine | **NON-RT supervisor** |
| `vendor-sdo` (A6) | **NON-RT supervisor** (SDO illegal in step(), M1) | **NON-RT supervisor** |

⇒ **the A6 always recovers via the non-RT supervisor.** **NEVER auto-retry CLASS B in-loop** — repeated OP-entry IS the wedge-hammering CLAUDE.md forbids (Er74 → NO-CARRIER → power-cycle).

**In-loop CLASS-A (bit7) recovery REUSES the proven #18 machine** (M3, servo_controller.cpp:466-485):
bit7 rising EDGE (edge-count==1, no level-spin) → fault must STAY clear K cycles → then →Enabling; re-fault during
the hold → revert to Faulted WITHOUT re-spinning.

**The NON-RT recovery SUPERVISOR (CLASS-B + vendor-sdo) — a bounded retry around `Runner::start()`:**
```
RT step() on a CLASS-B / vendor-sdo fault:
   latch reason + ctx.request_stop()          // fault already dropped torque; teardown → close()→INIT
   in-flight waiter THROWS;  last_error = raw 0x603F

NON-RT supervisor (module wrapper; watches the latched recoverable-stop reason):
   for k = 1..N (N=3, backoff ≈ 200·2^k ms):
     1. WEDGE? (NIC NO-CARRIER link-state | WKC==0 sustained, S4) → FAULTED_LATCHED now
     2. Runner.start()   // == the ec_sample recovery: on_configured does the confirmed-PRE-OP settle
                        //    (CLAUDE.md L1) + vendor reset 0x2031:01 + DC re-arm → OP
     3. reached & HELD OP (WKC sustained ≥ ~400cyc, S3) → OPERATIONAL (re-energized — R2 honored)
     4. else backoff, retry
   exhausted → FAULTED_LATCHED → Degraded (cleared only by explicit fault_reset()/reconfigure())
```
Each attempt IS the full proven bring-up (the PRE-OP settle + vendor reset already live in `on_configured`). The
user's "auto-re-energize" is **preserved** — as a bounded non-RT re-bring-up, not an in-RT cw edge — honoring both
the ask and the wedge lesson. Wedge-detect (S4) only STOPS further retries; **M2 (don't retry class-B in-loop) is the real guard.**

---

## §9 Hook mapping

| piece | hook |
|---|---|
| field resolve, quick-stop SDO setup, **vendor fault-reset on recovery** | `on_configured` (NON-RT, pre-spawn, throwing) |
| seed FSM, capture enable-position, **always echo-check 0x6061==commanded** | `on_operational` |
| drain, mode-switch, generic policy, in-flight slot, R1 hold, **CLASS-A bit7 in-loop recovery (#18)**, publish | `step` (PDO-only) |
| bring-up: report **raw 0x603F** (informational) | `sync_faulted` |
| StopReason → two-tier fault + cancel completion gens | `on_stop` |
| **Viam API, units, CommandQueue, completion futures, in-flight slot, the recovery SUPERVISOR, Degraded, reconfigure** | **module wrapper (non-RT)** |

---

## §10 Phasing & status

- **P3a — Runner adoption, behavior-PRESERVING.** Hand-rolled loop → `Runner` + module `SlaveControl`; deletes the hand-rolled RT loop + unbounded join; adds Degraded-alive (§8).
- **P3b — generic policy + A6 profile + R1/R2/R3 lifecycle.** Module `SlaveControl` AND a6_validate `A6Control` both wrap the generic policy. **Gate: the 10 #53 `a6_control_test` cases MUST all survive (generic policy + A6 profile) — the regression guard (N2)** + module API tests + HW bench.
- **P3c — runtime PP↔PV mode-switch HW-verify** (§6): the one new behavior; user-gated bench.

**Confirmed:** PV-hold = PP-at-current-counts ✓ · fault-retry cap = 3 + backoff ✓ · ready-for-DA ✓.
**Open (small, §S2):** `set_rpm(0)` under a blocking move → throws "operation ongoing" (`halt()` is the stop verb) — confirm or prefer set_rpm(0)=stop.

**Next:** DA **re-gates** the redesigned R2 (location-by-mechanism + the supervisor-as-bounded-`Runner::start()` + #18 reuse) and the M4–M7/S1–S5 fixes → then cpp-expert starts P3a.
