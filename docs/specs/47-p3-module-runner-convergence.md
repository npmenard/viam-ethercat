# #47-P3 / #37 — Viam module → Runner + SlaveControl convergence

**Status:** DRAFT — **Revision 8** (DA rev-7 sign + P3a cleared; folds DA's 4 P3b/P3c findings). Design DA-SIGNED; **P3a in progress** (task #54). No re-gate pending.
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
- **(b) slot reclaim:** `claim_motion_slot()` **reclaims a slot whose active gen is already TERMINAL** via a SINGLE CAS (`compare_exchange(expected = FREE | terminal-gen, desired = new-gen)`) — not check-then-claim (TOCTOU between two gRPC claimers). Claim reclaims-if-terminal; the waiter does not own release.
- **(c) terminal precedence:** RT sets gen N's terminal state **write-once / first-terminal-wins** (immutable); cancel of an already-completed gen = no-op. Classify: done→true | failed→throw(raw 0x603F) | stopped→throw "motor stopped" | disabled→throw "motor disabled" | timeout→throw.

---

## §4 Units — conversion is NON-RT (wrapper); the policy is PURE COUNTS

`counts = revs × counts_per_rev × gear_ratio` in the non-RT wrapper; the policy sees only counts.
**`counts_per_rev`** = encoder resolution (counts per motor rev; A6 = 131072 = 2¹⁷) — the datum that converts raw counts ↔ revs/rpm for the Viam API. Per-motor config, never code.

---

## §5 Config (regrouped)

- **bus / RT:** `ifname`, `target_loop_rate_hz`, `require_realtime`, `rt_priority`, `use_distributed_clocks`, `sync0_cycle_ns`
- **motion / units (per-motor):** `counts_per_rev`, `gear_ratio`, `max_motor_speed_rpm`
- **standard CiA402 tunables:** `quick_stop_decel` (0x6085), `quick_stop_option` (0x605A), `position_tolerance`, `zero_vel_threshold`, `mode_switch_settle_timeout` (T_switch, §6), `ramp_stop_timeout` (§6 step 1)
- **recovery (generic app-policy):** `fault_recovery_max_attempts` (=3), `fault_recovery_backoff_ms`
- **device-specific (the residual ONE):** `fault_reset_mechanism` (default bit7; A6 = vendor-sdo) — also selects recovery location (§A R2)
- **optional:** `peak_current_limit`/`rated_current` (torque-permille, absent by default)

_Live-apply vs respawn (M5):_ live-apply = an atomic POINTER-swap to an IMMUTABLE config snapshot (RT `step()`
acquire-loads the pointer ONCE at cycle-top, uses that snapshot the whole step — no in-place mutation / torn read).
**SOFT (live-appliable, no teardown, load HELD):** `position_tolerance`, `zero_vel_threshold`, `max_motor_speed_rpm`,
`fault_recovery_max_attempts`/`backoff` (all RT-consumed values, no drive-side write). **STRUCTURAL (FORCE respawn =
a documented LOAD-DROP window; brake/support first on a load axis):** `counts_per_rev`, `gear_ratio` (mis-live-applying
corrupts an in-flight move's units mid-run), `quick_stop_decel` (0x6085) — configure-time SDO write; A6 manual Effective Time = "Immediately", so a RESPAWN
re-applies it, but we treat it STRUCTURAL by POLICY (live-shrinking it under a streaming velocity breaks the PV
window-budget safety invariant — same class as counts_per_rev); `ifname`, dc/sync0/loop-rate/rt params, PDO map,
`fault_reset_mechanism`.
- **ASSERT-ONLY (neither live nor respawn can change it — operator control-power-cycle only):** `quick_stop_option`
  (0x605A) — A6 manual Effective Time = **"Upon re-power-on"** (verified, a6 manual §11.2.2 605Ah row); a warm SDO
  write is silently latched-old, and our respawn (close()→INIT keeps control power ON) is NOT a re-power-on, so it
  won't apply a change either. The driver READS + asserts the expected value at configure and refuses with
  "power-cycle the drive after changing 0x605A" if wrong — it NEVER writes it.

---

## §6 API → mode, is_moving

- `go_to`/`go_for` → PP; `set_rpm` → PV; `set_power(p)` → PV velocity = `p × max_motor_speed_rpm` (velocity-scaled, NOT torque).
- `is_moving` = ALWAYS internal `|target−actual| ≤ position_tolerance && |vel| ≤ zero_vel_threshold`.

### THE CANONICAL GENERIC MODE-SWITCH (per-command; supersedes the M6 PV→PP-specific form)
`Cmd` carries `mode`. In `step()`, when a dequeued command needs `mode ≠ current_mode`, run this **before** executing it:
```
1. PRECONDITION — motor STOPPED (|vel| ≤ zero_vel_threshold). If still moving, ramp to zero
     FIRST in the CURRENT mode; do NOT switch yet. BOUNDED by ramp_stop_timeout — if |vel| never
     reaches threshold (a load resisting stop), the switch FAILS "motor didn't stop for mode-switch"
     (don't hang the queue).                                                [never switch mid-motion]
2. SEED the new mode's RxPDO command objects to SAFE values BEFORE the switch (no-lunge):
     PP → 0x607A = THIS-cycle CycleContext.load<PositionActual> (actual counts);  PV → 0x60FF = 0
3. WRITE 0x6060 = new mode.
4. TRANSITION WINDOW — for up to T_switch (mode_switch_settle_timeout) cycles, treat 0x6061 +
     the new mode's TxPDO feedback as UNDEFINED: command NO motion, do NOT fault on mismatch yet.
     HOLDS ENERGIZED through the window — cw stays 0x0F (OperationEnabled), per R1; "no motion" ≠ de-energize.
5. CONFIRM — 0x6061 == commanded → proceed (enable the new motion). If no match within T_switch
     OR a drive error/EMCY fires during the window → mode-switch FAILED → the command's waiter
     throws "mode-switch failed" (reject / Degraded).
```
**Device variation handled GENERICALLY (not per-device code):** `mode_switch_settle_timeout` (T_switch) is a
config tunable (A6 transition short; M56S "takes time" — same knob, different value). Step 5 catches **both**
failure shapes: a SILENT `0x6061` mismatch (A6 #45 silently-ignores) AND a drive ERROR/EMCY (M56S errors on an
unsupported mode). **Provenance:** verified against the M56S/MDX+ manual §4.2.3 — a real second device that
*explicitly* requires stop-before-switch + tolerate-undefined-transition + errors-on-unsupported-mode. This is the
**generic CiA402 mode-switch contract**, not an A6 quirk — the A6 is its short-window/silent-ignore instance.
Composes with R3 (the single-in-flight slot means step 1 only drains the accepted command's own prior-mode motion)
and the rev-4 generic `0x6061==commanded` echo-check (now applied per-switch, not only at first enable).

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
  - **DONE (sub-step 5, #47-P3b M6):** on a **switch-capable** PV map (`0x6060` + `0x607A` both RxPDO-mapped) a Halt of a PV move runs the generic mode-switch (ramp PV→0 → `0x6060`=PP) and latches the **rest** position as the PP setpoint, so the drive's **position loop LOCKS the shaft** — zero **POSITION** drift under load. The seed target is the LIVE actual (latched once on the handshake's bit4 edge, which fires only *after* the stop-first ramp reaches rest → no back-jump/lunge). **Failure disposition:** if the switch can't confirm (`0x6061` never echoes PP), the module does **not** throw (internal hold, not an operator command) and does **not** de-energize — it reverts to the interim zero-**VELOCITY** hold, now commanded PV-**at-0** (`0x60FF`=0). **Fallback** (map NOT switch-capable): Halt stays the interim bit8 zero-VELOCITY hold — under load the axis drifts (safe on the no-load sim/bench). Tests: `controller_offline_test` M6 positive (switches → PP, |vel|~0, HELD, energized) + M6 failure (unconfirmed switch → stays energized, reverts to PV).
- **LIFECYCLE-stop** (resource remove / respawn-reconfigure / exit) → de-energize via Runner teardown.
- **`disable()`** → explicit **operator** de-energize (its own disposition; overrides the hold contract, S1).

**M6 — ordered PV→PP hold-switch (no lunge):** ramp vel→0 in PV → set `0x6060=PP` → **seed `0x607A`=ACTUAL counts** →
bit4 rising edge → hold. The seeded `0x607A` is THIS cycle's `CycleContext.load<PositionActual>()` (RT-fresh latched
feedback), NOT a stale published value. **Failure disposition:** mode-echo or bit4-ack fails mid-switch → **STAY in
PV-at-0 hold** (accept small drift), do NOT de-energize.

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

**The NON-RT recovery SUPERVISOR (CLASS-B + vendor-sdo) — DROP + RECONSTRUCT the one-shot Runner:**
Ownership: the **Master is WRAPPER-OWNED** (persists the whole resource lifetime); the **Runner only BORROWS it**
(`Runner(Master&)`) and is **one-shot + disposable** — no public `stop()`, move-deleted, teardown = `~Runner`
(bounded-join + `master.close()→INIT`). So recovery cannot "restart" a Runner; it **destroys + reconstructs** one.
```
RT step() on a CLASS-B / vendor-sdo fault:
   latch reason + ctx.request_stop()    // fault already dropped torque; this just makes the loop EXIT
   in-flight waiter THROWS;  last_error = raw 0x603F

NON-RT supervisor (module wrapper; Master persists here across Runner lifetimes):
   for k = 1..N (N=3, backoff ≈ 200·2^k ms):
     1. DROP the Runner  → ~Runner: bounded-join the RT thread + master.close()→INIT
          // close()→INIT happens ONLY in ~Runner — and it IS the from-INIT state the A6 PRE-OP-settle
          //   re-bring-up needs (CLAUDE.md L1). That's WHY recovery must DESTROY, not "re-start".
          // ABORT-ON-WEDGE (#52): a WEDGED step() → dtor bounded-join → std::abort → process FAIL-STOP.
          //   The supervisor recovers FAULTS, not WEDGES; a wedge is terminal, not a catchable retry.
     2. [old Runner FULLY destroyed: join + close complete]   ← NEVER-TWO-RUNNERS barrier
     3. CONSTRUCT fresh Runner(master, cfg); attach(control); start()
          // start() → on_configured: confirmed-PRE-OP settle + vendor reset 0x2031:01 + DC re-arm → OP
     4. reached & HELD OP (WKC sustained ≥ ~400 cyc, S3) → OPERATIONAL (re-energized — R2 honored)
     5. else → backoff, next attempt
   exhausted OR same-fault-latch (below) → FAULTED_LATCHED → Degraded (explicit fault_reset()/reconfigure only)
```
- **NEVER-TWO-RUNNERS invariant:** the old Runner is FULLY destroyed (RT thread joined + close→INIT complete)
  BEFORE the new one is constructed — **sequential drop→construct, never overlapped** (two live Runners on one
  Master = two threads on the port = CLAUDE.md L3, PD corruption / 0x001B). No double-teardown: `close()` lives
  ONLY in `~Runner`; `request_stop()` just flags the loop to exit.
- **RE-ATTACH = FRESH SESSION (P3b impl-nit):** the control object is WRAPPER-OWNED and PERSISTS across the
  drop+reconstruct (only the Runner is dropped). So the fresh Runner's `on_operational` MUST re-seed the control
  to a clean operational baseline — CLEAR any Faulted/Resetting sub-state + the cancelled move's in-flight slot +
  any pending generation (already terminated at the fault, §R2a); RE-CAPTURE the enable-position (the jump-avoidance
  origin, session-mechanical) from THIS session's actual feedback. The monotonic gen counter may continue (just an
  id source); no *in-flight/pending* status survives. Re-attach = fresh session — the lifecycle is re-seeded by
  `on_operational`, never inherited.
  **BUT `zero_offset_counts` (set_zero, the USER's home frame; `position() = position_counts − zero_offset`) is
  WRAPPER-PERSISTENT — it lives in the non-RT §4 unit layer and SURVIVES the drop+reconstruct.** It is DISTINCT from
  the enable-position: clearing it on recovery would silently re-zero the user's frame → `go_to(X)` would land at a
  different PHYSICAL position post-recovery. So: re-capture the enable-position (mechanical baseline); **PRESERVE
  `zero_offset_counts` (user-semantic)**; clear the in-flight/Faulted lifecycle state. **ALSO CLEAR the pending
  CommandQueue entries + the motion slot** on recovery (DA P3a finding): `commands_` persists across the
  drop+reconstruct, so a stale `set_rpm`/`set_target` enqueued after the fault would otherwise drain into SURPRISE
  motion in the recovered session — flush it as part of the fresh-session re-seed.
- **TWO bounds (transient vs wedge):** **N=3 + backoff** bounds a TRANSIENT fault (a clean re-bring-up reaches
  OP next attempt). **SAME fault recurs at OP-ENTRY → LATCH IMMEDIATELY** (don't exhaust N): each attempt is a
  full INIT→OP bounce, and an Er74-at-OP-entry fault re-triggers every attempt = the repeated-OP-entry WEDGE
  pattern; the reactive WKC==0/NO-CARRIER detector (S4) LAGS (the wedge is *caused by* the retries). The
  same-fault-at-OP-entry latch (caps OP-entry contributions to ~2) is **the real wedge guard**; S4 only stops further retries.

The CLASS-A bit7 in-loop path is unchanged (a controlword PDO edge, no Runner drop). Drop+reconstruct is for
CLASS-B / vendor-sdo — i.e. **always the A6**. The user's "auto-re-energize" is preserved as this bounded non-RT
re-bring-up; the Runner being one-shot is a FEATURE — it guarantees the `close→INIT` every attempt needs.

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
- **P3c — runtime PP↔PV mode-switch HW-verify** (§6): the one new behavior; user-gated bench. **Gate via a WIRE
  TRACE** (like #53's decel capture), not just "0x6061 confirms": the `0x6060` switch must show `0x606C`/torque
  with **NO coast / output-drop** across the switch cycles — i.e. the drive holds torque through its own mode change
  (a per-switch coast on a load axis = a position drop). Criterion: continuous `0x606C` ≈ 0, no transient spike/drop, energized throughout.
  - **P3c PREREQUISITE (sub-step 5 groundwork):** sub-step 5 maps `0x6060` (mode-of-operation, i8) into the **RxPDO**
    so the runtime switch can write it cyclically (SDO in `step()` is illegal, M1). This CHANGES the wire layout the
    #53 bring-up proved on hardware (A6 PP map 14 B → 15 B; `0x6060` is standard RxPDO-mappable, so the A6 should
    accept it). **P3c MUST FIRST re-verify A6 bring-up with the mode-in-RxPDO map** (DC → OP, WKC 3/3, no Er74) —
    add `0x6060` to `etc/a6-hardware.example.json` and confirm bring-up survives — **BEFORE** the energized
    mode-switch test. Do NOT assume the #53 bring-up carries to the new map.

**Confirmed:** PV-hold = PP-at-current-counts ✓ · fault-retry cap = 3 + backoff ✓ · ready-for-DA ✓.
**Open (small, §S2):** `set_rpm(0)` under a blocking move → throws "operation ongoing" (`halt()` is the stop verb) — confirm or prefer set_rpm(0)=stop.

**Next:** DA **re-gates** the redesigned R2 (location-by-mechanism + the supervisor-as-bounded-`Runner::start()` + #18 reuse) and the M4–M7/S1–S5 fixes → then cpp-expert starts P3a.
