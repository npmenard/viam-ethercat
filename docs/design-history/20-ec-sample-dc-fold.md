# Spec #20 — Fold the ec_sample working DC sequence into Master::configure() + the one RT loop

**Status:** ready for cpp-expert (after #19 lands the v2 base). Architect-owned.
**Source of truth:** `~/SOEM/samples/ec_sample/ec_sample.c` (brings THIS A6 to OP in DC, WKC 3/3) + `CLAUDE.md`. **Diff against ec_sample + capture the wire (tcpdump) before permuting configs** — that's the methodology that ended the 15-iteration wall.
**Big idea:** v2's `config_map_group` + stock `ecx_dcsync0` + our single-RT-I/O-thread architecture make the DC bring-up **simple**. Most of #17's hand-rolled machinery was working around v1.4.0 + a self-inflicted bug; this spec **deletes** it. See §1.

---

## 0. THE invariant (enshrined, non-negotiable): single port owner = the RT thread

**ALL EtherCAT port I/O — process data AND mailbox/SDO — goes through the single current port owner, and once the RT thread is running that owner is the RT thread.** (Temporal: `configure()` owns the port on the lifecycle thread *before* the RT thread spawns; once spawned, the RT thread is the sole owner; **never both concurrently**.) The ec_sample 0x001B failure (~40% OP-fail) was a 2nd thread running `ecx_statecheck` (no-sleep BRD flood) concurrently with the PD thread on the non-thread-safe SOEM port → the flood starved LRW → output-SM watchdog `AL 0x001B`. Our design avoids this by construction: the non-RT API side only ever touches **published atomics**, never the port.

**Consequence (load-bearing for the SDO/fault-reset path):** runtime SDO operations (the vendor fault-reset `0x2031:01`, diagnostics) **CANNOT** be issued from the non-RT API thread while the RT loop runs — that would be two threads on the port. They must be **QUEUED to the RT loop** (across the existing command-queue boundary), and the RT loop services the mailbox (`ecx_mbxhandler`) + the SDO **in-loop**. This is *why* v2's cyclic `ecx_mbxhandler` exists and why it lives in the RT loop, not a public backend method. Any `ecx_*` port I/O off the RT thread (a "quick status read", a concurrent statecheck, a direct runtime SDO) re-introduces 0x001B. This invariant is *why* our architecture beats the reference — keep it absolute.

### 0.1 `ecx_mbxhandler` + runtime SDOs — NOT needed for first light (librarian, source-verified)
- **First light (#20 + #21): `exchange()` stays PDO-only — NO `ecx_mbxhandler`.** Per the librarian's v2 delta: our runtime is PDO-only (statusword/fault-code via TxPDO-mapped `0x6041`/`0x603F`), and the only blocking SDOs (PDO-remap + the bring-up vendor reset) run single-threaded in `configure()` where `ecx_SDOwrite` self-services the mailbox (mbxpool inited by `ecx_init`). So neither `ecx_mbxhandler` nor `ecx_slavembxcyclic` is required to reach or hold OP. `exchange()` = send + receive processdata, unchanged. (Even simpler than mirroring ec_sample, which calls mbxhandler each cycle — we don't need to.)
- **Steady-state runtime SDO (the ONLY case that ever needs mbxhandler) = #22, post-first-light:** an in-OP async SDO (the steady-state vendor fault-reset) is **queued to the RT loop** (a command-queue entry carrying `{index, sub, value}`) and serviced **in-loop** via `ecx_mbxhandler` — which the librarian confirmed is RT-compatible in the PD envelope (no-alloc, bounded by its `limit` arg, timed datagram I/O + one uncontended leaf mutex, same class as the PD exchange) **provided it's called only from the RT thread** (never concurrent — §0). Deferred to #22; do NOT build it for first light.

---

## 1. Simplification thesis — what #17 built that #20 DELETES

CLAUDE.md disproved the premises behind #17's DC gate. With v2 + ec_sample proven, **delete:**
- **The hand-rolled SYNC0 arm** (`arm_dc_sync` ESC-register writes, lines 314–383) → replace with **stock `ecx_dcsync0`**. The `start_delay_ns`/SyncDelay workaround is moot with stock v2 (CLAUDE.md §5).
- **The `0x0984/0x098E` "armed/pulsing" gate** (`dc_sync_status`) → those registers are **PDI-consumed; they read 0 from the master even when SYNC0 is firing** (CLAUDE.md). The whole `DcSyncStatus`/`dc_sync_status()` FPRD-poll mechanism gets **removed** (§7).
- **The cycle-converge gate** (#17's `0x1C32:02 → 1e6` wait, 1db6ed9) → `0x1C32:02` is **NOT** the cause of anything; it's 0 in SAFE-OP, only measured AT OP (CLAUDE.md §2). Delete the gate.
- **The manual `0x1C32:01=2` force** → **NEVER write it.** That was the self-inflicted `AL 0x0030`. v2's `config_map_group` makes the drive self-select DC (reads back `0x0002`).
- **The elaborate "arm-after-lockStreak≥200 + verify-not-wait"** → unnecessary. ec_sample arms early and works; our in-loop arm (§3) just needs PD flowing, not a 200-cycle lock proof.

**What SURVIVES from #17/the (b) design:** the single continuous RT loop owning all frames, the `ec_sync` PI phase-lock (`dc_sync.hpp`, keep), the gapless SAFE-OP→OP transition, the PRE-arm/POST-arm regime split, and `#18`'s fault-reset Resetting (with the vendor-SDO wrinkle, §6). The skeleton was right; the DC-gate internals were chasing wrong signals.

---

## 2. `Master::configure()` — non-RT setup (ends at SAFE-OP)

**⚠ CORRECTED BY BENCH (commit 976b5d9) — the arm is in PRE-OP, see §5.** The original sequence below (arm-after-SAFE-OP) reached OP then Er74.1'd ~1s in with `0x1C32:01=1` (SM-sync); the A6 latches its sync-type at the PRE-OP→SAFE-OP transition from whether SYNC0 is **already armed**. The SHIPPED `configure()`, replicating ec_sample's exact order, NO `0x1C32:01` write anywhere:
```
config_init        -> PRE-OP
preop SDO writes   (drive-tuning)
apply PDO map SDO  (0x1600/0x1A00 entries + 0x1C12/0x1C13 assign)   [existing remap sub-protocol]
arm_dc_sync        (stock ecx_dcsync0(TRUE,cycle,shift) per slave -- IN PRE-OP, BEFORE the map; drive self-selects DC)
config_map_group   (v2; builds the IOmap)
configdc           (configure_dc_configdc: reference clock + 0x0920 offset + 0x0928 delay -- AFTER the map, ec_sample order)
reach SAFE-OP      (request_state) ; then fault_reset clear (vendor SDO, §6) -- still single port owner, pre-RT-spawn
```
configure() **arms SYNC0 in PRE-OP** (so the drive latches sync-type=DC at the SAFE-OP transition) but does **NOT** request OP — it returns at SAFE-OP. The no-PD window between the PRE-OP arm and the RT loop's first pump is **harmless because the first SYNC0 edge is ~100 ms out** (the stock `ecx_dcsync0` SyncDelay), and config_map_group + configdc + the RT-loop's first pumps all complete well before that first edge — so PD is flowing before any SYNC0 pulse is due. The single-port-owner invariant holds: the arm runs on the lifecycle thread, pre-RT-spawn.

---

## 3. The RT loop — single-thread, gapless bring-up (the heart of #20)

One thread, owns every frame from cycle 0. The bring-up is a prelude inside the loop, then steady control:
**SHIPPED FSM (bench-finalized — SYNC0 armed in PRE-OP by `configure()`, §5; gate per §4):**
```
caller owns clock_nanosleep + dc_phase_correction each cycle; bringup_step() does exchange() + advances:
  [SYNC0 already armed in PRE-OP by configure() -- §5; nothing to arm in the loop]
  SETTLE:   pump phase-locked PD a bounded settle (~dc_op_gate_cycles, ec_sample's ~400ms);
            NOT gated on Er74.1 (NORMAL in SAFE-OP, §4); -> request OP ONCE -> AWAIT_OP
  AWAIT_OP: keep pumping; hold for (state==OP && WKC-full && Er74.1-CLEARED) ~5 cycles -> OPERATIONAL;
            else after kAwaitOpBound(500) cycles -> ABORT (no re-request; surface via #16 last_error)
  OPERATIONAL: hand to the CiA402 lifecycle FSM (step_lifecycle); steady loop phase-locks EVERY cycle
```
- **PD flows every cycle from cycle 0** — including across SETTLE→request-OP→AWAIT_OP→OP. SYNC0 was armed in PRE-OP (configure, §5); the gapless requirement is the **SAFE-OP→OP transition** (watchdog live), which AWAIT_OP keeps pumping through. The PRE-OP arm window is SyncDelay-covered (§5).
- **PRE-arm / POST-arm regime** (from #17): the only blocking ops (SDO, request_state, **the PRE-OP `ecx_dcsync0` arm**) are in `configure()` *before* the loop (single port owner, pre-RT-spawn); everything in the loop is non-blocking cyclic PD. Overrun recovery: phase-preserving catch-up (`while next<=now: next+=period`) on BOTH the bring-up and steady loops — never a rebase, since the steady state is phase-locked to a live SYNC0.
- This bring-up prelude is **factored for reuse** (#21): the same loop drives both the ServoController (then continues to steady control) and the thin test program (then just streams feedback). See §9.

---

## 4. The OP-gate — settle → request OP → hold-on-(WKC-full + Er74.1-CLEARED) (bench-finalized)

Computed **in the RT loop from data it already has** — no acyclic FPRD, no `dc_sync_status`. **This section was corrected TWICE by the bench; the final form below is what shipped (45d0b50) + is bench-validated.** Two non-obvious facts the bench taught (CLAUDE.md: capture the wire):

1. **WKC-full cannot hold in SAFE-OP** (output SyncManager inactive pre-OP → `wkc < expected` until OP) — so WKC-full can't be a pre-OP gate; it's the OP *confirmation*.
2. **Er74.1 (`0x603F == 0x8700`) in SAFE-OP is NORMAL, not a failure** — ec_sample reads `0x8700` in SAFE-OP, the vendor reset doesn't clear it there, and it **requests OP anyway → reaches OP → Er74.1 clears AT OP.** So the "synced" signal is **Er74.1 CLEARING at OP**, NOT its absence in SAFE-OP. Gating the pre-OP request on `!drive_sync_faulted` (my earlier wording) would never pass / would falsely abort — the drive *always* shows Er74.1 until it's actually in synced OP.

**The shipped `bringup_step` FSM (SYNC0 already armed in PRE-OP per §5):**
- **SETTLE (pre-OP):** pump phase-locked PD a bounded settle (`dc_op_gate_cycles`, ~400 to match ec_sample's ~400 ms RT-PD settle; non-DC = 1). **NOT gated on Er74.1** (it's normal here). → **request OP ONCE.**
- **AWAIT_OP:** hold for **(`state == OP` && `wkc == expected_wkc` && Er74.1-CLEARED)** sustained ~5 cycles → `Operational`. Er74.1 clearing + WKC-full is the real "synced and holding" proof. `drive_sync_faulted` (#16's live drive tier, `0x603F == 0x8700`, passed into `bringup_step` — keeps Master CiA402-free) is used HERE (must clear), not in SETTLE.
- **ABORT:** if AWAIT_OP doesn't reach the held-synced state within `kAwaitOpBound` (500 cycles), OP didn't take → `Aborted`, **NO re-request** (a SINGLE OP request with Er74.1 present is safe — ec_sample does it; *hammering* re-requests is what wedges the A6 → NO-CARRIER → control-power cycle, CLAUDE.md). Surface via #16's `last_error`. Recovery is reconfigure/restart, never a tight re-bring-up loop.

So the corrected gate: **a settle (not a no-Er74.1-wait) decides when to request OP; the OP-confirmation is WKC-full + Er74.1-cleared held a few cycles; a single bounded request, never hammered.** #16's drive tier still provides the Er74.1 signal — just at the OP-confirmation (must-clear), the inverse of "no-Er74.1 pre-OP." No new mechanism, no FPRD poll.

This is a clean synthesis: **#16's drive-fault legibility provides #20's pre-OP gate signal; WKC-full is the OP-confirmation.** No new mechanism, and the wrong-signal FPRD poll is deleted.

---

## 5. Arm placement: PRE-OP, before config_map_group (ec_sample order) — BENCH-CORRECTED

**This section originally specified arm-in-loop; the bench (976b5d9) disproved it. The truth: arm in PRE-OP, before `config_map_group`, exactly as ec_sample does.**

My original reasoning was: ec_sample's PRE-OP arm works only because it never gaps PD after the arm, so our `reach_op=false` split would risk an Er74.1 from the post-arm no-PD window → therefore arm in-loop. **That was a wrong theory, killed by the bench (CLAUDE.md methodology — capture the wire, don't reason blind):**
- **The real constraint is sync-type latching, not a PD gap.** The A6 latches its SM sync-type (SM vs DC) at the **PRE-OP→SAFE-OP transition, from whether SYNC0 is ALREADY armed.** Arm-in-loop (after SAFE-OP) → the drive had already chosen SM-sync (`0x1C32:01=1`) → SYNC0 never truly established → reached OP, then Er74.1 ~1s in. tcpdump vs ec_sample (same drive): ours `0x1C32:01=1`, ec_sample `=2` (DC) though neither writes it. (The ARMW/FRMW frame-count theory was a red herring — ec_sample shows 0 DC frames too.)
- **The post-arm no-PD window I feared is harmless:** the first SYNC0 edge is ~100 ms out (stock `ecx_dcsync0` SyncDelay), and config_map_group + configdc + the RT loop's first pumps all complete well before that edge — so PD is flowing before any pulse is due. No missed edge, no watchdog trip.

**SHIPPED:** `configure()` arms `ecx_dcsync0` in PRE-OP (before the map); `arm_dc_sync` drops its `hasdc` guard (hasdc isn't set until `config_map_group`/`configdc`; ec_sample arms unconditionally, and `configdc` still validates DC-capability afterward, throwing if no DC slave). The `bringup_step` FSM correspondingly **loses the SETTLE and ARM phases** — it starts at GATE (PD-flowing + no-Er74.1 K cycles) → request OP → AWAIT_OP. The blessed gate-phasing (§4: no-Er74.1 pre-OP, WKC-full at OP), single-thread (arm on the lifecycle thread pre-spawn), gapless-through-OP, and abort-no-hammer are all unchanged. (Lesson: the "gapless" invariant is specifically about the **SAFE-OP→OP** transition where the sync watchdog is live — NOT the PRE-OP arm window, which the SyncDelay covers.)

---

## 6. Vendor fault-reset `0x2031:01 = 1` — config-data, A6-specific (+ #18 interaction)

CLAUDE.md §4: the A6 fault-reset is **write `1` to `0x2031:01`** (a CoE SDO), **NOT** CiA402 controlword bit7. This is **A6-specific → lives as config-data**, never hardcoded:
- **Config:** an optional `fault_reset` descriptor in `ServoConfig` (e.g. `{ index, sub, value }` for a vendor-SDO reset; absent ⇒ generic CiA402 bit7). The A6 hardware JSON sets `0x2031:01 = 1`. Generic CiA402 drives leave it absent and use bit7.
- **RT interaction (per §0 single-port-owner; flag for a #18 revisit):** a vendor-SDO reset is a mailbox round-trip — by §0 it can NEVER be issued from a non-RT thread while the RT loop runs, and it is NOT the cyclic bit7 edge #18's `Resetting` drives. Two regimes:
  - **CiA402 bit7 (generic):** #18's `Resetting` cyclic machinery applies as-is (bit7 is a controlword bit in the RxPDO — already RT-loop-written).
  - **Vendor SDO (A6):** the fault-reset is a **queued-to-RT-loop SDO** (§0.1 mechanism — the command queue carries `{0x2031:01, value=1}`; the RT loop services `ecx_mbxhandler` + the SDO write **in-loop**), NOT a non-RT-thread write and not a cyclic edge. #18's hold-N-cycles-debounce reframes as "RT loop issues the queued SDO, then watches the drive clear over the bounded window."
- **For #20/#21 bring-up specifically (simplest path, no queue needed):** clear any latent fault in **`configure()`** — PRE-OP/SAFE-OP phase, **before the RT thread spawns**, so the lifecycle thread is the single port owner and the `0x2031:01=1` SDO is a plain direct write (no queue, no concurrency). This is cleaner than ec_sample (which fault-resets concurrently *after* its RT thread starts — a latent two-thread touch we avoid). The **steady-state** operator reset (queued-to-RT, §0.1) is the #22/#18 follow-up; #20 establishes the config-data location + the bring-up-in-configure use.

---

## 7. `backend.hpp` deltas (DELIBERATE design evolution — distinct from #19's no-leak rule)

#19 forbade v2 *leaks*. #20's interface changes are **intentional DC-design simplification**, and they REMOVE surface (good):
- **Remove `DcSyncStatus` + `dc_sync_status()`** — the gate moved to PD-health + #16 (§4); the FPRD poll is gone. (Removes the `0x0984/0x098E/0x092C` machinery + `prev_sync0_evt` from the Impl.)
- **Simplify `arm_dc_sync`** → call **stock `ecx_dcsync0`**; drop the hand-rolled ESC writes + the `start_delay_ns` SyncDelay hack + the silicon-readback diagnostics. Signature can shrink to `(cycle_ns, shift_ns)`.
- **Remove `configure_dc_sync`** (the self-contained prime+arm variant) — the in-loop arm (§5) is the only path now; configure() no longer arms.
- **Keep `configure_dc_configdc`** (configdc in PRE-OP) and `dc_time()` (phase-lock input).
- **SimBackend (coordinated):** drop its `dc_sync_status`/`configure_dc_sync` overrides; model the new gate instead — a sim slave that holds WKC full + no Er74.1 after a sim-arm → gate passes → OP (so offline tests cover the bring-up gate). Add a sim hook to inject Er74.1 (0x603F=0x8700) so a test exercises the ABORT path.

These are backend.hpp changes, but they're *removals/simplifications* the DC fold justifies — not v2 concepts leaking. Net interface gets smaller + cleaner.

---

## 8. Tests (offline, SimBackend)

1. **DC bring-up happy path:** sim DC slave, configure(reach_op=false) → RT loop SETTLE→ARM→GATE→OP → reaches OPERATIONAL (poll-not-sleep, cycle-based where bounding — per #18's discipline). 
2. **Gate holds OP until PD-healthy + no-Er74.1:** sim withholds full WKC / injects Er74.1 in the window → loop does NOT request OP; clears → proceeds. 
3. **ABORT on Er74.1:** sim injects Er74.1 (0x603F=0x8700) during GATE → bounded abort, surfaced via #16 `last_error` ("drive fault 0x8700 / no sync"), NO OP request, no hammer.
4. **Vendor fault-reset config:** a config with the `0x2031:01=1` reset descriptor → bring-up issues that SDO (assert via SimBackend's `recorded_sdo`); a config without it uses bit7.
5. **Gapless:** assert PD `exchange()` is called every cycle from cycle 0 through the OP transition (no gap) — the structural 0x001B/Er74 guard.

---

## 9. Factor for #21 (forward ref)

The §3 bring-up loop is the thing #21 wraps as a thin API. Factor it so the bring-up (pump→settle→arm→gate→OP) is callable **without** the full ServoController CiA402 control — e.g. a `Master`-level "run the cyclic loop until OPERATIONAL (or fault)" step/driver that both ServoController's `run_rt_loop` and the #21 test program invoke. #21 specs the exact method shape; #20 just ensures the bring-up sequence isn't entangled with the servo control FSM so it's reusable. (The single-thread invariant §0 is what makes a shared bring-up safe.)

---

## 10. Sequence

After #19 (v2 base): cpp-expert implements §2 (configure) + §3 (RT-loop bring-up) + §4 (gate) + §6 (vendor-reset config-data) + §7 (backend deltas + SimBackend gate model) + §8 (tests). Joint review: me (sequence/gate/regime-split/the #16-reuse + the single-thread invariant) + DA (RT-determinism, the gapless guarantee through OP, the abort-not-hammer bound, sim-gate determinism). Bench-validate against ec_sample with tcpdump per CLAUDE.md. Then #21 wraps the bring-up as the thin API.