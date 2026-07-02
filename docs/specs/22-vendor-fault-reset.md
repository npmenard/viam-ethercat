# Spec #22 — A6 vendor-SDO fault-reset (steady-state operator recovery)

**Status:** spec'd now (team-lead); **implementation POST-first-light** — must NOT gate #19/#20/#21. Architect-owned.
**Problem:** #18's `Resetting` drives the **CiA402 controlword bit7** edge — correct for *generic* CiA402 drives, but the **A6's fault-reset is a vendor SDO `0x2031:01 = 1`** (CLAUDE.md §4), an acyclic mailbox op, not a controlword bit. So live A6 operator-recovery is unaddressed by #18. (The #20 §6 *bring-up* clear handles configure-time; this is the *steady-state, in-OP* operator reset.)
**Builds on:** #18 (the `Resetting` two-latency/give-up framework — reused, generalized), #20 §0/§0.1 (single-port-owner + the queued-SDO-to-RT vehicle), the librarian's v2 mailbox findings.

---

## 0. The mechanism constraint (read first) — REVISED off the real v2 API (librarian)

Original assumption (an in-OP **async** SDO via `ecx_mbxhandler` post/poll) is **not reachable**: librarian's source read (b410bf6) found **v2 exposes NO public async-SDO API** — `ecx_SDOread/SDOwrite` are blocking, `ecx_mbxsend/mbxreceive` are themselves blocking, and the async ticket-queue plumbing (`ecx_mbxaddqueue`/`donequeue`) is internal/undeclared (not public). So "post + per-cycle mbxhandler completion" would require forward-declaring SOEM internals — brittle, rejected.

And a **blocking** `ecx_SDOwrite` in the RT loop is ~2 ms (EC_TIMEOUTRET) — longer than the 1 ms cycle — so inline it would overrun → miss a SYNC0 frame. **So the reset mechanism must avoid the in-OP mailbox entirely.** Two realistic mechanisms (librarian-ranked), selected by config:

- **A — PDO-map the reset object (clean IF mappable; the happy path).** If the A6's `0x2031:01` is **RxPDO-mappable** (add it to the `0x1600` map), the fault-reset becomes a **cyclic PDO field**: the RT loop writes `1` on a rising edge, `0` otherwise — *exactly like the CiA402 bit7 path.* Pure PDO, **zero mailbox, zero blocking, fully RT-safe regardless of fault type**, **collapses #22 into #18's cyclic-field framework** (§2). Make it the design **if the bench confirms mappability** (§3).
- **D — one bounded blocking `ecx_SDOwrite` in an already-degraded window (the EXPECTED default).** Librarian's manual check leans **D**: `0x2031` is an *undocumented* vendor object (the manual's only documented fault-reset is CiA402 bit7), it's in none of the fixed-PDO presets, and vendor command/trigger objects are conventionally RW-SDO-only — so it's **probably not PDO-mappable.** D: the reset only runs when the drive is already **faulted** — for the primary A6 case (Er74, a *sync* fault) the drive has already dropped SYNC0, so a single stretched/missed cycle during recovery adds nothing. A single `ecx_SDOwrite(0x2031:01,1)` (U16 per ec_sample's `reset_cmd`) with a **tight timeout** is acceptable. ⚠ Caveat: a *non-sync* CiA402 fault (e.g. overcurrent) can leave the drive still in synced EtherCAT-OP — there a blocking SDO mid-cycle could induce a *second* (sync) fault. So D is safe for the Er74 path, acceptable-with-a-note elsewhere.

**The choice costs nothing to defer:** the mechanism is *derived* from "is the reset object in the RxPDO map?" (§1) — so the spec **defaults to D** (the likely outcome) and **flips to A for free** if the bench confirms mappability (just add `0x2031:01` to the `0x1600` JSON map). No code branches on the guess; only the JSON does.

(B — hand-roll the CoE download over public mailbox primitives [`ecx_getmbx`+`ecx_mbxsend`+`ecx_mbxreceive(timeout=0)`, RT-thread-only, bounded per cycle] — true non-blocking but reimplements `ec_SDOt` framing against internal layout; only if A unavailable AND D's one-cycle hit is unacceptable. C — internal ticket queue — rejected, non-public.)

This is also why #19/#20 correctly omit `ecx_mbxhandler`: runtime is PDO-only. #22 keeps it that way under Option A (the reset is just another PDO field); only Option D/B touch the mailbox at all, and only in the already-faulted window.

---

## 1. Config-data: the `fault_reset` mechanism descriptor

`ServoConfig` gains an OPTIONAL descriptor selecting the reset mechanism (no A6 constants in code — it's config):
```cpp
struct FaultResetSpec {                       // absent => generic CiA402 bit7
    std::uint16_t index;                      // e.g. 0x2031
    std::uint8_t  sub;                         // e.g. 0x01
    std::uint32_t value;                       // e.g. 1
    std::uint8_t  size;                        // SDO payload width (1/2/4)
};
std::optional<FaultResetSpec> fault_reset;    // A6 JSON: {0x2031, 0x01, 1, 1}
```
- **Absent** → CiA402 bit7 (the #18 `Resetting` cyclic edge) — unchanged generic path.
- **Present** → vendor reset (§3). The A6 hardware JSON sets `{0x2031,0x01,1,1}`. **The mechanism is DERIVED, not a separate flag:** if the descriptor's object is **RxPDO-mapped** (present in the `0x1600` map config) → Option A (cyclic PDO-field write); else → Option D (bounded blocking SDO in the recovery window). So adding `0x2031:01` to the RxPDO map in the A6 JSON is what selects the clean Option-A path.

---

## 2. Generalize #18's `Resetting` — dispatch the reset ACTION by mechanism

#18's `Resetting` framework is mechanism-agnostic and **reused whole**: the clear-confirm debounce (`clear_streak_ >= K`), the window countdown, the two-latency split (type-a transient / type-b persistent / type-c flicker), the bounded **give-up → `FaultResetFailed`**, and the #16 compose-both legibility — all identical. Only the **reset action dispatched on entering `Resetting`** differs:
- **bit7 mechanism (generic):** the cyclic `fault_reset_with_rearm` edge (existing).
- **Option A — PDO-mapped vendor field (A6, preferred):** *identical shape to bit7* — the RT loop writes `value` to the RxPDO-mapped vendor field on the rising edge, `0` otherwise. The reset "edge each cycle while in Fault" logic is **the same code path** as bit7, just a different field/value. **#22 collapses into #18 with zero new mechanism** — `dispatch_fault_reset()` writes the configured field instead of the controlword bit; the clear-confirm/window/give-up logic is untouched.
- **Option D — blocking SDO (fallback, non-mappable):** on entering `Resetting`, the RT loop issues ONE bounded `ecx_SDOwrite` (tight timeout) in the already-faulted window (§0 caveat re non-sync faults); then the *same* watch-for-clear + window + give-up logic runs. Re-issue at most once mid-window, never per-cycle.

So `step_lifecycle`'s `Faulted→Resetting` entry calls `dispatch_fault_reset()` which branches on the resolved mechanism (bit7 / mapped-field / blocking-SDO). Everything downstream is #18 as-built. Clean reuse — no second state machine — and under Option A there is no new I/O mechanism at all.

---

## 3. The realized mechanism (Option A preferred, Option D fallback)

Single-port-owner (§20 §0) holds in both: all I/O is the RT thread's; the non-RT API only enqueues the reset *intent* (the existing `request_fault_reset()` flag crossing the command queue — no new queue entry kind needed under Option A).

**Option A — PDO-mapped vendor field (preferred; verify mappability first):**
1. **Config:** add `0x2031:01` to the RxPDO `0x1600` map (A6 JSON) alongside controlword/target. `resolve_fields()` resolves it to a `FieldLocation f_vendor_reset_` (optional; `byte_width==0` ⇒ not mapped ⇒ fall to Option D).
2. **RT loop:** `dispatch_fault_reset()` / the `Resetting` cyclic step writes `value` (1) to `f_vendor_reset_` on the reset rising edge, `0` otherwise — a plain `store_le` into the RxPDO image, **exactly like the controlword/target writes already on the hot path**. No mailbox, no blocking, no overrun, RT-safe by construction, fault-type-independent.
3. **Done** — this is the whole mechanism; #18's framework drives the rest. **VERIFY (the one bench fact that picks A vs D; ~30s when the bench is up):** either (a) `slaveinfo <ifname> -sdo` → object `0x2031:01`'s access flags → the RxPDO-mappable bit; or (b) the decisive empirical trial in PRE-OP — write the map entry `0x1600:00←0`, `0x1600:01←0x20310110` (index `0x2031`, sub `0x01`, len `0x10`=16 bits, matching ec_sample's `uint16 reset_cmd`; use `…0108` if the OD shows U8), `0x1600:00←1`. **SDOwrite OK → mappable → Option A; abort `0x06040041` ("object cannot be mapped to PDO") → not mappable → Option D.** (`0x06040042` = length-exceeded, a different error.) Librarian gives the definitive call from the `slaveinfo -sdo` block when #22 is picked up.

**Option D — bounded blocking SDO (only if A's object isn't mappable):**
1. On `Resetting` entry, the RT loop issues ONE `ecx_SDOwrite(0x2031:01, 1)` with a **tight timeout** (≤ a few hundred µs, not EC_TIMEOUTRET's ~2 ms). Single port owner (RT thread). 
2. Safe in the **Er74 path** (drive already unsynced — a stretched cycle adds nothing); §0's caveat applies for non-sync faults (accept the one-cycle risk in the already-faulted recovery state, or prefer A). Re-issue at most once mid-window.
3. NOT `ecx_mbxhandler`/async (no public v2 API; §0). NOT every cycle.

(Option B — hand-rolled non-blocking CoE over `ecx_getmbx`/`mbxsend`/`mbxreceive(0)` — held in reserve only if A is unmappable AND D's one-cycle hit is unacceptable; it leans on internal `ec_SDOt` layout, so it's a last resort. Librarian has offered to extract the exact framing if needed.)

---

## 4. SimBackend + tests

- **SimBackend:** model the vendor reset for both mechanisms — **Option A:** the sim device reads the PDO-mapped vendor field and clears its fault on the configured `value` (mirror the bit7-clear path, just a different RxPDO field). **Option D:** an `ecx_SDOwrite` to `{index,sub}=value` clears the sim fault (reuse `recorded_sdo` to assert it was issued).
- **Tests:** re-run #18's matrix (type-a recover / type-b give-up / type-c flicker / no-spin / instant) with a config carrying the `fault_reset` descriptor → **identical recovery + give-up behavior via the vendor mechanism** (proves #18 generalized cleanly). Plus:
  - **RT-budget (Option A):** the reset is a PDO-field write — assert the reset cycle has **no PD gap and no missed cycle** (`loop_cycle` advances normally; trivially true since it's just another RxPDO store). The §0 risk doesn't exist under A.
  - **RT-budget (Option D, if used):** assert the single bounded blocking SDO occurs **only in the faulted/recovery window**, never in steady OP, and is bounded (tight timeout) — the one acceptable stretched cycle.
  - **Mechanism dispatch:** absent descriptor → bit7 (existing tests); present + RxPDO-mapped → Option A cyclic-field; present + not-mapped → Option D.

---

## 5. Scope + sequence

- **Post-first-light only.** #20 §6's configure-time bring-up clear (direct SDO, pre-RT-spawn) is sufficient to reach OP; #22 is the *live, in-OP, operator-initiated* recovery. Do not let #22 touch #19/#20/#21.
- **Sequence:** after first light + #20 lands the RT-loop structure → **first verify `0x2031:01` RxPDO-mappability** (decides A vs D) → implement #22: config descriptor (§1) + `dispatch_fault_reset` (§2) + the resolved-mechanism write (§3: Option A field-write, the common case; Option D bounded blocking only if unmappable) + SimBackend model + tests (§4).
- **Review:** me (the #18-generalization + the mechanism dispatch + config-data correctness) + DA (the RT-budget guarantee — Option A is a plain PDO write [trivially safe]; Option D's single blocking SDO is bounded + confined to the faulted window; single-port-owner discipline). Librarian verifies `0x2031:01` mappability (A) and, only if D/B, the exact API.
- **Bench:** on the A6, fault the drive in OP, operator `request_fault_reset()` → the vendor reset applies (Option A: a PDO field on the rising edge; Option D: one bounded SDO in the faulted window) → drive clears → recovers, with no new Er74 (Option A is gapless by construction).

---

## 6. Why this matters

Without #22, a live A6 that faults during operation can't be recovered by the operator without a full reconfigure/restart — the generic bit7 reset is silently a no-op on the A6 (it ignores bit7; it wants `0x2031:01`). #22 makes operator-recovery work on the A6 while honoring the gapless DC contract. The clean realization (Option A) is that **the vendor reset is just another RxPDO field written on a rising edge — mechanically identical to bit7** — so #22 **collapses into #18's existing cyclic framework with no new I/O mechanism**, the mechanism is config-data, and the recovery logic is shared. (Option D is the fallback only if `0x2031:01` isn't PDO-mappable.) The librarian's "no public async-SDO API in v2" finding is what redirected this from an over-engineered async path to the simpler PDO-field truth.

---

## 7. Post-#39 addendum (2026-06-10) — redefinition + the seam-shape disposition

- **#39 redefined #22's split:** mechanism = LIBRARY (an SDO request queue serviced by the RT thread between cycles — IF the SDO path is needed at all); payload = CONSUMER (the vendor object/value from config). Reconcile with §6's "Option A" above when #22 activates: **if `0x2031:01` is PDO-mappable, the PDO-field realization needs neither the queue nor the seam** — check mappability FIRST; the queue is the fallback mechanism, not the default.
- **Seam-shape disposition (DA, recorded at the #39/#40 train close):** the `ServoController::master_for_sdo()` seam (landed in #39's set-true test commit) is ACCEPTED but **`with_sdo(F&&)` (assert-not-started; `Master&` scoped to the callable under the lock) is the PREFERRED shape** — it structurally eliminates both residual hazards (concurrent-call AND cached-pointer-across-reconfigure) that the current seam only documents against. **Adopt `with_sdo` in whichever commit next touches the seam with a real second user in hand** — #22's consumer payload or the #38-era one-time 0x605A readback — i.e. re-shape with an actual caller, not speculatively.
- **Pre-staged #22 review points (DA, recorded at the round close so the gate doesn't reconstruct them cold):**
  1. **The mapped path inherits the #18 pulse discipline, sharpened:** a PDO-resident `0x2031:01` is a LEVEL on every cyclic frame — the reset must be a rising EDGE (write 1 for N cycles, return to 0), NEVER a held 1: a stuck-at-1 field masks every future edge (the drive's detector never re-arms) — the PDO-shaped version of the bit7-hold wart #18 fixed. Spec line for the implementation: *mapped form = pulse-not-level, falling edge mandatory, mirror of #18/the #38 FSM's bit7 semantics.*
  2. **RxPDO budget is a real cost of the mapped path:** the field rides EVERY frame forever, and the A6's freely-mappable RPDO space is small — so **mappable ≠ automatically better**; if the map is tight, the queue/seam fallback wins even when mapping is possible. The mappability check must report the SPACE COST, not just yes/no.
  - Vendor-lens: both paths clean (mapped form = 0x2031 in the CONFIG map, where vendor objects belong; queue form = payload consumer-side).
