# Spec #31 — Library RT-loop + bring-up abstraction (dedup tool + controller)

**Status:** design+plan (LOCKED dispatch). Architect-owned; cpp-expert impl + DA review.
**Goal:** the cyclic RT loop is DUPLICATED in a6_validate AND servo_controller (both re-roll mlockall, the abs-deadline phase-locked pacer, `dc_phase_correction`, the `bringup_step` loop, the steady loop). Factor the **mechanics** into the library; keep the **loop body** caller-specific. Unblocks #21 (thin program → ~15 lines).

---

## 0. Where it lives — MY CALL: a `realtime` library namespace, NOT Master, NOT ServoController
- **NOT Master methods.** Master is the bus-policy layer (config, remap, the per-step `bringup_step`, PDO access). The RT *thread ownership* + *cadence* are the CALLER's concern, not Master's — adding `setup_realtime`/`pace` to Master muddies that boundary. (`bringup_step` correctly already lives on Master as a per-step, caller-paced primitive — keep it there.)
- **NOT ServoController-owned.** That's the #21-redundancy trap again — a6_validate + the thin #21 program would have to drag in the servo FSM to reuse the loop.
- **→ A free-standing `ethercat::realtime` namespace** (e.g. `src/ethercat/realtime.hpp`): the two duplicated MECHANICS as library components, consumed by a6_validate, servo_controller, AND the #21 thin program. **The loop BODY stays caller-specific** (a6_validate streams a sine; servo_controller runs the CiA402 servo FSM) — we factor the plumbing, not a one-size loop (which would fight the different bodies).

---

## 1. The three factored pieces

**(a) `realtime::setup()` — the RT-thread setup** (replaces servo_controller's `setup_realtime()` + a6_validate's inline RT prelude):
```cpp
namespace ethercat::realtime {
  // mlockall(MCL_CURRENT|MCL_FUTURE) + mallopt(no-trim) + SCHED_FIFO + STACK PRE-FAULT.
  // Returns false on RT-sched failure (caller decides: throw if require_realtime, else warn).
  bool setup(int priority = 80, std::size_t prefault_stack_bytes = 512*1024) noexcept;
}
```
- **Stack pre-fault is the new bit** (note: "no page-fault jitter on the RT loop"): touch `prefault_stack_bytes` of stack (a `volatile char buf[N]; memset`) BEFORE the loop so the first deep call doesn't fault a page mid-cycle. Combined with mlockall(MCL_FUTURE) this gives a fault-free hot loop.
- **The configure-time `mlockall(MCL_CURRENT)` pre-lock stays separate** (it runs pre-RT-spawn to avoid the MCL_FUTURE-thread-create EAGAIN — #20). Expose it as `realtime::lock_current() noexcept` so configure uses the same library helper. So: `lock_current()` pre-spawn (configure), `setup()` in the RT thread (full lock + SCHED_FIFO + prefault).

**(b) `realtime::DcPacer` — the abs-deadline phase-locked pacer** (replaces both `sleep_until` + the inline `dc_phase_correction` + `next`/`integral` bookkeeping):
```cpp
class DcPacer {
 public:
  DcPacer(std::uint32_t period_ns, std::int64_t sync0_shift_ns) noexcept;
  // Sleep to the next phase-locked absolute deadline: next += period + dc_phase_correction(dc_time, ...);
  // overrun catch-up is phase-PRESERVING (while(next<=now) next+=period), never a rebase post-arm.
  void pace(std::int64_t dc_time_ns) noexcept;
 private:
  std::uint64_t next_; std::int64_t integral_; std::uint32_t period_ns_; std::int64_t shift_ns_;
};
```
- Owns `next` + the PI `integral`; the caller passes `master.dc_time()` each cycle. One pacer used continuously across bring-up → steady (gapless — the #20 invariant). `dc_phase_correction` (in `dc_sync.hpp`) is **kept + gets a provenance comment** (note 10): it's the standard SOEM `ec_sync` PI/PLL (P+I on the DC-time phase error, the canonical `ethercat.org`/SOEM gains) — confirmed-standard, not bespoke. **DcPacer CALLS the existing inline `dc_phase_correction` — it does NOT reimplement the wrap/clamp/anti-windup** (re-deriving the `(-cycle/2, cycle/2]` wrap or the ±50µs clamp is the divergence risk; only the pacer *loop* is being deduped, not the math).
- **⚠ THE TWO CURRENT LOOPS ALREADY DIFFER IN THE OVERRUN PATH (DA pre-read) — so DcPacer can NOT be "bit-identical to both" there; it adopts servo_controller's behavior (the safe superset):**
  - **Steady-state IS identical** in both (`next += period + corr`, sleep `TIMER_ABSTIME` to absolute `next`, `corr` from the prior cycle's `dc_time`) → DcPacer's steady trajectory must be bit-identical to both.
  - **Overrun differs:** servo_controller does a phase-PRESERVING skip-catch-up (`for(now=monotonic_ns(); next<=now;) next+=period;` — skip the missed WHOLE periods, realign to the SYNC0 grid, ONE sleep, NO `process()` for skipped cycles — deliberately avoids an off-phase burst). a6_validate's `sleep_until` has NO catch-up → on a missed deadline `clock_nanosleep` returns immediately and it runs `process()` back-to-back = an off-phase LRW burst — **precisely the LRW-starvation pattern the runbook ties to `0x001B` (SM watchdog).**
  - **→ DcPacer takes servo's skip-catch-up.** Consequences: **P3c (servo)** — verify the catch-up is preserved EXACTLY (whole-period skip, NO `corr` applied in the skip, re-read `now` each iteration); production + timing-critical, byte-equivalent. **P3b (a6_validate)** — it **GAINS** skip-catch-up (loses its burst): a behavior CHANGE — beneficial (the anti-0x001B behavior) + low-stakes (non-prod tool), but **acknowledged as a DELTA, NOT claimed "byte-identical."** Don't assert a6_validate is byte-for-byte unchanged in the overrun path. (The HW gate is steady-state, so the gain won't show on the bench — it's offline-reasoned-safe by being servo's proven production behavior.)
- **`next` underflow guard:** when `corr < 0` (phase ahead), `next += period + corr` must not underflow / go backwards — needs `period > max_correction` (true at 1 kHz: 1ms period vs ±50µs clamp; guard or doc the invariant for higher rates). Same care in the timespec-normalize vs uint64-ns representation.

**(c) Bring-up: `bringup_step` is already the shared per-step (Master).** The only duplication left is the *loop around it*. Provide a thin helper for simple consumers (the #21 program), while servo_controller keeps its inline prelude (it must publish drive-fault state on abort):
```cpp
// Pace + step to OPERATIONAL/ABORTED. `sync_faulted()` is the caller's 0x603F==0x8700 read
// (kept out of Master — #20 layering). Returns the terminal BringupStatus.
BringupStatus realtime::run_to_operational(Master&, DcPacer&, std::function<bool()> sync_faulted,
                                           std::chrono::milliseconds timeout);
```
- **#21 thin program** uses `run_to_operational` (one call) — it has no per-cycle diagnostics. **servo_controller** keeps its inline prelude loop (Pacer + bringup_step + the publish-on-abort #16 drive-tier) — it publishes `drive_fault_code`/`NotOperational` per cycle, so it uses Pacer + bringup_step directly. **a6_validate ALSO keeps its Phase-1 loop EXPLICIT (corrected after P3b review — was over-named a run_to_operational consumer):** its bring-up prints the per-200-tick dcPhase-convergence diagnostics (load-bearing for the DC bring-up debugging — CLAUDE.md leaned on them) + honors g_stop, which `run_to_operational`'s callback-less signature can't carry. So a6_validate uses the DEDUPED mechanics (`setup` + `DcPacer`) but keeps its explicit diagnostic Phase-1 loop. **Net: `run_to_operational` is the vehicle for SIMPLE consumers (#21) that don't need a custom loop body; the mechanics (`setup`/`DcPacer`) dedup everywhere; the loop BODY stays caller-specific where it carries diagnostics/FSM/publish.** All share `setup()` + `DcPacer` + `bringup_step`.
- **THE DEDUP PRINCIPLE (sharpened — governs P3b + P3c, resolves the "no hand-rolled loop" tension):** the dedup target is the **PACER**, not the loop. The rule is **"no hand-rolled PACER left behind"** — i.e. NO surviving hand-rolled `sleep_until`/`dc_phase_correction`/`next`-bookkeeping; ALL pacing goes through `DcPacer.pace()`. It is NOT "everything → `run_to_operational`." An explicit loop BODY that calls the deduped `DcPacer.pace()` fully satisfies the dedup (the hand-rolled pacer is gone); keeping that body is correct where it carries per-cycle work `run_to_operational`'s callback-less signature can't: **a6_validate's Phase-1 (dcPhase-convergence diagnostics)** and **servo_controller's bring-up (drive-fault publish-on-abort)** both KEEP explicit loop bodies that USE `DcPacer.pace()`.
  - **P3c implication (prevents a real mistake):** servo's bring-up loop must NOT collapse into `run_to_operational` (that would drop its per-cycle publish). P3c migrates servo's PACER sites (steady + second loop + the bring-up loop) all onto `DcPacer.pace()`; the bring-up loop BODY stays explicit (publish). "All servo pacer sites migrated" = "no hand-rolled `sleep_until`/`dc_phase_correction` remains," NOT "the bring-up loop became run_to_operational."

---

## 2. What does NOT change (hard constraints)
- The validated bring-up SEQUENCE + the CSP sine (HW) — untouched; this is a refactor of the *plumbing*, behavior identical. **HW re-validate (OP + sine) after the migration.**
- The single-port-owner invariant, the gapless SAFE-OP→OP, abort-no-hammer, the gate semantics (#20) — all preserved (they live in `bringup_step`, unchanged).
- RT path stays noexcept/no-alloc/no-throw (`setup`/`pace` are noexcept; `run_to_operational` is the bring-up phase, not the steady hot loop).
- 531f075 readstate + b9f7b5f teardown — untouched.

---

## 3. Phasing + testing (this spec = P3; after #30)
- **P3a — the helpers:** `realtime::setup()`/`lock_current()` + `DcPacer` + `dc_phase_correction` provenance comment. **Tests (offline):** `DcPacer` math is unit-testable (feed a synthetic dc_time ramp → assert the deadline converges + the overrun catch-up is phase-preserving — reuse the dc_sync.hpp test pattern). `setup()` is environment-dependent (SCHED_FIFO needs CAP_SYS_NICE) — test the no-throw/returns-false path offline; the real RT effect is HW.
- **P3b — migrate a6_validate** to `setup()` + `DcPacer` + (optionally) `run_to_operational`. **HW re-validate: OP + sine.**
- **P3c — migrate servo_controller** `run_rt_loop` to `setup()` + `DcPacer` (keep its inline bring-up prelude using them — option (i), keeps the per-cycle drive-fault publish). Offline controller tests + TSan; **HW re-validate.**
  - **P3c VERIFICATION CRITERIA (DA — the teeth on "no hand-rolled PACER"):**
    1. **Zero residual hand-rolled pacer in servo** — post-migration grep shows NO surviving `sleep_until`/`clock_nanosleep`/`dc_phase_correction`/manual `next += period` bookkeeping anywhere in servo_controller; ALL pacing via `DcPacer.pace()`. (The partial-migration trap — the actual thing the principle guards.)
    2. **ONE DcPacer across servo's phases (gapless #20)** — bring-up + steady + teardown share a SINGLE `DcPacer` carried continuously; NO `reset()`/reconstruct at the bring-up→steady boundary. **Distinct check beyond #1:** a per-phase fresh pacer would pass "uses pace()" yet still gap the handoff → 0x001B. (Same check DA ran on a6_validate's single continuous pacer.)
    3. **Byte-equiv (true no-delta) behavior** — servo ALREADY re-reads, so servo→`DcPacer.pace()` (which re-reads) is a TRUE no-delta migration (unlike a6_validate's catch-up GAIN). Verify each of the 3 sites (615 steady / 672 second loop / bring-up prelude) preserves its prior behavior exactly.
    4. Explicit loop bodies are fine if they pace via `DcPacer` (the sharpened principle).
  - **+ the `run_to_operational` unit test** (folds here): offline, drive a Sim Master through `bringup_step` to Operational via run_to_operational + the bounded-timeout→Aborted give-up case (it's unit-testable: `Master& + DcPacer& + sync_faulted + timeout` → terminal `BringupStatus`). Closes the unexercised-through-all-of-P3 gap before #31 ships; #21 is the production consumer.
- **`run_to_operational` OWES A TEST before #31 closes (DA, P3b forward note).** It's built in P3a but UNCONSUMED in P3 (a6_validate keeps its explicit Phase-1 for diagnostics; servo keeps its explicit bring-up for the publish — per the "no hand-rolled PACER, explicit body OK" principle) AND currently UNTESTED (realtime_test covers DcPacer/step/pace/setup, not run_to_operational). So it's defined-but-unexercised library code. Its production consumer is **#21** (imminent, post-#31). **Add an offline unit test by end of P3** (drive a mock/Sim Master through `bringup_step` to Operational via `run_to_operational`, + the bounded-timeout→Aborted case) so it ships VERIFIED, not dead/unexercised — fold into P3c or a small standalone commit. (Keeping it in #31 is correct — it's the bring-up building block #21 consumes; the test resolves the "unverified" half now, #21 resolves the "unconsumed" half next.)
- **Then #21** (thin program) falls out: `realtime::setup()` → `DcPacer` → `run_to_operational` → `read_rpdo` feedback → done (~15 lines, the spec-C ServoController-level form OR a Master-level form both now trivial).
- **Per-phase gate:** offline + HW (OP+sine) + both -Werror.

DA focus: `DcPacer` keeps the gapless/phase-preserving guarantees (no rebase post-arm), `setup()`'s prefault + lock ordering is correct (MCL_CURRENT pre-spawn vs MCL_FUTURE in-thread — the #20 EAGAIN lesson), and the migration is behavior-identical (the HW re-validate is the proof).