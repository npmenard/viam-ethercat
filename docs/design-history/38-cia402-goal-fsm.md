# Spec #38 — Stateful goal-walking Cia402Fsm (sw-in / intents / cw-out)

**Status:** architect spec, awaiting **DA review BEFORE implementation** (user-directed gate). API shape LOCKED with the user (review round 2, 2026-06-10) — this spec elaborates routing/edge semantics + folds the A6 grounding (librarian's #38 T0–T16 table, manual Fig 8-3, a6.txt:9171-9246); it does not re-litigate the API.
**Replaces:** the pure `step()`+caller-owned-edges model (and plan-note #8's driver-owned bit-7 re-arm wart). Consumers (servo_controller, a6_validate) migrate off hand-rolled ladders/edges (task #41).

---

## 0. Locked API (from the task — restated for reference, not open)
```cpp
class Cia402Fsm {
  void update(std::uint16_t statusword) noexcept;          // 1/cycle — THE cycle boundary
  Cia402State state() const noexcept;                      // reflects last update()
  bool operation_enabled() const noexcept;                 // + the usual predicates
  std::expected<void, FsmError> set_state(Cia402State goal) noexcept;  // goal-driven walk
  struct { bool active() const noexcept; void reset() noexcept; } fault;  // bit-7 edge, FSM-owned
  std::expected<void, FsmError> set_point(bool relative = false) noexcept; // PP bit-4 handshake
  SetpointPhase setpoint_phase() const noexcept;
  void halt(bool) noexcept;                                // sticky bit-8 level
  void quick_stop() noexcept;                              // transition INTENT (not a goal)
  void set_mode(Cia402Mode) noexcept;                      // intent VALIDATION only; never 0x6060
  std::uint16_t get_cw() const noexcept;                   // pure/idempotent within a cycle
  std::uint32_t cycles_in_state() const noexcept;
  auto enable()  { return set_state(Cia402State::OperationEnabled); }
  auto disable() { return set_state(Cia402State::SwitchOnDisabled); }
};
```
RT-resident: everything noexcept, `std::expected`/error-enum, NO exceptions, no alloc. The existing pure transition table stays the unit-tested core; the stateful class is a thin shell over it.

**Toolchain footnote — the `std::expected` spelling requires C++23 (the project pinned C++20).** Verified enablement recipe (branch tip **bbebeca**, docker-gated end-to-end: 53/53 targets + 17/17 in-image tests, module ON): (1) `CMAKE_CXX_STANDARD 20→23`; (2) module image: ubuntu:jammy's default g++-11 has NO `<expected>` — install **g++-12** (in-distro; its libstdc++ has it) + update-alternatives (~6 Dockerfile lines; clang-19-in-image auto-picks libstdc++-12) → ghcr image republish, amd64+arm64; (3) boost 1.74's asio uses `std::exchange` without including `<utility>` (libstdc++-12 dropped the transitive include at C++23) — 1-line `<utility>` include in main.cpp. Host gates (clang-19 + GCC-14) take C++23 with zero changes. **Fallback (option B) if the bump is declined:** stay C++20 + a minimal in-house `ethercat::Expected<void,E>` (~30 lines, identical shape; deviates from the user-locked SPELLING — needs a user heads-up). **RESOLVED: the USER ruled B (2026-06-10)** — stay C++20; the in-house `ethercat::Expected<void,E>` ships (spelling-only deviation from the locked API, user-sanctioned); the 3 bbebeca infra deltas REVERTED. Frozen at **7562f5e**. The verified C++23 recipe above stays recorded as the road-not-taken (if the project ever bumps, it's pre-paid).

---

## 1. Architecture: pure core + stateful shell
- **Pure core (existing, kept):** state decode (sw→Cia402State via the standard masks) + a **routing table** `next_cw(Cia402State current, Cia402State goal) → expected<uint16_t, FsmError>` — pure, exhaustively unit-testable.
- **Stateful shell:** owns goal, fault-reset edge phase, bit-4 handshake phase, halt level, mode, cycles_in_state. `update()` mutates; `get_cw()` composes (base cw from routing ∥ bit7 pulse ∥ bit4 phase ∥ bit8 halt) and is a pure read.
- **Cycle contract:** `update(sw)` exactly once per cycle, BEFORE any `get_cw()` for that cycle. All edge machinery advances in `update()` only. Multiple `get_cw()` calls in one cycle return the same value (idempotent). Intents (`set_state`/`set_point`/`halt`/`quick_stop`/`fault.reset`) may be called between updates; they take effect at the next `get_cw()` composition (no immediate cw mutation — single composition point).
- **Initial goal = ADOPT-CURRENT at the first update() (D3, DA review):** at construction there is no goal; the FIRST `update()` adopts the decoded state as the goal (HOLD it via its sustain cw). **Explicit consequence: constructing the FSM while the drive is already OperationEnabled SUSTAINS 0x0F — energization is maintained, not initiated** (dropping it would be the worse surprise: an FSM swap-in must not de-energize a running axis). "Goal cancelled" (post-fault §5 / post-quick-stop §3) means the same thing operationally: **HOLD-current** (the current state's sustain cw — with B1's SOD sustain = 0x00, genuinely inert) until the caller issues a fresh `set_state`.
- **`cycles_in_state()` resets exactly when `update()` decodes a DIFFERENT state than the previous update** (minor-d) — it counts consecutive updates in the same decoded state; intents/goal changes do NOT reset it.

## 2. State decode (A6-verified masks — librarian, a6.txt:3153-3212, 9171-9246)
| Cia402State | mask | value | A6 name |
|---|---|---|---|
| NotReadyToSwitchOn | sw&0x4F | 0x00 | Initialization |
| SwitchOnDisabled | sw&0x4F | 0x40 | No fault for servo |
| ReadyToSwitchOn | sw&0x6F | 0x21 | Servo ready |
| SwitchedOn | sw&0x6F | 0x23 | Waiting for S-ON |
| OperationEnabled | sw&0x6F | 0x27 | Servo running |
| QuickStopActive | sw&0x6F | 0x07 | Quick stop |
| FaultReactionActive | sw&0x4F | 0x0F | Stop at fault |
| Fault | sw&0x4F | 0x08 | Fault |
- **Decode hygiene (A6 quirk 4, scoped per DA minor-e):** bit10 is ALWAYS 1 on the A6 (unsupported) and bit14 unsupported — both outside the masks above (safe). **No FSM-INTERNAL predicate may read bit10/bit14** (state decode, routing, handshake, fault — none of them). This scopes to the FSM: cia402.hpp's standalone `target_reached()` (bit10) legitimately exists for conformant drives and stays — its A6-unreliability is a consumer concern (#41's clean-list), not this FSM's. Bench-consistent: fault sw 0x218/0x238 &0x4F = 0x08 ✓.

## 3. The routing table (deterministic, one legal transition per cycle)
`next_cw(current, goal)`, applied fresh EVERY update from the CURRENT decoded state (see §4):

**Per-state SUSTAIN cw (the deterministic base for HOLD and WAIT — DA review, B1 + minor-c):**
| state | sustain cw | why |
|---|---|---|
| NotReadyToSwitchOn | 0x00 | inert pre-init |
| SwitchOnDisabled | **0x00** | **B1: 0x06 would FIRE T2 (DS402 + bench-proven, a6_validate:574 "0x06 → ReadyToSwitchOn") → SOD↔RTSO ping-pong at loop rate. 0x00 is genuinely inert in SOD.** |
| ReadyToSwitchOn | 0x06 | level that holds RTSO |
| SwitchedOn | 0x07 | level that holds SwitchedOn |
| OperationEnabled | 0x0F | sustains energization |
| QuickStopActive | 0x02 | sustains the quick stop (A6 default auto-T12s out regardless) |
| FaultReactionActive | 0x06 | inert during the auto fault-stop (§5 baseline) |
| Fault | 0x06 | inert; lands SOD-stable after T15 (§5) |

| current ↓ \ goal → | SwitchOnDisabled | ReadyToSwitchOn | SwitchedOn | OperationEnabled |
|---|---|---|---|---|
| NotReadyToSwitchOn | WAIT (T1 auto) | WAIT | WAIT | WAIT |
| SwitchOnDisabled | **HOLD 0x00 (B1)** | 0x06 (T2) | 0x06 (T2) | 0x06 (T2) |
| ReadyToSwitchOn | 0x00 (T7) | HOLD 0x06 | 0x07 (T3) | 0x07 (T3) |
| SwitchedOn | 0x00 (T10) | 0x06 (T6) | HOLD 0x07 | 0x0F (T4 — ENERGIZES) |
| OperationEnabled | 0x00 (T9) | 0x06 (T8) | 0x07 (T5) | HOLD 0x0F |
| QuickStopActive | 0x00 (T12) | 0x00 (T12)† | 0x00 (T12)† | 0x0F (T16)‡ |
| FaultReactionActive | WAIT (T14 auto) | WAIT | WAIT | ERROR FaultActive |
| Fault | ERROR FaultActive (unless reset armed → see §5) | ← same | ← same | ← same |

- **WAIT** = no cw advances this transition; emit the CURRENT state's sustain cw (deterministic, not "last emitted" — history-independent); `cycles_in_state` keeps counting (caller timeout policy — never hammer).
- **HOLD** = at goal; emit the goal state's sustain cw. **HOLD-at-SOD = 0x00 (B1, DA-corrected):** 0x06 is a TRANSITION trigger from SOD (it IS the T2 cell in this very row) — emitting it as a hold would oscillate SOD↔RTSO; this also infected post-fault-reset (T15 lands SOD) and post-quick-stop (auto-T12 lands SOD). Walk-resumability needs nothing from the hold cw — re-planning emits 0x06 fresh the cycle the goal changes. **Regression test (test 12): a sim drive honoring T2-on-0x06, goal=SOD → state stays SOD for N cycles** (fails on 0x06-hold, passes on 0x00).
- **† QSA→RTSO/SwitchedOn = 0x00 (T12), NOT WAIT (DA gate amendment):** the path is T12→SOD→re-walk. WAIT would emit QSA's sustain (0x02), which actively SUSTAINS the quick stop — on a HOLDING drive (605A∈5..7, exactly the configs the spec keeps T16 for) that makes RTSO/SwitchedOn permanently unreachable, contradicting the table's own genericity. **The override principle: `quick_stop()` CANCELS the goal (§3/§8), so any goal present during QSA was issued AFTER the stop = a fresh, deliberate caller override — honor it** (route T12, land SOD, re-walk). The ramp-cut concern (0x00 during a quick-stop ramp may coast) is already accepted by the goal=SOD cell routing 0x00 — no asymmetry. On the A6 default this is invisible (auto-T12 exits QSA regardless; identical landing).
- **‡ T16 (QSA→OE, 0x0F) is in the table per the locked API** — but see §4: on the A6 DEFAULT it never fires.
- **Goal=QuickStopActive is NOT a valid `set_state` goal** → `FsmError::InvalidGoal`. QuickStop is the `quick_stop()` transition INTENT (locked API): it forces cw=0x02 while current==OperationEnabled; once the state leaves OE the intent is consumed and the drive lands wherever 0x605A dictates (SOD on the A6 default). The active goal is CANCELLED by quick_stop() (it's an abort, not a detour).

## 4. Per-cycle RE-PLANNING — the load-bearing design point (reconciles T16 with the A6)
**The walker stores only the GOAL, never a path.** Every `update()` decodes the current state and `get_cw()` routes ONE hop from it. Consequences:
- **Drive auto-transitions are absorbed, not fought.** The A6 auto-walks T1 (init), T12 (quick-stop done → SOD, because 0x605A default=2 — a6.txt:13499), T13/T14 (fault path). The walker never holds a stale plan: state moved underneath → next cycle re-routes from where the drive actually IS.
- **T16 reconciliation (A6 quirk 2):** the routing table KEEPS the standard T16 edge (the FSM is generic CiA402 — valid for drives/configs with 0x605A∈5..7). On the A6 default the drive auto-T12s out of QSA before T16 could apply → the walker simply re-plans from SOD (T2→T3→T4 re-walk). NO special-casing, NO "resume" modeling — re-planning makes T16-unreachable a non-event. (Document in code: "T16 present per standard; unreachable on A6 default 605A=2 — the re-planner handles it.")
- **One-legal-transition-per-cycle** matches the A6's documented 6→7→15 sequence exactly (librarian's sequencing note) — no transition-skipping (e.g. no 0x06→0x0F shortcut), even where some drives tolerate it.
- **0x605A facts for the record (librarian, a6.txt:13499 + 14656-14663):** SDO-RW (I16, 0–7, default 2, NOT PDO-mappable). Values 5–7 are REAL, documented A6 values ("keeping position lock" = the stay-in-QSA variants that enable T16 — standard-CiA402-inferred from the position-lock wording, high confidence). **Value 4 is UNDEFINED on the A6 (absent from its table) — never write it.** ⚠ **605A is effective only UPON RE-POWER-ON** — so a T16 HW-verify requires: SDO-write 605A=5/6 → control-power cycle → bring-up → quick-stop → confirm QSA holds → 0x0F → OE. Within any power session 605A is FROZEN → the session's quick-stop regime (lands-SOD vs holds-QSA) is a stable fact. **Consumer recommendation (NOT the FSM — it has no SDO access, §11): a one-time 605A SDO read at configure** records the session regime for diagnostics/runbook truth (the re-planner handles either regime regardless; the readback just replaces assumption with fact). Record for #41 (servo_controller/a6_validate could log it at configure).

## 5. Fault semantics (T13/T14/T15 + A6 quirks 1)
- **Fault occurrence (update decodes FaultReactionActive or Fault): the ACTIVE GOAL IS CANCELLED.** Safety rationale: auto-resuming a goal of OperationEnabled after a fault reset would RE-ENERGIZE the motor without a fresh caller decision. Explicit > implicit re-energize. Post-fault, the caller must re-issue `enable()`. [The one deliberate divergence from "the goal persists until changed" — flagged for DA.]
- **UNCOMMANDED QSA entry cancels the goal (B3, DA implementation-gate finding): an external stop's release must not auto-restart the axis.** A drive can enter QuickStopActive externally (e.g. an estop input driving the drive's quick-stop function) with goal=OE active — without this rule the walker re-walks after the auto-T12 landing and the motor RE-ENERGIZES the moment the external stop releases (the restart hazard, same class as fault-cancel above; the Fault-only asymmetry was indefensible). **Form: an ENTRY-EDGE rule in update()** — QSA decoded where the previous state ≠ QSA and the FSM did not itself issue `quick_stop()` → cancel the goal. Entry-edge (not level) preserves the §3 T12-override principle: a goal issued DURING QSA is post-entry = a fresh, deliberate caller decision — honored. Test 15: external QSA injection mid-walk-to-OE → goal cancelled, no re-walk after the stop releases; goal issued during QSA still routes (T12-override intact).
- **cw during Fault / FaultReactionActive:** baseline 0x06 (shutdown level — inert in fault, lands SOD-stable after T15). FaultReactionActive: WAIT (T14 is auto; no cw helps; never hammer — CLAUDE.md).
- **`fault.reset()`:** arms the **bit-7 RISING edge: cw `= 0x80` (clean, the bench-proven #18 form — NOT or-ed) for EXACTLY ONE update-cycle** — FSM-owned (the plan-note-#8 wart fix). A6 quirk 1: while bit7=1 ALL other control references are masked (a6.txt:3067-3071), so clean 0x80 matches what the drive sees anyway. **The post-pulse cycle returns to the base cw with bit7=0 — the FALLING edge matters: it's what re-arms the drive's rising-edge detector for any future reset.** No-op if no fault. **One pulse per reset() call** — retry policy stays the CALLER's (bounded give-up; never hammer a persistent fault — the wedge lesson).
- **Er74-class faults do NOT clear via bit7** (vendor 0x2031:01 — consumer-side policy per #39). The FSM surfaces this honestly: after a reset pulse, `fault.active()` stays true and `cycles_in_state()` grows → the caller's bounded-retry/give-up sees "reset ineffective" and escalates (vendor SDO pre-RT, or operator). The FSM does NOT know about vendor resets — mechanism-only.

## 6. The bit-9 "remote" gate (A6 quirk 3 — NEW surface, flag for DA)
sw bit9=0 ⇒ the drive is IGNORING controlword writes (a6.txt:3190-3192). The walker must not interpret non-progress as "try harder":
- `update()` records bit9; expose **`bool remote() const noexcept`**.
- When bit9=0: routing still composes the same next-hop cw (harmless — it's being ignored), but the state won't advance → `cycles_in_state()` grows → the caller's timeout fires. `remote()` lets the caller's diagnostic distinguish "drive offline/local" from "transition stuck."
- NO FSM-internal timeout (locked: timeout stays caller policy); `remote()` + `cycles_in_state()` are the two diagnostic primitives that make the caller's timeout a one-liner.

## 7. PP set_point (bit-4 4-phase handshake, FSM-sequenced)
Phases (advanced in `update()`, exposed via `setpoint_phase()`): **Idle → Arm** (cw |= bit4, + bit6=relative if requested) → **AwaitAck** (hold bit4 until sw bit12=1) → **Drop** (clear bit4) → **AwaitAckClear** (until sw bit12=0) → Idle.
- `set_point()` errors: `FsmError::WrongMode` unless mode==ProfilePosition (mode-aware validation only — locked); `FsmError::NotOperational` unless state==OperationEnabled; `FsmError::Busy` if a handshake is already in flight.
- **ABORT RULE (B2, DA review — prevents Busy-forever): `update()` resets the handshake to Idle whenever the decoded state ≠ OperationEnabled.** A fault / quick_stop / any exit from OE mid-handshake kills the in-flight setpoint on the drive side anyway (it dropped it on disable/fault); without this rule bit12 may never edge → the phase wedges in AwaitAck → every future `set_point` returns Busy permanently even after full recovery. Test 13: fault mid-AwaitAck → phase==Idle after recovery → set_point succeeds.
- Timeout = caller policy (`setpoint_phase()` + `cycles_in_state()`-style observation; a phase counter `cycles_in_phase()` is NOT in the locked API — the caller can count its own cycles between phase observations; keep the API as locked).
- PV decode hygiene: cw bits 4-6/9-10 reserved in PV (a6.txt:3394) — the composer only sets bit4/bit6 inside an ACTIVE PP handshake (mode-gated), so PV cw never carries them. halt (bit8) composes in all modes (standard).

## 8. Composition order (get_cw, deterministic)
1. Base = routing `next_cw(current, goal)` (or quick_stop's 0x02 override while OE; or the fault sustain 0x06).
2. If fault-reset pulse armed THIS cycle → cw `= 0x80` (clean — #18's proven form; A6 masks other refs anyway).
3. Else compose: | bit4/bit6 (active PP handshake phase) | bit8 (halt level).
4. quick_stop intent (current==OE): base = 0x02 (overrides routing; halt/bit4 suppressed — quick stop IS the priority).
- **quick_stop() outside OE = NO-OP (minor-b, DA review):** the intent does NOT linger armed until OE is next reached — a stale quick-stop firing minutes later on a fresh enable would be a hazard. Consumed-or-dropped at the next update.
- **quick_stop-vs-reset-pulse precedence is VACUOUS by state exclusivity (DA's simplification):** quick_stop applies only while current==OperationEnabled; the reset pulse only arms in Fault — the two can never compose. Reset-pulse-during-armed-handshake resolves via §7's B2 abort rule: the fault that precedes any reset already reset the handshake to Idle, so the pulse always emits a clean 0x80.

## 9. Errors (`FsmError`)
`FaultActive` (goal refused: decoded state ∈ {Fault, FaultReactionActive}) · `InvalidGoal` (QSA/Fault/NotReady/FRA as goals) · `WrongMode` (set_point outside PP) · `NotOperational` (set_point outside OE) · `Busy` (handshake in flight). All returned via `std::expected` at the INTENT call (immediate, per the locked API) — `update()`/`get_cw()` never error.
- **`FaultActive` fires UNCONDITIONALLY while decoded ∈ {Fault, FRA} — even with a reset armed** (implementation refinement, architect-ratified): an accepted-then-immediately-cancelled goal would be a misleading success, and a goal accepted during Fault that survived into post-T15 SOD would be exactly the implicit-re-energize §5 forbids. The locked API's "notably FaultActive when fault active and no reset armed" is an example of unreachability, not an iff — during Fault, every goal IS unreachable until T15 completes, reset armed or not. **Caller sequence: `fault.reset()` → await SOD (post-T15) → `enable()`** — the re-enable is a fresh decision AFTER recovery, by construction. (The PURE routing table keeps its §3 Fault/FRA cells — the refusal lives in the shell's set_state validation.)

## 10. Tests (exhaustive, offline — the gate for implementation)
1. **Exhaustive (from,goal) routing enumeration** — all 8 states × 4 valid goals + the invalid goals: every cell of §3 asserted (cw value / WAIT / HOLD / ERROR).
2. **Multi-hop walks:** SOD→OE in exactly 3 updates (0x06,0x07,0x0F) with simulated sw advancing; stalled sw (no advance) → same cw held, cycles_in_state grows.
3. **Re-planning:** mid-walk drive auto-transition (simulate QSA→SOD auto-T12) → next cw re-routes from SOD; goal preserved (non-fault case).
4. **bit-7 edge timing:** reset() → exactly one update-cycle of 0x80, then base resumes; reset with no fault = no-op; reset does NOT auto-route (goal stays cancelled post-fault until re-issued).
5. **Fault cancels goal:** walking to OE, inject fault sw → goal cancelled; post-T15 (SOD) the FSM holds (no auto re-walk); fresh enable() walks again.
6. **Full 4-phase bit-4 handshake** incl. ack, ack-clear, Busy on double set_point, WrongMode in PV/CSP, NotOperational outside OE; caller-timeout observability (phase stays AwaitAck while bit12 never set).
7. **quick_stop:** OE→0x02; intent consumed on leaving OE; goal cancelled; A6-default path lands SOD (simulated auto-T12) → no resume.
8. **T16 path + the T12-override (gate amendment):** with simulated 605A∈5..7 behavior (drive HOLDS QSA): (a) goal=OE re-issued from QSA → 0x0F emitted (T16 works where the drive supports it); (b) goal=RTSO re-issued from QSA → 0x00 (T12) routed → sim lands SOD → re-walks to RTSO (the override principle — RTSO reachable on holding drives).
9. **remote() gate:** bit9=0 → state frozen, cw stable, cycles_in_state grows, remote()==false.
10. **Decode hygiene:** sw with bit10/bit14 randomized → identical decode/behavior.
11. **get_cw idempotence:** N calls between updates → identical value.
12. **HOLD-at-SOD stability regression (B1):** a sim drive that honors T2-on-0x06 (DS402-conformant + A6 per bench), goal=SOD → assert the state STAYS SOD for N cycles (fails on a 0x06-hold, passes on 0x00).
13. **Handshake abort (B2):** fault injected mid-AwaitAck → after recovery (reset + re-enable), `setpoint_phase()==Idle` and a fresh `set_point()` succeeds (no Busy-forever).
14. **Adoption / initial goal (D3):** construct + first update() in each state → goal adopts current (sustain cw emitted; notably OE → 0x0F sustained, NOT dropped); no transition commanded until a set_state arrives.
15. **Uncommanded-QSA goal-cancel (B3):** external QSA injection mid-walk-to-OE (previous state ≠ QSA, no quick_stop() issued) → goal cancelled; stop releases (sim returns to SOD/OE-capable) → NO re-walk; AND a goal issued DURING the external QSA still routes (T12-override intact — entry-edge, not level).

## 11. Boundaries / non-goals
- **Never touches 0x6060** (mode writes are configure-time, library-side) — mode is validation-only state.
- **No vendor-fault knowledge** (Er74/0x2031:01 = consumer policy, #39/#22).
- **No timeouts inside the FSM** (caller policy via cycles_in_state + remote + setpoint_phase).
- **No port I/O, no Master dependency** — pure sw-in/cw-out; testable without any bus.

## 12. Migration (task #41, after DA-gated implementation)
servo_controller + a6_validate migrate off their hand-rolled enable ladders / bit-7 edges / bit-4 handshakes onto update()/set_state()/get_cw(). Phased like #30/#31: library+tests first (offline), consumers after (HW-gated: OP + sine + a PP move for the bit-4 path).

---
**DA gate: CONFIRMED (2026-06-10)** after revisions — B1 (HOLD-at-SOD=0x00 + sustain table), B2 (handshake abort on leaving OE), D3 (adopt-current initial goal), the QSA→RTSO/SO T12-override amendment, minors a–e. DA concurred: fault-cancels-goal (strongly), per-cycle re-planning, remote()-no-internal-timeout, composition order (+ the quick_stop/reset state-exclusivity simplification). Vendor-leak lens: CLEAN (the #41 pattern). **Implementation-review gates (cpp-expert pre-empt):** (1) routing core matches §3 cell-for-cell (DA diffs exhaustively); (2) all 14 tests present + non-vacuous (test 12 must FAIL against a 0x06-hold — prove it the broken-baseline way; test 13 fault-mid-AwaitAck; test 8 holding-sim T16 + T12-override); (3) RT-residency: noexcept/no-alloc/no-exceptions on update()/get_cw(); (4) vendor-leak: no A6 constants/objects/timeouts in the implementation.