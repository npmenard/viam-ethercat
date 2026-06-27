# Spec #47 — `SlaveControl` interface + library-owned cyclic `Runner`

**Status:** architect spec, awaiting **DA pre-implementation gate** (standing user directive). USER-REQUESTED (2026-06-11, un-parks review-round-2 comment 6). **Lands BEFORE #37** (the module finale then validates the final architecture); #21/#25 ride behind it.
**The inversion:** today consumers own the RT thread/pacing and call library primitives. After #47, **the LIBRARY owns the RT thread, realtime setup, bring-up, PD exchange, pacing, and teardown**; the consumer derives `SlaveControl` and implements per-cycle policy in `step()`. DcPacer/pacing disappears from ALL consumer code.

---

## 0. Locked shape (from the task — restated, not open)
- `class SlaveControl { virtual void step(CycleContext&) = 0; /* optional lifecycle hooks */ };`
- attach control to a slave; the library runs everything from RT-spawn to teardown.
- Pacing strategy selected by the LIBRARY from bus config (DC → phase-locked; free-run → pure period); **consumer `step()` identical either way** (the user's motivation: DC↔free-run = a config change, zero consumer-code change).
- step() RT contract: once per cycle at a fixed point (inputs latched → step() → outputs staged/sent); bounded, no alloc, no blocking, **noexcept**; not called before OPERATIONAL; `on_operational()` marks the transition.
- **The inversion's biggest enforceability WIN (DA gate): the cadence is enforced BY CONSTRUCTION** — the library owns the call site, so once-per-cycle/fixed-point CANNOT be violated by a consumer (contrast #38's update()-contract-on-the-caller). The strongest tier of the §3b hierarchy, for free.
- **Misbehaving-step degradation direction (documented fails-safe):** a blocking/slow step() → a pacer overrun → DcPacer's phase-preserving catch-up + WKC/watchdog VISIBILITY — degrades observable-and-recoverable (skipped cycles, counters move), never silent corruption, never an off-phase burst.
- Errors: `ctx.request_stop()` + library-latched fault state readable non-RT; NO exceptions across the RT boundary.
- Layering proof: ServoController, a6_validate's sine, and #21 all become derived `SlaveControl`s — the library eats its own interface.
- Multi-slave: one control per slave, stepped in slave order each cycle (1 today; design for N).
- All locked invariants carry: PD-flows-through-transitions, single-port-owner, copy-based RT boundary for non-RT consumers, vendor-clean library (the #39 sweep test stays green).

## 1. ARCHITECT CALL: a separate `ethercat::Runner`, NOT `Master::run()`
Master stays **pure bus policy** (config/remap/bringup_step/process — caller-paced primitives, thread-free), exactly the #31 boundary that has held all project. The Runner is the **orchestration layer** that composes them:
```cpp
class Runner {
  Runner(Master& master, RunnerConfig cfg);          // master: open()+init()+configure() DONE (throwing, consumer's phase)
  void attach(std::uint16_t slave_id, SlaveControl& control);  // pre-start only; throws ConfigError after start/unknown slave/dup
  void start();    // non-RT hooks → lock_current → spawn RT thread → (in-thread) realtime::setup → bring-up → steady
  void stop();     // graceful: stopping-window → loop exit → join → master.close()
  void run();      // convenience: start() + block until stopped (the a6_validate / #21 shape); SIGINT via request_stop
  RunnerStatus status() const noexcept;  // non-RT: Running / BringingUp / Stopped{StopReason} + latched detail
};
```
- Rationale: (a) Master's surface stays small + thread-free (testable as today); (b) the Runner is where the `set_rt_active` bracket (#39), the spawn/join, and the teardown live — ONE audited place instead of N consumer paths; (c) consumers that genuinely need a custom loop (none after the migration) still have the primitives.
- **Consumer split:** `open/init/configure` remain the consumer's throwing config-phase (clear errors at config time — unchanged); `Runner` owns everything from `start()` on.

## 2. `SlaveControl` (the consumer interface)
```cpp
class SlaveControl {
 public:
  virtual ~SlaveControl() = default;
  // NON-RT, pre-spawn, MAY THROW: resolve typed fields (resolve_rx/resolve_tx<F> + the width assert),
  // read one-time SDOs (e.g. the 0x605A regime readback), validate config. The ONLY throwing hook.
  virtual void on_configured(Master& master, std::uint16_t slave_id) {}
  // RT, once, first cycle AT Operational (before the first step()).
  virtual void on_operational(CycleContext& ctx) noexcept {}
  // RT, once, on entering the stopping window (fault, request_stop, or stop()): reason provided.
  virtual void on_stop(StopReason reason) noexcept {}
  // RT, every steady cycle: inputs latched → step() → outputs sent. Bounded/no-alloc/no-block/noexcept.
  virtual void step(CycleContext& ctx) noexcept = 0;
  // RT, every BRING-UP cycle: "is the drive reporting a sync fault?" — the bring-up gate's
  // Er74.1-class signal. DEFAULT false. The CiA402-aware control implements it (e.g. mapped
  // 0x603F == 0x8700). KEEPS Master AND Runner vendor/CiA402-free (the #20 layering, preserved).
  virtual bool sync_faulted(const CycleContext& ctx) const noexcept { return false; }
};
```
- Hook set is minimal-but-sufficient: `on_configured` (the throwing home — field resolution moves here from ServoController::resolve_fields), `on_operational`, `on_stop(reason)` (covers fault/requested/teardown in one — `on_fault` is `on_stop(StopReason::BusFault/BringupAborted)`), `step`, `sync_faulted`. All default-no-op except step.

## 3. `CycleContext` — RT-safe surfaces ONLY (resolves the #30-API tension honestly)
The task says "typed rpdo/tpdo access (the #30 API)". **The #30 API is DUAL, and only its RT form is admissible in step()** — `Rpdo`/`Tpdo` are the THROWING copy forms (per-call resolve), banned by the step() contract. So:
```cpp
class CycleContext {
 public:
  // THE typed RT form (#30): noexcept load/store at consumer-cached FieldLocations,
  // resolved ONCE in on_configured(). Slave-scoped images.
  template <PdoScalar T> T load(FieldLocation loc) const noexcept;          // from the latched input image
  template <PdoScalar T> void store(FieldLocation loc, T v) noexcept;      // into the output image — SHIPS WITH CYCLE N+1's
                                                                            // process() (the same 1-cycle command latency P2b
                                                                            // verified on HW; step authors: a store this cycle
                                                                            // is on the wire NEXT cycle)
  std::uint64_t cycle() const noexcept;          // steady-cycle counter (0 at first step)
  std::int64_t dc_time_ns() const noexcept;      // 0 when DC disabled
  bool stopping() const noexcept;                // the stopping-window flag (§5)
  void request_stop() noexcept;                  // the RT-side error/stop channel (latches StopReason::Requested)
  WkcStats wkc() const noexcept;                 // the #40 stats (relaxed)
};
```
- **Non-RT consumers keep the copy forms:** the module's gRPC accessors etc. continue using atomics/`read_rpdo` OUTSIDE step() — the copy-based RT boundary invariant unchanged. `Rpdo`/`Tpdo` are deliberately NOT on CycleContext (they throw + copy; a derived class wanting them off-thread uses Master, not ctx).
- No Master&, no SDO, no map access on ctx — structurally unreachable from the hot path. No raw image pointers escape ctx (load/store at handles only — the librarian's fastcat lesson: their devices touch the PD image directly, the shortcut our boundary exists to fix). One virtual `step()` call per slave per cycle is the accepted dispatch cost (fastcat's model at the same rate; the typed-handle path INSIDE step is where per-cycle cost matters).

### 3b. step()-contract ENFORCEMENT TIERS (DA criterion 1 — which tier each element gets)
| Element | Tier | Mechanism / failure direction |
|---|---|---|
| step()/hooks noexcept | **compiler** | `noexcept` on the virtuals — an escaping exception = terminate (fails CLOSED, loudly) |
| no exceptions across RT | **structural** | every ctx surface noexcept; the only throwing hook (on_configured) is pre-spawn by construction |
| no map-walk / typed access | **structural** | ctx has no map/resolve surface; FieldLocations are pre-resolved handles |
| no SDO during RT | **runtime-checked** | the #39 `rt_active` guard — Runner-bracketed; violation THROWS in release (fails CLOSED) |
| once-per-cycle cadence | **runtime-checked (debug)** | a `live_` window flag on ctx; a ctx use outside its dispatch window asserts in debug. **Defined release behavior: impossible by construction (the Runner is the only step() caller, one dispatch per control per cycle in one loop) — the debug assert guards REFACTORS, not consumers** |
| ctx escape (stash + use out-of-window) | **owned-data + debug assert** (TODO-1, supersedes the original "shared spans + always-on violations counter") | ctx holds its input/output images BY VALUE (arrays @ kMaxPdoBytes), NOT spans into the live process buffers; dispatch() copies in before / seeds + copies out after each hook. So an escaped handle reads a VALID object with STALE data (never a dangling/in-flight read at 1 kHz) and a store through it lands in the owned buffer + never reaches the wire. The escape is made HARMLESS rather than merely counted — so there is NO release-mode violations counter (the original §3b counter is DELETED); the debug assert is the only check, and the worst release case is safe-stale, not corruption. CAVEAT: safe-stale holds only within the Runner's lifetime (the ctx lives in controls_); a handle outliving the Runner is UAF (TODO-3 territory). |
| bounded / no-alloc / no-block in step() | **doc-only (last resort)** | unverifiable in-library; fails OPEN (an overrunning step = a pacer overrun → the re-read catch-up absorbs it, WkcStats/cycle-overrun counters make it VISIBLE non-RT). Document: "a persistently-overrunning step() degrades to skipped cycles, never to an off-phase burst" |
| hooks must not re-enter Master/Runner | **runtime-checked (debug)** | a debug re-entrancy flag around dispatch; doc the rule (same-thread, no master calls — ctx is the whole legal surface) |

## 4. Pacing: ONE DcPacer, both strategies — selection is an INPUT, not a class hierarchy
P3c already proved the unification: **`pacer.pace(dc_enabled ? master.dc_time() : 0)`** — dc_time==0 → correction 0 → pure-period sleep (the DcPacer contract since a32f651). So "the library selects the pacing strategy from bus config" is literally: the Runner constructs ONE `DcPacer(period)` (period/2 target, #40 default) and feeds it `dc?dc_time:0`. No strategy classes (YAGNI), no consumer knowledge, and the **ONE-pacer-gapless-across-bring-up→steady invariant becomes structurally unsplittable** — the pacer is a Runner local that consumers cannot touch.

## 5. The Runner's RT thread (the proven sequence, formalized in ONE place)
```
start():  for each control: on_configured(master, slave)   [non-RT, may throw → start() throws, nothing spawned]
          realtime::lock_current(); master.set_rt_active(true); spawn RT thread
RT:       realtime::setup(rt_priority)  [require_realtime semantics preserved: failure + require → latched abort]
          BRING-UP: pace + master.bringup_step(any control's sync_faulted())   [the run_to_operational internals,
                    inlined so the pacer is THE one pacer; bounded by bringup_timeout → Aborted]
          on Operational: per control on_operational(ctx)
          STEADY:   process() → latch inputs → per control (slave order): step(ctx) → outputs already staged → pace()
          STOP (fault latch | request_stop | stop()):  per control on_stop(reason);
                    stopping WINDOW: `teardown_cycles` more steady cycles with ctx.stopping()==true
                    [the control's POLICY disables its drive here — e.g. CiA402 cw→0x00; PD KEEPS FLOWING]
          exit loop
stop():   join; master.set_rt_active(false); master.close()   [drive→INIT — the b9f7b5f-proven close, unchanged]
```
- **Teardown layering:** the PD-flowing disable is CONSUMER policy (CiA402 knowledge) → it lives in step()-during-stopping, NOT in the Runner. The Runner provides the bounded window + the flag; `master.close()`'s proven INIT teardown is unchanged. A control that ignores `stopping()` still gets a safe outcome (window expires → close → INIT). **Window default: `teardown_cycles = 100` (100ms @ 1kHz), config-tunable, floor 1** — the window is for POLICY (the CiA402 disable needs only cw-delivery + observation); **safety does not depend on it.**
- **Fault DURING the stopping window → the window CONTINUES to expiry, then close (FINAL — settled at the gate; DA's exit-early proposal was examined and WITHDRAWN by DA; this row + test 11 pin it; any future change = a fresh gate, not a crossed message):** a BusFault latch (consecutive short WKC) is NOT a provably-dead bus — in the partial-fault case, continuing gives the control's disable policy a chance to still reach the drive; in the truly-dead case the cost is ≤ teardown_cycles of harmless no-reply cycles before close() (note: a dead bus makes receive block up to EC_TIMEOUTRET per cycle, so wall-clock ≈ 2-3× the cycle count — still bounded, still harmless). **The multi-slave escalation makes window-continues STRICTLY better (DA's settling argument):** the bus-wide stop means ANY cause stops ALL controls — in a multi-slave PARTIAL failure (one slave's WKC contribution gone, others alive — exactly what a latched short-WKC can mean), exit-early would slam close()→INIT on the HEALTHY drives without their disable policy ever running; window-continues lets the survivors disable gracefully while the dead one loses nothing. close()'s b9f7b5f INIT path is the safety backstop either way (window = policy, not safety). Test-pinned (P1.11, the window-COMPLETES direction; baseline: an early-exit implementation fails it — the test pins the decided behavior against exactly the flip-flop that produced it).
- **Steady-state health reliance (documented so nobody "improves" it later):** steady health = **the WKC latch** — an AL-state regression manifests as a WKC change (the canonical CLAUDE.md signal). The Runner deliberately does NOT poll AL state in steady (a readstate poll on the hot path is the ec_sample 0x001B flood class).
- **Why on_stop-on-RT + status()-polling suffices (no library-side non-RT completion callback — DA-concurred, recorded so it isn't re-litigated):** the consumer that needs to LEARN of unexpected stops (the module's gRPC state) already owns the atomic-publish pattern — its derived on_stop(reason) publishes to its own atomics on the RT thread; non-RT accessors read them; status() corroborates. A library-side non-RT callback would create a who-runs-it third-context question for zero gain.
- **Faults:** Master's existing WKC latch + bring-up abort → StopReason::{BusFault, BringupAborted}; ctx.request_stop() → Requested; stop() → Requested; **+ RtSetupFailed** (require_realtime abort — as-built, architect-ratified: every abort cause has a name). First-cause wins (CAS latch). All latched, readable via `status()` non-RT (the no-exceptions-across-RT rule). Bring-up abort does NOT auto-retry (the wedge lesson — bounded give-up; restart is the consumer's call). **Bring-up abort gets NO stopping window (as-built, ratified):** the window exists for the DISABLE policy, and pre-OP nothing was ever enabled — abort → close directly. **Window semantics precise (ratified):** exactly `teardown_cycles` stopping steps, entry cycle included, floor 1. **Public `Runner::request_stop()`** (the signal-handler shape for blocking run()) — as-built, ratified. (TODO-1 **removed** the always-on violations counter that earlier sat here: ctx is now owned-by-value, so an out-of-window escape is safe-stale rather than corrupting — the §3b table's "ctx escape" row carries the justification; a debug assert is the only check, no release counter.)
- **Multi-slave:** one process() exchanges all PD; controls stepped in attach/slave order; per-control slave-scoped ctx. sync_faulted during bring-up = OR over controls. **Fault escalation default (fastcat's ALL_DEVICE_FAULT analog): any per-slave stop cause (request_stop from ANY control, bus fault, bring-up abort) stops the WHOLE Runner** — bus-wide, conservative, correct at N=1; per-slave detach/degraded modes are explicitly OUT of #47 (future, with a real multi-slave user in hand — the with_sdo principle).
- **DC-mandatory is CONFIG data (DA criterion 2):** the Runner reads `MasterConfig::use_distributed_clocks` (+ per-slave `sync_cycle_granularity_ns`) — it never hardcodes which drives need DC (vendor lens; the A6's DC-required-ness lives in the consumer's config, as today).

### 5b. UNCOMMANDED-ENTRY SWEEP (DA criterion 3b — the B3-generalized question, per Runner phase)
What happens when the BUS/DRIVE moves while the Runner holds an intent — enumerated:
| Runner phase | uncommanded event | defined outcome |
|---|---|---|
| BringingUp | drive faults / sync_faulted persists | bringup_step's bounded give-up → Aborted → on_stop(BringupAborted). **No re-request (the no-hammer invariant survives every abort path — the Runner requests OP exactly once per start())** |
| BringingUp | slave drops off the bus | WKC/bringup failure → same bounded abort path |
| Steady | WKC collapses / slave leaves OP | Master's existing fault latch → StopReason::BusFault → on_stop → stopping window → close. **The Runner NEVER auto-re-walks to OP** (restart = consumer's call via a fresh start() — the B3 no-uncommanded-re-energize principle at the Runner tier) |
| Steady | drive faults (CiA402) but bus stays healthy | NOT the Runner's concern — drive-level state is the CONTROL's policy (its FSM sees the statusword in step(); the #38 FSM's own B3/fault-cancel rules apply INSIDE step) |
| Stopping window | further bus fault | window continues (PD keeps flowing if possible); close proceeds at expiry regardless |
| Stopped | anything | inert; status() reads the latch |
- The two-tier split is the load-bearing line: **bus-level uncommanded events = Runner (latch + stop, never auto-recover); drive-level uncommanded events = the control's step() policy (the #38 FSM already carries B3).** No event is silently absorbed.

## 6. Migration (the layering proof — each phase gated)
- **P1 (offline):** Runner + SlaveControl + CycleContext + tests (SimBackend), **with broken-baseline requirements DECLARED up front (DA cross-cutting — "prove it fails against X" in the list, not discovered at gate):**
  1. Full lifecycle (configured→operational→step×N→stop window→closed) + hook ORDER. *Baseline: reorder hook dispatch → fails.*
  2. **not-stepped-before-OP** — step never dispatched pre-Operational. *Baseline: remove the OP gate → fails.*
  3. **Exhaustive (phase, event) matrix (DA criterion 3a):** every §5/§5b cell value-asserted (outcome + StopReason + hook calls), incl. the give-ups — the #38.1 pattern.
  4. request_stop latching + fault→on_stop(reason) per cause (the defined-return-per-abort-cause rule). *Baseline: drop the latch → fails.*
  5. **No-hammer:** exactly ONE OP request per start() across every abort path. *Baseline: allow a re-request on abort → fails.*
  6. Multi-slave ordering (2 sim slaves) + the bus-wide stop escalation (one control's request_stop stops both).
  7. Pacing-input selection (dc vs 0) — both regimes through the SAME pacer, step() identical.
  8. attach-after-start throws; on_configured-throw → clean no-spawn abort (nothing locked, no thread, master untouched).
  9. **Stopping window:** ctx.stopping() visible to step(); window-bounded; a window-ignoring control still reaches close. *Baseline: skip the window → the disable policy never runs → the test's drive-state assert fails.*
  10. **Owned-data value semantics (TODO-1):** an escaped ctx (stashed, used out-of-window post-join) reads stale-but-DEFINED data (no UB), and a store through it never reaches the wire (lands in the owned buffer); the stop()-from-RT guard degrades (no self-join deadlock). NDEBUG-only (the out-of-window touch asserts in debug — the manual broken-baseline: remove the `live_` guard → the assert no longer fires). *Runs sanitized only under an NDEBUG TSan lane (#50); the rt_tid_ guard access is atomic (#49).*
  11. **Fault-during-stopping-window: the window COMPLETES** (the settled §5/§5b rule): stop() → mid-window BusFault injection → the window runs to expiry (PD attempts continue) → closed. *Baseline: an exit-early implementation fails the cycle-count assert (window cut short).*
  Both compilers + TSan (the Runner thread vs status()/stop()). **GATE PROCEDURE RULE (DA, learned at the P1 gate on themselves): every gate's reproduction runs MUST include a RELEASE-config run** — NDEBUG-gated test branches silently stub out in Debug (the P1 instance: test 10's real self-join branch is NDEBUG-gated; a Debug-only reproduction exercises the stub and reads green). Applies to every #47 phase gate and onward.
- **P2 (HW-gated):** a6_validate's sine → a small derived `SineControl` (its CSP policy in step(); Phase-1 diagnostics via a Runner-status poll or the observer-equivalent — design detail for cpp-expert within the contract). HW: non-energized probe, then **user-gated energized sine**.
- **P3:** ServoController **refactors into a derived SlaveControl** (its CiA402 FSM/command policy = step(); resolve_fields → on_configured; its spawn/join/bracket/pacer code DELETED — the Runner owns them). Offline + TSan; HW rides the next bench. The api_mutex/lifecycle surface stays ServoController's (it wraps Runner start/stop).
  - **Post-layering OWNERSHIP TABLE (DA criterion 4a — exactly one owner each):**
    | mechanism | owner after P3 |
    |---|---|
    | RT thread spawn/join | Runner (only) |
    | the pacer | Runner (only; a local — untouchable) |
    | set_rt_active bracket | Runner (only — supersedes the #39 placement in ServoController; named ruling change) |
    | realtime::setup / lock_current | Runner (only) |
    | bring-up pump + give-up | Runner (only) |
    | CiA402 FSM / command policy / drive teardown-disable | the derived control (only) |
    | field resolution | the derived control's on_configured (only) |
    | gRPC accessors / api_mutex / lifecycle API | ServoController shell (only) |
  - **Phase-3 invariant SURVIVAL list (DA criterion 4b — how each survives the re-layering):** master_-free atomic accessors — UNCHANGED (the control publishes to the same atomics from step(); accessors never touch master_); join-before-destroy / UAF-vs-reconfigure — **the invariant TRANSPOSES to: `Runner::stop()` (join inside) BEFORE `master_.reset()`, ALWAYS** (the Runner holds Master& — resetting master under a live runner is the NEW UAF shape; the shell's reconfigure = stop runner → reset master → rebuild → start runner; carried into P3's test list); lock ordering — api_mutex stays the shell's outermost lock, the Runner takes NO locks shared with the shell (structural: it has none).
  - **Zero-duplicated-mechanism evidence (DA criterion 4c):** post-P3 grep of servo_controller — no jthread/spawn/join/setup_realtime/DcPacer/pace/set_rt_active/bringup_step (the no-hand-rolled-X pattern applied to everything the Runner now owns).
  - **SDK lifecycle → Runner teardown mapping (TODO-3, librarian SDK-contract evidence).** viam-cpp-sdk gives exactly ONE deterministic pre-destruction hook and NO process-exit hook; the de-energize story is two software tiers + a HW backstop. **The Runner ALREADY satisfies this by construction (§5) — this row PINS the wiring + ownership, it adds no new mechanism:**
    - **Tier 1 — deterministic, SDK-fired.** `rdk:component:motor`'s `Stop()` IS `Stoppable::stop()`; the SDK calls it pre-destruction on `RemoveResource` and on reconfigure-replace, **on the gRPC thread**. Wire `ServoMotor::stop()` → `ServoController::stop()` → `Runner::stop()`. **Single-port-owner preserved BY CONSTRUCTION:** the gRPC thread NEVER touches the drive — `Runner::stop()` only flips `stop_flag_` + joins; the actual de-energize is the control's disable policy in `step()` **on the RT thread**, during the stopping window. This IS the "command de-energize THROUGH the RT boundary" handshake — already built. `Runner::stop()` blocks on `join`, which cannot unblock until the bounded window (`teardown_cycles`) has shipped the disable with PD flowing → `stop()` is inherently command-and-confirm with a bounded ceiling (the window is the timeout; a *wedged* RT loop is the separate join-liveness question, not this hook's concern).
    - **Tier 2 — process-exit / `stop()`-never-called.** The resource DTOR. Reference pattern (UR/yaskawa): resource teardown lives in the DTOR (noexcept, abort-guarded); `~Runner()` ALREADY calls `stop()` → same graceful window+disable+close. **Ownership pin (NEW P3 constraint): ServoController owns its Runner by value / `unique_ptr`** so `~ServoController → ~Runner` de-energizes even when the SDK never called `stop()` (bare `shared_ptr` drop, a test, an exception path). **Idempotency REQUIRED and as-built:** a double teardown (SDK `stop()` THEN dtor) de-energizes once — the 2nd `stop()` sees `rt_` not-joinable (skips join), `set_rt_active(false)` is an idempotent store, `close()` is "idempotent at the backend" (b9f7b5f). Test below.
    - **Tier 3 — abrupt death (SIGKILL, no dtor runs).** The drive's own SM watchdog (~50 ms PD-gap → fault-to-safe; the 0x001B mechanism). SOFTWARE makes NO guarantee here by design — the HW backstop does.
    - **Honest GAP (librarian):** the SDK installs NO process-exit hook. A clean SIGTERM de-energizes only if the module `main` runs resource dtors on the signal (orderly shutdown → Tier 2); otherwise → Tier 3. SIGKILL → Tier 3 only. Documented in a6-quirks/runbook, NOT engineered around (the SM watchdog is the contract).
    - **Ordering vs the UAF rule:** Tier-1 `stop()` (join inside) precedes `master_.reset()` ALWAYS (the transposed invariant above) — the SDK's `stop()`-before-drop satisfies it for remove/replace; the dtor-chain satisfies it for process-exit (Runner destroyed → joined → only THEN may the shell drop Master).
    - **P3 test (TODO-3, declared baseline):** (a) `stop()` then destroy → the disable cw appears EXACTLY once on the sim wire; `close()` twice is no-op-safe. (b) destroy WITHOUT `stop()` → still de-energizes (the Tier-2 dtor path). *Baseline: hold the Runner non-owning (raw ptr / external lifetime) → case (b)'s dtor no longer chains → the no-`stop()`-then-destroy assert fails.*
- **Then #21 = trivial** (a ~10-line derived control + run()).

## 7. Invariant carry-list (DA gate checklist)
1. step() unreachable before OPERATIONAL; on_operational precedes first step (test-pinned).
2. ONE pacer, gapless, consumer-untouchable (structural — it's a Runner local).
3. PD flows through SAFE-OP→OP and through the stopping window (no gap → no 0x001B).
4. set_rt_active bracket = Runner-owned spawn/join (the #39 guard, now ONE site; the seam/with_sdo note in #22 §7 unaffected).
5. No throw/alloc/block on the RT path: step noexcept; ctx surfaces noexcept; the only throwing hook (on_configured) is pre-spawn.
6. Vendor/CiA402-clean Runner+Master: sync_faulted + all CiA402 policy live in derived controls (the #39 sweep test green; grep: no 0x603F/0x8700/0x6040 in Runner).
7. require_realtime preserved; bringup_timeout bounded give-up (no hammer); b9f7b5f close unchanged.
8. Copy-based RT boundary for non-RT consumers unchanged (Rpdo/read_rpdo stay the off-thread forms; NOT on ctx).
9. **De-energize-on-destruction is STRUCTURAL** (P3 SDK-mapping row): `~Runner()→stop()` runs the graceful disable window even when `Stoppable::stop()` was never called; ServoController owns the Runner by value/`unique_ptr` so destruction chains; idempotent across SDK-`stop()`-then-dtor. HW SM-watchdog is the SIGKILL backstop; software makes no claim past the dtor.

**DA gate focus:** §1 Runner-vs-Master call; §3's honest resolution of "the #30 API" (RT form only on ctx — the throwing copy forms excluded by the locked step() contract); §5 teardown layering (policy-in-control, window-in-Runner) vs the b9f7b5f invariant; the sync_faulted hook as the vendor-clean bring-up gate; the stopping-window default (teardown_cycles) sizing; whether on_stop on the RT thread is sufficient or a non-RT completion callback is also needed (I say status() polling suffices — challenge it); **the P3 SDK-lifecycle mapping (TODO-3): the `Stop()`=`Stoppable::stop()`→`Runner::stop()` gRPC→RT handshake, the by-value Runner ownership for the dtor de-energize path, and the SIGTERM/SIGKILL gap honesty (challenge whether the two-tier+watchdog story leaves any de-energize hole).**