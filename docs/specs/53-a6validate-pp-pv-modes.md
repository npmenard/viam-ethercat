# Spec 53 — a6_validate: PP move-to-position + PV velocity modes

Two new **energized** motion modes for `a6_validate` (the Runner/`SlaveControl`
bench tool, `A6Control`). Units are raw **counts** throughout (POS in counts, VEL in
counts/s) — no rpm/gear conversion (that is the *module's* job, not this low-level
tool). The tool is **mode-fixed per invocation**: the CLI flag picks PP | PV | CSP;
there is **no runtime PP↔PV switching**.

Decisions ruled by team-lead (this spec transcribes them; DA gates).

## CLI

- `--move-pos POS [VEL]` — absolute Profile-Position move-to. Requires `--enable`
  (fail-closed: a move flag without `--enable` refuses to energize). `POS` = absolute
  target (counts); `VEL` = profile velocity (counts/s, default if omitted). Selects
  `Cia402Mode::ProfilePosition` (`0x6060` = 1).
- `--move-vel VEL` — Profile-Velocity, runs until Ctrl-C. Requires `--enable`. `VEL` =
  target velocity (counts/s). Selects `Cia402Mode::ProfileVelocity` (`0x6060` = 3).
- Mutually exclusive with each other AND `--move-pp` / `--move-sine` / `--csp-probe`
  (extend the existing exclusivity check).

## PDO map (DONE — 137eecb)

RxPDO `0x1600` superset: `{ctrl 0x6040, 0x607A target-pos, 0x6081 profile-vel,
0x60FF target-vel}` (~14 B, limit 40). The drive acts only on the objects its current
`0x6060` mode uses, so one map serves PP/PV/CSP. `cia402::TargetVelocity`
(`0x60FF/0/i32`) already exists on master.

## Mode set + echo (fail-closed)

`0x6060` set **once at configure** (`a6.default_mode` from the CLI mode). After SAFE-OP,
read `0x6061` and require it == commanded **before** enabling; mismatch → **refuse to
enable** (the A6 silently ignores unsupported mode-sets — TODO-45). No mid-run switch.
- **IMPLEMENTATION MUST-DO (DA-B, load-bearing for PV):** the current `a6_validate.cpp:390`
  only STORES `0x6061` to telemetry — there is NO `== commanded` check. TODO-45's echo gate
  was the MODULE path, NOT this tool. #53 adds PV (`0x6060=3`); if the A6 doesn't honor `=3`,
  `--move-vel` streams `0x60FF` into a drive still in PP/CSP → undefined ENERGIZED behavior.
  `mode_loc_` is already resolved — add the `0x6061==requested` gate at enable, refuse otherwise.

## A6Control — absolute PP (`--move-pos`)

- On enable (first `OperationEnabled`): set `0x607A = POS` **absolute** (controlword
  **bit6 = 0**, NOT relative) and `0x6081 = VEL`.
- New-setpoint handshake — a **true rising edge** per setpoint: controlword bit4 0→1 →
  await sw **bit12** (set-point acknowledge) → clear bit4 → await bit12 clear. Exactly
  one edge; never leave bit4=1 across setpoints (else the next target is silently not
  latched).
- **Reached** = `|0x6064_actual − POS| ≤ kPosReachedTol` counts AND `|0x606C| ≤ kZeroVelThresh`
  (debounced, same as PV §). Report "reached", then hold (re-send cw=enable, no new bit4 edge).
  Do NOT use A6 sw bit10 ("target reached" is quirkily always-set — TODO-43).
  - **`kPosReachedTol` (DA-C, must-fix):** the existing code uses `< 300` (a6_validate.cpp:104,
    HW-tested); the task asked for `≤ 50`. Tightening 300→50 risks **never-completes** (infinite
    settle-spin) if the A6's positioning deadband parks outside ±50. **Default to the PROVEN 300**
    (configurable via `--pos-tol`); 50 is achievable ONLY if a bench measurement shows the A6
    deadband ≤ 50 — verify before tightening. A "reached 300 counts early" report is a benign
    validation outcome; an infinite spin is not.
- The absolute target is measured against the **same zero reference** as feedback/SetZero.
- **DA-C, must-fix — do NOT flip the existing `--move-pp`:** `--move-pp` is a working, HW-tested
  path (bit6 currently unset → 0 → it's an absolute-move-to-a-computed-relative-target). #53
  ONLY ADDS `--move-pos` (bit6=0, reusing the existing bit4↔bit12 handshake verbatim). Do NOT
  retro-flip `--move-pp` to bit6=1 (true relative) as part of #53 — that is a behavior change to
  a green path requiring its own HW re-validation. Leave `--move-pp` byte-for-byte as-is.
- `--follow-err-limit` still aborts + disables on `|cmd − actual|` runaway.

## A6Control — PV (`--move-vel`)

- Mode `ProfileVelocity` (`0x6060` = 3).
- On enable: stream `0x60FF = VEL` each cycle. Drive ramps via `0x6083` (accel).
- **(DA-G) defensive: write `0x607A = live 0x6064` each PV cycle.** `0x607A` (target position)
  is over-mapped but inert in PV per the superset map — but if it holds a STALE value and the
  A6 cross-supervises position, it could throw a following-error. Mirroring `0x607A` to the
  live actual position keeps it benign. HW-confirm: no `0x603F` following-error during PV.
- **Stop (Ctrl-C → `ctx.stopping()`): CiA402 Quick-Stop.** Command the Quick-Stop
  transition (T11, OperationEnabled→QuickStopActive) — **controlword `0x0B`: CLEAR bit2
  while KEEPING bit1 (Enable Voltage) and bits 0,3** (`0x0F & ~0x04`). *Polarity matters:
  the Quick-Stop command is bit2 = 0, NOT bit2 = 1 — enable-operation `0x0F` already has
  bit2 SET; a wrong polarity = no quick-stop = the torque-cut-at-speed we are preventing.*
  (DA: `0x0B` is the mask-style command; `0x02` is the textbook-minimal exact value — if the
  A6 turns out to decode the QS command by EXACT match rather than mask, `0x02` is the
  conformant fallback. `0x0B` should work; HW-checkable.) The motor transitions to
  QuickStopActive **IMMEDIATELY at full speed** and decelerates DURING QuickStopActive (an
  ENERGIZED decel state, `sw & 0x6F == 0x07`) — NOT during OperationEnabled.
- **WHO de-energizes — regime depends on `0x605A` (Quick-Stop option code); this is
  load-bearing (DA Finding 2 + a6-quirks Q4):**
  - **PRIMARY (A6 factory default `0x605A = 2` — "decel on `0x6085` ramp, then AUTO-transition
    to SwitchOnDisabled"):** the **DRIVE** ramps via `0x6085` and **self-de-energizes** at its
    OWN zero (→ SwitchOnDisabled). The control just holds `cw = 0x0B` and observes; it does NOT
    need to command the disable.
  - **BACKSTOP (`0x605A ∈ {5,6,7}` "stay in QuickStopActive"):** the drive ramps but holds
    QSA energized at zero → the control's event-driven path takes over: watch `0x606C`; once
    `|0x606C| < kZeroVelThresh` for **`kZeroVelDebounce` consecutive cycles** (debounce so a
    velocity-estimate noise dip can't disable mid-decel), command `cw → 0x00` (disable voltage
    → SwitchOnDisabled). This path is also the no-op-safe fallback under `=2`.
  - **ASSERT, do NOT set (DA + Q4):** READ `0x605A` at configure and **require `== 2`; refuse
    to energize otherwise.** A drive reconfigured to `0` (coast — no decel!) or `1` (decel on
    `0x6084`, not `0x6085`) silently breaks the `0x6085` premise. Do NOT `sdo_write` `0x605A` —
    Q4: a `0x605A` change takes effect only after a control-power cycle, so a warm write won't
    apply; read-and-assert is the only correct move.
  - So the control's event-driven `cw→0x00` is correctly a **BACKSTOP**, not the primary, at the
    default — the spec earlier framed it backwards. Both regimes converge on SwitchOnDisabled;
    the design is regime-robust.
- **`kZeroVelThresh` concrete:** set just ABOVE the measured `0x606C` velocity-estimate
  noise floor (a one-time bench data-point). Start at **≈ 500 counts/s** (≈ 0.23 rev/s at
  2^17 counts/rev — genuinely stopped, a negligible cut if disabled there), `kZeroVelDebounce
  = 5` cycles. NOTE: ≈ 50 counts/s (the earlier guess) is likely BELOW the noise floor →
  the event may never fire → it safely falls back to the window upper bound (disable at
  window-end, motor long stopped) but the bench-exit is slow; 500 c/s reliably fires early.
  **(DA refine) the bench tune must capture the velocity-estimate noise CORRELATION TIME, not
  just amplitude:** a 5-cycle debounce defeats noise correlated < 5 cycles, but a low-frequency
  estimator wander correlated > 5 cycles would slip through — size `kZeroVelDebounce` above the
  observed correlation length. (Moot under `0x605A=2` where the DRIVE owns the disable; the
  debounce only matters in the `0x605A∈{5,6,7}` backstop regime, which we refuse anyway — so
  in practice this is belt-and-suspenders for the fallback path.)
- **`0x6085` (quick-stop decel) CONCRETE value** — set in `A6Control::on_configured` via
  `cfg.sdo_write` (the `on_configured` setup-SDO home). Principle: `0x6085 := VEL_ceiling /
  t_qs` for a brisk target stop-time `t_qs ≈ 200 ms`. For the bench's expected velocity
  ceiling (~600 rpm = `600/60 × 131072` ≈ **1.31 Mcounts/s**, `kCountsPerRev = 131072`),
  **`0x6085 ≈ 6,553,600 counts/s²` (= 50 rev/s², stops 600 rpm in ~200 ms; stops 60 rpm in
  ~20 ms)** — brisk, well inside the 2 s window for every sane VEL. **Default to this**; tune
  DOWN if the drive trips the quick-stop on regen/overcurrent (and tune the window UP to
  match). HW-verify it's honored (next bullet + quirk #45).
  - **(DA-A) readback-echo + use the ECHOED value:** immediately after the `sdo_write(0x6085)`,
    `cfg.sdo_read(0x6085)`. If absent / read-only / zero → **REFUSE to energize** at configure
    (gross failure, before any motion). If the drive CLAMPED it to a valid-but-different value,
    **use the ECHOED value for ALL downstream math** — the VEL guard (§ below) and the expected
    decel slope — never the commanded value (DA: else a clamped-lower decel means a too-fast VEL
    passes a guard computed against the wrong, too-high decel). The subtle "accepts+stores but
    doesn't APPLY in quick-stop" residual still needs the HW decel-slope check (kept).
  - **(DA-E) `0x6085`/`a_decel_max` must come FROM the bench decel-ceiling measurement, not a
    guess:** if `0x6085` is set ABOVE the drive's true decel ceiling, the quick-stop can't
    achieve it → the motor isn't stopped by window-end → the `close()`→INIT backstop fires,
    which **IS a torque-cut at speed — just a LOGGED one** (the loud diagnostic is POST-HOC,
    after the cut). So the backstop's defensiveness is load-bearing on `0x6085 ≤ true ceiling`.
    Set it from the measured ceiling before any energized run is trusted; the default 6.5 M is
    a STARTING point pending that measurement.
- **VEL guard — CONFIGURE-TIME, pre-energize (DA-H); compute from the ECHOED `0x6085`:** with
  the readback-confirmed `0x6085` (DA-A above) and window `W`,
  a too-large VEL can't stop in time. `VEL` and `0x6085` are BOTH known at configure, so this
  is a **configure-time refusal BEFORE `--enable` acts** (consistent with the `--enable`
  fail-closed posture — never energize then discover it). Refuse `--move-vel VEL` where
  `|VEL| / 0x6085 + t_margin > W` (i.e. `|VEL| > 0x6085 × (W − t_margin)`; with `0x6085 = 6.5 M`,
  `W = 2 s`, `t_margin = 100 ms` → `VEL_max ≈ 12.4 Mcounts/s ≈ 5680 rpm` — far above any bench
  use, so it never bites in practice, but it makes "the window covers worst-case decel" TRUE
  by construction rather than assumed).
- `RunnerConfig.teardown_cycles` = a **generous upper bound** (≈ 2000 = 2 s @ 1 kHz);
  the control disables **early** within it on the velocity event. Leftover cycles are
  harmless no-op `cw=0` (cost: ≤ ~2 s bench-exit after Ctrl-C). (a6_validate is the TOOL,
  not the module, so the #47-C2 SIGTERM→SIGKILL grace upper-bound on `teardown_cycles`
  does NOT apply — a 2 s window is fine; Ctrl-C is the tool's own SIGINT.)
- The `0x6085`-honored assumption is **load-bearing** (quirk #45 — the A6 silently ignores
  unsupported objects): the HW gate confirms not just "`0x606C` reaches 0" but that the
  observed decel SLOPE matches the commanded `0x6085` (≈ `VEL/0.2 s`), proving the
  quick-stop decel is real. If `0x6085`/quick-stop proves unreliable, fall back to CSV
  (mode 9, master-streamed `0x60FF` ramp — fully master-controlled trajectory); noted, not adopted.
- **RESIDUAL (stated, not implied-graceful):** PV + a *wedged* drive → the Runner's
  abort / SM-watchdog path de-energizes at residual speed (uncontrolled). A wedged drive
  cannot ramp; fail-stop is the only option.

## Offline tests (sim)

- PP: sim drive moves `0x6064` toward `0x607A`; assert reached at `|Δ| ≤ kPosReachedTol`;
  assert the bit4 rising-edge handshake — **(DA-I) assert the bit4 edge COUNT == 1** (a
  level-not-edge bug re-asserts bit4 every cycle and silently fails to latch the next setpoint;
  count==1 + bit12 ack observed + bit4 cleared between is the non-vacuous pin); assert
  zero-jump-free (first enabled frame carries POS, no spurious jump).
- PV: assert `0x60FF` streamed == VEL; on stop, assert cw goes `0x0B` (quick-stop, bit2
  cleared) first. Two sim regimes for the disable (model both):
  - **`0x605A=2` (default/primary):** the sim drive ramps via `0x6085` and AUTO-transitions to
    SwitchOnDisabled at its own zero — assert the de-energize (`sw → SwitchOnDisabled`) happens
    AFTER `|0x606C| < thresh`, driven by the DRIVE (the control's `cw→0x00` is a no-op here).
  - **`0x605A∈{5,6,7}` (backstop):** the sim holds QuickStopActive at zero — assert the
    CONTROL's `cw→0x00` fires ONLY AFTER `|0x606C| < thresh` for the debounce count.
  *Baseline (both regimes): a control/sim that de-energizes on a fixed cycle-counter instead of
  the `0x606C` velocity event → de-energizes at residual speed → the "vel < thresh before
  SwitchOnDisabled" assert FAILS (pins the event-driven/auto disable — the whole point).* Also
  assert the window is a sufficient UPPER bound: at `VEL_max`, decel reaches `<thresh` before
  window-end.
- **`0x605A` assert (DA Finding 2):** sim returns `0x605A ≠ 2` (e.g. 0 coast) → configure
  REFUSES to energize; `0x605A == 2` → proceeds.
- Mode echo (DA-B): wrong-mode (sim `0x6061 ≠` commanded `0x6060`) → refuse to enable
  (fail-closed) — pins the `==commanded` check the current code lacks.
- `0x6085` readback (DA-A): sim clamps `0x6085` → the VEL guard uses the ECHOED (clamped)
  value; sim returns `0x6085=0`/absent → refuse.
- Exclusivity: two move flags → error.

## HW gate (team-lead, energized, USER-gated per run)

- `--move-pos`: zero-jump-free latch (cmd==pos at enable), moves to POS, reaches
  (`|Δ| ≤ kPosReachedTol`), holds; badWKC=0, no Er74.
- `--move-vel` — **CORRECTED LANDMARK (DA Finding 1): the de-energize landmark is
  SwitchOnDisabled, NOT "leaving OperationEnabled".** On Ctrl-C the statusword leaves
  OperationEnabled IMMEDIATELY (→ QuickStopActive) at full speed — that's expected, not the
  hazard. The safety proof is: **`0x606C` ramps to ~0 while STILL ENERGIZED in QuickStopActive
  (`sw & 0x6F == 0x07`), and only THEN does the statusword reach SwitchOnDisabled
  (`sw & 0x4F == 0x40`)** = the actual de-energize. DA verifies: vel→~0 in QSA *before*
  SwitchOnDisabled (ramp-then-disable, NOT torque-cut-at-speed), AND the observed `0x606C`
  decel SLOPE matches the (echoed) `0x6085` (proves it's honored, quirk #45); badWKC=0.

## DA gate (spec + impl)

PV: Quick-Stop (`cw=0x0B`, bit2-clear polarity confirmed); disable regime per `0x605A`
(default `=2` → DRIVE auto-disables at its zero, primary; `∈{5,6,7}` → control event-driven
`cw→0x00`, backstop), `0x605A==2` read-asserted at configure; `0x6085` readback-echoed +
VEL-guard from the echoed value; HW landmark = vel→~0 in QuickStopActive BEFORE
SwitchOnDisabled (`sw&0x4F==0x40`), decel-slope matches echoed `0x6085`; wedge residual
stated. PP: bit6=0, true rising-edge bit4 + bit12 ack + clear-between (edge count==1),
reached=actual-vs-target (not bit10) at `kPosReachedTol` default 300, same zero ref,
`--move-pp` NOT flipped. Mode-fixed-per-invocation; `0x6061` echo fail-closed (impl gap at
a6_validate.cpp:390 — must add the `==commanded` check).
