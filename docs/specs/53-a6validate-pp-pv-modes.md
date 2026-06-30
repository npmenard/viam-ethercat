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

## A6Control — absolute PP (`--move-pos`)

- On enable (first `OperationEnabled`): set `0x607A = POS` **absolute** (controlword
  **bit6 = 0**, NOT relative) and `0x6081 = VEL`.
- New-setpoint handshake — a **true rising edge** per setpoint: controlword bit4 0→1 →
  await sw **bit12** (set-point acknowledge) → clear bit4 → await bit12 clear. Exactly
  one edge; never leave bit4=1 across setpoints (else the next target is silently not
  latched).
- **Reached** = `|0x6064_actual − POS| ≤ 50` counts AND `|0x606C| ≈ 0`. Report "reached",
  then hold (re-send cw=enable, no new bit4 edge). Do NOT use A6 sw bit10 ("target
  reached" is quirkily always-set — TODO-43).
- The absolute target is measured against the **same zero reference** as feedback/SetZero.
- `--follow-err-limit` still aborts + disables on `|cmd − actual|` runaway.

## A6Control — PV (`--move-vel`)

- Mode `ProfileVelocity` (`0x6060` = 3).
- On enable: stream `0x60FF = VEL` each cycle. Drive ramps via `0x6083`/`0x6084`.
- **Stop (Ctrl-C → `ctx.stopping()`): CiA402 Quick-Stop.** Command the Quick-Stop
  transition (T11, OperationEnabled→QuickStopActive) — **controlword `0x0B`: CLEAR bit2
  while KEEPING bit1 (Enable Voltage) and bits 0,3** (`0x0F & ~0x04`). *Polarity matters:
  the Quick-Stop command is bit2 = 0, NOT bit2 = 1 — enable-operation `0x0F` already has
  bit2 SET; a wrong polarity = no quick-stop = the torque-cut-at-speed we are preventing.*
  In QuickStopActive the *drive* performs its own controlled decel via **`0x6085`
  (quick-stop deceleration)** and ignores streamed `0x60FF`. Watch `0x606C`; once
  `|0x606C| < kZeroVelThresh` for **`kZeroVelDebounce` consecutive cycles** (debounce so a
  velocity-estimate noise dip can't disable mid-decel), THEN command cw → `0x00`
  (disable voltage → SwitchOnDisabled). **Event-driven on real feedback — not a
  fixed-counter race.**
- **`kZeroVelThresh` concrete:** set just ABOVE the measured `0x606C` velocity-estimate
  noise floor (a one-time bench data-point). Start at **≈ 500 counts/s** (≈ 0.23 rev/s at
  2^17 counts/rev — genuinely stopped, a negligible cut if disabled there), `kZeroVelDebounce
  = 5` cycles. NOTE: ≈ 50 counts/s (the earlier guess) is likely BELOW the noise floor →
  the event may never fire → it safely falls back to the window upper bound (disable at
  window-end, motor long stopped) but the bench-exit is slow; 500 c/s reliably fires early.
- **`0x6085` (quick-stop decel) CONCRETE value** — set in `A6Control::on_configured` via
  `cfg.sdo_write` (the `on_configured` setup-SDO home). Principle: `0x6085 := VEL_ceiling /
  t_qs` for a brisk target stop-time `t_qs ≈ 200 ms`. For the bench's expected velocity
  ceiling (~600 rpm = `600/60 × 131072` ≈ **1.31 Mcounts/s**, `kCountsPerRev = 131072`),
  **`0x6085 ≈ 6,553,600 counts/s²` (= 50 rev/s², stops 600 rpm in ~200 ms; stops 60 rpm in
  ~20 ms)** — brisk, well inside the 2 s window for every sane VEL. **Default to this**; tune
  DOWN if the drive trips the quick-stop on regen/overcurrent (and tune the window UP to
  match). HW-verify it's honored (next bullet + quirk #45).
- **VEL guard (CLI, fail-closed):** with a fixed `0x6085` and the window `W`, a too-large
  VEL can't stop in time. Refuse `--move-vel VEL` where `|VEL| / 0x6085 + t_margin > W`
  (i.e. `|VEL| > 0x6085 × (W − t_margin)`; with `0x6085 = 6.5 M`, `W = 2 s`, `t_margin =
  100 ms` → `VEL_max ≈ 12.4 Mcounts/s ≈ 5680 rpm` — far above any bench use, so it never
  bites in practice, but it makes "the window covers worst-case decel" TRUE by construction
  rather than assumed).
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

- PP: sim drive moves `0x6064` toward `0x607A`; assert reached at `|Δ| ≤ 50`; assert the
  bit4 rising-edge handshake (bit12 ack observed, bit4 cleared between); assert
  zero-jump-free (first enabled frame carries POS, no spurious jump).
- PV: assert `0x60FF` streamed == VEL; on stop, the sim models quick-stop decel via `0x6085`
  — assert cw goes `0x0B` (quick-stop, bit2 cleared) first, then cw→`0x00` happens ONLY
  AFTER `|0x606C| < thresh` for the debounce count (disable NOT issued at speed). *Baseline:
  a control that disables on a fixed cycle-counter instead of the `0x606C` velocity event →
  disables at residual speed → the "vel < thresh before cw→0" assert FAILS (pins the
  event-driven disable — the whole point).* Also assert the window is a sufficient UPPER
  bound: at `VEL_max`, the modeled decel reaches `<thresh` before window-end.
- Mode echo: wrong-mode (sim ignores `0x6060`) → refuse to enable (fail-closed).
- Exclusivity: two move flags → error.

## HW gate (team-lead, energized, USER-gated per run)

- `--move-pos`: zero-jump-free latch (cmd==pos at enable), moves to POS, reaches
  (`|Δ| ≤ 50`), holds; badWKC=0, no Er74.
- `--move-vel`: runs at VEL (`0x606C ≈ VEL`); on Ctrl-C the **wire/feedback shows
  `0x606C` ramping to ~0 BEFORE statusword leaves OperationEnabled / cw→0** (DA verifies:
  ramp-then-disable, NOT torque-cut-at-speed); badWKC=0.

## DA gate (spec + impl)

PV: Quick-Stop + event-driven disable on `0x606C≈0`, window covers worst-case decel,
HW-verified ramp-before-disable, residual stated. PP: bit6=0, true rising-edge bit4 +
bit12 ack + clear-between, reached=actual-vs-target (not bit10), same zero ref.
Mode-fixed-per-invocation; echo fail-closed.
