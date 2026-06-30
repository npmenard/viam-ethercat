# A6-EC Drive Quirks — What a Module Consumer / Operator Must Know

Consolidated list of A6-EC (ANCTL AS715N) behaviors that **differ from what a
generic CiA402/EtherCAT consumer would expect**. Each entry: what the drive
actually does, where the manual says it (A6-EC manual section; `a6.txt:` line
refs point into the extracted manual text), where our stack handles it today,
and what an operator/consumer of the module needs to do about it.

**Read alongside:** [`a6-bringup-runbook.md`](./a6-bringup-runbook.md) (bench
procedure), [`a6-hardware-wiring.md`](./a6-hardware-wiring.md) (wiring/safety),
[`deployment-capabilities.md`](./deployment-capabilities.md) (process caps).

> Scope note: items Q1–Q2 are **code-fix items** tracked as tasks (cross-ref
> only). Q3–Q12 are the consolidated consumer/operator quirk list (librarian
> omission-check items 3–12).

---

## Cross-referenced code-fix items (tracked as tasks, not just docs)

### Q1. Loop rate must yield a 250 µs-multiple SYNC0 cycle — else Er74.0 at OP
- **Quirk:** the drive faults `Er74.0` ("EtherCAT synchronization cycle setting
  error", 0x603F=0x6320) if the DC SYNC0 cycle is not an **integer multiple of
  250 µs**. A `target_loop_rate_hz` of e.g. 600 Hz (1.667 ms) produces a
  cryptic OP-entry fault.
- **Manual:** C13.05 note ("the synchronization cycle must be an integer
  multiple of 250 μs, otherwise the servo drive will report Er74.0",
  §11.3.7 / a6.txt:14437; fault table a6.txt:11040).
- **Handled:** ⚠ pending config-validation task. Note: the 250 µs granularity
  is **A6 policy, not EtherCAT law** (other drives differ) — the validation
  belongs in **config data** (e.g. a per-slave `sync0_cycle_granularity_ns`
  the A6 config sets to 250000, checked by generic code), never as a literal
  in the generic library.
- **Guidance:** pick loop rates whose period is a 250 µs multiple: **4000,
  2000, 1000 (recommended), 800, 500, 250… Hz**. If the drive faults Er74.0
  immediately at OP entry, check the loop rate first.

### Q2. Unsupported mode written via PDO is *silently ignored*
- **Quirk:** writing an unsupported value to 0x6060 (modes of operation) via
  **SDO** returns an SDO error; via **PDO** the change is silently ignored —
  no error, 0x6061 (mode display) just never changes.
- **Manual:** a6.txt:14710-14711. Supported modes on this drive: 0x6502 = 941
  → PP(1), PV(3), PT(4), HM(6), CSP(8), CSV(9), CST(10). **Not** supported:
  VL(2), IP(7).
- **Handled:** the module sets the mode via SDO at configure (errors surface);
  explicit 0x6061-echo verification is the pending code-fix.
- **Guidance:** after any mode change, confirm `0x6061 == commanded mode`
  before commanding motion. If 0x6061 stays at its previous value, the mode
  write did not take.

---

## Consumer / operator quirk list

### Q3. Multi-turn absolute-encoder overflow: ErA0.1 at ±32767 / −32768 revolutions
- **Quirk:** the absolute encoder's multi-turn counter overflows when
  accumulated rotation exceeds **+32767 or −32768 revolutions** → fault
  `ErA0.1` "Multi-turn overflow" (0x603F = 0x7305, resettable). Long
  unidirectional running (conveyor / continuous-rotation use) **will**
  eventually hit this. Separately, changing the electronic gear ratio shifts
  the mechanical position mapping (a6.txt:6897).
- **Manual:** fault table a6.txt:11186-11193; code table a6.txt:10157.
- **Handled:** nowhere in code — fault surfaces via 0x603F like any other.
- **Guidance:** for continuous-rotation applications, plan a periodic
  `reset_zero_position` (or a homing cycle) well before ±32 k revs; treat
  ErA0.1 as expected-and-resettable in such duty. For position-critical axes,
  power-cycle + re-home after clearing it. Do not change the drive's
  electronic gear (C10.18/C10.19) without re-homing.

### Q4. Quick stop lands in SwitchOnDisabled — full re-enable required (605A default = 2)
- **Quirk:** with the factory default quick-stop option code **0x605A = 2**
  (range 0–7), a Quick Stop (controlword bit2 → 0) ramps the motor down per
  0x6085 and then the drive **automatically transitions to SwitchOnDisabled**.
  The CiA402 "resume from QuickStopActive" transition (T16) exists **only for
  605A ∈ 5–7** and is therefore unreachable at default.
- **Manual:** 0x605A entry (RW, I16, 0–7, default 2, **not PDO-mappable**,
  modifiable during operation but **effective only upon re-power-on**;
  a6.txt:13499); value meanings (a6.txt:14656-14663): 0–3 = stop "keeping
  de-energized status" (→ SwitchOnDisabled), 5–7 = stop "keeping **position
  lock** status" (the stay-in-QuickStopActive variants; value 4 is undefined);
  stop-mode Table 6-3 (a6.txt:7912-7919).
- **Handled:** the module's normal `Stop()` uses **Halt (bit 8)**, which stays
  in OperationEnabled and is re-commandable — Quick Stop is reserved for the
  emergency path. The CiA402 FSM models the automatic QuickStopActive →
  SwitchOnDisabled walk.
- **Guidance:** after any emergency Quick Stop, the axis must be **fully
  re-enabled** (controlword 0x06 → 0x07 → 0x0F) before it will move again.
  Don't expect "resume". If you need stay-in-quick-stop semantics, set 605A to
  5–7 — but note a 605A change **takes effect only after a control-power
  cycle**, and the FSM's default model no longer matches; prefer the default.
  ⚠ Once SwitchOnDisabled is reached the output stage is OFF — see **Q13**: the
  velocity reading (0x606C) is then a meaningless estimator artifact, and on a
  loaded/vertical axis the shaft can back-drive for real. Don't treat the axis
  as "safely stopped" on the velocity reading alone.

### Q5. Statusword bit 10 ("target reached") is ALWAYS 1; bit 14 unsupported
- **Quirk:** bit 10 "Position reach — **Not supported. This bit remains 1 all
  the time**" and bit 14 "Manufacturer-specific — Not supported". A generic
  CiA402 consumer that polls bit 10 for move-complete will think every move is
  instantly done.
- **Manual:** PP statusword table (a6.txt:3194, 3208).
- **Handled:** the controller's move-complete predicate ignores bit 10 and uses
  `|target − actual| ≤ tol && |vel| ≤ vthresh` (plus statusword bit 13 as the
  drive-side cross-check). The sim slave reproduces the bit10-always-1
  behavior behind a model flag.
- **Guidance:** never use `target_reached()`/bit 10 on this drive. Use the
  module's `is_moving`/position readback, or bit 12 (setpoint-acknowledge) for
  the PP handshake and bit 13 (deviation) for runaway detection.

### Q6. Fault-reset bit 7 masks ALL other control references — pulse it, never hold it
- **Quirk:** while controlword bit 7 = 1, "**other control references are
  inactive**" — holding 0x80 freezes all other commanding. Reset triggers on
  the **rising edge** only; repeated resets need repeated 0→1 edges.
- **Manual:** PP controlword table (a6.txt:3067-3071); same text in the other
  mode tables.
- **Handled:** the fault-reset path pulses bit 7 for one cycle and returns the
  controlword to the ladder value; the FSM encodes this as the only legal
  Fault-state action.
- **Guidance:** if driving the controlword manually (commissioning), write
  0x80 for one cycle then return to 0x06. A controlword stuck at 0x80 looks
  like a dead drive — everything is ignored.

### Q7. Er74-class faults don't clear via CiA402 bit 7 — vendor reset 0x2031:01 or power cycle; repeated Er74 wedges the drive
- **Quirk:** EtherCAT-sync faults (Er74.x family) are not cleared by the
  CiA402 fault-reset bit. The working reset is the **vendor object
  0x2031:01 ← 1** (⚠ **not documented in the manual** — bench-discovered, also
  used by SOEM's ec_sample). Additionally, **repeated Er74 OP-entry faults
  wedge the drive entirely** (NIC reports NO-CARRIER, no enumeration) and only
  a **control-power cycle** recovers it.
- **Manual:** Er74.x causes/remedies (a6.txt:11040-11063) — lists only "correct
  the master configuration"; 0x2031 absent from the manual.
- **Handled:** bring-up bounds its retry attempts (bounded give-up) so it never
  hammers a persistent Er74; the vendor reset is consumer-side config-data
  (see `docs/specs/22-vendor-fault-reset.md`). **Wiring (#39):** the servo module
  takes it from the hardware JSON's `vendor_fault_reset`
  (`{"index": 8241, "subindex": 1, "value": 1, "value_bytes": 2}` — 8241 = 0x2031)
  and runs it once pre-RT-spawn; `a6_validate --reset-fault` does the same via
  `Master::sdo_write`. The old per-slave `fault_reset` config key is obsolete and
  rejected. The library itself carries zero vendor-object knowledge.
- **Guidance:** if the module reports repeated sync faults at startup: **stop
  retrying, power-cycle the drive's control power, fix the master timing**
  (see Q1), then retry. Do not write scripts that loop OP attempts against a
  faulting drive — that is the wedge path.

### Q8. Statusword bit 9 "remote" gates whether the controlword does anything
- **Quirk:** bit 9 = 1 means "the control word has taken effect". While
  bit 9 = 0 the drive **ignores controlword writes** — transitions silently
  don't progress.
- **Manual:** statusword tables (a6.txt:3190-3192).
- **Handled:** the enable ladder waits on statusword feedback per step; the FSM
  treats bit 9 = 0 as "hold — do not hammer transitions".
- **Guidance:** when commissioning manually, check bit 9 before concluding the
  drive is ignoring you for some deeper reason. bit 9 = 0 typically means the
  comms state isn't fully up (not OP, or sync not established).

### Q9. PV mode: controlword bits 4–6 and 9–10 are reserved/unsupported
- **Quirk:** the PP new-setpoint handshake bits (bit 4 latch, bit 5 immediate,
  bit 6 relative) **do not exist in PV mode** — the A6 marks cw bits 4–6 and
  9–10 "Reserved for PV mode / Not supported".
- **Manual:** PV controlword table (a6.txt:3394, 3407-3409).
- **Handled:** the controller's handshake sub-FSM runs only in the PP branch;
  PV holds plain 0x0F.
- **Guidance:** in PV, command velocity via 0x60FF only; leave cw bits 4–6
  zero. Setting them is harmless but meaningless — don't build logic on them.

### Q10. Velocity/accel units are encoder counts-per-second (not rpm) — and the electronic gear silently rescales them
- **Quirk:** 0x60FF / 0x606C / 0x607F / 0x6081 / 0x6083 / 0x6084 are in
  **reference units per second** = encoder counts/s: **131072 (2¹⁷) per
  revolution at the default 1:1 electronic gear**. This scale is **derived**
  from the manual's example ("writing 1310720 → 600 rpm at gear 1:1",
  a6.txt:3529-3531) — there is no explicit "unit = X" line — and it was
  bench-validated. Changing the drive's electronic gear (C10.18/C10.19 =
  0x2010:19/:1A) **silently changes the effective counts-per-rev** for both
  position and velocity.
- **Manual:** a6.txt:3529-3531 (60FF example), 14752/14759 (606C compared to
  60FF in the same unit), 7102-7105 (e-gear params).
- **Handled:** `motion_profile` converts rpm ↔ counts/s via `counts_per_rev`
  (config datum, 131072 at 1:1); velocity readback uses the same scale.
- **Guidance:** `counts_per_rev` in the module config **must match the drive's
  e-gear setting**. If someone changes C10.18/C10.19 on the drive, every
  reported position/velocity is silently wrong until the config is updated.
  Sanity check after any drive re-configuration: command a known rpm and
  compare the reported velocity. **Note the noise floor:** 0x606C reads
  ±~2000–4000 c/s even at a commanded standstill (≈ ±1 rpm) — set velocity-zero
  thresholds above that. And once the drive de-energizes, 0x606C stops being
  meaningful entirely — see **Q13**.

### Q11. Following-error window default is 24 REVOLUTIONS — far looser than you'd assume
- **Quirk:** 0x6065 (excessive position deviation threshold) defaults to
  **3,145,728 reference units = 24 full revolutions** at 131072/rev, and
  0x6066 (following-error timeout) defaults to 0 ms. The drive will tolerate
  enormous following error before declaring Er and flipping statusword
  bit 13 — consumers expecting tight, fast drive-side following-error
  protection don't get it by default.
- **Manual:** 0x6065/0x6066 OD rows (a6.txt:3240, 3243); bit 13 semantics
  (a6.txt:3204-3206).
- **Handled:** the energized-move path monitors statusword bit 13 + 0x603F per
  cycle as a runaway net (it won't false-trip — a gentle move sits ~3 orders
  of magnitude under the window). The window itself is left at default.
- **Guidance:** if your application needs tight position-following protection
  (e.g. collision/jam detection), **tighten 0x6065 deliberately** (units:
  reference units; 131072 = 1 rev) and optionally set 0x6066 — don't assume
  the default protects the mechanism. Otherwise rely on application-level
  supervision of position error.

### Q12. What the shaft does during a fault depends on the FAULT CLASS
- **Quirk:** stop-at-fault behavior is **not uniform**: "No.1" faults stop by
  **dynamic braking** (per C05.03, shaft resists rotation, stops hard);
  "No.2" faults **ramp to quick stop** per 0x605E/0x6085 (controlled decel).
  Which class a given fault belongs to is per the fault tables. Other stop
  paths: S-ON-OFF stop per 0x605C, overtravel per C05.02, halt per 0x605D.
- **Manual:** stop-mode comparison Table 6-3 (a6.txt:7894-7924); fault-class
  assignments in the troubleshooting chapter.
- **Handled:** nowhere in code — this is physical behavior of the drive.
- **Guidance:** for **mounted-mechanism safety review**: do not assume a
  faulting axis ramps down gently — a No.1 fault dynamic-brakes the shaft
  (abrupt). If the mechanism can't tolerate an abrupt stop, that's a
  mechanical-design constraint, not something the module can change. Include
  fault-stop behavior in any safety assessment of the machine.

### Q13. 0x606C (velocity actual) is MEANINGLESS once the output stage is disabled — and is never a safety signal on a loaded axis
- **Quirk (HW finding, #53 energized bench run; DA-confirmed from the wire trace):**
  after a Quick Stop reaches **SwitchOnDisabled** (output stage off, see Q4),
  0x606C does **not** read ~0. It shows a large **damped reverse excursion** —
  observed a single ring peaking **≈ −23000 c/s**, decaying to the at-rest noise
  floor within **~50 ms** — with **zero re-energize**. This is a velocity-
  **estimator artifact**, not real motion: the back-acceleration it implies is
  physically impossible on a free shaft with no applied torque. The estimator
  simply produces garbage once the drive stops controlling current.
- **Standstill noise floor:** even **energized and holding position**, 0x606C
  carries a **±~2000–4000 c/s** noise floor (≈ ±0.9–1.8 rpm at 131072 c/rev,
  Q10). Any "velocity ≈ 0" threshold must sit **above** this band or it will
  never read stopped.
- **⚠ SAFETY COROLLARY (matters for #37 on a real/loaded axis):** on a **loaded
  or vertical axis**, motion *after* de-energize **can be REAL** — gravity or a
  load back-drives the shaft once torque is removed. So 0x606C-after-
  SwitchOnDisabled **cannot distinguish an estimator artifact from a genuine
  runaway**. **Do NOT use "post-SOD velocity is just noise" as a safety
  assumption on a loaded axis.** Gate de-energize-safety on **position
  deviation** (is the shaft actually moving in encoder *position*, which is real
  even when de-energized), a **mechanical brake** (0x6040 brake DO / a holding
  brake), or **re-engage logic** — not on the post-disable velocity reading.
- **Manual / source:** velocity-unit basis Q10 (a6.txt:3529-3531); SwitchOnDisabled
  reached per Q4 / 0x605A=2 (a6.txt:13499); the excursion + noise-floor numbers
  are bench-measured (#53), not in the manual.
- **Handled:** the controller's move-complete predicate already cross-checks
  position + a velocity *threshold* (Q5), so it tolerates the noise floor; but no
  code today treats post-SOD velocity as a safety signal — and per the corollary
  it must not. Cross-refs: **Q4** (quick-stop → SwitchOnDisabled, the state this
  occurs in), **Q10** (the counts/s unit + noise magnitude), **Q12** (fault-class
  stop behavior — a related "what the shaft physically does" safety concern).

---

## Quick reference — statusword bits with non-standard behavior on the A6

| Bit | Standard meaning | A6 behavior |
|-----|------------------|-------------|
| 9   | Remote | Gates controlword effect — 0 ⇒ cw ignored (Q8) |
| 10  | Target reached | **Always 1 — unusable** (Q5) |
| 12  | Setpoint ack (PP) | Works — use for PP handshake |
| 13  | Following error | Works — but window default = 24 revs (Q11) |
| 14  | Manufacturer | Not supported (Q5) |

## Quick reference — fault families an operator will actually meet

| Code (panel) | 0x603F | Meaning | Reset |
|---|---|---|---|
| Er74.0 | 0x6320 | Sync cycle not a 250 µs multiple (Q1) | Fix loop rate; vendor reset / power cycle |
| Er74.1/.2 | 0x8700 | No sync signal / sync incomplete (master DC config) | Vendor 0x2031:01 or power cycle (Q7) |
| ErC1.x / ErC2.0 | 0x8700 | Sync jitter / frame loss / watchdog / SYNC loss | Resettable after cause fixed |
| ErA0.1 | 0x7305 | Multi-turn overflow at ±32 k revs (Q3) | Resettable; re-home |
| (any) bit3 set | per table | Drive faulted, shaft per fault class (Q12) | bit7 pulse (Q6) or vendor reset (Q7) |

> **Velocity-as-safety-signal warning (Q13):** 0x606C is a ±2–4 k c/s-noisy
> estimate when energized and an outright meaningless artifact once de-energized
> (SwitchOnDisabled). On a loaded/vertical axis, real gravity back-drive is
> indistinguishable from the artifact — gate de-energize safety on position
> deviation / a brake, never on post-disable velocity.

---

*Sources: A6-EC series servo drive manual (object dictionary, CiA402 mode
chapters §4.1.x, EtherCAT chapter §8, parameter list §11, troubleshooting
§10); bench findings from the Phase-7 bring-up (CLAUDE.md). Line refs
(`a6.txt:`) index the extracted manual text used during development.*
