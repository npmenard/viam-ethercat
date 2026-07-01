# M56S / MDX+ EtherCAT Drive — Device Profile

Device-profile reference for the **M56S-EC** (and MDX+) servo drive — our
**second** target device for the generic servo driver (#47-P3 / #37). Companion
to [`a6-quirks.md`](./a6-quirks.md): where that doc catalogs the A6's *quirks*
(how it deviates from generic CiA402), **this doc's point is the opposite — the
M56S is almost entirely standard**, which is exactly what validates the
"generic driver + per-device config data" thesis.

> **The thesis, validated.** The A6 needed a specific set of config knobs to tame
> its quirks (vendor fault-reset 0x2031:01, mandatory DC, bit10-always-1, the
> 250 µs SYNC0 rule, …). The M56S needs **almost none of them** — its profile is
> ≈ `{ counts_per_rev = 10000 } + standard CiA402 defaults`, and where it does
> need a knob it wants the **OPPOSITE** value from the A6 (fault-reset = standard
> controlword bit 7, *not* a vendor SDO). Two drives demanding opposite settings
> through the *same* config surface = the genericity proof. Sources:
> `M56S_MDX+ EtherCAT Communication Manual` (920-0179 RevA); `m56s.txt:` line
> refs index the extracted text.

---

## Identity

| | |
|---|---|
| Device name | **M56S-EC** (M5 / MDX+ series) |
| Protocol | **CoE** (CANopen-over-EtherCAT), CiA402 |
| 0x1018 product code | **0x0000000F** |
| Diagnostics | sends CoE **Emergency (EMCY)** messages on fault (§3.6, m56s.txt:990-1003), *supplementary* to the standard 0x603F fault-code PDO read |

---

## Profile vs. the A6 — the near-empty-profile comparison

| Aspect | A6-EC (quirky) | **M56S (standard)** | Driver impact |
|---|---|---|---|
| **fault_reset** | vendor SDO **0x2031:01 ← 1** (bit 7 doesn't clear Er74) | **controlword bit 7 rising edge** (standard CiA402) | `fault_reset_mechanism` = bit7 = the **generic default**; M56S needs the *opposite* of the A6's one big knob |
| **DC / sync** | **DC SYNC0 mandatory** (free-run → AL 0x0027) | **DC OPTIONAL** — FreeRun + SM-event + DC all supported; PP/PV run free-run, DC only for CSP/CSV/CST | M56S can skip the whole DC bring-up dance for PP/PV |
| **statusword bit 10** | **always 1** — unusable (A6 Q5) | works **normally** = "Positioning Finish" | generic move-complete (bit 10) is usable here |
| **quick-stop** | 0x605A=2 default, lands in SwitchOnDisabled | 0x605A=2 default (range 0~8), 0x6085=30,000,000 pulses/s² | **same standard objects**; only the decel *value* differs |
| **mode-set of unsupported value** | **silently ignored** (A6 Q2/#45) | **raises an ERROR** (§4.2.3.5) | opposite failure mode — see the flag below |
| **units** | counts/s, 131072/rev (e-gear, silent rescale) | pulses, **10000/rev** (e-gear 0x2A90, silent rescale) | same *shape* (e-gear config datum), different value |

Everything else (enable ladder, modes, statusword/controlword bit semantics) is
plain CiA402 — no per-field handling needed.

---

## Profile facts (config-data for the M56S `SlaveConfig`)

### fault_reset = controlword bit 7 (STANDARD — the generic default)
- CiA402 **bit 7 rising edge** clears faults (m56s.txt:1273 "Fault → Servo Disabled … 0x80, bit7: 0→1"; controlword bit-7 table m56s.txt:1548). **No vendor SDO.** This is the exact opposite of the A6's 0x2031:01 (a6-quirks Q7) — so the M56S exercises the *default* `fault_reset_mechanism` path while the A6 exercises the override. Two drives, opposite values, one config field = genericity validated.

### DC is OPTIONAL — free-run works for PP/PV
- §3.7 (m56s.txt:1018-1045): supports **FreeRun / SM-event / DC(SYNC0 Time Event)**. Per the table, FreeRun = "simple processing, poor real-time"; DC = "high precision, requires master-side compensation." PP/PV run fine in free-run; **DC (SYNC0) only needed for CSP/CSV/CST**. Contrast the A6, which *mandates* DC in every mode (a6-quirks: free-run → AL 0x0027). ⇒ M56S can bring up PP/PV without `ecx_configdc`/SYNC0 at all; the library's DC path is opt-in per slave.

### statusword bit 10 works normally ("Positioning Finish")
- m56s.txt:1606-1607, 1710, 1721: bit 10 = **Target reached / Positioning Finish**, normal CiA402 semantics (with a window/threshold, 0x6067-style). **Not** the A6's hardwired-1 (a6-quirks Q5). ⇒ on the M56S, the generic `Status::target_reached()` / bit-10 move-complete is usable directly — the A6's `|tgt−act|≤tol` workaround is *not* required (though it remains a safe superset).

### quick-stop — standard objects, default value differs
- **0x605A** Quick Stop Option Code: INTEGER16, range **0~8**, default **2** (m56s.txt:1488/1993). Same value-table family as the A6 (0–3 de-energized, 5–6 position-lock; 0–2 vs 5–6 branch noted m56s.txt:1259/1275). **0x6085** Quick Stop Deceleration: UNSIGNED32, pulses/s², default **30,000,000** (m56s.txt:1504). ⇒ generic quick-stop handling; the A6's quick-stop→SwitchOnDisabled note (a6-quirks Q4) applies identically (605A=2 default), only the decel magnitude is a config datum.
  - **Effective-time contrast (for symmetry with a6-quirks Q4):** the A6 annotates 0x605A with an explicit **"Effective Time = Upon re-power-on"** attribute (§11.2.2) — a change doesn't take effect until a control-power cycle (this bit the A6 T16 HW-verify). The **M56S OD tables carry NO per-object effective-time column** (columns are just Index/Sub/Name/Access/Type/Unit/Range/Default/PDO), and the M56S manual's "re-power the drive after setting parameters" notes apply to *other* objects (0x1014 EMCY COB-ID m56s.txt:6479; a homing option list m56s.txt:7344), **not** 0x605A. The M56S instead uses the standard CoE **0x1010 "Store parameters"** (write `0x65766173` "save" → NVM; 0x1011 restores defaults; m56s.txt:6434-6472) for persistence. ⇒ Whether an M56S 0x605A change is live-immediate vs needs a persist+restart is **not documented per-object** — **confirm on the bench** if it matters. Net: the A6's per-object effective-time quirk has **no M56S equivalent**; the M56S uses the generic store-to-NVM model.

### units — pulses; electronic gear 0x2A90 default 10000 (configurable; same silent-rescale risk as A6)
- Position/velocity in **pulses**. Electronic gear **0x2A90** (param **P3-05**, "Command pulses per revolution"): UNSIGNED32, range **200 ~ 2²⁶**, default **10000** (m56s.txt:11/5747/6139). ⇒ **`counts_per_rev` ≈ 10000** in the M56S config (vs the A6's 131072). ⚠ Same trap as A6 Q10: if someone changes 0x2A90/P3-05 on the drive, every reported position/velocity is silently rescaled until the config's `counts_per_rev` is updated. Velocity objects (0x60FF/0x606C/0x6083/0x6084) in **pulses/s** (e.g. 0x60FF default 100000, 0x6083/0x6084 default 1,000,000 pulses/s²; m56s.txt:2008-2012).

### torque / current — 0.1%-of-rated (device-specific basis; torque mode out of scope)
- 0x6077 Torque Actual / 0x6078 Current Actual: INTEGER16, **0.1%** of rated, 0~±3000 (m56s.txt:1497-1498). 0x6073 Max current: UNSIGNED16, **0.1%**, 0~3000, default 3000 (m56s.txt:2004). ⇒ a *different* current-limit basis than the A6's ‰-of-rated-torque (a6 appendix) — but torque mode is out of scope, so this is optional config; if surfaced, `peak_current` converts via the 0.1%-of-rated basis (a per-device datum, like the A6's rated-current).

### modes + enable — fully standard
- 0x6060/0x6061 standard; supported: **PP(1), PV(3), Torque(4), Homing(6), CSP(8), CSV(9), CST(10)** (m56s.txt §4.3+). Enable = standard **0x06 → 0x07 → 0x0F** ladder (controlword tables m56s.txt:1288-1312). No A6-style sequencing quirks.

---

## THE ONE BEHAVIORAL FLAG — §4.2.3 Control Mode Switching Precautions

This is the **generic mode-switch pattern** the driver adopts (#47-P3, rev 7) —
it's not M56S-specific, it's the correct CiA402 mode-switch discipline that the
M56S happens to state explicitly. **Verbatim** (m56s.txt:1412-1420, §4.2.3):

> 1. Please do not switch control modes while the motor is in motion.
> 2. When switching control modes, please first update the objects in the RxPDO that are related to the control mode in 0x6060.
> 3. Switching from one control mode to another requires some time. During this transition, the values in 0x6061 and the objects related to the control mode in the TxPDO are undefined.
> 4. In the modified control mode, the value of unsupported object is uncertain.
> 5. If a control mode not supported by the drive is set, an error will occur.
> 6. Full closed-loop control is only supported in position control modes (PP, CSP, HM). Full closed-loop control is not supported in other modes.

Driver implications (the generic mode-switch contract):
- **(1) Stop first.** The driver must reach a stopped/safe state before changing 0x6060 — never mid-motion.
- **(2) Seed the new mode's command objects BEFORE/with the mode write.** e.g. switching to PP → set 0x607A (and profile vel/accel) before commanding motion; switching to PV → set 0x60FF. (Same spirit as the A6 CSP "seed target = actual at enable" gotcha.)
- **(3) Transition is not instantaneous — 0x6061 + mode-related TxPDO are UNDEFINED during it.** The driver must **not** trust mode-display or mode feedback for a window after a mode write; wait for 0x6061 to settle to the commanded mode before acting on feedback. (Pairs with the A6 0x6061-echo-verify, #45 — confirm-the-echo, but also *tolerate the undefined window* first.)
- **(4)** Unsupported objects in the new mode read garbage — don't consume objects outside the active mode's set.
- **(5) Setting an unsupported mode RAISES AN ERROR** — the **opposite** of the A6, which *silently ignores* it (a6-quirks Q2 / #45). ⇒ the generic mode-set path must handle **both**: verify the 0x6061 echo (catches the A6's silent no-op) **and** be ready for an SDO/EMCY error return (catches the M56S's loud reject). Belt-and-suspenders covers both devices through one code path.
- **(6)** Full closed-loop only in PP/CSP/HM — informational for control-quality expectations.

---

## Cross-references
- [`a6-quirks.md`](./a6-quirks.md) — the A6's quirk list. The M56S column above maps each A6 quirk to its M56S (mostly standard) counterpart: A6 Q7↔M56S bit7 fault-reset, A6 DC-mandatory↔M56S DC-optional, A6 Q5 bit10↔M56S bit10-normal, A6 Q4 quick-stop↔M56S same-objects, A6 Q10 units↔M56S 10000/rev, A6 Q2/#45 silent-mode-ignore↔M56S §4.2.3.5 mode-error.
- The genericity payoff: the only per-device data the M56S needs are `counts_per_rev=10000`, `fault_reset_mechanism=bit7` (the default), DC-optional (off for PP/PV), and the quick-stop/torque magnitudes — all through the **same** `SlaveConfig` surface the A6 uses, with *opposite values*. No M56S-specific code.

---

*Sources: M56S and MDX+ EtherCAT Communication/User Manual (920-0179 RevA) —
§3.6 EMCY, §3.7 Distributed Clock, §4.2.3 mode-switch precautions, §4.3+ control
modes, §5.7 electronic gear, and the object-dictionary tables. `m56s.txt:` line
refs index the extracted manual text used during this review. Facts cross-checked
against the manual; torque-mode objects noted as out-of-scope/optional.*
