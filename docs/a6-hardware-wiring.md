# A6-EC Drive — Power / Motor / Comms Wiring (Bench Bring-up)

Reference wiring for bringing up an **A6-EC** servo drive on the bench (Phase 7).
All facts here are transcribed from the *A6-EC Series Servo Drive Manual*
(`a6-manuals/A6-EC_series_servo_drive_manual.pdf`), with section/figure/table
numbers cited inline.

> ⚠️ **This document is reference information only.** Terminal layout, voltage
> class, and braking arrangement **differ by frame size and model**. You **must**
> confirm every connection against the **nameplate and terminal diagram on your
> physical unit** before applying power. When in doubt, follow the manual, not
> this summary.

---

## ⚠️ SAFETY — read first

Mains voltage is present. Mistakes here can kill.

- **Qualified personnel only.** "Only electrical engineering specialists can
  perform wiring." (§3.2 System Wiring, WARNING)
- **Power off and wait ≥ 10 minutes before wiring or touching power terminals.**
  Residual high voltage remains on the DC bus after power-off. (§3.2 WARNING;
  §3.4 CAUTION; Safety chapter — "Do not touch terminals with power on or within
  10 minutes after disconnecting the power.")
- **Verify the nameplate voltage class FIRST.** Connecting a 220 V-class drive to
  380–440 V (or vice-versa) will destroy it. (§1.3 Nameplate; Table 3-1 — power
  input "as per the rated voltage class on the nameplate.")
- **Ground PE.** Connect the drive PE terminal to the control-cabinet earth and
  ground the whole system. Use a grounding cable of the same cross-section as the
  main-circuit cable (≥ 2.0 mm² if the main cable is < 1.6 mm²). (§3.2 WARNING;
  §3.4 CAUTION; §3.4.3 Grounding)
- **Install protection on the input:** a circuit breaker / MCCB **and** an
  electromagnetic contactor in series between the supply and the drive main
  circuit, so power can be cut on a fault. **Do not** use the contactor to
  start/stop the motor. (§3.1 System Topology; §3.2 WARNING; §3.4.1; §13.4
  Circuit Breaker)
- **Never** connect the motor output terminals **U/V/W to mains**, and never
  connect mains to U/V/W. Connect the drive to the motor **directly** — no
  contactor between drive and motor. (§3.2 WARNING/CAUTION)
- Use the **TN or TT** grid, **never an IT** grid. (§3.2 WARNING)
- Separate main-circuit cables from I/O and encoder cables by **≥ 30 cm**; use
  shielded twisted pair for signal/encoder. (§3.2 CAUTION; §3.4 CAUTION)
- Do **not** cycle power frequently (no more than once a minute). (§3.4 CAUTION)

---

## 1. Identify the voltage class by nameplate (§1.3)

Read the `INPUT:` line on the drive nameplate. Example from the manual (§1.3,
A6-400EC):

```
MODEL:  A6-400EC
INPUT:  1PH/3PH AC200-240V 4.0A 50/60Hz     <-- 220 V class
OUTPUT: 3PH AC0-240V 2.8A 0-500Hz 400W
```

- `…AC200-240V…` → **220 V class** (this is the bench target).
- Three-phase **380–440 V** variants exist (e.g. A6-2000EC, A6-3000EC, §3.4.1
  NOTICE) — **different wiring**, different terminals. Verify the nameplate; do
  not assume.

Frame size ↔ model ↔ supply (§1.5 Rated Data; §3.4.1 NOTICE):

| Frame | Model | Power | Main-circuit supply |
|---|---|---|---|
| SIZE A | A6-200EC / A6-400EC | 0.2 / 0.4 kW | Single-phase 200–240 V AC |
| SIZE B | A6-750EC | 0.75 kW | Single-phase 200–240 V AC |
| SIZE C | A6-1000EC | 1.0 kW | **Single- or three-phase** 200–240 V AC |
| SIZE D | A6-1500EC | 1.5 kW | **Single- or three-phase** 200–240 V AC |
| SIZE D | A6-2000EC / A6-3000EC | — | **Three-phase 380–440 V AC** |

> SIZE C/D 220 V models accept single-phase **or** three-phase main power on
> R/S/T, "depending on which one is available on site." (§1.5 NOTICE; §3.4.1)

---

## 2. Terminal layout by frame size (§3.3 Ports; Table 3-1)

### SIZE A / SIZE B — single-phase 220 V

Power/motor terminals (§3.3 Ports; Table 3-1 "Main circuit terminals of SIZE
A/B"; §3.4.1 Fig. 3-2):

| Terminal | Function | Notes |
|---|---|---|
| `L1`, `L2` | **Main power input** | 200–240 V AC **across L1–L2**. **No separate neutral terminal** — a 220 V single-phase supply connects line+neutral (or two hot legs) to L1 and L2 per nameplate class. |
| `U`, `V`, `W` | **Motor power output** | → motor phases U/V/W. Wire colors in Fig. 3-2: U=white, V=black, W=red, PE=yellow/green. |
| `PE` | **Motor / system ground** | Protective earth to cabinet earth. |
| `P`, `C` | External braking resistor | **SIZE A:** connect resistor between `P` and `C`. |
| `P`, `D`, `C` | External braking resistor | **SIZE B:** connect resistor between `P` and `C`, and **remove the P–D jumper bar first** — otherwise the braking transistor is damaged by overcurrent. (Table 3-1 Note; §3.1 NOTICE) |
| `P`, `N` | Servo DC bus | Shared DC bus when multiple drives share one bus; `N` = bus negative. |

> SIZE A braking is an **external** resistor; SIZE B has a **built-in** braking
> resistor but still exposes P/D/C for an external one. (§1.5 Rated Data)

### SIZE C / SIZE D — single- or three-phase 220 V

These frames add a **separate control-circuit power input** distinct from the
main circuit (§3.3 Ports; Table 3-1 "SIZE C"; §3.4.1 Fig. 3-3):

| Terminal | Function | Notes |
|---|---|---|
| `L1C`, `L2C` | **Control circuit power input** | Single-phase 200–240 V AC per nameplate class — powers the drive's control electronics independently of the main bus. |
| `R`, `S`, `T` | **Main circuit power input** | Single- **or** three-phase 200–240 V AC. For single-phase use, follow the manual's single-phase terminal assignment for R/S/T. |
| `U`, `V`, `W` | **Motor power output** | → motor phases. |
| `PE` | **Motor / system ground** | Protective earth. |
| `P`, `D`, `C` | External braking resistor | Connect resistor between `P` and `C`; **remove the P–D jumper first**. (Table 3-1 Note) |
| `P`, `N` | Servo DC bus | `N` = bus negative; for shared-DC-bus setups. |

> SIZE C/D have a **built-in braking resistor**; P/D/C are for adding an external
> one. (§1.5 Rated Data)

### Main-circuit wiring topology (§3.1, §3.4.1)

Supply → **Circuit breaker (MCCB)** → **Filter** → **Electromagnetic contactor**
→ drive main-power terminals. A surge-protection device and STOP/RUN control of
the contactor coil are shown in Fig. 3-2/3-3. The contactor is for fault
isolation, **not** for running/stopping the motor.

---

## 3. Communication & signal connectors (§3.3, §3.8, §3.9)

| Connector | Function | Notes |
|---|---|---|
| `CN3` | **EtherCAT network port IN** | From host controller / upstream slave. (§3.3; Fig. on §3.3 "Communication network port IN") |
| `CN4` | **EtherCAT network port OUT** | To the next downstream slave (daisy-chain). |
| `CN6` | **Commissioning & communication port** | Type-C → serial/USB for the PC commissioning tool (§3.9; Type-C to serial→USB). |
| `CN2` | **Encoder port** | To motor encoder. Pinout (§3.6): 1=+5V, 2=0V, 5=PS+, 6=PS−, shell=PE shield. Max encoder cable 10 m. |
| `CN1` | **User control / I/O port** | DI/DO. Per §3.7: DI1=positive limit, DI2=negative limit, DI3=home, DI4=probe2, DI5=probe1; +24 V internal supply (20–28 V), COM+/COM−; DO1=Servo ready, DO2=Fault, DO3=Brake (each DOx is a ± pair, max 30 VDC / 50 mA). Max I/O cable 3 m. |

EtherCAT cabling (§3.2, CN3/CN4 pinout): standard 100BASE-TX twisted pair —
TD+/TD−/RD+/RD−. **Topology: host → CN3 (IN) of slave 1 → CN4 (OUT) → CN3 (IN) of
slave 2 → …** Daisy-chain IN-to-OUT down the line.

---

## 4. Bench bring-up checklist

1. **Power off; wait ≥ 10 min.** Confirm the charge indicator (⑥, §1.4) is off
   before touching terminals.
2. **Verify the nameplate** voltage class and frame size; pick the matching
   terminal map above.
3. **Main power:**
   - SIZE A/B: supply → MCCB → contactor → `L1`/`L2`.
   - SIZE C/D: control supply → `L1C`/`L2C`; main supply → `R`/`S`/`T`.
4. **Motor:** drive `U`/`V`/`W` → motor `U`/`V`/`W` (white/black/red), directly,
   no contactor between.
5. **Ground:** drive `PE` and motor ground → cabinet earth (same cross-section as
   main cable).
6. **Braking resistor (if used):** remove the **P–D jumper** (B/C/D), connect
   resistor across `P`–`C`.
7. **Encoder:** motor encoder → `CN2`.
8. **I/O (optional for bring-up):** limits/home/brake → `CN1`; keep I/O and
   encoder cables ≥ 30 cm from main-circuit cables.
9. **EtherCAT:** host NIC → `CN3` (IN); `CN4` (OUT) → next slave if chaining.
10. **PC commissioning (optional):** Type-C → `CN6`.
11. Double-check no loose screws/strands, insulation on power terminals intact,
    then power on. Do not re-cycle power more than once per minute.

---

## 5. Cable sizing (reference, §3.4.2 Tables 3-2 / 3-3)

| Frame | Model | Rated input | Max output | Input cable |
|---|---|---|---|---|
| SIZE A | A6-200EC | 2.3 A | 5.8 A | 0.75 mm² |
| SIZE A | A6-400EC | 4 A | 10.1 A | 0.75 mm² |
| SIZE B | A6-750EC | 7.9 A | 16.9 A | 0.75 mm² |
| SIZE C | A6-1000EC (1φ) | 9.6 A | 23 A | 1 mm² |
| SIZE D | A6-1500EC (1φ) | 8 A | 32 A | 0.75 mm² |

Main-circuit cable: rated ≥ 600 V AC, ≥ 75 °C, bend radius ≥ 10× OD, shielded for
EMC. (§3.4)

---

### Document status

Transcribed from the A6-EC manual for Phase-7 bench planning. **Reference only —
confirm against the physical nameplate and the terminal diagram printed on the
unit before wiring.** Section/figure/table citations above point back to the
manual for authoritative detail.
