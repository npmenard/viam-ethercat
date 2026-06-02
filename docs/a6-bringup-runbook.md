# A6-EC Bench Bring-up Runbook (Phase 7)

Step-by-step procedure to bring the `viam:ethercat:servo` motor module up on a **real A6-EC drive** on a bench. This is the hardware-independent prep + the on-bench sequence; do the prep now, run the bench steps when the drive + NIC are in front of you.

**Read alongside:**
- [`docs/a6-hardware-wiring.md`](./a6-hardware-wiring.md) — power/motor/comms wiring + the ⚠️ mains-voltage safety section. **Do the wiring (and its safety steps) before anything here.**
- [`docs/deployment-capabilities.md`](./deployment-capabilities.md) — granting `CAP_NET_RAW`/`CAP_NET_ADMIN`/`CAP_SYS_NICE`/`CAP_IPC_LOCK` + RT/memlock limits to the module process.
- A6-EC drive manual (`a6-manuals/`) — object dictionary, CiA402, fault codes. Section cites below (e.g. "§10.x") refer to the A6 manual.

> ⚠️ **Safety:** the wiring doc's safety section governs. Mains voltage; qualified electrician for power wiring; power off ≥10 min before touching terminals; PE ground; MCCB + contactor on the input. On the bench, **couple the motor to nothing** (or a known-safe inertia) for the first enable/jog — a mis-scaled velocity or a runaway will spin the shaft.

---

## 0. What "done" looks like (acceptance)
The drive enables through the CiA402 ladder, a PV jog and a PP move both work, reported position/current are sane, the mode guards + fault-reset behave, and the working counter (WKC) is stable over a sustained run. Full checklist in §6.

---

## 1. Pre-flight (do this before the bench)

**1.1 Build + package the module.**
```
make module.tar.gz        # CPack TGZ -> module.tar.gz (bin/ethercat-servo + meta.json)
```
Install it as a local module in your robot config (point the robot at the unpacked entrypoint, or upload the tarball). Confirm it loads in **sim** first (`"simulate": true` / `"interface": "sim"`) — that's the Phase-6 offline gate and proves the module + config parse before hardware is involved.

**1.2 Grant capabilities** to whatever process will exec the module (per `docs/deployment-capabilities.md`):
- `CAP_NET_RAW` (+ `CAP_NET_ADMIN`) — SOEM's raw socket; without it `ec_init`/NIC open fails.
- `CAP_SYS_NICE` (or `RLIMIT_RTPRIO`) — `SCHED_FIFO` for the RT thread.
- `CAP_IPC_LOCK` (or `RLIMIT_MEMLOCK`) — `mlockall` to avoid page-fault jitter.

Recommended: the systemd **ambient-capabilities** drop-in on the viam-server/viam-agent unit (survives exec + re-deploys), plus `LimitRTPRIO=99` / `LimitMEMLOCK=infinity`. Verify with the steps in that doc's "Verifying the grant" section **before** you expect RT to work.

**1.3 Dedicated NIC.** Use a **dedicated** wired NIC for EtherCAT — not the management interface. Note its name (`ip link`, e.g. `enp3s0`). The bus must be the only thing on that NIC; no switch, no other traffic. A PREEMPT_RT host is assumed (see deployment doc → "Host requirements").

**1.4 Wiring.** Complete `docs/a6-hardware-wiring.md` end-to-end: nameplate voltage class → main power (L1/L2 or R/S/T per frame size) → U/V/W to motor → PE → CN3 (EtherCAT IN) from the host NIC → CN2 encoder. Leave CN4 (EtherCAT OUT) open for a single drive. Power on the drive only after the wiring checklist there is signed off.

---

## 2. Bus scan (confirm the drive is on the wire)

With the drive powered and CN3 cabled to the NIC, run the standalone scanner (links SOEM, no SDK, needs `CAP_NET_RAW`):
```
sudo ./build/ec_scan <nic>          # e.g. ec_scan enp3s0   (or grant caps and drop sudo)
```
Expect:
```
ec_scan: found 1 EtherCAT slave(s) on 'enp3s0':
  slave 1: <A6 device name>
```
**Confirm:** exactly **1** slave, and the name is the A6. To confirm vendor/product id, read the CoE **identity object 0x1018** (`:01` vendor, `:02` product code) — either extend `ec_scan` to print it or read it via an SDO tool; match against the A6 nameplate/manual.

- **0 slaves found** → §7 (link/caps/wiring).
- **>1 slaves** → you've got the wrong NIC (seeing other devices) or a daisy-chain you didn't expect.

---

## 3. Configure the module against the real drive

**3.1 Swap SimBackend → SoemBackend.** In the component config, set `"interface"` to the **real NIC name** (e.g. `"enp3s0"`) and **remove `"simulate"` / don't use `"sim"`**. That alone flips the backend factory from SimBackend to SoemBackend (`interface != "sim" && !simulate` → real). Use the **hardware profile `etc/a6-hardware.example.json`** (#14, the unified PP+PV map below) as the base; `etc/a6-servo.example.json` is the sim quickstart.

**3.2 The A6 `SlaveConfig` is CONFIG DATA** (never hardcoded; cpp-expert's `etc/a6-hardware.example.json` (#14) encodes this same map in decimal, since JSON has no hex). Authoritative map, verified against the A6 manual §10 (object dictionary + PDO config). Mapping-word format: each entry packs as `index<<16 | subindex<<8 | length_bits` (length byte `08`=8b, `10`=16b, `20`=32b) — e.g. `6040:00`/16b → `0x60400010`; entries pack byte-aligned in map order, little-endian on the wire (our cursor handles LE).

The recommended map is a **single unified RxPDO/TxPDO that serves BOTH PP and PV** (mirrors the drive's own fixed preset `0x1703`), with the active mode selected by `0x6060`:

**RxPDO `0x1600` (master→drive, SM `0x1C12`), 5 entries / 15 B:**
| Object | Hex | Type | Bits | Role |
|---|---|---|---|---|
| Controlword | `0x6040:00` | U16 | 16 | command |
| Mode of operation | `0x6060:00` | I8 | 8 | **1 = PP, 3 = PV** |
| Target position | `0x607A:00` | I32 | 32 | PP |
| Target velocity | `0x60FF:00` | I32 | 32 | PV |
| Profile velocity | `0x6081:00` | U32 | 32 | PP profile speed |

**TxPDO `0x1A00` (drive→master, SM `0x1C13`), 6 entries / 15 B:**
| Object | Hex | Type | Bits | Role |
|---|---|---|---|---|
| Fault code | `0x603F:00` | U16 | 16 | error code |
| Statusword | `0x6041:00` | U16 | 16 | status |
| Mode display | `0x6061:00` | I8 | 8 | echoes `0x6060` |
| Position actual | `0x6064:00` | I32 | 32 | feedback |
| Velocity actual | `0x606C:00` | I32 | 32 | feedback |
| Torque actual | `0x6077:00` | I16 | 16 | ‰ rated (current proxy) |

> **⚠ `0x6060` mode-of-operation must be set** — with the unified map it's an RxPDO field, so the RT loop must **write the configured mode value (1 or 3) into it every cycle**; otherwise the drive sees mode 0 (no mode) and won't enter PP/PV. *Alternative (simpler for our single-mode-per-config MVP):* drop `0x6060` from the RxPDO and **set it once via SDO** at `configure()` (`SDO 0x6060 ← 1` or `3`). cpp-expert + I are settling which (#14 + a small controller change) — see the note at the end of this section. Whichever is chosen, **the mode must be set somewhere; the SimBackend doesn't exercise this, so it's a hardware-only failure mode if missed.**

Limits: max 10 entries / 40 B per map. The A6 `0x1600`/`0x1A00` are the *configurable* PDOs we remap; `0x1701–0x1705`/`0x1B01–0x1B04` are **fixed presets** (no entry-write needed). **No-mapping-write fallback:** if the SDO remap ever misbehaves on the bench, select fixed preset `0x1703` (Output: 6040,607A,60FF,6060,60B8,60E0,60E1) + `0x1B03` (Input) via the SM assignment alone (`0x1C12:01 ← 0x1703`, `0x1C13:01 ← 0x1B03`) — covers PP+PV without writing any map entries.

**3.3 Re-applied every power-on.** The A6's PDO mapping is **writable only in PRE-OP and is NOT stored in EEPROM** (manual §10) → `configure()` re-applies it on every start/power-on. The exact CoE sequence the module runs (all in PRE-OP, per slave): `0x1C12:00 ← 0` (clear RPDO assign) → `0x1600:00 ← 0` (clear entry count) → `0x1600:01..N ← ` the packed mapping words → `0x1600:00 ← N` (entry count, **5** for the unified RxPDO) → `0x1C12:01 ← 0x1600` → `0x1C12:00 ← 1`; identical for `0x1C13`/`0x1A00` (count **6**) → `map_process_data` → SAFE-OP → prime → OP. You don't do this by hand — but **confirm it ran**: a clean `configure()` (no `PdoMappingError`, drive reaches OP) means it took. The `applied-image-size == configured-byte_size` guard throws loudly if the remap was silently rejected (§7).

**3.4 Set `require_realtime: true`** for the real run (you want a hard error if RT scheduling isn't available, not silent best-effort). On a properly-capped PREEMPT_RT host this succeeds; if it throws `InitError("real-time scheduling unavailable …")`, fix caps/limits (§1.2) — don't paper over it with `false` on real hardware.

**3.5 ⚠ Confirm-before-energize flags** (verified-on-bench data the manual does *not* fully pin down — check each before the first enable, or motion/torque will be mis-scaled):

| # | Confirm | How | Default / risk if wrong |
|---|---|---|---|
| a | **Electronic gear is 1:1** (so `counts_per_rev = 131072`) | read `0x2010:19`/`0x2010:1A` (panel C10.18 / C10.19, numerator/denominator) | default 1/1; if changed, effective counts/rev changes → set `counts_per_rev` to match |
| b | **Velocity unit** = reference-units/s, `(rpm/60)×counts_per_rev` | jog a **known rpm** and confirm the shaft speed (this scaling is *derived* from the manual's "`60FF ← 1310720` = 600 rpm @ 1:1" example, not a stated unit line) | wrong scale → `SetRPM` runs at the wrong speed |
| c | **Motor rated current** (for amps→torque‰) | read the **motor nameplate/datasheet** — it is **NOT in the drive manual** | until known, leave torque limits at the drive default **3000 ‰ (=300 %)** or set raw ‰; **do NOT guess** the amps→‰ conversion |
| d | **Written map matches the drive's ESI/XML** | compare the entries in §3.2 against the drive's ESI file object definitions | a mismatched entry → `PdoMappingError` / wrong field offsets (the size guard catches gross cases) |
| e | **Distributed-clocks need** | does the drive demand SYNC0 for PV/CSP? | default off; decide on bench (§5.2) |

---

## 4. The bench sequence (enable → PV jog → PP move)

Drive these via the SDK (motor methods) or `DoCommand`. Watch the statusword each step (read it via `DoCommand {"status": true}` → `last_error`/flags, or an SDO read of `0x6041`).

**4.1 Enable ladder (automatic).** On `start()` the RT loop runs the CiA402 ladder: `0x06` (shutdown) → `0x07` (switch-on) → `0x0F` (enable-operation). The statusword should decode **SwitchOnDisabled → ReadyToSwitchOn → SwitchedOn → OperationEnabled** within a few cycles. Confirm `is_powered() == true`.
- ⚠️ At OperationEnabled the drive is **energized and holding** — shaft is now live.

**4.2 PV jog (if `control_mode: "PV"`).** `SetRPM(small rpm)` — e.g. 60 rpm. Confirm the shaft turns at ~the commanded speed and the **sign/direction** matches expectation (flip via `0x607E` polarity in config if reversed). `is_moving() == true` while turning. `SetRPM(0)` / `Stop()` → halts and **stays stopped** (Stop = Halt, bit 8, sticky; the drive stays enabled/re-commandable). Verify reported velocity (`0x606C`) tracks.

**4.3 PP move (if `control_mode: "PP"`).** `GoTo(rpm, target_revs)` or `GoFor(rpm, revs)`. Watch the **new-setpoint handshake**: controlword `0x0F → 0x1F` (bit 4 rising) → **statusword bit 12 (set-point acknowledge) goes 1** → controlword drops bit 4 → bit 12 clears. The module's handshake sub-FSM does this with a timeout; if the drive never acks within `handshake_timeout_cycles`, `GoTo` throws "set-point acknowledge timed out" (§7).
- **Move-complete is by the position predicate**, `|target − actual| ≤ position_tolerance_counts && |vel| ≤ velocity_threshold` — **NOT statusword bit 10**. On the A6, **bit 10 ("target reached") is hard-wired to 1 and unusable**; the module never reads it. Confirm `GoTo(X)` lands at `Position() ≈ X` (within tolerance) and `is_moving()` goes false at completion.
- After `ResetZeroPosition(0)`, `Position()` reads ~0 at the current shaft position, and a subsequent `GoTo(X)` lands at `Position()==X` (absolute, zeroed frame).

**4.4 Scaling — two distinct gears, don't conflate them.**
- **`counts_per_rev = 131072`** (2¹⁷ absolute encoder) — this holds at the **drive's electronic gear 1:1** (`0x2010:19`/`:1A`, panel C10.18/C10.19). The drive's e-gear changes the effective reference-units/rev; if it's not 1:1, set `counts_per_rev` to match (flag a, §3.5).
- **`gear_ratio`** is the **mechanical motor→load ratio**, applied by **our** conversion layer for Viam-reported position/velocity — the **drive doesn't know about it**. (So `0x607A`/`0x6064` are in motor reference units; our layer maps motor↔load.)
- **Velocity unit** (`0x60FF`/`0x606C`/`0x6081`) = **reference-units/second**: `device_vel = (rpm/60) × counts_per_rev`. Confirm against a known-rpm jog (flag b) before trusting `SetRPM`.
Confirm one commanded motor revolution = one physical motor revolution; if off by a constant factor, `counts_per_rev` or the drive e-gear is wrong.

---

## 5. Decisions to make on the bench (settle these here, they can't be guessed)

1. **Motor rated current** (for amps → torque ‰). The A6 expresses torque/current limits — Max torque `0x6072`, positive `0x60E0`, negative `0x60E1` (all U16, **default 3000**, range 0–4000) — and actual torque `0x6077` (I16) in **‰ of motor RATED torque** (1000 = 100 %, 4000 = 400 %/peak), **NOT amps**. Since torque ∝ current for a PMSM: **`‰ = round(1000 × target_amps / motor_rated_current_amps)`**. The **rated current is on the MOTOR nameplate, not in the drive manual** (flag c, §3.5) — read it on the bench, set `motor_rated_current_amps` so `peak_current_amps` maps to a correct ‰ (sanity: at rated current → ~1000 ‰). **Until known, leave the limits at the default 3000 ‰ or set raw ‰ — do not guess.** `0x6077` is the closest thing to a current readback (torque ‰ → amps via the same ratio).
2. **Distributed clocks (DC) on/off.** Default off. If the A6's CSP/CSV sync mode (or jitter) needs DC, enable it — decide based on the jitter histogram + whether the drive demands SYNC0. For PP/PV at ≤1 kHz, free-run is usually fine; measure before enabling DC.
3. **`target_loop_rate_hz`.** Start at the configured rate (≤1000). Watch the WKC stability + the loop jitter; back off if the host can't hold the deadline on this NIC. (PREEMPT_RT + isolated NIC should hold 1 kHz.)

Record the chosen values back into the A6 profile config (they're config data, not code).

---

## 6. Validation / acceptance checklist

- [ ] `ec_scan` finds exactly 1 slave; identity (`0x1018`) matches the A6.
- [ ] `configure()` completes with no `PdoMappingError`; drive reaches **OPERATIONAL**; applied image size == configured `byte_size` (guard didn't throw).
- [ ] Enable ladder reaches **OperationEnabled**; `is_powered() == true`.
- [ ] **PV:** `SetRPM` turns the shaft at the commanded speed, correct direction; `Stop` halts and stays halted; reported velocity tracks.
- [ ] **PP:** new-setpoint handshake observed (bit 4 ↑ → **bit 12 ack** → bit 4 ↓); `GoTo(X)` lands at `Position() ≈ X` via the |Δ| predicate (**bit 10 never used**); `is_moving()` true→false correctly.
- [ ] `Position()` reports sane revs; `ResetZeroPosition` zeroes it and absolute `GoTo` respects the new zero.
- [ ] Reported **current/torque** is sane (`0x6077` torque actual) and `peak_current` limit maps to the right ‰.
- [ ] **Mode guards:** `SetRPM` rejected in PP (clear error), `GoTo` rejected in PV.
- [ ] **Fault path:** trip a fault (e.g. brief over-travel / e-stop), confirm `is_powered()` goes false + `last_error` reports the CiA402 fault, then `DoCommand {"fault_reset": true}` recovers to OperationEnabled (bit 7 rising edge); a *persistent* bus fault still requires reconfigure.
- [ ] **WKC stable:** over a sustained run (minutes) the working counter stays at expected; no `BusError` latched (`max_consecutive_wkc_errors` not hit).
- [ ] **Move completion / blocking:** `GoTo`/`GoFor` block until complete and return; a jammed/over-time move throws "move stalled/timed out" without de-powering a healthy drive.

---

## 7. Troubleshooting

**No slaves found (`ec_scan` = 0).**
- Caps: `CAP_NET_RAW` missing → `ec_init` can't open the socket (`docs/deployment-capabilities.md` → "Verifying the grant"). Run `ec_scan` with caps or `sudo` to isolate.
- Wrong/inactive NIC: `ip link set <nic> up`; confirm you're on the dedicated EtherCAT NIC, not the management one.
- Cabling: host NIC → drive **CN3 (IN)**, not CN4 (OUT). Check link LEDs. Drive powered (control power present).

**`InitError: real-time scheduling unavailable …`** → `CAP_SYS_NICE`/`RLIMIT_RTPRIO` not granted, or not PREEMPT_RT. Fix per §1.2; only set `require_realtime:false` for dev, never to mask a real-hardware cap problem.

**`PdoMappingError` during `configure()`** (the SDO PDO-remap dance):
- "SDO write … CoE abort 0x06010002" / similar → the object isn't mappable, or you're not in PRE-OP, or you used a fixed-preset PDO (`0x1701…`) instead of the configurable `0x1600`/`0x1A00`. Use the configurable map objects (§3.2).
- "applied image RxPDO N B / TxPDO M B != configured …" → the remap was silently rejected or partially applied; the wire image doesn't match the config. Re-check the entry list (index/subindex/bit_length) against the A6 manual's object dictionary; confirm widths (ctrl/status 16-bit, target/actual/velocity 32-bit).

**WKC errors / `BusError` latched** (`working-counter fault: got X, expected Y`):
- Cabling/EMC: reseat CN3, check shield/ground, separate motor power from the EtherCAT cable. A flaky link drops the WKC.
- A slave dropped to SAFE-OP (sync error) with otherwise-nominal WKC — the current `process()` watches WKC only; a SAFE-OP drop is a Phase-7 recover-path item (re-read AL state, `ec_recover_slave`, re-apply map). For now, reconfigure to recover.
- If it's a single transient cycle, the consecutive-error threshold (`max_consecutive_wkc_errors`, default 5) absorbs it; sustained errors latch the fault → reconfigure (the latch is sticky-until-configure by design).

**CiA402 won't leave a state** (stuck below OperationEnabled):
- Read `0x6041` and decode (the module's mask table). Stuck in `SwitchOnDisabled` → check `EnableVoltage`/main power; stuck in Fault → read the A6 **fault code `0x603F`** and the manual's fault table; clear the condition then `fault_reset`.
- `QuickStopActive` → the drive took a quick-stop; recover via disable-voltage → re-enable.

**PP move never completes / handshake timeout:**
- "set-point acknowledge timed out" → the drive isn't asserting statusword **bit 12** within `handshake_timeout_cycles`. Confirm the drive is actually in PP mode (`0x6060 == 1`, display `0x6061`), and that `0x607A` (target) is in the RxPDO map. Raise `handshake_timeout_cycles` only if the drive is genuinely slow to ack.
- "move stalled / no progress" → the shaft isn't advancing toward target (mechanical jam, or profile velocity `0x6081` too low / zero, or current limit clamping). Check the torque-limit ‰ (§5.1) isn't starving the move.

**Wrong direction / wrong distance:**
- Direction reversed → set polarity `0x607E` in config, or flip the sign convention expectation.
- Distance off by a constant factor → `counts_per_rev` (should be 131072) or `gear_ratio` wrong.

---

### Document status
Hardware-independent prep + bench procedure authored ahead of Phase 7. The on-bench steps (§2, §4, §5) require the physical A6 + NIC; the decisions in §5 (rated current, DC, loop rate) are settled on the bench and recorded back into the A6 profile config. A6 `SlaveConfig` data (PDO maps, mode codes, units, torque‰) is from the librarian's manual-verified extract (§10 object dictionary + PDO config). Cross-references: `docs/a6-hardware-wiring.md` (wiring + safety), `docs/deployment-capabilities.md` (caps + RT), `etc/a6-hardware.example.json` (the A6 hardware profile, #14) + `etc/a6-servo.example.json` (sim quickstart), A6-EC manual (`a6-manuals/`, object dictionary + fault codes).
