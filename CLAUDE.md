# CLAUDE.md — EtherCAT master lib + servo Viam module

Project: a SOEM-based EtherCAT master **library** + a generic **servo Viam module**
(`rdk:component:motor`, CiA402). First HW target: ANCTL **AS715N** (A6-EC) servo on
`enp86s0`. C++20, boost, cmake, clang-19. Bench: `build-hw/` (`-DETHERCAT_BUILD_MODULE=OFF`),
run `sudo ./build-hw/a6_validate enp86s0 ...` (NIC must be UP: `sudo ip link set enp86s0 up`,
carrier takes a few s; sudo is passwordless here).

---

# A6-EC / AS715N DISTRIBUTED-CLOCK (DC/SYNC0) BRING-UP — debugging reference

The hard part. This drive **requires DC SYNC0** (free-run = `AL 0x0027` "Freerun not
supported"). Everything below is hard-won from on-hardware debugging — read it before
touching the DC path again.

## ✅ SOLVED on OUR driver (2026-06-03). Stable DC OP hold on the `Master` API, matching ec_sample.
`a6_validate enp86s0` reaches `*** OPERATIONAL *** WKC=3/3`, `0x1C32:01=2` (DC self-selected),
`0x603F=0x0` (Er74.1 cleared at OP), `badWKC=0` for the full run. Three fixes, in order, each
a diff against ec_sample (commits f145c79 → 976b5d9 → 45d0b50):
1. **`open()` must do a confirmed-PRE-OP settle before ANY SDO** (ec_sample.c:301-327):
   `ctx.manualstatechange=1` → `ecx_readstate` → drive bus to PRE-OP → `ecx_statecheck(PRE_OP,
   3*EC_TIMEOUTSTATE)`. WITHOUT it the A6 CoE mailbox isn't ready and every `ecx_SDOread`/write
   returns **WKC 0** ("SDO ... working counter 0"). This is ALSO the "recover from a prior
   faulted run" step — bounce a non-PRE-OP slave through INIT. (A symptom that looked like a
   wedge needing a power-cycle was really this missing settle — ec_sample recovers in software.)
2. **Arm `ecx_dcsync0` in PRE-OP, BEFORE `config_map_group`** (ec_sample order: dcsync0@351 →
   config_map_group@355 → configdc@374). The drive latches its sync-type at the PRE-OP→SAFE-OP
   transition from whether SYNC0 is already armed: arm-first → it **self-selects DC**
   (`0x1C32:01=2`); arm-after-SAFE-OP → it stays **SM** (`=1`) → Er74.1 ~1s after OP → WKC
   collapse. Stock `ecx_dcsync0`, default delay — the ~50ms-watchdog worry (lesson 5 below) does
   NOT bite when the arm sits in PRE-OP with the long config_map+configdc+settle before OP.
3. **Do NOT gate OP on Er74.1.** `0x603F=0x8700` (Er74.1 "no sync") is the NORMAL pre-sync state
   in SAFE-OP — ec_sample reads it, its `0x2031:01` reset does NOT clear it, and it **requests
   OP anyway → succeeds**; Er74.1 clears AT OP once cycling+SYNC0 alignment completes. The real
   "sync proven" gate is a phase-locked-PD settle (~400 cycles, ec_sync PI). Request OP ONCE
   (single request is safe; only HAMMERING wedges the drive), then verify the HOLD post-OP
   (full WKC && Er74.1 cleared); abort cleanly with NO re-request if it won't hold.
- Red herring killed: **ARMW/FRMW frame count is NOT a DC-health signal** — ec_sample holds OP
  with **0 ARMW/FRMW frames** on the wire too. Don't chase DC-datagram distribution.

## Golden rule: there is a KNOWN-WORKING reference. Compare against it FIRST.
`~/SOEM/samples/ec_sample/ec_sample.c` (build: `~/SOEM/build-2/samples/ec_sample/ec_sample`)
brings **this exact drive** to OPERATIONAL in real DC mode (WKC 3/3, live PDO). It is on
**SOEM v2** (`soem/soem.h`, `ecx_config_map_group`, `ecx_mbxhandler`). When DC misbehaves,
diff our sequence against ec_sample and **capture frames with tcpdump** — do NOT permute
configs blind (that wasted ~15 bench iterations).

## What actually makes DC work (lessons that cost us days)
1. **DO NOT manually force `0x1C32:01` (SM sync-type).** ec_sample NEVER writes it, yet it
   reads back `0x0002` (DC SYNC0) in SAFE-OP. SOEM's `config_map_group` + `ecx_dcsync0`
   establish DC and the drive sets its own sync-type. **Forcing `0x1C32:01=2` by hand is an
   incomplete DC-SM config and the drive rejects it with `AL 0x0030` "invalid DC sync
   config".** Our long `0x0030` wall was SELF-INFLICTED by this manual force. (On SOEM
   v1.4.0, no-force left it at `0x0001`/SM-sync → `AL 0x0027` at OP; the right fix is the v2
   `config_map_group` DC-SM setup, not a manual `0x1C32:01` write.)
2. **`0x1C32:02` (cycle time) is NOT the cause of `0x0030`** — disproven (it read `999680`
   once and still `0x0030`'d; on a clean drive it's `0` in SAFE-OP and is only measured AT
   OP). Don't gate on it. (The old "*** :02=0 -> 0x0030 ***" annotation was a wrong guess.)
3. **Keep process data (LRW) flowing through the SAFE-OP→OP transition.** If the master's
   cyclic LRW gaps during the transition, the slave's output **SyncManager watchdog**
   expires → `AL 0x001B`. THE ec_sample bug we found: its main thread runs a tight
   `ecx_statecheck(OP, 5*EC_TIMEOUTSTATE)` (no-sleep BRD flood) **concurrently** with the
   RT process-data thread on the same non-thread-safe SOEM port → the flood starves LRW →
   `0x001B` → intermittent OP (~40% fail). **Fix:** gentle poll
   (`osal_usleep(10000)` between `ecx_readstate`) so PD keeps flowing → 8/8 OP. Our own
   design (single RT I/O thread, RT↔non-RT boundary) avoids this by construction — never run
   two threads doing port I/O.
4. **Vendor fault reset = write `1` to `0x2031:01`** (NOT CiA402 controlword bit 7).
5. If you ever hand-roll the SYNC0 arm (instead of `ecx_dcsync0`): the A6 sync watchdog is
   ~50 ms, SHORTER than SOEM's hardcoded 100 ms `SyncDelay` (ethercatdc.c:24). Arming with
   the 100 ms delay → drive faults Er74.1 ~50 ms after arm, before the first edge. Use a
   ~15 ms start delay. (Moot if you use the stock v2 `ecx_dcsync0` + the working sequence.)

## ec_sample's bring-up order (the working sequence)
init → config_init → manualstatechange=1 → PRE-OP → write PDO map (0x1600/0x1A00 +
0x1C12/0x1C13 assign) → `ecx_dcsync0(slave,TRUE,cycletime,0)` per slave → `config_map_group`
→ `ecx_configdc` → start RT thread (PD + `ec_sync` PI on `ec_DCtime`) → `dorun=1` →
request SAFE-OP → (diagnostics, fault-reset via 0x2031:01) → request OP **while PD flows**.
(Note: ec_sample calls `dcsync0` before `configdc`; canonical is configdc→dcsync0, but it
did NOT measurably change OP reliability here — the `0x001B` flood bug dominated.)

## Drive quirks / gotchas
- Repeated **Er74 OP-entry faults WEDGE the drive**: NIC goes `NO-CARRIER`, no enumeration →
  needs a **control-power cycle**. Don't hammer a persistent fault (bounded give-up).
- **Enumeration is intermittent** on the first attempt(s) after a prior run / power-up —
  retry 2–4×; if `SDO ... working counter 0`, the mailbox is wedged.
- `0x0984`/`0x098E` (SYNC0 activation/status) read 0 from the master even when SYNC0 IS
  firing — they're **PDI-consumed** (the drive's MCU acks them). Don't gate on them; the
  real "synced" signal is the **absence of the Er74.1 no-sync fault** + WKC holding.
- AL codes: `0x0030`=invalid DC sync config, `0x0027`=freerun not supported, `0x001B`=SM
  watchdog. CiA402 `0x603F`: `0x8700`=Er74.1 "no sync signal", `0x6320`=Er74.0 "cycle error".

## Debugging tooling (all built against the bench SOEM)
- **tcpdump + pcap parser** (THE most useful): `sudo tcpdump -i enp86s0 -w x.pcap 'ether
  proto 0x88a4' -U` during a run, then parse with `tools/bench/ecpcap.py` (EtherCAT datagram
  decoder: cmd-type histogram, LRW inter-frame gaps, ARMW/FRMW DC frames, per-500ms timeline).
  LRW count + gaps + BRD-flood vs steady-LRW instantly shows process-data health at the
  transition. Datagram cmds: LRW=0x0C, FPRD=0x04, FPWR=0x05, BRD=0x07, ARMW=0x0D, FRMW=0x0E.
- **slaveinfo / eepromtool**: compile SOEM's `test/linux/{slaveinfo,eepromtool}` against
  `build-hw/_deps/soem-build/libsoem.a`. `slaveinfo enp86s0 -sdo` dumps the full CoE OD
  (0x1C32/0x1C33 sub-tree, supported sync types 0x1C32:04, modes 0x6502). SII DC category
  (cat 60) had AssignActivate `0x0300` = SYNC0-only.
- **dc_probe*.c** (`/tmp/`): standalone SOEM probes for isolating DC behavior fast (compile
  vs libsoem.a, run with sudo) — faster than rebuilding a6_validate per hypothesis.

## METHODOLOGY (the meta-lesson)
When stuck on hardware behavior: (1) get a KNOWN-GOOD reference and diff against it, (2)
CAPTURE THE WIRE (tcpdump) to see what actually happens, (3) verify each hypothesis by
testing the fix, don't claim a cause you haven't proven (we were wrong about `0x1C32:02`,
the `0930` red herring, and the dcsync0/configdc order — all "obvious" theories that the
bench disproved). Ask the user for a working reference / capture EARLY.
