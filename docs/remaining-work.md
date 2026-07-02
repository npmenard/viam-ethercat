# Remaining Work — EtherCAT master lib + servo Viam module

_Status as of 2026-07-02. master @ `e1d5992`._

## Where we are (context)
The library + module are functionally complete and offline-gated. On the real A6 (`enp86s0`),
the **library/tool path** is HW-proven: DC/SYNC0 bring-up, PP move, and the runtime PP→PV
mode-switch (P3c) all validated energized with clean wire traces (`cw=0x0F` held through the
`0x6060` switch, no coast). The **generic policy + config ergonomics** are landed and DA-signed:
`#59` (noise-robust reach), `#61` (control_mode intent → derived PDO map + `switchable`),
`#64` (minimal config), `#67` (false-reached regression). A self-contained trixie
`module.tar.gz` (glibc ≥ 2.38 host) is built + staged.

What has **not** happened yet: the **module** (viam-server / RDK path) has never driven the
**real** drive — only sim. That's the finale.

---

## 1. The finale — #37: module-on-real-A6 (RDK), energized
This is the last substantive milestone. Two energized runs, **both user-gated + shaft-clear**.
⚠️ The module **auto-energizes on load** (lifecycle climbs Init→Enabling→OperationEnabled with
no command), so *loading the module against the real A6 is itself an energize event* — shaft
must be clear before viam-server starts.

### 1a. First RDK run — PP, single-mode
- **Prereqs:** user's viam-server up (glibc ≥ 2.38 host / container); caps granted
  (`CAP_NET_RAW`/`NET_ADMIN`/`SYS_NICE`/`IPC_LOCK` + RT — `tools/p37/grant-caps.sh`); NIC up.
- **Artifacts (staged):** `/home/viam/ethercat-servo-trixie.tar.gz` (+ unpacked
  `/home/viam/ethercat-servo-module/`), config `/home/viam/a6-robot.trixie.json`
  (PP, `0x6060` SDO-only, `0x6061` mapped → mode-echo gate active), Python client
  `tools/p37/servo_smoke.py`.
- **Steps:** load module → bring-up to OperationEnabled → `is_powered`/`GetPosition` →
  small `GoTo` → `Stop` (Halt, stays energized) → teardown (Quick-Stop ramp → de-energize).
- **Acceptance (DA evidence bar, tcpdump):** DC OP / WKC=3 / no Er74 / `0x6061`==PP (gate
  passed) / `is_powered` / Position readable; `cw=0x0F` held through the move, `badWKC=0`,
  torque never 0 (no-coast); Stop = bit8 (not `0x02`); teardown = `0x0B` ramp (the 5d landmark).

### 1b. `switchable` on real A6 (module analog of P3c) — HELD until 1a passes
Enabling `control_mode: switchable` maps `0x6060`, which activates the `#56` seed + `#57`
gate + the §6 runtime switch **on live hardware** (currently bench-/sim-only). This is the
exact class where P3c's sim-fidelity gap bit us, so it gets its own energized validation.
- **Config:** `control_mode: switchable` (driver derives the 15-byte superset RxPDO).
- **Acceptance (DA, tcpdump):** `GoTo`→`SetRPM` drives a confirmed PP→PV switch — `cw=0x0F`
  held through the `0x6060` write, no coast, `0x6061` follows; **plus** the 500-count velocity
  backstop check under the A6's ±3300 c/s noise (mode-switch STOP-FIRST completes, no
  de-energize hang — the offline-ungatable part of `#59`, see `#66`).

_Single-mode PP/PV ships on the 1a path; `switchable` must not ship on offline-green alone._

---

## 2. Hardening / coverage (low-priority, non-blocking)
- **#66 — make the 500-count velocity backstop offline-gatable.** Extend SimBackend
  `report_noise` to also jitter `0x606C` (velocity), not just position, so the `0→500`
  quick-stop backstop regression can be caught offline. Its real-HW verification is folded
  into 1b above; this is the offline complement.
- **#65 — pin the module build to the noble container.** The host keeps dropping
  `pkg-config`/`libgrpc++-dev`/`libprotobuf-dev`; the module only reliably builds in
  `ethercat-noble-gate:amd64`. Make a container-only build target so no one depends on
  host-installed grpc/protobuf. (Relates to #58.)
- **#58 — refresh the stale published ghcr module image.** `ghcr.io/viam-modules/ethercat:amd64`
  is Ubuntu 22.04/gcc-11 (no `<format>`) → can't build current code; push the noble image.

---

## 3. Older backlog (from Phase-7, deferred)
- **#25 — harden `bringup_step`: require a plausible statusword before OP.** The #25 zombie-PDO:
  the 1st bring-up after power-on can reach DC OP + WKC 3/3 but with `sw=0x0` (dead PDO); the
  2nd is healthy. Gate OP on a plausible statusword, not just WKC.
- **#22 — A6 vendor-SDO fault-reset (`0x2031:01`) for steady-state operator recovery.** The
  bring-up-time reset exists (`#39`); this is the runtime operator-facing recovery path.
- **#28 — trim now-redundant `mbxempty`/warm-up in `open()`** (wire-confirm first).
- **#21 — thin ec_sample-equivalent test program using our Master API** (a minimal API smoke).
- **arm64 CI (TODO-9)** — native arm64 runner for the seqlock weak-ordering tests (x86 TSO
  masks ARM ordering bugs) + image republish. Currently a documented coverage gap.

---

## Process notes (learned this cycle)
- **Isolated worktree per agent** — execution gates run in a throwaway `git worktree add
  --detach`; never mutate an implementer's shared worktree (a blanket `git checkout` clobbered
  uncommitted WIP once). Implementers commit every slice on compile.
- **Run-don't-trust** — DA's execution gates caught three issues a code read missed this cycle
  (retry-storm coverage, offline-ungatable backstop, false-reached guard). Energized HW is the
  final arbiter for any drive-behavior claim (the P3c mode-seed bug passed sim green).
