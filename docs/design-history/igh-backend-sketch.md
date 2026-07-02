# IgH EtherCAT master backend — scoping sketch

> **⚠ SUPERSEDED — DESIGN HISTORY (2026-07-02). The core premise below is WRONG.**
> This sketch assumes "SOEM rejects the A6's DC unconditionally (AL `0x0030`) → SOEM
> cannot bring up this servo class → we may need an IgH backend." That was **disproven
> by our own #17/#20 work**: the `0x0030` was **self-inflicted** (we were manually forcing
> `0x1C32:01`), and SOEM **v2** + the ec_sample sequence (arm SYNC0 in PRE-OP, `config_map_group`,
> never force the SM sync-type) brings the A6 to **DC OP** cleanly — validated energized (P3c).
> The IgH backend was **never needed and never built**. Kept only as a record of a considered-
> and-rejected path. Authoritative DC lessons: `CLAUDE.md` + `docs/a6-bringup-runbook.md`.

**Original status (historical):** PREP / decision input. Not implemented. Pending user's call on
the DC pivot.

## Why this exists

The ANCTL A6-EC drive **requires** Distributed Clocks (free-run → AL `0x0027`
"Freerun not supported") but **rejects SOEM's DC configuration unconditionally**
(AL `0x0030` "Invalid DC SYNC configuration"). Exhaustively ruled out on
hardware: CiA mode (PP/CSP), `0x1C33` sync type, vendor `0x2013:06`, the ETG.1020
`0x1C32:08/:0a` cycle-time handshake (writes accept but `0x1C32:02` stays 0),
SYNC0 cycle 250 µs…4 ms, SYNC1 (`dcsync01`), and every SafeOp ordering. At the
silicon (FPRD) `0x0981`/`0x09A0`/`0x0990` are all correct, **but `0x0984`
Activation-status and `0x098E` SYNC0-status stay 0** — the ESC SYNC-out unit
never arms. This matches the known SOEM-vs-CiA402-servo DC limitation (SOEM #142
EL7211, #160, #29; ICube ros2 #154 ZeroErr eRob). IgH and TwinCAT bring up this
class fine; SOEM does not — IgH is itself the DC reference clock and distributes
application time to the slaves **every cycle**, which SOEM's
first-slave-is-reference model does not do.

## Two options (cheapest first)

### Option 1 — capture-diff a working master (do this FIRST, low cost)
Wireshark a TwinCAT / IgH / linuxcnc-ethercat bring-up of THIS drive, diff the
register writes vs ours. If the only delta is **continuous DC time distribution**
(a per-cycle `FRMW`/`ARMW` on `0x0910` + `0x0900` system-time) and/or one extra
`0x0980`/`0x0981` write sequence, that may be a **small `SoemBackend` addition**
— far cheaper than a new backend. SOEM's `ecx_dcsync0` arms once and never
re-distributes; the fix could be a custom DC-arming path in `configure_dc_sync` +
a per-`exchange()` time-distribution datagram. **Estimate: 1–3 days IF the diff
is small.** Risk: SOEM may lack the datagram primitives, pushing to Option 2.

### Option 2 — add an IgH `EcatBackend` (the heavier, surer path)
IgH brings up this servo class reliably. Our `EcatBackend` abstraction exists
exactly so a second backend can slot in (`SoemBackend`/`SimBackend` already prove
the seam). Keep SOEM for sim/dev/CI and free-run/simple-DC buses; use IgH for
real CiA402 DC servos, selected by config.

## IgH `ecrt` → `EcatBackend` mapping

IgH's userspace API is `ecrt.h` (lib `libethercat`). Note: API specifics below
are from working knowledge of `ecrt.h`; verify against the installed headers.

| `EcatBackend` method      | IgH `ecrt` call(s)                                                                 | Notes / impedance |
|---------------------------|------------------------------------------------------------------------------------|-------------------|
| `open(ifname)`            | `ecrt_request_master(0)` → `ec_master_t*`                                           | **No `ifname`** — IgH selects by master index; the NIC is bound at the daemon (`/etc/ethercat.conf`), not by us. Enumerate via `ecrt_master_get_slave`. |
| `slave_info(slave)`       | `ecrt_master_get_slave(m, pos, &ec_slave_info_t)`                                   | vendor/product/name direct; image sizes known only after domain registration. |
| `sdo_write` / `sdo_read`  | `ecrt_master_sdo_download` / `ecrt_master_sdo_upload`                               | Blocking; valid pre-activate. Clean 1:1. |
| `map_process_data()`      | `ecrt_slave_config_pdos(sc, n, ec_sync_info_t[])` + `ecrt_slave_config_reg_pdo_entry(...)` + `ecrt_master_activate(m)` | **Biggest impedance.** IgH is **declarative**: the full PDO map is handed over as `ec_sync_info_t`/`ec_pdo_info_t`/`ec_pdo_entry_info_t`, NOT applied via our SDO-remap sub-protocol. `reg_pdo_entry` returns each entry's byte offset (replaces our field table). |
| `request_state(PreOp/SafeOp/Op)` | `ecrt_master_activate(m)` (one-shot → OP)                                    | **Impedance.** IgH has no granular per-state request; `activate()` drives the whole transition. We'd synthesize the state model (report via `ecrt_master_state` → `al_states`). |
| `configure_dc_sync(cycle, shift)` | `ecrt_slave_config_dc(sc, 0x0300, cycle, shift, 0, 0)`                      | **THE fix.** `assign_activate=0x0300` (SYNC0-only — exactly the SII value). Must be called **before** `activate()`, at config time — not at SAFE-OP as our interface assumes. |
| `dc_time()`               | `ecrt_master_reference_clock_time(m, &t)`                                           | For our phase-lock PI (still useful). |
| `exchange()`              | `ecrt_master_receive(m); ecrt_domain_process(d); ecrt_master_application_time(m, t); ecrt_master_sync_reference_clock(m); ecrt_master_sync_slave_clocks(m); ecrt_domain_queue(d); ecrt_master_send(m)` | **The win:** `application_time` + `sync_reference_clock` + `sync_slave_clocks` every cycle = the continuous time distribution this drive needs. WKC from `ecrt_domain_state` → `working_counter`. |
| `slave_io(slave)`         | `ecrt_domain_data(d)` + per-entry byte offsets from `reg_pdo_entry`                 | Offsets captured at config time. |
| `expected_wkc()`          | derived from domain (`ec_domain_state_t.wc_state` / `working_counter`)              | IgH reports `EC_WC_COMPLETE` vs a count. |
| `close()`                 | `ecrt_release_master(m)`                                                            | Clean. |

## Interface impact

The current `EcatBackend` is **SOEM-shaped**: incremental state transitions +
master-driven SDO remap (`Master::configure()` applies the PDO map via
`apply_pdo_map`'s SDO sub-protocol, then `map_process_data`, then steps
PreOp→SafeOp→Op). IgH is **declarative + one-shot-activate** and wants the PDO
map + DC params as data up front. So an IgH backend is **not a drop-in**; it
needs one of:

- **(pref) Pass the declarative config to the backend.** Give the backend the
  `SlaveConfig`s (PDO maps + DC params) at a configure-time hook so IgH can call
  `slave_config_pdos`/`slave_config_dc`/`activate` in one place. `SoemBackend`
  keeps doing its SDO remap internally (or `Master` keeps driving it for SOEM
  only). The `request_state`/`map_process_data`/`configure_dc_sync` split stays
  for SOEM; IgH folds them into `activate()`. This is a **focused refactor of the
  configure path**, not a rewrite — the cyclic hot path (`exchange`/`slave_io`/
  `dc_time`/`expected_wkc`) maps cleanly.
- (alt) Keep the interface; the IgH backend caches the map from the `sdo_write`
  remap calls + `configure_dc_sync` and defers all real work to the first
  `map_process_data`/state request. Hacky; fights both models.

## Scope estimate vs `SoemBackend`

`SoemBackend` is ~400 LOC. For Option 2:

- **IgH backend TU:** ~400–600 LOC (ecrt mapping; the cyclic path is simpler than
  SOEM, the declarative config setup is more upfront bookkeeping).
- **Interface + `Master::configure()` refactor:** ~1–2 days to route declarative
  config to the backend without regressing SOEM/Sim. Covered by existing
  `master_test`/`sim_backend_test` + a new IgH smoke path.
- **Build:** link `libethercat` (find_package / pkg-config), gated `ETHERCAT_BUILD_IGH=ON`
  so host CI (no IgH) stays green. Sim/SOEM paths unchanged.
- **⚠ Deployment — the real cost.** IgH is **not** pure userspace. It needs, on
  the **host**: the `ec_master` **kernel module** (must match the running kernel),
  the master **daemon** (`ethercatctl start`), `/dev/EtherCAT0`, and the NIC bound
  to IgH's native or generic driver. A Viam module shipped as a container/binary
  **cannot self-contain this** — the kernel module is host-kernel-specific. This
  is a deployment-architecture change: the end user must install + configure the
  IgH master on their machine. SOEM (raw socket + `CAP_NET_RAW`) ships in the
  container and runs anywhere. **This is the dominant trade-off, not the code.**

**Rough total (Option 2):** ~1–1.5 weeks engineering for a working IgH backend +
refactor, **plus** an unavoidable host-side IgH-master install requirement for
any deployment that drives this servo class. Document it in
`deployment-capabilities.md` as a distinct "DC servo (IgH host master required)"
tier vs the self-contained SOEM tier.

## Recommendation

1. **Probe Option 1 first** (capture-diff) — if the gap is just continuous DC
   time distribution, a small `SoemBackend` patch keeps the self-contained
   deployment and is far cheaper. One bench session + a Wireshark capture decides.
2. If SOEM genuinely can't arm the SYNC-out unit even with continuous time
   distribution, **commit to the IgH backend (Option 2)** behind the existing
   `EcatBackend` seam, with the configure-path refactor, and surface the
   host-IgH-master requirement explicitly in the deployment docs.
3. Keep SOEM as the default/sim/CI backend regardless — the abstraction earns its
   keep here.
