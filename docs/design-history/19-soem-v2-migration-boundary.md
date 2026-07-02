# Spec #19 — SOEM v2.0.0 migration boundary

**Status:** ready for cpp-expert (unblocks the mechanical migration). Architect-owned boundary spec.
**Goal:** upgrade SOEM v1.4.0 → 2.0.0, **contained entirely to `soem_backend.cpp`** (+ its private `Impl`). `backend.hpp` and `soem_backend.hpp` MUST NOT change. SimBackend, Master, ServoController, the module — untouched.
**Pairs with:** the librarian's v1.4.0→v2.0.0 API delta (the authoritative symbol mapping — this spec is the *boundary + change-map*, not the symbol list). **Scope-fences #20** (the DC-sequence rework) — see §5.

---

## 0. The containment thesis (confirmed by inspection)

The SOEM seam is already a clean pimpl, so v2 is contained **by construction**:
- **`backend.hpp`** — the `EcatBackend` interface is SOEM-type-free (`EcatState`, `SlaveInfo`, `SlaveIo`, `DcSyncStatus`, raw-byte spans). No `ec_*`/`ecx_*` type, no `<soem/...>` include. **→ zero changes.**
- **`soem_backend.hpp`** — pimpl: `struct Impl;` forward-declared + `std::unique_ptr<Impl>`. No SOEM header, no SOEM type. Method signatures are `EcatBackend`'s (unchanged). **→ zero changes.**
- **`soem_backend.cpp`** — the *only* `#include <soem/...>`, the only `ec_*`/`ecx_*` usage (all inside `Impl` + the method bodies). **→ all v2 changes land here.**

**The acceptance gate for "contained":** `git diff` touches **only `soem_backend.cpp`** (and `CMakeLists`/the soem pin). If the diff touches `backend.hpp` or `soem_backend.hpp`, a v2 concept leaked — stop and reshape it back behind the pimpl. (One *possible* exception flagged in §4: the `ecx_mbxhandler` RT-contract question — if it forces an interface change, escalate; it shouldn't.)

---

## 1. What must NOT change (the invariant)

- **No `ec_*`/`ecx_*`/`soem` token in any header.** `grep -rl 'soem/\|ec_\|ecx_' src/**/*.hpp` stays empty except none. (The Impl is the firewall.)
- **`EcatBackend`'s contract is unchanged**, including the RT-hot-path guarantee on the cyclic methods (`slave_io`/`exchange`/`expected_wkc`): **noexcept, no allocation, no blocking.** v2's `exchange()` must still honor this (see §4).
- **`DcSyncStatus` / the `configure_dc_*`/`arm_dc_sync`/`dc_sync_status` signatures stay put for #19.** Their *bodies* port mechanically; their *semantics* are #20's problem, not this migration's. Do NOT redesign them here.
- **SimBackend is the offline truth and never touches SOEM** — so the entire offline test suite (controller/module/host) must pass **unchanged** after the migration. Any offline-test change ⇒ something leaked through the seam.

---

## 2. What changes in `soem_backend.cpp` (the change-map)

Exact symbols per the librarian's delta; this is the map of *where* each lands. The big one is the context restructure.

1. **Header:** `#include <soem/ethercat.h>` → `#include <soem/soem.h>` (v2 umbrella).
2. **`ecx_contextt` restructure (`Impl`, lines ~87–142) — the largest change.** v1.4.0 wires the context by hand: separate `ec_slavet slavelist[]`, `ec_groupt grouplist[]`, `ecx_portt port`, the ESI/PDO/SM scratch arrays, then ~25 lines assigning `ctx.port = &port; ctx.slavelist = &slavelist[0]; …` in the ctor. v2.0.0 **embeds** these in `ecx_contextt` (the context owns its arrays; `ecx_init`/`ecx_config_init` populate them) — so most of the manual wiring (lines 117–141) **deletes**, and `Impl` shrinks to `ecx_contextt ctx{}` + the `iomap` + the backend's own bookkeeping (`expected_wkc`, `dc_cycle_ns`, `prev_sync0_evt`, etc.). Per-instance reentrancy (no globals) is preserved — that was the whole point of the pimpl and v2 keeps it.
3. **`ecx_config_map_group`** (line 209) — already group-based; confirm the v2 signature `(ctx, iomap, group)` + return (IOmap bytes) and the group index (0) are unchanged, adapt if the delta says otherwise.
4. **`ecx_mbxhandler` in the cyclic path — NEW (see §4 for the RT question).** v2 split mailbox servicing out; the cyclic loop must call it so CoE/EoE mailbox traffic is pumped. Placement + RT-safety is the one real design decision in this migration.
5. **group-WKC field names** (line 215, `(g.outputsWKC * 2) + g.inputsWKC`) — v2 may rename the `ec_groupt` WKC fields; the `expected_wkc` formula is unchanged, only the field names.
6. **`ec_slavet` field access** — `Obytes/Ibytes/outputs/inputs/configadr/hasdc/ALstatuscode/eep_man/eep_id/name/state` (used in `slave_info`, `slave_io`, `request_state`, the DC methods). Some may rename in v2; map per the delta.
7. **`ecx_*` free functions** — `ecx_init/config_init/config_map_group/configdc/dcsync0/send_processdata/receive_processdata/statecheck/writestate/readstate/SDOread/SDOwrite/FPRD/FPWR/close/poperror/iserror`. Context-first-arg is stable; confirm any signature/timeout-constant changes.
8. **constants** — `EC_STATE_*`, `EC_TIMEOUT*`, `EC_MAXSLAVE`, `EC_MAXGROUP`, `EC_MAX_MAPT`, `ec_ALstatuscode2string` — map per the delta.

---

## 3. No-leak verification checklist (how cpp-expert + reviewers confirm containment)

1. `git diff --stat` shows only `soem_backend.cpp` + `CMakeLists.txt` (soem pin). **backend.hpp / soem_backend.hpp untouched.**
2. No `<soem/...>` include outside `soem_backend.cpp`.
3. The full **offline** suite (SimBackend-backed controller/module/host tests) passes **unchanged** — no test edits required (if a test needs editing, the seam leaked).
4. `soem_backend.cpp` **compiles + links** against soem 2.0.0 in the build-module-OFF config (CI can't *run* it — CAP_NET_RAW/NIC — only build+link).

---

## 4. `ecx_mbxhandler` — RESOLVED: NOT needed at all (librarian, source-verified)

The §4 worry is **fully cleared** by the librarian's v2 delta (read of `~/SOEM` b410bf6): **`exchange()` stays PDO-only — do NOT call `ecx_mbxhandler` anywhere.** Reasoning:
- Our **runtime is PDO-only**: statusword/fault-code come via the TxPDO-mapped `0x6041`/`0x603F`, not async mailbox. So no cyclic mailbox servicing is needed to reach or hold OP.
- **Blocking SDO is single-threaded at `configure()`** (PDO-remap + the vendor fault-reset), where `ecx_SDOwrite`/`ecx_SDOread` handle their own mailbox internally and the `mbxpool` is already inited by `ecx_init`. The single port owner there is the lifecycle thread (pre-RT-spawn).
- Therefore **neither `ecx_mbxhandler` nor `ecx_slavembxcyclic` is required** for #19/#20/#21. `exchange()` = `ecx_send_processdata` + `ecx_receive_processdata` only, exactly as today. **No `backend.hpp` change** — the one thing that could have breached containment does not.
- (FYI for #22 only: if a *runtime, in-OP* async mailbox SDO is ever needed — the steady-state vendor fault-reset — `ecx_mbxhandler` IS RT-compatible in the PD envelope per librarian: no-alloc, bounded by its `limit` arg, timed datagram I/O + one uncontended leaf mutex — same class as the PD exchange. It would live **inside the RT loop** next to send/recv, called only from the RT thread, never concurrently. That's #22 territory, post-first-light. Don't build it now.)

The §0 single-port-owner invariant (now #20 §0) still holds and is still *why* we beat ec_sample — it just turns out we need zero extra cyclic mailbox machinery to honor it. Everything else is mechanical symbol-swapping behind the pimpl.

---

## 5. Scope fence with #20 — do NOT rework the DC sequence here

`#20` (the ec_sample DC fold) **replaces** the DC path: stock `ecx_dcsync0` instead of the hand-rolled ESC-register arm (`arm_dc_sync`, lines 314–383), and an **Er74.1/PD-health** OP-gate instead of the `0x0984/0x098E`-pulsing `dc_sync_status` gate (CLAUDE.md: those regs are PDI-consumed and read 0 even when SYNC0 fires — the current gate is on the wrong signal). 

**Therefore, for #19, port the DC methods only enough to COMPILE + LINK** — do not carefully preserve their v1.4.0 logic, because #20 rewrites them within days. Mechanical symbol-swap (or even a minimal stub that compiles) is the right investment level for `arm_dc_sync`/`configure_dc_sync`/`dc_sync_status` here; their real form is #20's. The non-DC methods (`open`/`slave_info`/`sdo_*`/`map_process_data`/`request_state`/`set_state`/`slave_state`/`slave_io`/`exchange`/`expected_wkc`/`close`) are the ones to port properly in #19, since they're stable across #20.

(#20 will also remove the manual `0x1C32:01` force if any survives, drop the hand-rolled SyncDelay workaround in favor of stock `ecx_dcsync0`, and re-point the OP-gate — all inside `soem_backend.cpp` + `Master::configure`, still no `backend.hpp` change. The `DcSyncStatus` struct *may* get simpler fields in #20; if so that IS a backend.hpp change, but it's #20's deliberate design evolution, not a v2 leak — I'll spec it there.)

---

## 6. Sequence

cpp-expert: pin soem 2.0.0 in CMake → port §2 (non-DC methods properly, DC methods minimally per §5) → confirm §3 checklist (esp. the empty backend.hpp/soem_backend.hpp diff) → resolve §4 (mbxhandler placement) → build+link green + offline suite unchanged. Then #20 reworks the DC path on the now-v2 base. Ping me on §4 if mbxhandler resists internal placement; otherwise this is a clean mechanical migration behind a seam that was built for exactly this.
