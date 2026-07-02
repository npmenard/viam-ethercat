# Spec #30 — PDO access API (typed Field + copy-based RT boundary)

**Status:** design+plan (team-lead/user decisions LOCKED — this spec elaborates, doesn't re-litigate). Architect-owned; cpp-expert impl + DA review per phase.
**Goal:** replace ad-hoc `input_image()`/`outputs()` + `le16`/`read_tx`/`write_rx` poking with a typed, frame-consistent PDO API: an immutable read **snapshot** (`Rpdo`) and a seeded write **builder** (`Tpdo`), over a typed `Field<Index,Sub,T>`, with a dual (ergonomic-throwing / RT-noexcept) boundary.

---

## 0. Hard constraints (gate every phase)
- **The validated bring-up + CSP sine MUST keep working on HW after each phase** (a6_validate reaches OP + the sine runs). The 531f075 readstate + b9f7b5f teardown fixes are load-bearing — untouched.
- **RT hot path: no throw, no alloc, no map-walk per cycle.** Offsets resolved ONCE at configure (cached handle). The throwing `get`/`put` cursor is NOT on the 1 kHz path (see §4 — the dual API).
- Clear-text exceptions only. Offline SimBackend tests + HW re-validate per phase; both compilers `-Werror` green.

---

## 1. `Field<Index, Sub, T>` — the typed CoE field descriptor
Compile-time bundle of CoE object index + subindex + the C++ wire type:
```cpp
template <std::uint16_t Index, std::uint8_t Sub, typename T>
struct Field { static constexpr std::uint16_t index = Index; static constexpr std::uint8_t sub = Sub; using type = T; };
```
**`cia402::` aliases (the canonical, type-safe access — the way to never get T wrong):**
```cpp
namespace cia402 {
  using ControlWord     = Field<0x6040, 0, std::uint16_t>;
  using Statusword      = Field<0x6041, 0, std::uint16_t>;
  using TargetPosition  = Field<0x607A, 0, std::int32_t>;
  using PositionActual  = Field<0x6064, 0, std::int32_t>;
  using ModeDisplay     = Field<0x6061, 0, std::int8_t>;
  using VelocityActual  = Field<0x606C, 0, std::int32_t>;
  using FaultCode       = Field<0x603F, 0, std::uint16_t>;
  using ProfileVelocity = Field<0x6081, 0, std::uint32_t>;
  using TargetVelocity  = Field<0x60FF, 0, std::int32_t>;
}
```
Call sites use the aliases: `rpdo.get<cia402::Statusword>()` → `std::uint16_t`. **The alias encodes the correct T** — that is where per-field type-correctness comes from (the locked decision drops runtime width-validation; the aliases are the contract). A raw `get<Field<…, wrongT>>` is the caller's-contract risk the design accepts.

---

## 2. `Rpdo` — immutable, frame-consistent read snapshot
```cpp
class Rpdo {
 public:
  template <class F> typename F::type get() const;  // resolve F's offset in the field table; read sizeof(F::type) LE
 private:
  // a COPY of one feedback frame's bytes + a (non-owning) ref to the resolved offset-only field table.
};
Rpdo Master::read_rpdo(std::uint16_t slave) const;  // one seqlock read of the PdoCache RxSnapshot -> COPY
```
- **Frame-consistent:** every `get<>` on one `Rpdo` comes from the SAME frame (the snapshot copy). No tearing across fields. To see a newer frame, call `read_rpdo` again. NOT a live view.
- Backed by the existing **PdoCache RxSnapshot seqlock** (the copy is the seqlock read). This is the path note 11 wants exercised by servo_controller's read side.
- **THROW TAXONOMY (LOCKED by team-lead — the SPLIT; final, no further revision. as-built dee03b5):** clear-text, never `BusError` (a field-access miss is a programming error, not a WKC/cable fault).
  - **`PdoMappingError` = "the map can't satisfy you"** — the PDO map is wrong/incomplete for the request. Spans BOTH apply-time (apply_pdo_map mapping-object writes) AND runtime **not-in-map** (`Rpdo::get`/`Tpdo::put`/`resolve` of an object absent from the configured map). Operator fix: **add the object to the map config.**
  - **`PdoAccessError` = "your access is malformed"** — **width-mismatch (§5)** (the Field's typed width disagrees with the mapped width) OR **past-frame** (the access runs past the buffer). Operator fix: **fix your Field type / access.**
  - **Why the split (team-lead ruling + DA's catch-distinction):** a caller can `catch (PdoMappingError)` to mean "my MAP is incomplete" (a config-level problem) distinctly from `catch (PdoAccessError)` "my ACCESS is wrong" (a code-level problem) — the operator fix genuinely differs, so the two types earn their keep. Both are logic/configure-time throws OFF the RT path, so there is NO behavioral cost either way — it's purely which is more debuggable, and the split wins. (This supersedes my transient "uniform" position; team-lead locked the split — do NOT re-open.)
  - **Doc reconciliation (errors.hpp + master.hpp inline, dee03b5):** `PdoMappingError`'s comment documents BOTH apply-time AND runtime not-in-map; `PdoAccessError`'s comment is the malformed-access tier (width / past-buffer) and notes "object-not-in-map is PdoMappingError." All Rpdo/Tpdo/resolve inline doc-comments must say not-in-map→PdoMappingError, width/past→PdoAccessError (no stale "uniform" language).
  - **Note (process — owned):** this tier oscillated split→uniform→split across reviewers/commits; team-lead's ruling locks it. The lesson: anchor an exception-type choice on the *operator-fix distinction* (does the catcher act differently?) ONCE, write it authoritative, and don't re-derive. Net rework was ~zero but the churn was real and avoidable.

## 3. `Tpdo` — seeded write builder, submit-to-transmit
```cpp
class Tpdo {
 public:
  template <class F> void put(typename F::type v);  // resolve offset; write sizeof(F::type) LE into the staged copy
  void submit() noexcept;                            // hand the staged frame to TxStaging -> sent next cycle
 private:
  // a COPY of the CURRENT command image (the SEED) + the field table + slave id + a submitted flag.
};
Tpdo Master::make_tpdo(std::uint16_t slave);  // seed from the current outputs (so unchanged fields carry over)
```
- **Seeded from the current/LIVE output image** → CiA402 fields you don't `put` carry over unchanged (mutate-deltas; e.g. you put TargetPosition, the ControlWord you set last cycle persists). 
- **⚠ SEED MUST BE THE LIVE OUTPUTS IMAGE, NOT ZERO (DA, ties to #24).** Seed-from-zero re-introduces the #24 CSP enable-jump footgun: `put<ControlWord>` but not target → the un-put target goes out as **0** = commanding target=0 = the exact jump we fixed. `make_tpdo` MUST copy the last-commanded output bytes as the seed; a freshly-constructed Tpdo carrying zeros that gets submitted is a bug. Test 2 (§8) asserts the seed equals the live image, not zero.
- **`submit()` → transmitted next cycle** (via the existing TxStaging, rewritable-until-sent). **An UNSUBMITTED `Tpdo` never touches the bus** (drop it = no-op; no dirty-on-construct). 
- `put<F>` throws per the §5 split: `PdoMappingError` on not-in-map, `PdoAccessError` on width-mismatch / end-of-buffer.
- **Tpdo USAGE CONTRACT (DA, doc-comment on `make_tpdo`):** (1) **don't MIX** a direct `outputs()` write with a `Tpdo`/`submit()` on the same slave/cycle — `make_tpdo` snapshots the seed at call time, so `make_tpdo → direct-write → submit` silently overwrites the direct write with the pre-write seed (the make-early case bites); submit() wins at drain. (2) **don't HOLD a Tpdo across cycles** — the seed goes stale and a late submit lands a stale frame. Tpdo is a per-cycle transient: make → put → submit, then drop. (Bench-only single-consumer API → the contract is a doc-comment, not runtime machinery — DA confirmed defensive guards would be over-engineering.)

---

## 4. The DUAL API — how it maps to RT vs non-RT (the central design point; clarification flagged to team-lead)
The hard constraint "no throw / no map-walk per cycle" means **`Rpdo`/`Tpdo` (throwing, per-call resolve, a copy) are NOT the 1 kHz hot-path form.** The dual API:

| Form | Mechanism | Used by |
|---|---|---|
| **Ergonomic (throwing)** | `read_rpdo`/`make_tpdo` → `get<F>`/`put<F>`/`submit` (per-call resolve, copy, bounds-throw) | non-RT/setup; **a6_validate** (bench tool — RT-paced but copy-tolerant); **servo_controller's NON-RT accessors** (`position_revs`/`feedback`/`last_error` read `read_rpdo`); the **#21 thin program** |
| **RT hot-path (noexcept)** | offsets resolved ONCE at configure (cached `FieldLocation`), then `load_le<T>`/`store_le<T>` on the live image — no copy, no throw, no map-walk | **servo_controller's `run_rt_loop`** per-cycle FSM read + command write |

**So "migrate servo_controller to read_rpdo/Tpdo" means:** its **non-RT read side** (the accessors) uses `read_rpdo` (exercising the PdoCache snapshot — note 11); its **RT loop keeps cached-offset `load_le`/`store_le`** (the noexcept form), just unified onto the new `Field<>` + offset-only `FieldLocation` (retiring the ad-hoc `le16`/`read_tx`/`write_rx`). The Field<> types + the offset-only table are SHARED by both forms — the hot path caches the offset at configure; the ergonomic form resolves per-call. **The copy is what lets both coexist** (the user holds an immutable snapshot/Tpdo; the RT path never allocates/throws). 

**The split is STRUCTURAL, not by convention (DA).** The throwing `get`/`put` exist ONLY on `Rpdo`/`Tpdo` (the copy types). The RT hot path NEVER constructs an `Rpdo`/`Tpdo` — it holds cached `FieldLocation`s (resolved at configure) and calls the noexcept free `load_le<T>(image, loc)`/`store_le<T>(image, loc, v)` directly on the live image. So the throwing surface is physically absent from the 1 kHz path — it's a different type, not "the same object you're trusted not to misuse in the loop." Someone CAN'T accidentally call a throwing `put` per cycle because the RT loop has no `Tpdo` to call it on.

→ **team-lead: confirm this RT/non-RT split is the intent.** It's forced by the hard constraint (the RT loop can't use throwing `put` per cycle); I'm reading "migrate to read_rpdo/Tpdo" as "the non-RT/bench/ergonomic surfaces use Rpdo/Tpdo; the production RT loop uses the cached-offset noexcept form of the same Field<> table." If you intended the RT loop itself to use Tpdo per cycle, that violates no-throw-on-1kHz — flag and we reconcile.

---

## 5. Offset-only `FieldLocation`
```cpp
struct FieldLocation { std::size_t byte_offset; };  // offset-only — the END STATE
```
- **Physical byte_width drop is a P2c step, NOT P2a (as-built 133404d keeps byte_width inert).** The new API already behaves offset-only (width from `T`), but physically removing the `byte_width` member couples into two no-migration surfaces — servo_controller's `byte_width==0 => unmapped` optional-field sentinel (P2c) and a6_validate's load/store width (P2b). So byte_width rides on `FieldLocation` until those call sites migrate; the member is removed when the last reader does. Approved — keeps P2a truly no-migration.
- `get<F>/put<F>/load_le<T>/store_le<T>` read/write `sizeof(T)` at `byte_offset`. The width comes from `T` (the Field's type / the alias), not the table.
- **KEEP `PdoEntry.bit_length`** — still needed to COMPUTE offsets during map resolution + to represent padding gaps (the `0x0000` filler entries). It's a map-description field, not an access-time field.
- Resolution (`resolve<F>() → FieldLocation`, templated so it knows `sizeof(F::type)`): throws `PdoMappingError` (clear text) if F's index:sub isn't in the configured map (§5 split — not-in-map is map-membership); `PdoAccessError` on width-mismatch.
- **Configure-time WIDTH ASSERTION (DA refinement — PROVISIONAL, pending USER confirm; build it in P2a, do NOT gate on it, trivially removable if the user holds the bounds-only line):** `resolve<F>` asserts the Field's `sizeof(F::type)` matches the mapped entry's width, throwing clear-text on mismatch. **Width source while byte_width is retained (P2a/P2b):** check `sizeof(F::type) == loc.byte_width` (equivalent, no need to thread bit_length into FieldLocation yet). **At the P2c offset-only drop:** switch the check to `sizeof(F::type) * 8 == bit_length` of the mapped entry. This recovers the WIDTH half of the dropped CoeType tag *for free* (we keep bit_length anyway) and catches the silent-wrong-read gap DA flagged — e.g. a `Field<0x607A,0,int16_t>` alias against a 32-bit-mapped object would otherwise read 2 of 4 bytes *in bounds, silently wrong, no throw*. The check is at RESOLVE (configure-time for the RT cached offsets — ONE-time, off the hot path; per-call for the throwing get/put, which is off-RT anyway). **The runtime `FieldLocation` STAYS offset-only** (width is verified-then-discarded, not stored) — consistent with the locked drop-width decision; only the signedness/float half of the old tag stays dropped (that one is genuinely us-checking-us). This nudges the "validation = bounds-throw ONLY" locked line, hence the team-lead opt-out FYI.
- The RT path resolves all its fields ONCE at configure into cached `FieldLocation`s (no per-cycle resolve, no per-cycle width check).
- **P2c byte_width-drop mechanics (the public struct goes offset-only):**
  - **Width-assert moves to the INTERNAL table.** The public `FieldLocation` returned to callers is `{byte_offset}` only; the internal resolve table retains the mapped width (`bit_length`) so `resolve<F>` can still assert `sizeof(F::type)*8 == bit_length` at configure, then hand back an offset-only `FieldLocation`. Width is verified-then-discarded from the public type.
  - **`FieldLocation::mapped()` replaces the `byte_width==0 => unmapped` sentinel** (servo_controller's optional-field sites — today `byte_width != 0`, with unmapped = `FieldLocation{}`). Contract: `loc.mapped()` returns whether the field is present in the configured map (for OPTIONAL TxPDO fields like velocity feedback that may legitimately be absent).
    - **⚠ HARD CORRECTNESS CONSTRAINT (DA, production safety path): `mapped()` MUST be offset-INDEPENDENT — never `byte_offset != 0`.** `byte_offset == 0` is a LEGITIMATE mapped location (controlword/statusword normally sit at offset 0). If `mapped()` regressed to "offset != 0" (or the unmapped sentinel were a zero-init `FieldLocation` reading as offset 0), an UNMAPPED FaultCode would resolve to offset 0 = the statusword bytes → a **phantom or missed drive fault on the production safety path.** So: use a presence bool, or a sentinel offset in the `SIZE_MAX` class — NOT offset 0. Resolved/cached ONCE at configure; the RT loop checks the cached `loc.mapped()`, never a per-cycle resolve.
    - Migration gate: (1) `mapped()` is offset-independent; (2) EVERY `byte_width != 0` site → `mapped()`; (3) EVERY `: FieldLocation{}` unmapped assignment yields `!mapped()`.

---

## 6. Migration (the call-site sweep — review notes 1,7,8,9,11)
**Scope guard (team-lead, LOCKED): the seeded-`Tpdo`+`submit` COPY API is for the BENCH/direct-PDO path ONLY (a6_validate, #21). The servo MODULE's command path is UNTOUCHED** — it stays CommandQueue + CiA402 FSM (GoTo/SetRPM → typed commands → the RT loop applies them via the FSM). Do NOT replace the module's per-cycle output writes with a `Tpdo::submit()`.
- **a6_validate** (bench, direct-PDO) → `read_rpdo`/`make_tpdo`+`put`+`submit`; drop `le16`/`read_tx`/`write_rx`. (Per-cycle copy is fine.) The CSP sine: seed Tpdo, `put<cia402::TargetPosition>(sine)`, `submit()`; read `get<cia402::Statusword>()`/`PositionActual`/`FaultCode`.
- **servo_controller** (corrected after cpp-expert's pre-read — my original "(ii) accessors → read_rpdo" was a SPEC ERROR vs the module's load-bearing concurrency contract):
  - **(i) — corrected (DA precision): the module's RT spans are ALREADY hardcoded literals (`subspan(loc.byte_offset, 4)`, `subspan(loc.byte_offset, 2)`), NOT byte_width-derived.** The ONLY byte_width use in the module is the optional-field sentinel (`byte_width != 0`, see below). So **the module's byte_width-drop = PURELY the mapped() sentinel swap — ZERO change to the RT spans.** Leave the literal-width `store_le`/`load_le` calls BYTE-IDENTICAL; do NOT "helpfully" re-derive them from `sizeof(T)` (gratuitous risk on the noexcept production path). The RT loop already uses cached offsets (resolved once in `resolve_fields`) + is `noexcept` — UNTOUCHED. The `byte_width`→`sizeof(T)`/`bit_length` width-source change lives in the LIBRARY only (resolve_field's width-assert, §5), NOT the module. The FSM/CommandQueue command path is untouched.
  - **(ii) — REVISED to Option A: the production NON-RT accessors STAY master_-free / atomic-backed / `noexcept` / derived — UNCHANGED.** They must NOT route through `read_rpdo`, because: (a) `read_rpdo` derefs `master_`, which the accessors deliberately never touch (UAF protection vs `reconfigure()`'s exclusive-lock + RT-join + `master_.reset()`); (b) `read_rpdo`+`get<>` THROWS (`resolve_tx`), and the accessors are `noexcept` → throw = `std::terminate`; (c) the accessors return DERIVED state (zero-offset position, powered/moving/faulted gates, composed multi-tier `last_error`) that the RAW wire fields from `read_rpdo` cannot supply. The atomics' per-field freshness is sufficient (the motor API needs no cross-field frame-consistency).
  - **Note 11 ("the snapshot seqlock read path is exercised, not dead code") is satisfied WITHOUT touching the accessors:** P2b already routes a6_validate's per-cycle reads through `read_rpdo` (HW-validated), so the path is live + proven; ADD a TSan concurrent-read test (call `read_rpdo` while the RT loop runs) to prove the cross-thread seqlock read under the sanitizer. Do NOT rewire production accessors to satisfy note 11.
  - No `Tpdo`/`submit` anywhere in the module.
- **retire** `le16`/`read_tx`/`write_rx` once both call sites are migrated.

---

## 7. DEFERRED (YAGNI — locked, do NOT build now)
- **`CoeType` semantic tag** — width is already OD-checked by the drive's mapping-accept at configure; the tag only self-asserts signedness/float (us-against-us), and it forecloses bit-packed/sub-byte PDOs. Dropped.
- **`make_pdo_map<Fields...>` static variant** — fights "A6 specifics = config data, never code"; the template error-spew is poor UX (a reason it's out).
- **The real future correctness feature is SDO-info OD-readback at configure** (read the drive's object dictionary + verify the map against it) — separate, more valuable, not now. Note it; don't build it.

---

## 8. Phasing + testing (this spec = P2; see the dispatch reply for the P1/P2/P3 order)
- **P2a — the library API (no migration):** `Field<>` + `cia402::` aliases + offset-only `FieldLocation` (drop byte_width, keep PdoEntry.bit_length) + `Rpdo`/`read_rpdo` + `Tpdo`/`make_tpdo`/`submit` + resolve-throws. **Tests (offline, SimBackend):** (1) snapshot frame-consistency (two fields from one `read_rpdo` are from one frame even if the sim advances between gets); (2) seeded-Tpdo carry-over AND **seed==live-image-not-zero** (put one field; the others retain the live seed, NOT zero — the #24 guard); (3) unsubmitted-Tpdo never transmits (assert the bus image unchanged, no dirty-on-construct); (4) submit → next-cycle transmit; (5) resolve-throw (`PdoMappingError`) on not-in-map; (6) bounds-throw (`PdoAccessError`) on offset+sizeof past the frame; (7) round-trip via the `cia402::` aliases; (8) **width-mismatch resolve-throw** (a `Field<…,int16_t>` against a 32-bit-mapped entry throws clear-text at resolve, not a silent 2-of-4-byte read). Both compilers -Werror.
- **P2b — migrate a6_validate:** swap to `read_rpdo`/`Tpdo`; drop le16/read_tx/write_rx. **HW re-validate: OP + the CSP sine still run** (this is the gate). Offline: a6_validate's logic unaffected.
- **P2c carry-in (from P2b review):** hoist a6_validate's Phase-1 bring-up `resolve_tx<cia402::FaultCode>` out of the per-cycle loop (resolve once before the loop; not-mapped→treat-as-no-fault). Cosmetic/zero-behavior (per-call resolve is the API's normal pattern + bring-up is bounded, not hot-path) — deferred to P2c because `FieldLocation::mapped()` (introduced here) is a cleaner vehicle than a `std::optional` band-aid, and to avoid shifting the frozen+reviewed P2b SHA mid-flight.
- **P2c — migrate servo_controller (scope corrected via DA pre-read — see §6):** module footprint = (a) `FieldLocation::mapped()` offset-INDEPENDENT + migrate the optional-field sentinel sites (the safety item); (b) accessors UNCHANGED (Option A — byte-identical, NOT rewired through read_rpdo). Library footprint = byte_width physical drop + width-assert→internal bit_length (P2a.8 must STILL trip, non-vacuous). The module's RT spans are literal-width → UNTOUCHED. **Tests:** existing controller_offline_test green; a test that the RT cycle does no resolve/throw/alloc; P2a.8 still trips after the bit_length switch.
  - **Note-11 TSan concurrent-read test — must be provably NON-VACUOUS (DA):** (1) a genuine second thread calls `read_rpdo` in a tight loop WHILE a publish writer (RT loop or process()) runs concurrently, many iterations — a real interleave, not read-after-park; (2) **vacuous-guard: it must be demonstrable that the test FLAGS a deliberately-broken seqlock** (e.g. swap the `atomic_ref` publish for a plain store → TSan fires). A test that can't catch a broken seqlock doesn't prove the clean one. Acceptance is "shown to race + catch a break," not just "labeled TSan-clean."
  - **Coupling to keep in view (DA):** "accessors unchanged" = byte-identical diff, but the faulted/last_error accessor surfaces RT-published state, and the RT fault decision reads FaultCode via the new `mapped()` sentinel — so a bad `mapped()` (the offset-0 hazard) produces a phantom/missed fault that flows OUT through the untouched accessor. The accessor body being unchanged does NOT immunize its output; gate-1's offset-0 check is what protects it. **HW re-validate.**
  - **HW GATE RESULT (PASSED, tip 7311900 on SOEM v2/b410bf6):** --csp-probe 3/3 OP; energized sine OPERATION ENABLED, followErr peak ~130, 0x603F=0, WKC 3/3, badWKC=0, mode=8, 0 LRW gaps, enable-jump held. **Offset-0 sentinel proven safe ON HW** — controlword@Rx-0 write enabled the drive AND fault-code@Tx-0 read returned 0x0, so `mapped()`-is-a-bool + the byte_width drop did NOT break offset-0 in either direction.
  - **Coverage honesty (DA):** HW proved the **`mapped()==true`** branch (the A6 maps controlword/fault-code/etc → present=true, reads/writes land at offset 0). The **`!mapped()` absent-field fallback** (velocity-estimate / fault-tier omission) was NOT HW-exercised (this A6 config maps the fields, so that branch didn't fire on silicon) — it is OFFLINE-proven (the `present=false`→`!mapped()` direction is pinned by the inverse-trap check + the default-`FieldLocation{}` unit test) and behavior-preserving (same fallback as pre-P2c `byte_width==0`). So: present-field branch HW-proven, absent-field branch offline-proven — both bool directions covered; don't later assume the fallback was silicon-tested.
- **Per-phase gate:** offline suite green + HW (OP + sine) + both -Werror. No phase lands without the HW re-validation.

DA review focus: the snapshot frame-consistency (seqlock copy), the seeded-Tpdo semantics (carry-over + unsubmitted-never-sends), the RT-path stays noexcept/no-throw/no-map-walk (the §4 split), and the resolve/bounds throw paths are clear-text + can't fire on the 1 kHz path.