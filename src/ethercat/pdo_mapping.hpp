#pragma once

// PDO mapping config data + the SDO sub-protocol that applies it to a slave.
//
// The driver supplies the map (the library never parses manufacturer defaults).
// CiA402 / A6 detail: the configurable PDO map is writable ONLY in PRE-OP and is
// NOT stored in EEPROM, so Master::configure() re-applies it on every power-on
// via apply_pdo_map() -- this must not be skipped.

#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include "ethercat/backend.hpp"
#include "ethercat/cia402.hpp"

namespace ethercat {

// One entry of a PDO map: an object (index:subindex) of `bit_length` bits.
// A padding/gap entry uses index 0x0000 (advances the offset, no named object).
struct PdoEntry {
    std::uint16_t index = 0;
    std::uint8_t subindex = 0;
    std::uint8_t bit_length = 0;
};

// PDO direction = which SyncManager carries it. Fixed by the EtherCAT/CiA402
// standard: outputs (master->slave, RxPDO) go through SM2, inputs (slave->master,
// TxPDO) through SM3 -- so the SM PDO-assignment objects are fixed too.
enum class PdoDirection : std::uint8_t { Rx, Tx };

// The SM PDO-assignment object index for a direction (#TODO-8): RxPDO -> SM2 0x1C12,
// TxPDO -> SM3 0x1C13. Universal for CiA402 servos -- the library derives it; the
// user does not supply it.
constexpr std::uint16_t sm_assign_index(PdoDirection dir) noexcept {
    return dir == PdoDirection::Rx ? 0x1C12 : 0x1C13;
}

// A Sync-Manager PDO assignment: which PDO(s) (e.g. 0x1600) are assigned to the SM,
// and the entry list of each (keyed by PDO index). The assign-index is DERIVED from
// the map's direction (#TODO-8): the user names only the PDO(s) + entries. An escape
// hatch (assign_index_override) covers exotic non-standard SM layouts; no CiA402
// servo should need it.
struct PdoMap {
    // 0 = derive from direction (the universal case). Set non-zero ONLY for a
    // non-standard SM layout; then THIS index is used verbatim, ignoring direction.
    std::uint16_t assign_index_override = 0;
    std::vector<std::uint16_t> pdo_indices;
    std::map<std::uint16_t, std::vector<PdoEntry>> entries;

    // Effective SM assign-index: the override if set, else derived from direction.
    std::uint16_t assign_index(PdoDirection dir) const noexcept {
        return assign_index_override != 0 ? assign_index_override : sm_assign_index(dir);
    }

    // Total mapped size in bytes (sum of all entry bit lengths / 8). Throws
    // PdoMappingError if the bit total is not byte-aligned.
    std::size_t byte_size() const;
};

// A raw SDO write descriptor: {object index:subindex, little-endian value bytes}.
// `data`'s length MUST match the object's CoE data type or the drive aborts.
// CONSUMER-issued (#TODO-2): the Master no longer runs lists of these at configure()
// time -- setup-SDO POLICY belongs to the consumer, which issues its writes via
// Master::sdo_write() while it is the single port owner (post-configure, pre-RT).
// The surviving use is a data carrier for the consumer's vendor fault-reset (#39:
// ServoConfig::vendor_fault_reset, A6 0x2031:01). Best-effort vs mandatory is the
// CONSUMER's call at the call site (e.g. run_vendor_fault_reset try/catches), not a
// flag here -- the old `optional` field served only the deleted apply_sdo_writes.
struct SdoWrite {
    std::uint16_t index = 0;
    std::uint8_t subindex = 0;
    std::vector<std::byte> data;
};

// Per-slave configuration (config DATA; the A6 specifics live here, never in
// generic code).
struct SlaveConfig {
    std::uint16_t slave_id = 1;  // 1-based ring position
    PdoMap rxpdo;                // outputs -> SM2 0x1C12 (assign-index derived, #TODO-8)
    PdoMap txpdo;                // inputs  -> SM3 0x1C13 (assign-index derived, #TODO-8)
    Cia402Mode default_mode = Cia402Mode::ProfilePosition;
    // NOTE (#TODO-2): the generic preop_sdo_writes / postremap_sdo_writes lists are
    // GONE -- "run these extra SDOs for me at configure()" was the same orchestration
    // anti-pattern #39 evicted for the vendor fault-reset. Setup-SDO POLICY is the
    // consumer's: it issues its own writes via Master::sdo_write() while still the
    // single port owner (post-configure, pre-RT-spawn -- see the servo module's
    // vendor_fault_reset / a6_validate's --reset-fault). The STRUCTURAL remap SDOs
    // (0x1C12/0x1C13 assign + 0x1600/0x1A00 entries) STAY in configure() -- they're
    // intrinsic to the init->map sequence, not consumer policy. (No A6 setup write
    // needs to run before the remap, audited at #TODO-2, so no pre-remap hook exists;
    // a drive that needed one would get a narrow named hook, not a generic list.)
    // OPTIONAL SYNC0 cycle granularity this slave accepts, in ns (#44). Some drives only
    // accept SYNC0 cycles that are an integer multiple of a base tick -- the A6 requires a
    // 250 us multiple and otherwise faults AT OP ENTRY (Er74.0 "cycle error"), a cryptic
    // failure far from its cause. Declare it here (config DATA -- the 250'000 lives in the
    // drive's config, never in library code) and the Master ctor validates the configured
    // loop rate against it UP FRONT with clear text + nearest valid rates. 0 = no
    // constraint declared (no check). Only meaningful with use_distributed_clocks.
    std::uint32_t sync_cycle_granularity_ns = 0;
};

// Master-level configuration.
struct MasterConfig {
    std::string ifname;
    std::uint32_t target_loop_rate_hz = 1000;
    std::vector<SlaveConfig> slaves;
    bool use_distributed_clocks = false;
    // DC bring-up (the ec_sample fold, #20): SYNC0 is armed in PRE-OP inside configure()
    // (stock ecx_dcsync0, before config_map_group -- the drive self-selects DC sync-type
    // from the armed SYNC0). The RT loop then pumps phase-locked PD a bounded SETTLE and
    // requests OP ONCE.
    //   SETTLE: cycles of phase-locked PD to run BEFORE requesting OP, so the master's
    //   send cadence is disciplined onto SYNC0 first (ec_sample effectively settles its RT
    //   PD ~400 ms before OP). We do NOT gate this on Er74.1 -- 0x603F=0x8700 in SAFE-OP is
    //   the NORMAL pre-sync state and clears AT OP, so the gate is a fixed settle, not a
    //   no-Er74.1 wait. Only applied when DC is on (non-DC needs no settle). 0 ⇒ 1.
    std::uint32_t dc_op_gate_cycles = 400;
    // Post-OP grace: suppress the consecutive-WKC-error fault latch for this many
    // cycles after reaching OP, so any residual DC phase transient settles without
    // tripping a BusError (the phase PI needs ~hundreds of cycles to fully lock; the
    // latch fires in ~5). 0 = latch immediately. Bench-tunable.
    std::uint32_t dc_settle_cycles = 0;
    // SYNC0 pulse CyclShift (ns) passed to ecx_dcsync0: the SYNC0 edge fires this long
    // after the DC base time. The single DC phase knob (#20 consolidated the old
    // dc_sync_shift_ns/dc_sync0_shift_ns pair): it sets BOTH the ecx_dcsync0 CyclShift
    // and the master's phase-lock target, so the master's send and the drive's SYNC0
    // hold a fixed relationship. The phase-lock TARGET is derived as cycle/2 from the
    // SYNC0 edge (mid-cycle margin -- locking on the edge leaves no room for jitter).
    // Bench-swept at first light. 0 = SYNC0 on the DC base.
    std::int32_t dc_sync0_shift_ns = 0;
    // Latch a BusError only after this many CONSECUTIVE short/abnormal WKC
    // cycles (a single transient bad cycle should not hard-fault). Reset on any
    // good cycle.
    std::uint32_t max_consecutive_wkc_errors = 5;
    // --- AWAIT_OP bounds (bringup_step's OP-await phase; #42 -- previously hardcoded) ---
    // Defaults carry the ec_sample/A6 bring-up rationale: the A6's SAFE-OP->OP is SLOW
    // (wire-measured >11 s, up to ~24 s) and ec_sample reaches OP by WAITING IT OUT with
    // PD flowing + periodic SAFE-OP recovery nudges. Aborting early was the bug that
    // failed bring-up ~20x too early. These are give-up/confirm bounds with early-exit:
    // a conformant drive confirms OP in ~op_hold_confirm_cycles, so fast drives are
    // unaffected; tune them for OTHER slow drives or non-default loop rates.
    //
    // Declare Operational only after this many CONSECUTIVE (full-WKC && !sync-faulted)
    // cycles -- filters a transient good cycle. Cycle-semantic (consecutive clean
    // exchanges), so a count like dc_op_gate_cycles. 0 => 1.
    std::uint32_t op_hold_confirm_cycles = 5;
    // While awaiting OP, run the backend's reack_op() (ec_sample's SAFE-OP recovery:
    // ACK SAFE_OP+ERROR / re-request OP; self-gating no-op once in OP) every this many
    // cycles. 0 => 1.
    std::uint32_t op_nudge_interval_cycles = 10;
    // Give up (Aborted) after this much WALL TIME awaiting OP. Converted to a cycle
    // bound against target_loop_rate_hz in the Master ctor, so the give-up patience is
    // RATE-INDEPENDENT (the old 30'000-cycle constant silently meant 30 s only at
    // 1 kHz -- 2 minutes at 250 Hz). Floor of 1 cycle.
    std::uint32_t op_await_timeout_ms = 30'000;
};

// Apply a PDO map to a slave via the CoE SDO sub-protocol. The slave MUST be in
// PRE-OP. Ordering is load-bearing (a non-zero count rejects entry writes):
//   (a) zero the SM assignment count   (assign_index:00 := 0)
//   (b) for each PDO: zero its entry count (pdo:00 := 0)
//   (c) for each PDO: write each entry (pdo:NN := (index<<16)|(sub<<8)|bitlen)
//   (d) for each PDO: set its entry count (pdo:00 := N)
//   (e) assign the PDO(s) to the SM (assign_index:01.. := pdo) and set the
//       assignment count (assign_index:00 := M)
// Throws PdoMappingError (clear text) if the map references a PDO with no entry
// list or has too many entries/PDOs for the 1-byte counts.
void apply_pdo_map(EcatBackend& backend, std::uint16_t slave, const PdoMap& map, PdoDirection dir);

}  // namespace ethercat
