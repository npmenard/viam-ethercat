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

// A Sync-Manager PDO assignment: which PDO(s) (e.g. 0x1600) are assigned to the
// SM (assign_index 0x1C12 for RxPDO, 0x1C13 for TxPDO), and the entry list of
// each. `entries` is keyed by PDO index.
struct PdoMap {
    std::uint16_t assign_index = 0;  // 0x1C12 (Rx) or 0x1C13 (Tx)
    std::vector<std::uint16_t> pdo_indices;
    std::map<std::uint16_t, std::vector<PdoEntry>> entries;

    // Total mapped size in bytes (sum of all entry bit lengths / 8). Throws
    // PdoMappingError if the bit total is not byte-aligned.
    std::size_t byte_size() const;
};

// A one-shot SDO write the driver wants applied at configure() time, in PRE-OP,
// BEFORE the PDO remap. This is how drive-specific TUNING params stay config DATA
// (not hardcoded): e.g. the A6's C13 sync-jitter-tolerance group, which must be
// written while the drive is quiescent (several C13 params are "at-stop only").
// `data` is the raw little-endian object value; its length MUST match the object's
// CoE data type or the drive aborts (the abort code surfaces via PdoMappingError).
struct SdoWrite {
    std::uint16_t index = 0;
    std::uint8_t subindex = 0;
    std::vector<std::byte> data;
    // Best-effort: if the drive rejects this write (read-only object, length/value
    // abort), LOG and CONTINUE instead of failing configure(). For diagnostic /
    // optional tuning writes where one rejected sub-index must not block the rest
    // (e.g. the A6's 0x1C33:01 input-SM sync type may be read-only while 0x1C32:01
    // output-SM is the one that matters). Default false = mandatory (throws).
    bool optional = false;
};

// Per-slave configuration (config DATA; the A6 specifics live here, never in
// generic code).
struct SlaveConfig {
    std::uint16_t slave_id = 1;  // 1-based ring position
    PdoMap rxpdo;                // assign_index 0x1C12
    PdoMap txpdo;                // assign_index 0x1C13
    Cia402Mode default_mode = Cia402Mode::ProfilePosition;
    // Driver-supplied SDO writes applied in PRE-OP before the remap (drive tuning,
    // e.g. A6 C13 sync tolerance). Empty for slaves that need none.
    std::vector<SdoWrite> preop_sdo_writes;
    // SDO writes applied in PRE-OP AFTER the PDO remap/assignment. Required for the
    // SM-synchronization objects 0x1C32:01 / 0x1C33:01 (sync type): the ETG startup
    // order is map -> assign (0x1C12/0x1C13) -> SM-sync, because several drives
    // RE-DEFAULT 0x1C32 when the PDO assignment changes -- so a sync-type write done
    // before the assignment gets clobbered back to its default. Empty for most slaves.
    std::vector<SdoWrite> postremap_sdo_writes;
    // OPTIONAL drive-specific fault-reset SDO, cleared once in configure() AFTER SAFE-OP
    // (single port owner, before the RT thread spawns -- so it's a plain blocking SDO, no
    // queue). The A6's fault-reset is a VENDOR SDO write `1` to 0x2031:01, NOT CiA402
    // controlword bit7 (CLAUDE.md). Config DATA: present ⇒ that SDO is the bring-up clear;
    // absent ⇒ no vendor reset (a generic CiA402 drive uses the controlword bit7 path the
    // controller already drives). The steady-state operator reset is a separate concern
    // (#22). `data` is the raw little-endian value (A6: a single 0x01 byte).
    std::optional<SdoWrite> fault_reset;
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
void apply_pdo_map(EcatBackend& backend, std::uint16_t slave, const PdoMap& map);

}  // namespace ethercat
