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
    // SDO writes applied in PRE-OP AFTER configure_dc_sync (SYNC0 configured + the ESC
    // cycle register 0x09A0 set + SYNC0 pulsing), immediately before the SAFE-OP
    // request. The SM sync-type switch to DC (0x1C32:01 = 2) belongs here for drives
    // that SNAPSHOT the read-only SM cycle time 0x1C32:02 from the live 0x09A0 at the
    // moment they enter DC mode -- switching earlier (while 0x09A0 is still 0) latches
    // 0x1C32:02 = 0 -> AL 0x0030 "Invalid DC SYNC config". Empty for most slaves.
    std::vector<SdoWrite> postdc_sdo_writes;
};

// Master-level configuration.
struct MasterConfig {
    std::string ifname;
    std::uint32_t target_loop_rate_hz = 1000;
    std::vector<SlaveConfig> slaves;
    bool use_distributed_clocks = false;
    // When DC is on: number of PACED, PHASE-LOCKING process-data exchanges (at the
    // SYNC0 cycle) to run after enabling SYNC0 and BEFORE requesting OP -- the warmup
    // runs the DC phase-lock PI until the phase is in-band (or this budget is spent),
    // so the bus enters OP already phase-aligned (WKC 3/3 from cycle 0) instead of
    // an unpaced burst (-> A6 refuses OP / WKC drops). 0 = skip the warmup (sim).
    std::uint32_t dc_lock_cycles = 0;
    // Post-OP grace: suppress the consecutive-WKC-error fault latch for this many
    // cycles after reaching OP, so any residual DC phase transient settles without
    // tripping a BusError (the phase PI needs ~hundreds of cycles to fully lock; the
    // latch fires in ~5). 0 = latch immediately. Bench-tunable.
    std::uint32_t dc_settle_cycles = 0;
    // Phase-lock TARGET: lock (dc_time mod cycle) to this offset (ns) instead of 0.
    // The default -1 = auto = cycle/2 (mid-cycle) -- locking at the SYNC0 EDGE (0)
    // leaves no margin, so normal +/-jitter pushes a frame past the pulse -> stale
    // latch -> intermittent WKC drop. Mid-cycle keeps the send far from both edges.
    std::int32_t dc_sync_shift_ns = -1;
    // SYNC0 pulse CyclShift (ns) passed to ecx_dcsync0: the SYNC0 edge fires this
    // long after the DC base time. With the send phase (dc_sync_shift_ns) this tunes
    // WHERE in the cycle the drive latches our output relative to its SYNC0 -- the A6
    // wants a FRESH frame just before SYNC0, not a stale mid-cycle one. Bench-swept.
    std::int32_t dc_sync0_shift_ns = 0;
    // SYNC0 first-edge START DELAY (ns) -- how far in the future the SYNC0 arm schedules
    // the FIRST pulse, replacing SOEM ec_dcsync0's hardcoded 100 ms SyncDelay. A drive
    // whose sync watchdog is SHORTER than 100 ms (the A6 ~50 ms) trips "no sync" in
    // SAFE-OP before SOEM's first edge is even due, and the fault deactivates SYNC0
    // before it fires. ~15 ms puts the first edge inside the watchdog window (master is
    // already phase-locked, so the start stays safely in the future). Bench-tunable.
    std::int64_t dc_sync_start_delay_ns = 15'000'000;
    // Arm SYNC0 in PRE-OP (after the phase-lock), BEFORE requesting SAFE-OP, so SYNC0 is
    // already FIRING (start_delay ~15 ms) when the drive validates its DC config at the
    // PRE-OP->SAFE-OP transition. Required by a drive that (a) rejects SM-sync at OP with
    // AL 0x0027 "freerun not supported" -- so DC sync-type (0x1C32:01=2) is mandatory --
    // AND (b) AL 0x0030-rejects DC config at SAFE-OP unless SYNC0 is already pulsing. The
    // synthesis: force DC type (postremap) + arm-firing-SYNC0 in PRE-OP. false = arm
    // post-SAFE-OP (caller-driven / self-contained paths).
    bool dc_arm_in_preop = false;
    // After the post-DC SDO writes (SM sync-type -> DC SYNC0), pump this many paced PD
    // cycles BEFORE requesting SAFE-OP so the drive APPLIES the DC config -- copies the
    // live ESC SYNC0 cycle (0x09A0) into the read-only CoE 0x1C32:02. Without it the
    // drive validates an incomplete DC config (0x1C32:02 still 0) at the PS transition
    // -> AL 0x0030. 0 = no settle.
    std::uint32_t dc_postwrite_settle_cycles = 0;
    // Optional CoE object (index:sub) polled each cycle during that settle: break the
    // settle early once it reads NON-ZERO (the drive has applied the DC config, e.g.
    // 0x1C32:02 went 0 -> SYNC0 cycle). index 0 = no poll, just the fixed window.
    std::uint16_t dc_settle_poll_index = 0;
    std::uint8_t dc_settle_poll_sub = 0;
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
