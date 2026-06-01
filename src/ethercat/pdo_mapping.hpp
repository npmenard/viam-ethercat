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

// Per-slave configuration (config DATA; the A6 specifics live here, never in
// generic code).
struct SlaveConfig {
    std::uint16_t slave_id = 1;  // 1-based ring position
    PdoMap rxpdo;                // assign_index 0x1C12
    PdoMap txpdo;                // assign_index 0x1C13
    Cia402Mode default_mode = Cia402Mode::ProfilePosition;
};

// Master-level configuration.
struct MasterConfig {
    std::string ifname;
    std::uint32_t target_loop_rate_hz = 1000;
    std::vector<SlaveConfig> slaves;
    bool use_distributed_clocks = false;
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
