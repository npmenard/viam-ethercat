#pragma once

#include <stdexcept>
#include <string>

namespace ethercat {

// Exception hierarchy for the EtherCAT master library.
//
// Every exception carries human-readable text describing exactly what went
// wrong (slave id, object index/subindex, byte offset, expected vs actual
// state, ...). Callers can catch the specific type they care about, or the
// `Error` base to handle anything originating from this library.
//
// RT-safety note: these are constructed/thrown only on the NON-RT path
// (configuration, init, SDO access, bounds violations in setup). The RT loop
// catches at its boundary and latches a fault flag + last-error string; it
// never lets an exception cross the RT/non-RT boundary (see pdo_cache).

// Base class for all errors raised by this library.
class Error : public std::runtime_error {
   public:
    explicit Error(const std::string& what) : std::runtime_error(what) {}
};

// Invalid/inconsistent configuration supplied by the caller (bad ifname,
// contradictory PDO map, out-of-range loop rate, ...). Detected before any I/O.
class ConfigError : public Error {
   public:
    explicit ConfigError(const std::string& what) : Error(what) {}
};

// Failure bringing the bus up: NIC open (needs CAP_NET_RAW), slave enumeration,
// or a slave failing to reach a requested EtherCAT state within a timeout.
class InitError : public Error {
   public:
    explicit InitError(const std::string& what) : Error(what) {}
};

// A MAP-MEMBERSHIP failure ("the map can't satisfy you"), spanning two cases:
//   - a mapping could not be APPLIED to a slave (apply_pdo_map: an entry overflows
//     the SM, an SDO write to a mapping object 0x1C12/0x1C13/0x1600/0x1A00 was
//     rejected, or the requested map is invalid for the slave); OR
//   - a RUNTIME PDO access referenced an object NOT in the applied map
//     (resolve_rx / resolve_tx) -- distinct operator fix: "add it to the map".
// A generic non-mapping SDO abort is SdoError; a MALFORMED access (wrong width /
// past frame) is PdoAccessError; a bad slave id is ConfigError -- so the error type
// matches the operator's mental model on the bench.
class PdoMappingError : public Error {
   public:
    explicit PdoMappingError(const std::string& what) : Error(what) {}
};

// A generic CoE SDO transfer was aborted by the drive (a non-mapping object: mode
// 0x6060, a vendor/tuning write, a consumer-side vendor reset, ...). Carries the drive's
// CoE abort code in the message. Distinct from PdoMappingError (which is specific
// to the 0x1C1x/0x16xx/0x1Axx mapping writes) and from BusError (WKC/transport).
class SdoError : public Error {
   public:
    explicit SdoError(const std::string& what) : Error(what) {}
};

// A MALFORMED PDO field access ("your access is malformed"): the Field's typed
// width disagrees with the mapping. Thrown by the resolve_rx/resolve_tx width
// check; the message names the cause + offset/size. (Object-not-in-map is the
// separate PdoMappingError -- a map-membership concern, not a malformed access.)
class PdoAccessError : public Error {
   public:
    explicit PdoAccessError(const std::string& what) : Error(what) {}
};

// A runtime bus fault during cyclic exchange: bad working counter, slave
// dropped out of OP, lost link. Raised/flagged from the process path.
class BusError : public Error {
   public:
    explicit BusError(const std::string& what) : Error(what) {}
};

}  // namespace ethercat
