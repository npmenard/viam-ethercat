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

// A PDO mapping could not be applied to a slave: an entry overflows the SM, an
// SDO write to a MAPPING object (0x1C12/0x1C13/0x1600/0x1A00) was rejected, or
// the requested map is otherwise invalid for the slave. Reserved for the mapping
// sub-protocol (apply_pdo_map) -- a generic non-mapping SDO abort is an SdoError,
// not this (so the error type matches the operator's mental model on the bench).
class PdoMappingError : public Error {
   public:
    explicit PdoMappingError(const std::string& what) : Error(what) {}
};

// A generic CoE SDO transfer was aborted by the drive (a non-mapping object: mode
// 0x6060, a vendor/tuning write, the fault-reset 0x2031, ...). Carries the drive's
// CoE abort code in the message. Distinct from PdoMappingError (which is specific
// to the 0x1C1x/0x16xx/0x1Axx mapping writes) and from BusError (WKC/transport).
class SdoError : public Error {
   public:
    explicit SdoError(const std::string& what) : Error(what) {}
};

// An invalid PDO field access: the object isn't in the slave's PDO map, the
// Field's typed width disagrees with the mapping, or the access runs past the
// buffer (cursor overrun / short frame). Thrown by PdoReader/PdoWriter and the
// Rpdo/Tpdo resolve path; the message names the cause + offset/size. (Distinct
// from PdoMappingError, which is RESERVED for the configure-time apply_pdo_map
// sub-protocol -- this is the RUNTIME access tier.)
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
