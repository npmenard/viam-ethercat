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
// SDO write to 0x1C12/0x1C13/0x1600/0x1A00 was rejected, or the requested map
// is otherwise invalid for the slave.
class PdoMappingError : public Error {
   public:
    explicit PdoMappingError(const std::string& what) : Error(what) {}
};

// An attempt to read/write past the end of a PDO buffer (cursor overrun, short
// frame). Thrown by PdoReader/PdoWriter; message names the offset and size.
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
