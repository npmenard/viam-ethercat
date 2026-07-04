#pragma once

#include <stdexcept>
#include <string>

namespace ethercat {

// Exceptions for the EtherCAT master library. Three types: the `Error` base, plus
// PdoMappingError and SdoError, which callers catch distinctly. Config, init,
// bring-up, and bus faults all throw the base `Error`; the message text, not the
// type, is the diagnostic.
//
// Every exception carries human-readable text describing what went wrong (slave id,
// object index/subindex, byte offset, expected vs actual state, ...). Callers catch
// PdoMappingError/SdoError when they care, or the `Error` base for anything from
// this library.
//
// These are thrown only on the non-RT path (configuration, init, SDO access, setup
// bounds violations). The RT loop catches at its boundary and latches a fault flag
// plus last-error string; an exception never crosses the RT/non-RT boundary (see
// pdo_cache).

// Base class for all errors raised by this library. Config validation, bring-up
// (NIC open / enumeration / AL-state), and cyclic bus faults all throw this
// directly; the message names the cause.
class Error : public std::runtime_error {
   public:
    explicit Error(const std::string& what) : std::runtime_error(what) {}
};

// A map-membership failure, in two cases:
//   - a mapping could not be applied to a slave (apply_pdo_map: an entry overflows
//     the SM, an SDO write to a mapping object 0x1C12/0x1C13/0x1600/0x1A00 was
//     rejected, or the requested map is invalid for the slave); or
//   - a runtime PDO access referenced an object not in the applied map
//     (resolve_rx / resolve_tx), which the operator fixes by adding it to the map.
// resolve_rx_optional/resolve_tx_optional catch this specifically to treat a
// not-in-map object as absent; a wrong-width access throws the base Error instead,
// since that is a malformed access rather than an absent one. A generic non-mapping
// SDO abort is SdoError.
class PdoMappingError : public Error {
   public:
    explicit PdoMappingError(const std::string& what) : Error(what) {}
};

// A generic CoE SDO transfer was aborted by the drive (a non-mapping object: mode
// 0x6060, a vendor/tuning write, a consumer-side vendor reset, ...). Carries the drive's
// CoE abort code in the message. Distinct from PdoMappingError (which is specific
// to the 0x1C1x/0x16xx/0x1Axx mapping writes). apply_pdo_map catches this to
// re-tag a mapping-object abort as a PdoMappingError.
class SdoError : public Error {
   public:
    explicit SdoError(const std::string& what) : Error(what) {}
};

}  // namespace ethercat
