#pragma once

#include <stdexcept>
#include <string>

namespace ethercat {

// Exceptions for the EtherCAT master library. THREE types only: the `Error`
// base and the two that a caller actually catches DISTINCTLY (PdoMappingError,
// SdoError). Config/init/bring-up/bus faults all throw the base `Error` -- no
// call site ever discriminated them by type, and every message already carries
// a self-identifying scope (component::method or a plain description of the
// config/bring-up/bus failure), so the text, not the type, is the diagnostic.
//
// Every exception carries human-readable text describing exactly what went
// wrong (slave id, object index/subindex, byte offset, expected vs actual
// state, ...). Callers catch PdoMappingError/SdoError when they care, or the
// `Error` base to handle anything originating from this library.
//
// RT-safety note: these are constructed/thrown only on the NON-RT path
// (configuration, init, SDO access, bounds violations in setup). The RT loop
// catches at its boundary and latches a fault flag + last-error string; it
// never lets an exception cross the RT/non-RT boundary (see pdo_cache).

// Base class for all errors raised by this library. Config validation, bring-up
// (NIC open / enumeration / AL-state), and cyclic bus faults all throw this
// directly; the message names the cause.
class Error : public std::runtime_error {
   public:
    explicit Error(const std::string& what) : std::runtime_error(what) {}
};

// A MAP-MEMBERSHIP failure ("the map can't satisfy you"), spanning two cases:
//   - a mapping could not be APPLIED to a slave (apply_pdo_map: an entry overflows
//     the SM, an SDO write to a mapping object 0x1C12/0x1C13/0x1600/0x1A00 was
//     rejected, or the requested map is invalid for the slave); OR
//   - a RUNTIME PDO access referenced an object NOT in the applied map
//     (resolve_rx / resolve_tx) -- distinct operator fix: "add it to the map".
// resolve_rx_optional/resolve_tx_optional catch THIS specifically to treat a
// not-in-map object as absent; a wrong-width access throws the base Error instead
// (a malformed access, deliberately NOT swallowed as "absent"). A generic
// non-mapping SDO abort is SdoError.
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
