#pragma once

// One shared hex formatter (#TODO-5), replacing three hand-rolled copies
// (pdo_mapping's hex16, soem_backend's hex32, servo_controller's to_hex16).
// std::format is the idiomatic implementation -- requires libstdc++ 13 / the
// GCC-13 toolchain floor (#TODO-9), which is why this could not land earlier.
//
// Both forms emit an "0x"-prefixed, zero-padded, UPPERCASE hex string sized to
// the type: hex(u16) -> "0x1C12", hex(u32) -> "0x00000000". Cold paths only
// (error messages, last_error()); never the RT loop.

#include <cstdint>
#include <format>
#include <string>

namespace ethercat {

inline std::string hex(std::uint16_t v) {
    return std::format("0x{:04X}", v);
}

inline std::string hex(std::uint32_t v) {
    return std::format("0x{:08X}", v);
}

}  // namespace ethercat
