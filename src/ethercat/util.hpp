#pragma once

// Small shared utilities (cold paths only -- error messages, last_error(); never
// the RT loop).
//
// hex(): one "0x"-prefixed, zero-padded, UPPERCASE hex formatter, sized to the
// argument type -- hex(u16) -> "0x1C12", hex(u32) -> "0x00000000". Templated on
// the unsigned width so a single definition serves every object index / AL code /
// abort code (#TODO-5). std::format is the idiomatic implementation -- requires
// libstdc++ 13 / the GCC-13 toolchain floor (#TODO-9).

#include <concepts>
#include <cstddef>
#include <format>
#include <string>

namespace ethercat {

template <std::unsigned_integral T>
std::string hex(T v) {
    return std::format("0x{:0{}X}", v, sizeof(T) * 2);
}

}  // namespace ethercat
