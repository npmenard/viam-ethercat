#pragma once

// Typed CoE field descriptors for the PDO access API (#30).
//
// A `Field<Index, Sub, T>` is a COMPILE-TIME bundle of a CoE object index +
// subindex + its C++ wire type. It is the way to never get the type wrong at a
// PDO access site: the `cia402::` aliases below pin the correct T to each
// canonical object, so `rpdo.get<cia402::Statusword>()` is statically a
// std::uint16_t and `tpdo.put<cia402::TargetPosition>(v)` only accepts an
// std::int32_t. (The locked design drops runtime width-validation; the alias IS
// the per-field type contract -- a raw Field<..., wrongT> is the caller's risk.)
//
// Field carries no storage and no logic -- Rpdo/Tpdo (master.hpp) resolve a
// Field's index:sub to a byte offset in the process image and read/write
// sizeof(T) little-endian there.

#include <cstdint>

namespace ethercat {

template <std::uint16_t Index, std::uint8_t Sub, typename T>
struct Field {
    static constexpr std::uint16_t index = Index;
    static constexpr std::uint8_t sub = Sub;
    using type = T;
};

// Canonical CiA402 fields, each with the CORRECT wire type. Use these at call
// sites (cia402::Statusword, cia402::TargetPosition, ...) rather than spelling a
// raw Field<>, so the type can't be gotten wrong. These are nested in `cia402`
// so the alias `cia402::ControlWord` (a Field) is distinct from the existing
// `ethercat::ControlWord` (the controlword-encoder struct in cia402.hpp) -- call
// sites qualify with `cia402::`.
namespace cia402 {

using ControlWord = Field<0x6040, 0, std::uint16_t>;
using Statusword = Field<0x6041, 0, std::uint16_t>;
using ModeDisplay = Field<0x6061, 0, std::int8_t>;
using FaultCode = Field<0x603F, 0, std::uint16_t>;
using TargetPosition = Field<0x607A, 0, std::int32_t>;
using PositionActual = Field<0x6064, 0, std::int32_t>;
using VelocityActual = Field<0x606C, 0, std::int32_t>;
using ProfileVelocity = Field<0x6081, 0, std::uint32_t>;
using TargetVelocity = Field<0x60FF, 0, std::int32_t>;

}  // namespace cia402

}  // namespace ethercat
