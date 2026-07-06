#pragma once

// Typed CoE field descriptors for the PDO access API.
//
// A Field<Index, Sub, T> bundles a CoE object index, subindex, and its C++ wire
// type at compile time, so a PDO access site cannot use the wrong type: the cia402::
// aliases below pin the correct T to each canonical object, making
// resolve_tx<cia402::Statusword>() statically a std::uint16_t field and
// resolve_rx<cia402::TargetPosition>() an std::int32_t one. There is no runtime
// width validation; the alias is the per-field type contract.
//
// Field carries no storage and no logic. Master::resolve_rx/resolve_tx (master.hpp)
// resolve a Field's index:sub to a byte offset, and the load_le/store_le free
// functions read and write sizeof(T) little-endian there.

#include <cstdint>

namespace ethercat {

template <std::uint16_t Index, std::uint8_t Sub, typename T>
struct Field {
    static constexpr std::uint16_t index = Index;
    static constexpr std::uint8_t sub = Sub;
    using type = T;
};

// Canonical CiA402 fields, each with its correct wire type. Use these at call sites
// (cia402::Statusword, cia402::TargetPosition, ...) rather than a raw Field<>. They
// are nested in `cia402` so the alias cia402::ControlWord (a Field) stays distinct
// from ethercat::ControlWord (the controlword-encoder struct in cia402.hpp); call
// sites qualify with cia402::.
namespace cia402 {

using ControlWord = Field<0x6040, 0, std::uint16_t>;
using Statusword = Field<0x6041, 0, std::uint16_t>;
using ModeOfOperation = Field<0x6060, 0, std::int8_t>;  // RxPDO-mapped so the runtime mode-switch can drive it
using ModeDisplay = Field<0x6061, 0, std::int8_t>;
using FaultCode = Field<0x603F, 0, std::uint16_t>;
using TargetPosition = Field<0x607A, 0, std::int32_t>;
using PositionActual = Field<0x6064, 0, std::int32_t>;
using VelocityActual = Field<0x606C, 0, std::int32_t>;
using ProfileVelocity = Field<0x6081, 0, std::uint32_t>;
using TargetVelocity = Field<0x60FF, 0, std::int32_t>;

}  // namespace cia402

}  // namespace ethercat
