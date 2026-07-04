#pragma once

#include <bit>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <type_traits>

#include "ethercat/errors.hpp"

namespace ethercat {

// Scalars that may be read from / written to a PDO buffer: trivially-copyable,
// fixed-width (1/2/4/8 bytes) integers and floating-point. `bool` is excluded
// on purpose -- its object representation is unspecified, so it has no defined
// wire encoding. This covers (u)int8/16/32/64 and float/double.
template <class T>
concept PdoScalar =
    std::is_trivially_copyable_v<T> && !std::is_same_v<std::remove_cv_t<T>, bool> &&
    (std::is_integral_v<T> || std::is_floating_point_v<T>) && (sizeof(T) == 1 || sizeof(T) == 2 || sizeof(T) == 4 || sizeof(T) == 8);

namespace detail {

// Unsigned integer type with exactly N bytes -- the bit container we shuffle
// scalars through so the byte order is explicit (and host-endianness-agnostic).
template <std::size_t N>
struct uint_of;
template <>
struct uint_of<1> {
    using type = std::uint8_t;
};
template <>
struct uint_of<2> {
    using type = std::uint16_t;
};
template <>
struct uint_of<4> {
    using type = std::uint32_t;
};
template <>
struct uint_of<8> {
    using type = std::uint64_t;
};

template <std::size_t N>
using uint_of_t = typename uint_of<N>::type;

}  // namespace detail

// ---------------------------------------------------------------------------
// RT hot-path little-endian helpers (free functions, noexcept, NO bounds check)
// ---------------------------------------------------------------------------
// The RT loop calls these on pre-resolved {offset,width} field spans (configure()
// builds that flat table), so they must not throw and must not bounds-check.
// Precondition: field.size() == sizeof(T).

template <PdoScalar T>
T load_le(std::span<const std::byte> field) noexcept {
    using U = detail::uint_of_t<sizeof(T)>;
    U raw = 0;
    for (std::size_t i = 0; i < sizeof(T); ++i) {
        const auto octet = std::to_integer<std::uint8_t>(field[i]);
        raw = static_cast<U>(raw | static_cast<U>(static_cast<U>(octet) << (8 * i)));
    }
    return std::bit_cast<T>(raw);
}

template <PdoScalar T>
void store_le(std::span<std::byte> field, T value) noexcept {
    using U = detail::uint_of_t<sizeof(T)>;
    const U raw = std::bit_cast<U>(value);
    for (std::size_t i = 0; i < sizeof(T); ++i) {
        const U octet = static_cast<U>((raw >> (8 * i)) & static_cast<U>(0xFF));
        field[i] = static_cast<std::byte>(static_cast<std::uint8_t>(octet));
    }
}

}  // namespace ethercat
