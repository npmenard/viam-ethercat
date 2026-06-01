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

// Cursor over a read-only PDO byte span. All multi-byte scalars are decoded as
// explicit little-endian (EtherCAT wire order), independent of host endianness.
// Any access that would read past the end throws PdoAccessError naming the
// offset and size; the cursor is not advanced on failure.
class PdoReader {
   public:
    explicit PdoReader(std::span<const std::byte> data) noexcept : data_(data) {}

    // Read a scalar at the cursor and advance past it.
    template <PdoScalar T>
    T get() {
        const T value = peek<T>();
        pos_ += sizeof(T);
        return value;
    }

    // Read a scalar at the cursor WITHOUT advancing.
    template <PdoScalar T>
    T peek() const {
        require(sizeof(T));
        using U = detail::uint_of_t<sizeof(T)>;
        U raw = 0;
        for (std::size_t i = 0; i < sizeof(T); ++i) {
            const auto octet = std::to_integer<std::uint8_t>(data_[pos_ + i]);
            raw = static_cast<U>(raw | static_cast<U>(static_cast<U>(octet) << (8 * i)));
        }
        return std::bit_cast<T>(raw);
    }

    // Advance the cursor by n bytes (must stay within bounds).
    void skip(std::size_t n) {
        require(n);
        pos_ += n;
    }

    // Move the cursor to an absolute byte offset (0..size()).
    void seek(std::size_t pos) {
        if (pos > size()) {
            throw PdoAccessError("pdo reader seek to offset " + std::to_string(pos) + " exceeds buffer size " + std::to_string(size()));
        }
        pos_ = pos;
    }

    std::size_t remaining() const noexcept {
        return size() - pos_;
    }
    std::size_t size() const noexcept {
        return data_.size();
    }
    std::size_t tell() const noexcept {
        return pos_;
    }

   private:
    void require(std::size_t n) const {
        if (n > remaining()) {
            throw PdoAccessError("pdo reader access of " + std::to_string(n) + " byte(s) at offset " + std::to_string(pos_) +
                                 " exceeds buffer size " + std::to_string(size()));
        }
    }

    std::span<const std::byte> data_;
    std::size_t pos_ = 0;
};

// Cursor over a writable PDO byte span. Scalars are encoded as explicit
// little-endian. Any access that would write past the end throws
// PdoAccessError; the cursor is not advanced on failure.
class PdoWriter {
   public:
    explicit PdoWriter(std::span<std::byte> data) noexcept : data_(data) {}

    // Write a scalar at the cursor and advance past it.
    template <PdoScalar T>
    void write(T value) {
        require(sizeof(T));
        using U = detail::uint_of_t<sizeof(T)>;
        const U raw = std::bit_cast<U>(value);
        for (std::size_t i = 0; i < sizeof(T); ++i) {
            const U octet = static_cast<U>((raw >> (8 * i)) & static_cast<U>(0xFF));
            data_[pos_ + i] = static_cast<std::byte>(static_cast<std::uint8_t>(octet));
        }
        pos_ += sizeof(T);
    }

    // Advance the cursor by n bytes (must stay within bounds).
    void skip(std::size_t n) {
        require(n);
        pos_ += n;
    }

    // Move the cursor to an absolute byte offset (0..size()).
    void seek(std::size_t pos) {
        if (pos > size()) {
            throw PdoAccessError("pdo writer seek to offset " + std::to_string(pos) + " exceeds buffer size " + std::to_string(size()));
        }
        pos_ = pos;
    }

    std::size_t remaining() const noexcept {
        return size() - pos_;
    }
    std::size_t size() const noexcept {
        return data_.size();
    }
    std::size_t tell() const noexcept {
        return pos_;
    }

   private:
    void require(std::size_t n) const {
        if (n > remaining()) {
            throw PdoAccessError("pdo writer access of " + std::to_string(n) + " byte(s) at offset " + std::to_string(pos_) +
                                 " exceeds buffer size " + std::to_string(size()));
        }
    }

    std::span<std::byte> data_;
    std::size_t pos_ = 0;
};

}  // namespace ethercat
