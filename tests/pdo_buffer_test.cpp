#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

#include "ethercat/errors.hpp"
#include "ethercat/pdo_buffer.hpp"
#include "test_harness.hpp"

using ethercat::PdoAccessError;
using ethercat::PdoReader;
using ethercat::PdoWriter;

namespace {

// Helper: a fixed byte buffer we can write into and read back out of.
template <std::size_t N>
using Buf = std::array<std::byte, N>;

}  // namespace

TEST("write/read round-trips every supported scalar width") {
    Buf<40> buf{};
    PdoWriter w{buf};
    w.write<std::uint8_t>(0x12);
    w.write<std::int8_t>(-7);
    w.write<std::uint16_t>(0xABCD);
    w.write<std::int16_t>(-12345);
    w.write<std::uint32_t>(0xDEADBEEF);
    w.write<std::int32_t>(-2000000000);
    w.write<std::uint64_t>(0x1122334455667788ULL);
    w.write<float>(3.5F);
    w.write<double>(-1234.5);

    PdoReader r{buf};
    CHECK_EQ(r.get<std::uint8_t>(), std::uint8_t{0x12});
    CHECK_EQ(r.get<std::int8_t>(), std::int8_t{-7});
    CHECK_EQ(r.get<std::uint16_t>(), std::uint16_t{0xABCD});
    CHECK_EQ(r.get<std::int16_t>(), std::int16_t{-12345});
    CHECK_EQ(r.get<std::uint32_t>(), std::uint32_t{0xDEADBEEF});
    CHECK_EQ(r.get<std::int32_t>(), std::int32_t{-2000000000});
    CHECK_EQ(r.get<std::uint64_t>(), std::uint64_t{0x1122334455667788ULL});
    CHECK_EQ(r.get<float>(), 3.5F);
    CHECK_EQ(r.get<double>(), -1234.5);
}

TEST("multi-byte values are encoded little-endian on the wire") {
    Buf<4> buf{};
    PdoWriter w{buf};
    w.write<std::uint32_t>(0x01020304);
    // Least-significant byte first.
    CHECK_EQ(std::to_integer<int>(buf[0]), 0x04);
    CHECK_EQ(std::to_integer<int>(buf[1]), 0x03);
    CHECK_EQ(std::to_integer<int>(buf[2]), 0x02);
    CHECK_EQ(std::to_integer<int>(buf[3]), 0x01);
}

TEST("reader decodes a known little-endian byte pattern") {
    const std::array<std::byte, 2> raw{std::byte{0xCD}, std::byte{0xAB}};
    PdoReader r{raw};
    CHECK_EQ(r.get<std::uint16_t>(), std::uint16_t{0xABCD});
}

TEST("cursor advances by the scalar width and remaining/tell track it") {
    Buf<8> buf{};
    PdoWriter w{buf};
    CHECK_EQ(w.size(), std::size_t{8});
    CHECK_EQ(w.tell(), std::size_t{0});
    CHECK_EQ(w.remaining(), std::size_t{8});
    w.write<std::uint16_t>(1);
    CHECK_EQ(w.tell(), std::size_t{2});
    CHECK_EQ(w.remaining(), std::size_t{6});
    w.write<std::uint32_t>(1);
    CHECK_EQ(w.tell(), std::size_t{6});
    CHECK_EQ(w.remaining(), std::size_t{2});
}

TEST("peek does not advance the cursor") {
    const std::array<std::byte, 4> raw{std::byte{0x04}, std::byte{0x03}, std::byte{0x02}, std::byte{0x01}};
    PdoReader r{raw};
    CHECK_EQ(r.peek<std::uint32_t>(), std::uint32_t{0x01020304});
    CHECK_EQ(r.tell(), std::size_t{0});
    CHECK_EQ(r.peek<std::uint16_t>(), std::uint16_t{0x0304});
    CHECK_EQ(r.tell(), std::size_t{0});
    // get() at the same spot still sees the same bytes.
    CHECK_EQ(r.get<std::uint32_t>(), std::uint32_t{0x01020304});
    CHECK_EQ(r.tell(), std::size_t{4});
}

TEST("skip advances without reading; seek repositions absolutely") {
    const std::array<std::byte, 6> raw{
        std::byte{0x00}, std::byte{0x00}, std::byte{0xEF}, std::byte{0xBE}, std::byte{0xAD}, std::byte{0xDE}};
    PdoReader r{raw};
    r.skip(2);
    CHECK_EQ(r.tell(), std::size_t{2});
    CHECK_EQ(r.get<std::uint32_t>(), std::uint32_t{0xDEADBEEF});

    r.seek(2);
    CHECK_EQ(r.tell(), std::size_t{2});
    CHECK_EQ(r.peek<std::uint8_t>(), std::uint8_t{0xEF});

    r.seek(r.size());  // seeking to exactly size() is allowed (empty remainder)
    CHECK_EQ(r.remaining(), std::size_t{0});
}

TEST("reading past the end throws PdoAccessError and leaves the cursor put") {
    Buf<3> buf{};
    PdoReader r{buf};
    CHECK_THROWS(r.get<std::uint32_t>(), PdoAccessError);  // 4 > 3
    CHECK_EQ(r.tell(), std::size_t{0});                    // not advanced on failure
    // A smaller read at the same spot still works.
    CHECK_EQ(r.get<std::uint16_t>(), std::uint16_t{0});
    CHECK_EQ(r.tell(), std::size_t{2});
    CHECK_THROWS(r.get<std::uint16_t>(), PdoAccessError);  // 2 > 1 remaining
}

TEST("writing past the end throws PdoAccessError") {
    Buf<2> buf{};
    PdoWriter w{buf};
    // The 4-byte write into a 2-byte buffer is a DELIBERATE overflow that exercises the
    // runtime bounds guard (it throws BEFORE any store). GCC-14's -Warray-bounds static
    // analysis false-positives on the un-taken OOB store path inside the inlined write();
    // suppress locally so the deliberate test compiles under -Werror. (clang does not warn.)
#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Warray-bounds"
#endif
    CHECK_THROWS(w.write<std::uint32_t>(0), PdoAccessError);
#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC diagnostic pop
#endif
    w.write<std::uint16_t>(0xFFFF);
    CHECK_THROWS(w.write<std::uint8_t>(0), PdoAccessError);  // full
}

TEST("load_le/store_le free functions round-trip and stay little-endian") {
    Buf<4> b{};
    ethercat::store_le<std::uint32_t>(b, 0x0A0B0C0D);
    CHECK_EQ(std::to_integer<int>(b[0]), 0x0D);  // LSB first
    CHECK_EQ(std::to_integer<int>(b[1]), 0x0C);
    CHECK_EQ(std::to_integer<int>(b[2]), 0x0B);
    CHECK_EQ(std::to_integer<int>(b[3]), 0x0A);
    CHECK_EQ(ethercat::load_le<std::uint32_t>(b), std::uint32_t{0x0A0B0C0D});

    Buf<8> d{};
    ethercat::store_le<double>(d, -987.625);
    CHECK_EQ(ethercat::load_le<double>(d), -987.625);

    Buf<2> s{};
    ethercat::store_le<std::int16_t>(s, std::int16_t{-2});
    CHECK_EQ(ethercat::load_le<std::int16_t>(s), std::int16_t{-2});

    // The throwing cursor and the free helpers must agree byte-for-byte.
    Buf<4> viaCursor{};
    PdoWriter w{viaCursor};
    w.write<std::int32_t>(-2000000000);
    CHECK_EQ(ethercat::load_le<std::int32_t>(viaCursor), std::int32_t{-2000000000});
}

TEST("skip and seek past the end throw PdoAccessError") {
    Buf<4> buf{};
    PdoReader r{buf};
    CHECK_THROWS(r.skip(5), PdoAccessError);
    CHECK_THROWS(r.seek(5), PdoAccessError);
    PdoWriter w{buf};
    CHECK_THROWS(w.skip(5), PdoAccessError);
    CHECK_THROWS(w.seek(5), PdoAccessError);
}

TEST_MAIN()
