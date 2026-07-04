#include <array>
#include <cstddef>
#include <cstdint>

#include "ethercat/pdo_buffer.hpp"
#include "test_harness.hpp"

namespace {

// Helper: a fixed byte buffer we can write into and read back out of.
template <std::size_t N>
using Buf = std::array<std::byte, N>;

}  // namespace

// #19: the throwing PdoReader/PdoWriter cursors were test-only dead code and are gone; the RT hot path
// uses only the load_le/store_le free functions. Round-trip them + pin the little-endian wire order.
TEST("load_le/store_le round-trip every supported scalar width and stay little-endian") {
    Buf<4> b{};
    ethercat::store_le<std::uint32_t>(b, 0x0A0B0C0D);
    CHECK_EQ(std::to_integer<int>(b[0]), 0x0D);  // LSB first
    CHECK_EQ(std::to_integer<int>(b[1]), 0x0C);
    CHECK_EQ(std::to_integer<int>(b[2]), 0x0B);
    CHECK_EQ(std::to_integer<int>(b[3]), 0x0A);
    CHECK_EQ(ethercat::load_le<std::uint32_t>(b), std::uint32_t{0x0A0B0C0D});

    Buf<1> u8{};
    ethercat::store_le<std::uint8_t>(u8, 0xAB);
    CHECK_EQ(ethercat::load_le<std::uint8_t>(u8), std::uint8_t{0xAB});

    Buf<2> s{};
    ethercat::store_le<std::int16_t>(s, std::int16_t{-2});
    CHECK_EQ(ethercat::load_le<std::int16_t>(s), std::int16_t{-2});

    Buf<4> i32{};
    ethercat::store_le<std::int32_t>(i32, -2000000000);
    CHECK_EQ(ethercat::load_le<std::int32_t>(i32), std::int32_t{-2000000000});

    Buf<8> i64{};
    ethercat::store_le<std::int64_t>(i64, std::int64_t{-1});
    CHECK_EQ(ethercat::load_le<std::int64_t>(i64), std::int64_t{-1});

    Buf<8> d{};
    ethercat::store_le<double>(d, -987.625);
    CHECK_EQ(ethercat::load_le<double>(d), -987.625);
}

TEST_MAIN()
