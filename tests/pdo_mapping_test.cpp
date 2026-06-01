#include <cstdint>
#include <vector>

#include "ethercat/errors.hpp"
#include "ethercat/pdo_buffer.hpp"
#include "ethercat/pdo_mapping.hpp"
#include "ethercat/sim_backend.hpp"
#include "test_harness.hpp"

using ethercat::apply_pdo_map;
using ethercat::PdoEntry;
using ethercat::PdoMap;
using ethercat::PdoMappingError;
using ethercat::SimBackend;
using ethercat::SimSlaveModel;

namespace {

std::uint32_t key(std::uint16_t index, std::uint8_t sub) {
    return (static_cast<std::uint32_t>(index) << 8U) | sub;
}

// RxPDO 0x1600 assigned to SM 0x1C12, mapping ctrl(0x6040,16b) + target(0x607A,32b).
PdoMap a6_rxpdo() {
    PdoMap m;
    m.assign_index = 0x1C12;
    m.pdo_indices = {0x1600};
    m.entries[0x1600] = {PdoEntry{0x6040, 0x00, 16}, PdoEntry{0x607A, 0x00, 32}};
    return m;
}

// SimBackend is non-movable (deleted move on the polymorphic base), so return
// the model list and let each test construct the backend in place.
std::vector<SimSlaveModel> one_slave() {
    SimSlaveModel model;
    model.output_bytes = 6;
    model.input_bytes = 6;
    return {model};
}

}  // namespace

TEST("apply_pdo_map emits the CiA remap SDO sequence in the load-bearing order") {
    SimBackend be{one_slave()};
    apply_pdo_map(be, 1, a6_rxpdo());

    const std::vector<std::uint32_t> expected = {
        key(0x1C12, 0x00),  // (a) disable SM assignment
        key(0x1600, 0x00),  // (b) zero entry count
        key(0x1600, 0x01),  // (c) entry 1: ctrl
        key(0x1600, 0x02),  // (c) entry 2: target
        key(0x1600, 0x00),  // (d) set entry count = 2
        key(0x1C12, 0x01),  // (e) assign 0x1600 to SM
        key(0x1C12, 0x00),  // (e) set assignment count = 1
    };
    CHECK_EQ(be.sdo_log(1).size(), expected.size());
    const auto log = be.sdo_log(1);
    for (std::size_t i = 0; i < expected.size() && i < log.size(); ++i) {
        CHECK_EQ(log[i], expected[i]);
    }
}

TEST("apply_pdo_map writes the packed entry value and final counts") {
    SimBackend be{one_slave()};
    apply_pdo_map(be, 1, a6_rxpdo());

    // entry 1 = (0x6040<<16)|(0<<8)|16 = 0x60400010, little-endian.
    const auto e1 = be.recorded_sdo(1, 0x1600, 0x01);
    CHECK_EQ(e1.size(), std::size_t{4});
    CHECK_EQ(ethercat::load_le<std::uint32_t>(e1), std::uint32_t{0x60400010});

    // entry 2 = (0x607A<<16)|(0<<8)|32 = 0x607A0020.
    const auto e2 = be.recorded_sdo(1, 0x1600, 0x02);
    CHECK_EQ(ethercat::load_le<std::uint32_t>(e2), std::uint32_t{0x607A0020});

    // Final entry count = 2; final SM assignment count = 1.
    CHECK_EQ(ethercat::load_le<std::uint8_t>(be.recorded_sdo(1, 0x1600, 0x00)), std::uint8_t{2});
    CHECK_EQ(ethercat::load_le<std::uint8_t>(be.recorded_sdo(1, 0x1C12, 0x00)), std::uint8_t{1});
    CHECK_EQ(ethercat::load_le<std::uint16_t>(be.recorded_sdo(1, 0x1C12, 0x01)), std::uint16_t{0x1600});
}

TEST("PdoMap::byte_size sums entry bits; rejects non-byte-aligned maps") {
    CHECK_EQ(a6_rxpdo().byte_size(), std::size_t{6});  // (16 + 32) / 8

    PdoMap bad = a6_rxpdo();
    bad.entries[0x1600].push_back(PdoEntry{0x0000, 0x00, 4});  // 52 bits -> not byte-aligned
    CHECK_THROWS_MSG(bad.byte_size(), PdoMappingError, "not byte-aligned");
}

TEST("apply_pdo_map throws PdoMappingError when a PDO has no entry list") {
    SimBackend be{one_slave()};
    PdoMap m;
    m.assign_index = 0x1C12;
    m.pdo_indices = {0x1600};  // but entries[0x1600] never populated
    CHECK_THROWS_MSG(apply_pdo_map(be, 1, m), PdoMappingError, "no entry list");
}

TEST_MAIN()
