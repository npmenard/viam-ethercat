#include "ethercat/pdo_mapping.hpp"

#include <array>
#include <cstddef>
#include <string>

#include "ethercat/errors.hpp"
#include "ethercat/pdo_buffer.hpp"
#include "ethercat/util.hpp"

namespace ethercat {

namespace {

// Write a little-endian scalar as an SDO download. These are the mapping-object writes
// (0x1C1x/0x16xx/0x1Axx): the backend's generic sdo_write throws SdoError on a CoE abort, so
// re-tag it as PdoMappingError here, the one place the mapping context makes that name correct.
// A transport or bounds base Error propagates unchanged (it is not a mapping rejection).
template <PdoScalar T>
void sdo_write_scalar(SoemBackend& backend, std::uint16_t slave, std::uint16_t index, std::uint8_t sub, T value) {
    std::array<std::byte, sizeof(T)> buf{};
    store_le<T>(buf, value);
    try {
        backend.sdo_write(slave, index, sub, buf);
    } catch (const SdoError& e) {
        throw PdoMappingError(std::string("PDO mapping write rejected: ") + e.what());
    }
}

}  // namespace

std::size_t PdoMap::byte_size() const {
    std::size_t bits = 0;
    for (const auto& [pdo, list] : entries) {
        for (const auto& e : list) {
            bits += e.bit_length;
        }
    }
    if ((bits % 8) != 0) {
        throw PdoMappingError("PDO map size " + std::to_string(bits) + " bits is not byte-aligned");
    }
    return bits / 8;
}

void apply_pdo_map(SoemBackend& backend, std::uint16_t slave, const PdoMap& map, PdoDirection dir) {
    const std::uint16_t assign_index = map.assign_index(dir);  // derived from direction (or override)
    // (a) Disable the SM PDO assignment (count := 0) so the entries are writable.
    sdo_write_scalar<std::uint8_t>(backend, slave, assign_index, 0x00, 0);

    for (const std::uint16_t pdo : map.pdo_indices) {
        const auto it = map.entries.find(pdo);
        if (it == map.entries.end()) {
            throw PdoMappingError("slave " + std::to_string(slave) + ": PDO " + hex(pdo) + " assigned to SM " + hex(assign_index) +
                                  " has no entry list");
        }
        const std::vector<PdoEntry>& list = it->second;
        if (list.size() > 0xFF) {
            throw PdoMappingError("slave " + std::to_string(slave) + ": PDO " + hex(pdo) + " has " + std::to_string(list.size()) +
                                  " entries (max 255)");
        }

        // (b) Zero the entry count, (c) write each entry, (d) set the count.
        sdo_write_scalar<std::uint8_t>(backend, slave, pdo, 0x00, 0);
        std::uint8_t sub = 1;
        for (const PdoEntry& e : list) {
            const std::uint32_t packed =
                (static_cast<std::uint32_t>(e.index) << 16U) | (static_cast<std::uint32_t>(e.subindex) << 8U) | e.bit_length;
            sdo_write_scalar<std::uint32_t>(backend, slave, pdo, sub, packed);
            ++sub;
        }
        sdo_write_scalar<std::uint8_t>(backend, slave, pdo, 0x00, static_cast<std::uint8_t>(list.size()));
    }

    if (map.pdo_indices.size() > 0xFF) {
        throw PdoMappingError("slave " + std::to_string(slave) + ": SM " + hex(assign_index) + " has " +
                              std::to_string(map.pdo_indices.size()) + " PDOs (max 255)");
    }

    // (e) Assign the PDO(s) to the SM, then set the assignment count.
    std::uint8_t sub = 1;
    for (const std::uint16_t pdo : map.pdo_indices) {
        sdo_write_scalar<std::uint16_t>(backend, slave, assign_index, sub, pdo);
        ++sub;
    }
    sdo_write_scalar<std::uint8_t>(backend, slave, assign_index, 0x00, static_cast<std::uint8_t>(map.pdo_indices.size()));
}

}  // namespace ethercat
