#include "ethercat/pdo_mapping.hpp"

#include <array>
#include <cstddef>
#include <string>

#include "ethercat/errors.hpp"
#include "ethercat/pdo_buffer.hpp"

namespace ethercat {

namespace {

// Write a little-endian scalar as an SDO download. These are the MAPPING-object writes
// (0x1C1x/0x16xx/0x1Axx): the backend's generic sdo_write throws SdoError on a CoE abort, so
// re-tag it as PdoMappingError HERE -- the one place the mapping context makes that name correct
// (#32 note 14). A transport/bounds BusError propagates unchanged (not a "mapping rejected").
template <PdoScalar T>
void sdo_write_scalar(EcatBackend& backend, std::uint16_t slave, std::uint16_t index, std::uint8_t sub, T value) {
    std::array<std::byte, sizeof(T)> buf{};
    store_le<T>(buf, value);
    try {
        backend.sdo_write(slave, index, sub, buf);
    } catch (const SdoError& e) {
        throw PdoMappingError(std::string("PDO mapping write rejected: ") + e.what());
    }
}

std::string hex16(std::uint16_t v) {
    std::array<char, 7> buf{};  // "0x" + 4 hex + NUL
    static constexpr char kDigits[] = "0123456789ABCDEF";
    buf[0] = '0';
    buf[1] = 'x';
    buf[2] = kDigits[(v >> 12) & 0xF];
    buf[3] = kDigits[(v >> 8) & 0xF];
    buf[4] = kDigits[(v >> 4) & 0xF];
    buf[5] = kDigits[v & 0xF];
    return std::string(buf.data());
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

void apply_pdo_map(EcatBackend& backend, std::uint16_t slave, const PdoMap& map, PdoDirection dir) {
    const std::uint16_t assign_index = map.assign_index(dir);  // derived from direction (or override) #TODO-8
    // (a) Disable the SM PDO assignment (count := 0) so the entries are writable.
    sdo_write_scalar<std::uint8_t>(backend, slave, assign_index, 0x00, 0);

    for (const std::uint16_t pdo : map.pdo_indices) {
        const auto it = map.entries.find(pdo);
        if (it == map.entries.end()) {
            throw PdoMappingError("slave " + std::to_string(slave) + ": PDO " + hex16(pdo) + " assigned to SM " + hex16(assign_index) +
                                  " has no entry list");
        }
        const std::vector<PdoEntry>& list = it->second;
        if (list.size() > 0xFF) {
            throw PdoMappingError("slave " + std::to_string(slave) + ": PDO " + hex16(pdo) + " has " + std::to_string(list.size()) +
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
        throw PdoMappingError("slave " + std::to_string(slave) + ": SM " + hex16(assign_index) + " has " +
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
