#pragma once

// Master -- the generic EtherCAT master policy layer over an EcatBackend.
//
// Owns the bus lifecycle (init/configure/process/close), the per-power-on PDO
// remap, the flat {offset,width} field tables, and one PdoCache per slave. It is
// backend-agnostic: a SoemBackend in production, a SimBackend in tests, injected
// as std::unique_ptr<EcatBackend>.
//
// RT boundary: process() runs on the RT thread and is noexcept/exception-free
// (a bus fault is LATCHED into an atomic flag, never thrown). Everything else
// runs non-RT at init/configure/shutdown and may throw with clear text.

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <map>
#include <memory>
#include <span>
#include <string>

#include "ethercat/backend.hpp"
#include "ethercat/errors.hpp"
#include "ethercat/pdo_buffer.hpp"
#include "ethercat/pdo_cache.hpp"
#include "ethercat/pdo_mapping.hpp"

namespace ethercat {

// Byte location of a mapped PDO object within a process-data image.
struct FieldLocation {
    std::size_t byte_offset = 0;
    std::size_t byte_width = 0;
};

class Master {
   public:
    // Validates config (no I/O). Throws ConfigError on a bad config.
    Master(MasterConfig config, std::unique_ptr<EcatBackend> backend);

    Master(const Master&) = delete;
    Master& operator=(const Master&) = delete;
    Master(Master&&) = delete;
    Master& operator=(Master&&) = delete;
    ~Master() = default;

    // --- non-RT lifecycle (may throw) ---------------------------------------

    // Open the NIC and enumerate; throws InitError if the slave count doesn't
    // match the config.
    void init();

    // PRE-OP -> apply the PDO remap per slave -> map the process image -> size
    // the PdoCaches + build the flat field tables -> SAFE-OP -> prime -> OP.
    // Re-applies the map every call (A6 map isn't in EEPROM). Throws
    // InitError/PdoMappingError naming the offending slave on failure.
    void configure();

    void close() noexcept;

    // --- RT hot path (noexcept, exception-free) -----------------------------

    // One cyclic exchange: drive the bus, interpret the WKC (latch BusError
    // after max_consecutive_wkc_errors consecutive bad cycles -- never throw),
    // and publish each slave's feedback into its PdoCache.
    void process() noexcept;

    // The RT loop writes a slave's command image (RxPDO: controlword, target)
    // directly into this span. 1-based slave id.
    std::span<std::byte> outputs(std::uint16_t slave) noexcept;

    // --- accessors (non-RT) -------------------------------------------------

    std::size_t slave_count() const noexcept {
        return slaves_.size();
    }
    bool all_operational() const noexcept {
        return operational_.load(std::memory_order_relaxed);
    }
    bool fault() const noexcept {
        return fault_.load(std::memory_order_relaxed);
    }
    int working_counter() const noexcept {
        return working_counter_.load(std::memory_order_relaxed);
    }
    int expected_wkc() const noexcept {
        return expected_wkc_;
    }
    std::string last_error() const;

    // Latest feedback snapshot for a slave (1-based).
    PdoSnapshot read_inputs(std::uint16_t slave) const noexcept;
    PdoCache& cache(std::uint16_t slave);

    // Flat field lookup (built at configure()). rx_field = command image
    // (outputs), tx_field = feedback image (inputs). Throws PdoMappingError if
    // the object isn't mapped.
    FieldLocation rx_field(std::uint16_t slave, std::uint16_t index, std::uint8_t sub) const;
    FieldLocation tx_field(std::uint16_t slave, std::uint16_t index, std::uint8_t sub) const;

    // Typed CoE SDO access (non-RT).
    template <PdoScalar T>
    T sdo_read(std::uint16_t slave, std::uint16_t index, std::uint8_t sub) {
        std::array<std::byte, sizeof(T)> buf{};
        const std::size_t n = backend_->sdo_read(slave, index, sub, buf);
        if (n < sizeof(T)) {
            throw BusError("SDO read of slave " + std::to_string(slave) + " object returned " + std::to_string(n) + " bytes, expected " +
                           std::to_string(sizeof(T)));
        }
        return load_le<T>(buf);
    }
    template <PdoScalar T>
    void sdo_write(std::uint16_t slave, std::uint16_t index, std::uint8_t sub, T value) {
        std::array<std::byte, sizeof(T)> buf{};
        store_le<T>(buf, value);
        backend_->sdo_write(slave, index, sub, buf);
    }

   private:
    // Per-slave runtime state. Holds a (non-movable) PdoCache, so it lives in a
    // std::deque (stable addresses, never moved) rather than a vector.
    struct SlaveRuntime {
        SlaveRuntime(std::uint16_t id, std::size_t rx_feedback_bytes, std::size_t tx_command_bytes)
            : slave_id(id), cache(rx_feedback_bytes, tx_command_bytes) {}
        std::uint16_t slave_id;
        SlaveIo io;                                        // spans into backend storage (valid after map_process_data)
        std::map<std::uint32_t, FieldLocation> rx_fields;  // command image (RxPDO/outputs)
        std::map<std::uint32_t, FieldLocation> tx_fields;  // feedback image (TxPDO/inputs)
        PdoCache cache;                                    // NOTE: rx snapshot = FEEDBACK (TxPDO), tx staging = COMMAND (RxPDO)
    };

    SlaveRuntime& runtime_for(std::uint16_t slave);
    const SlaveRuntime& runtime_for(std::uint16_t slave) const;

    static std::map<std::uint32_t, FieldLocation> build_field_table(std::uint16_t slave, const PdoMap& map);

    MasterConfig config_;
    std::unique_ptr<EcatBackend> backend_;
    std::deque<SlaveRuntime> slaves_;

    int expected_wkc_ = 0;
    std::uint64_t cycle_ = 0;                   // RT-only
    std::uint32_t consecutive_wkc_errors_ = 0;  // RT-only

    std::atomic<int> working_counter_{0};
    std::atomic<int> fault_wkc_{0};
    std::atomic<bool> fault_{false};
    std::atomic<bool> operational_{false};
};

}  // namespace ethercat
