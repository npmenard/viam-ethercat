#pragma once

// EcatBackend is the seam that isolates SOEM. ONLY soem_backend.cpp includes
// <soem/ethercat.h>; no SOEM types (ec_slavet, ecx_contextt, the global
// ec_slave[]) appear here, so the master library's public API stays clean,
// -Werror/-clang-tidy happy, and SimBackend is a drop-in for offline tests.
//
// Layering:
//   Master (generic policy: config validation, PDO-remap sub-protocol via SDO,
//           flat {offset,width} field table, WKC threshold, PdoCache refresh)
//     -> EcatBackend (low-level bus ops, SOEM-shaped but SOEM-free types)
//          -> SoemBackend  (real: reentrant ecx_* on a per-Master ecx_contextt)
//          -> SimBackend   (in-memory drive: echo a PDO map + toy CiA402)
//
// The backend deals in RAW BYTES only (SDO payloads, process-data images). All
// typing / little-endian encoding / CiA402 policy lives above it.
//
// ORIENTATION (do not transpose): names are from the MASTER's perspective, like
// SOEM's ec_slave[].outputs/.inputs --
//   outputs(slave) = the RxPDO COMMAND image (master -> slave: controlword,
//                    target, ...), WRITABLE by the RT loop.
//   inputs(slave)  = the TxPDO FEEDBACK image (slave -> master: statusword,
//                    actual, ...), READ-ONLY.
// This is the INVERSE of PdoCache, whose "inputs" are the feedback snapshot the
// non-RT side reads. So: Master publishes backend.inputs(slave) into
// PdoCache::publish_inputs; the RT loop writes commands into backend.outputs().

#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <string_view>

namespace ethercat {

// EtherCAT AL states (logical, not the wire encoding). slave 0 means "all".
enum class EcatState : std::uint8_t {
    None,
    Init,
    PreOp,
    SafeOp,
    Op,
};

const char* to_string(EcatState state) noexcept;

// SOEM-type-free projection of a slave's identity + image sizes (from
// ec_slave[]). Populated after open().
struct SlaveInfo {
    std::uint16_t position = 0;  // 1-based ring position (SOEM convention)
    std::uint32_t vendor_id = 0;
    std::uint32_t product_code = 0;
    std::string name;
    std::size_t input_bytes = 0;   // TxPDO feedback image size (slave -> master)
    std::size_t output_bytes = 0;  // RxPDO command image size (master -> slave)
};

// A slave's process-data windows inside the backend's IO image, valid after
// map_process_data(). Spans point into backend-owned storage that stays valid
// until close(). See the ORIENTATION note above: outputs = RxPDO command image
// (writable), inputs = TxPDO feedback image (read-only).
struct SlaveIo {
    std::span<std::byte> outputs;       // RxPDO command (master writes)
    std::span<const std::byte> inputs;  // TxPDO feedback (master reads)
};

// Abstract EtherCAT bus backend. One instance per master/NIC. Setup methods run
// non-RT at init/configure and MAY throw (InitError/PdoMappingError/BusError
// with clear text). The cyclic methods are on the RT hot path: noexcept, no
// allocation, no blocking.
class EcatBackend {
   public:
    virtual ~EcatBackend() = default;

    EcatBackend() = default;
    EcatBackend(const EcatBackend&) = delete;
    EcatBackend& operator=(const EcatBackend&) = delete;
    EcatBackend(EcatBackend&&) = delete;
    EcatBackend& operator=(EcatBackend&&) = delete;

    // --- setup (non-RT; may throw) ------------------------------------------

    // Open the NIC and enumerate the bus into PRE-OP. Returns the slave count
    // (>= 1) or throws InitError (no privileges / no NIC / no slaves). Throws
    // ConfigError on a second open of an already-open backend (double-init
    // guard; SOEM has process-global resources per NIC).
    virtual std::size_t open(std::string_view ifname) = 0;

    // Identity + image sizes for a slave (1-based).
    virtual SlaveInfo slave_info(std::uint16_t slave) const = 0;

    // CoE SDO write/read of raw bytes (used by the master's PDO-remap
    // sub-protocol and arbitrary object access). Blocking with a timeout. Throws
    // PdoMappingError/BusError (slave + index:sub + abort code in the text) on a
    // CoE abort. sdo_read returns the byte count read.
    virtual void sdo_write(std::uint16_t slave, std::uint16_t index, std::uint8_t sub, std::span<const std::byte> data) = 0;
    virtual std::size_t sdo_read(std::uint16_t slave, std::uint16_t index, std::uint8_t sub, std::span<std::byte> out) = 0;

    // Map the (already remapped) PDOs into the process-data image. Call AFTER the
    // SDO remap. After this, slave_io() returns valid spans. Throws
    // PdoMappingError/BusError naming the offending slave.
    virtual void map_process_data() = 0;

    // Drive `slave` (0 = all) to `target` and wait. Throws InitError naming the
    // slave + target (and the state actually reached) on timeout.
    virtual void request_state(std::uint16_t slave, EcatState target) = 0;
    virtual EcatState slave_state(std::uint16_t slave) const = 0;

    // Configure Distributed-Clock SYNC0 on every DC-capable slave at `cycle_ns`,
    // called at SAFE-OP before requesting OP, only when MasterConfig requests DC.
    // Default no-op (sim / free-run drives). Drives that support ONLY DC sync
    // (e.g. the A6-EC) fault out of OP immediately -- WKC -> 0, statusword Fault,
    // Er74.1 "no sync signal" -- unless SYNC0 is running. `cycle_ns` must be a
    // valid multiple for the drive (A6: integer multiple of 250000 ns).
    virtual void configure_dc_sync(std::uint32_t cycle_ns) {
        (void)cycle_ns;
    }

    // --- cyclic (RT hot path; noexcept, no alloc, no block) -----------------

    // Process-data windows for a slave (1-based), valid after map_process_data.
    virtual SlaveIo slave_io(std::uint16_t slave) noexcept = 0;

    // One cyclic exchange (send + receive process data). Returns the actual
    // working counter; < 0 signals a link error. Never throws -- the master
    // interprets the WKC (consecutive-error threshold -> latched BusError).
    virtual int exchange() noexcept = 0;

    // Expected WKC for a fully-operational bus, computed at map time.
    virtual int expected_wkc() const noexcept = 0;

    // --- teardown -----------------------------------------------------------

    virtual void close() noexcept = 0;
};

}  // namespace ethercat
