#pragma once

// SoemBackend -- the EtherCAT bus backend, the only place SOEM is used. Built on SOEM's
// reentrant ecx_* API with a per-Master ecx_contextt (no global ec_slave[] state), so a
// process could host more than one master. SOEM headers are confined to soem_backend.cpp
// (pimpl), so no SOEM types leak into the rest of the library -- the pimpl IS the SOEM
// firewall (no abstract backend interface; Master owns a SoemBackend directly).
//
// The backend deals in RAW BYTES only (SDO payloads, process-data images). All typing /
// little-endian encoding / CiA402 policy lives above it, in Master and its consumers.
//
// Setup methods run non-RT at init/configure and MAY throw (Error/PdoMappingError/SdoError
// with clear text). The cyclic methods are on the RT hot path: noexcept, no allocation, no
// blocking. Runtime requires CAP_NET_RAW (raw packet socket) and a dedicated NIC, so the
// I/O paths cannot run in CI -- there this only compiles and links.

#include <cstddef>
#include <cstdint>
#include <memory>
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
    std::uint32_t revision = 0;
    std::string name;
    std::size_t input_bytes = 0;   // TxPDO feedback image size (slave -> master)
    std::size_t output_bytes = 0;  // RxPDO command image size (master -> slave)
};

// A slave's process-data windows inside the backend's IO image, valid after
// map_process_data(). Spans point into backend-owned storage that stays valid
// until close(). Orientation is from the MASTER's perspective: outputs = RxPDO command image
// (writable), inputs = TxPDO feedback image (read-only).
struct SlaveIo {
    std::span<std::byte> outputs;       // RxPDO command (master writes)
    std::span<const std::byte> inputs;  // TxPDO feedback (master reads)
};

class SoemBackend final {
   public:
    SoemBackend();
    ~SoemBackend();

    SoemBackend(const SoemBackend&) = delete;
    SoemBackend& operator=(const SoemBackend&) = delete;
    SoemBackend(SoemBackend&&) = delete;
    SoemBackend& operator=(SoemBackend&&) = delete;

    std::size_t open(std::string_view ifname);
    SlaveInfo slave_info(std::uint16_t slave) const;
    void sdo_write(std::uint16_t slave, std::uint16_t index, std::uint8_t sub, std::span<const std::byte> data);
    std::size_t sdo_read(std::uint16_t slave, std::uint16_t index, std::uint8_t sub, std::span<std::byte> out);
    void map_process_data();
    void request_state(std::uint16_t slave, EcatState target);
    void set_state(std::uint16_t slave, EcatState target) noexcept;
    void reack_op(std::uint16_t slave) noexcept;
    EcatState slave_state(std::uint16_t slave) const;
    std::uint16_t al_status_code(std::uint16_t slave) const noexcept;  // cached ESC AL status code
    static std::string describe_al_code(std::uint16_t code);           // SOEM string for a latched code
    void configure_dc_configdc();
    void arm_dc_sync(std::uint32_t cycle_ns, std::int32_t sync0_shift_ns);
    std::int64_t dc_time() const noexcept;

    SlaveIo slave_io(std::uint16_t slave) noexcept;
    int exchange() noexcept;
    int expected_wkc() const noexcept;
    void close() noexcept;

   private:
    struct Impl;  // holds the ecx_contextt + buffers + IOmap; hides SOEM
    std::unique_ptr<Impl> impl_;
};

}  // namespace ethercat
