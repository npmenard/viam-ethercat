#pragma once

// SoemBackend -- the REAL EtherCAT backend, the ONLY place SOEM is used. It
// implements EcatBackend on top of SOEM's reentrant ecx_* API with a per-Master
// ecx_contextt (no global ec_slave[] state), so a process could host more than
// one master. SOEM headers are confined to soem_backend.cpp (pimpl), so no SOEM
// types leak into the rest of the library.
//
// Runtime requires CAP_NET_RAW (raw packet socket) and a dedicated NIC, so this
// cannot run in CI -- it must only COMPILE + LINK there. The offline path uses
// SimBackend.

#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <string_view>

#include "ethercat/backend.hpp"

namespace ethercat {

class SoemBackend final : public EcatBackend {
   public:
    SoemBackend();
    ~SoemBackend() override;

    std::size_t open(std::string_view ifname) override;
    SlaveInfo slave_info(std::uint16_t slave) const override;
    void sdo_write(std::uint16_t slave, std::uint16_t index, std::uint8_t sub, std::span<const std::byte> data) override;
    std::size_t sdo_read(std::uint16_t slave, std::uint16_t index, std::uint8_t sub, std::span<std::byte> out) override;
    void map_process_data() override;
    void request_state(std::uint16_t slave, EcatState target) override;
    void set_state(std::uint16_t slave, EcatState target) noexcept override;
    EcatState slave_state(std::uint16_t slave) const override;
    void configure_dc_sync(std::uint32_t cycle_ns, std::int32_t sync0_shift_ns) override;
    std::int64_t dc_time() const noexcept override;
    DcSyncStatus dc_sync_status() noexcept override;

    SlaveIo slave_io(std::uint16_t slave) noexcept override;
    int exchange() noexcept override;
    int expected_wkc() const noexcept override;
    void close() noexcept override;

   private:
    struct Impl;  // holds the ecx_contextt + buffers + IOmap; hides SOEM
    std::unique_ptr<Impl> impl_;
};

}  // namespace ethercat
