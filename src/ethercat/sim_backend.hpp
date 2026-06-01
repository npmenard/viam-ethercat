#pragma once

// SimBackend -- an in-memory EtherCAT bus for offline tests (no NIC, no SOEM, no
// SDK). It implements the DEVICE side of CiA402 (the inverse of Cia402Fsm):
// each cyclic exchange consumes the controlword from a slave's RxPDO image,
// advances a toy DS402 device state, and emits the statusword + feedback into
// the TxPDO image. It mirrors the A6-EC where it matters: statusword bit10 is
// held =1 always, and the Profile-Position bit12 set-point-acknowledge
// handshake is modeled.
//
// This is the primary CI integration path: a Master driving a SimBackend
// exercises the full RT loop + CiA402 enable ladder + setpoint propagation +
// snapshot readback with no hardware.

#include <cstddef>
#include <cstdint>
#include <map>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include "ethercat/backend.hpp"
#include "ethercat/cia402.hpp"

namespace ethercat {

// Per-slave simulation model. The test supplies it; it stays generic (A6
// specifics are config data, never code). Offsets are byte offsets into the
// respective process-data images.
struct SimSlaveModel {
    std::size_t output_bytes = 0;  // RxPDO command image size
    std::size_t input_bytes = 0;   // TxPDO feedback image size

    std::size_t ctrlword_off = 0;    // 0x6040 in outputs (u16)
    std::size_t statusword_off = 0;  // 0x6041 in inputs (u16)
    std::size_t target_off = 0;      // 0x607A target position in outputs (i32)
    std::size_t actual_off = 0;      // 0x6064 actual position in inputs (i32)

    Cia402Mode mode = Cia402Mode::ProfilePosition;
    std::int32_t counts_per_step = 1000;  // PP: how fast actual chases target per cycle
    std::int32_t velocity_off = -1;       // optional: 0x60FF target velocity offset in outputs (i32); <0 = none

    std::uint32_t vendor_id = 0;
    std::uint32_t product_code = 0;
    std::string name = "sim-slave";
};

class SimBackend final : public EcatBackend {
   public:
    explicit SimBackend(std::vector<SimSlaveModel> slaves);

    // EcatBackend -- setup
    std::size_t scan(std::string_view ifname) override;
    SlaveInfo slave_info(std::uint16_t slave) const override;
    void sdo_write(std::uint16_t slave, std::uint16_t index, std::uint8_t sub, std::span<const std::byte> data) override;
    std::size_t sdo_read(std::uint16_t slave, std::uint16_t index, std::uint8_t sub, std::span<std::byte> out) override;
    void map_process_image() override;
    void request_state(std::uint16_t slave, EcatState target) override;
    EcatState state(std::uint16_t slave) const override;

    // EcatBackend -- cyclic
    std::span<std::byte> outputs(std::uint16_t slave) noexcept override;
    std::span<const std::byte> inputs(std::uint16_t slave) const noexcept override;
    int send_receive() noexcept override;
    int expected_wkc() const noexcept override;
    void close() noexcept override;

    // --- test hooks (not part of EcatBackend) ---
    // Inject a fault on a slave (next exchange decodes to Fault).
    void inject_fault(std::uint16_t slave) noexcept;
    // Force the next send_receive() to report a short WKC (one cycle), to test
    // the master's WKC-fault latch.
    void force_short_wkc_once() noexcept;
    // Read back a recorded SDO value (for asserting the configure() remap order).
    std::vector<std::byte> recorded_sdo(std::uint16_t slave, std::uint16_t index, std::uint8_t sub) const;

   private:
    struct Slave {
        SimSlaveModel model;
        std::vector<std::byte> output_image;                         // RxPDO (master writes)
        std::vector<std::byte> input_image;                          // TxPDO (master reads)
        std::map<std::uint32_t, std::vector<std::byte>> dictionary;  // recorded SDO writes, key=(index<<8)|sub
        std::vector<std::uint32_t> sdo_write_order;                  // SDO write keys in order (configure() ordering test)
        EcatState state = EcatState::Init;
        Cia402State device_state = Cia402State::NotReadyToSwitchOn;
        std::uint16_t prev_ctrlword = 0;
        std::int32_t target = 0;
        std::int32_t actual = 0;
        bool setpoint_ack = false;  // PP bit12 latch
        bool faulted = false;
    };

    static void step_device(Slave& s) noexcept;

    std::vector<Slave> slaves_;
    int expected_wkc_ = 0;
    bool open_ = false;
    bool short_wkc_once_ = false;
};

}  // namespace ethercat
