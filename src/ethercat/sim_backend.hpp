#pragma once

// SimBackend -- a loopback STUB EtherCAT bus for offline tests (no NIC, no SOEM,
// no SDK). It is a PIPE, NOT A DRIVE: its only charter is to let a Master / the
// ServoController / the Runner CYCLE so the CONCURRENCY + PLUMBING suites can run
// off hardware -- the seqlock publish/read handoff, the command queue, the
// completion-waiter wakeups, the SDO-teardown races, the WKC-fault latch, the
// PDO-remap SDO ordering, and the DC-arm bookkeeping.
//
// It models EXACTLY the generic-CiA402 minimum those suites need to reach
// OperationEnabled and complete a Profile-Position move:
//   * the CiA402 enable ladder (controlword -> device state -> statusword),
//   * the Profile-Position set-point-acknowledge handshake (bit4 -> bit12),
//   * a position loopback (actual chases the latched target at counts_per_step)
//     and a Profile-Velocity integrator (actual += target velocity),
//   * a conformant instant mode echo (0x6060 -> 0x6061) so the driver's
//     enable-time mode gate energizes,
//   * a conformant quick-stop configure gate (0x605A reads 2; 0x6085 echoes the
//     write) so a PV/switchable configure() does not refuse.
//
// It deliberately models NO DRIVE FIDELITY: no fault injection / clear FSM, no
// quick-stop deceleration ramp, no mode-switch latency, no encoder noise, no
// unsupported-mode rejection, no forced/stale error codes. Those behaviors are
// HW-first now -- proven on the bench (a6_validate) and the RDK campaign, not in
// sim. See docs/offline-test-retirement.md for the retired-case -> HW-check map.

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <map>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include "ethercat/backend.hpp"
#include "ethercat/cia402.hpp"

namespace ethercat {

// Per-slave loopback model. Offsets are byte offsets into the respective
// process-data images. Sentinel-negative optional offsets are "not mapped".
struct SimSlaveModel {
    std::size_t output_bytes = 0;  // RxPDO command image size
    std::size_t input_bytes = 0;   // TxPDO feedback image size

    std::size_t ctrlword_off = 0;    // 0x6040 in outputs (u16)
    std::size_t statusword_off = 0;  // 0x6041 in inputs (u16)
    std::size_t target_off = 0;      // 0x607A target position in outputs (i32)
    std::size_t actual_off = 0;      // 0x6064 actual position in inputs (i32)

    // 0x60FF target velocity in outputs (i32); <0 = not mapped (PP-only map).
    std::int32_t velocity_off = -1;
    // 0x606C velocity-actual (i32) in inputs; <0 = not emitted. When mapped the stub
    // publishes the per-cycle position delta here -- the controller reads it as its
    // velocity source when the (fixed superset) map carries 0x606C.
    std::int32_t velocity_actual_off = -1;
    // 0x6060 mode-of-operation (i8) in outputs; <0 = SDO-set only.
    std::int32_t mode_of_op_off = -1;
    // 0x6061 mode-display (i8) in inputs; <0 = not emitted. When both this and
    // mode_of_op_off are mapped the stub echoes 0x6060 -> 0x6061 (conformant),
    // which the driver's enable-time mode gate requires to energize.
    std::int32_t mode_display_off = -1;

    Cia402Mode mode = Cia402Mode::ProfilePosition;  // the drive's fixed mode (loopback motion type)
    std::int32_t counts_per_step = 1000;            // actual chases target this fast per cycle (PP)

    std::uint32_t vendor_id = 0;
    std::uint32_t product_code = 0;
    std::string name = "sim-slave";
};

class SimBackend final : public EcatBackend {
   public:
    explicit SimBackend(std::vector<SimSlaveModel> slaves);

    // EcatBackend -- setup
    std::size_t open(std::string_view ifname) override;
    SlaveInfo slave_info(std::uint16_t slave) const override;
    void sdo_write(std::uint16_t slave, std::uint16_t index, std::uint8_t sub, std::span<const std::byte> data) override;
    std::size_t sdo_read(std::uint16_t slave, std::uint16_t index, std::uint8_t sub, std::span<std::byte> out) override;
    void map_process_data() override;
    void request_state(std::uint16_t slave, EcatState target) override;
    EcatState slave_state(std::uint16_t slave) const override;
    void arm_dc_sync(std::uint32_t cycle_ns, std::int32_t sync0_shift_ns) override;  // records the cycle (configured_dc_cycle_ns)
    std::int64_t dc_time() const noexcept override;  // synthetic ramp (advances per exchange) so phase-lock math is sane offline

    // EcatBackend -- cyclic
    SlaveIo slave_io(std::uint16_t slave) noexcept override;
    int exchange() noexcept override;
    int expected_wkc() const noexcept override;
    void close() noexcept override;

    // --- test hooks (not part of EcatBackend) ---
    // Force the next exchange() to report a short WKC (one cycle), to test the
    // master's WKC-fault latch.
    void force_short_wkc_once() noexcept;
    // STICKY short WKC: every exchange reports a short WKC while on, so an async RT
    // loop reliably accumulates enough consecutive bad cycles to latch a BUS fault.
    void force_short_wkc(bool on) noexcept;
    // SYNC0 cycle (ns) the bring-up armed via arm_dc_sync, or 0 if it never did.
    std::uint32_t configured_dc_cycle_ns() const noexcept;
    // SYNC0 CyclShift (ns) the bring-up passed to arm_dc_sync (config dc_sync0_shift_ns; #32 note 4).
    std::int32_t configured_dc_sync0_shift_ns() const noexcept;
    // How many times set_state(_, Op) was requested (the no-hammer invariant metric:
    // the Runner/bring-up must request OP exactly once per start).
    int op_requests() const noexcept {
        return op_requests_;
    }
    // Read back a recorded SDO value (latest write to that object).
    std::vector<std::byte> recorded_sdo(std::uint16_t slave, std::uint16_t index, std::uint8_t sub) const;
    // Ordered log of SDO write keys ((index<<8)|sub) for asserting the remap
    // sub-protocol ordering (configure() test).
    std::vector<std::uint32_t> sdo_log(std::uint16_t slave) const;

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
        std::int32_t target = 0;    // latched PP target (on the bit4 set-point edge)
        std::int32_t actual = 0;    // loopback actual position
        std::int32_t velocity = 0;  // per-cycle actual delta (velocity estimate the controller reads back)
        bool setpoint_ack = false;  // PP bit12 latch
    };

    static void step_device(Slave& s) noexcept;

    // deque, not vector: per-Slave access is O(1) and growth never moves existing
    // elements (the inner image vectors stay stable across construction).
    std::deque<Slave> slaves_;
    int expected_wkc_ = 0;
    std::uint32_t dc_cycle_ns_ = 0;       // last arm_dc_sync cycle (0 = never armed)
    std::int32_t dc_sync0_shift_ns_ = 0;  // last arm_dc_sync SYNC0 CyclShift (config dc_sync0_shift_ns; #32 note 4)
    std::int64_t synthetic_dc_ns_ = 0;    // synthetic DC clock, advanced each exchange() (dc_time())
    bool open_ = false;
    bool short_wkc_once_ = false;
    int op_requests_ = 0;                        // set_state(_, Op) call count (no-hammer metric)
    std::atomic<bool> short_wkc_sticky_{false};  // toggled non-RT while the RT loop reads it in exchange() -> atomic
};

}  // namespace ethercat
