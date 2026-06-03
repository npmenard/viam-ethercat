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

// Live DC-sync health of the bus, sampled from the ESC DC registers for the
// SAFE-OP -> OP gate (see EcatBackend::dc_sync_status). All fields are AND-reduced
// across every DC slave (the bus is ready only if every slave is).
struct DcSyncStatus {
    std::int32_t sys_time_diff_ns = 0;  // worst |0x092C| seen (signed sample, the real lock signal -- NOT 0x0930)
    bool clock_locked = false;          // every slave's |0x092C| within the lock band
    bool sync0_active = false;          // every slave's 0x0984 b0 set (SYNC-out unit ARMED)
    bool sync0_pulsing = false;         // every slave's 0x098E changed since the PREVIOUS dc_sync_status() poll (SYNC0 physically firing)
    std::uint16_t al_status = 0;        // worst 0x0134 AL status code (0x2D = DC start invalid)
    bool ready = false;                 // clock_locked && sync0_PULSING on ALL slaves -> safe to request OP (NOT just armed:
                                        // SYNC0's first edge is ~100ms after arm, so gate on real pulses to avoid a no-sync fault)
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
    // REQUEST `slave` (0 = all) to `target` -- writes the state request only; does
    // NOT pump process data, wait, or throw. The caller's cyclic loop drives the
    // transition (so a DC drive sees CONTINUOUS process data through SAFE-OP->OP,
    // no gap -> no Er74). Default: best-effort no-throw wrapper over request_state.
    virtual void set_state(std::uint16_t slave, EcatState target) noexcept {
        try {
            request_state(slave, target);
        } catch (...) {  // NOLINT(bugprone-empty-catch): caller polls slave_state()
        }
    }

    // DC step 1, in PRE-OP: ecx_configdc -- detect DC-capable slaves, designate the
    // reference clock, write each slave's system-time offset (0x0920) + propagation
    // delay (0x0928). Offsets only; SYNC0 is NOT armed here. Per the SOEM author
    // (Arthur Ketels), a DC drive proves sync from synchronized PDO traffic in
    // SAFE-OP, so the canonical order is configdc(PRE-OP) -> SAFE-OP -> dcsync0 ->
    // phase-locked PD -> OP. Default no-op (sim / free-run drives).
    virtual void configure_dc_configdc() {}

    // DC step 2, AFTER SAFE-OP is reached: arm the ESC SYNC-out unit (ecx_dcsync0 on a
    // FRESH live 0x0910). Self-contained variant -- PRIMES a few exchanges itself so
    // 0x0910 is live, then arms. Used by the OWN-the-bring-up path configure(reach_op=
    // true). Default no-op (sim / free-run drives). `cycle_ns` must be a valid multiple
    // for the drive (A6: multiple of 250000 ns). `sync0_shift_ns` is the SYNC0 pulse
    // phase offset (ecx_dcsync0 CyclShift): the SYNC0 edge fires `sync0_shift_ns` after
    // the DC base time. Tune it (with the master's send phase) so the drive latches a
    // FRESH output frame at SYNC0.
    // `start_delay_ns` replaces SOEM's hardcoded 100 ms SyncDelay: how far in the future
    // the FIRST SYNC0 edge is scheduled. Short (~15 ms) so a drive whose sync watchdog is
    // < 100 ms (the A6) sees the first pulse before its watchdog trips.
    virtual void configure_dc_sync(std::uint32_t cycle_ns, std::int32_t sync0_shift_ns, std::int64_t start_delay_ns) {
        (void)cycle_ns;
        (void)sync0_shift_ns;
        (void)start_delay_ns;
    }

    // DC step 2, IN-LOOP variant for the caller-driven path: arm SYNC0 ONLY, no prime
    // pump (the caller's RT loop is ALREADY pumping phase-locked PD). Call a few cycles
    // into the SAFE-OP loop, once synchronized PD is flowing AND the master is phase-
    // locking, so the drive arms SYNC0 against a live, disciplined clock it has just
    // proven in sync. `start_delay_ns` = first-edge delay (see configure_dc_sync).
    // Default no-op (sim / free-run drives).
    virtual void arm_dc_sync(std::uint32_t cycle_ns, std::int32_t sync0_shift_ns, std::int64_t start_delay_ns) {
        (void)cycle_ns;
        (void)sync0_shift_ns;
        (void)start_delay_ns;
    }

    // Distributed-Clock system time (ns) latched at the last exchange(), for
    // phase-locking the cyclic wakeup to the SYNC0 pulse (DCtime % cycle = the
    // offset to drive to 0 with a PI controller). 0 = no DC (phase-lock no-op).
    virtual std::int64_t dc_time() const noexcept {
        return 0;
    }

    // Live DC-sync health, read straight from the ESC DC registers (acyclic FPRD)
    // for the SAFE-OP -> OP gate. Per the SOEM author (Arthur Ketels): a DC drive
    // will NOT permit OP until it has OBSERVED synchronized, DC-phase-locked PDO
    // traffic in SAFE-OP -- the SYNC-out unit only ARMS (0x0984) once such traffic
    // proves the slave clock is in sync. So the caller must pump phase-locked PD in
    // SAFE-OP and poll this until `ready`, THEN request OP. Default = ready (non-DC
    // / sim never blocks). See SoemBackend for the per-slave register reads.
    virtual DcSyncStatus dc_sync_status() noexcept {
        return DcSyncStatus{.clock_locked = true, .sync0_active = true, .ready = true};
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
