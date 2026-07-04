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
#include <cstdio>
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
    std::uint32_t revision = 0;  // EEPROM revision (0x1018:3) -- library-sourced identity
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
// non-RT at init/configure and MAY throw (Error/PdoMappingError/SdoError with
// clear text). The cyclic methods are on the RT hot path: noexcept, no
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
    // (>= 1) or throws Error (no privileges / no NIC / no slaves). Throws
    // Error on a second open of an already-open backend (double-init
    // guard; SOEM has process-global resources per NIC).
    virtual std::size_t open(std::string_view ifname) = 0;

    // Identity + image sizes for a slave (1-based).
    virtual SlaveInfo slave_info(std::uint16_t slave) const = 0;

    // CoE SDO write/read of raw bytes (used by the master's PDO-remap
    // sub-protocol and arbitrary object access). Blocking with a timeout. Throws
    // SdoError (slave + index:sub + abort code in the text) on a CoE abort, or
    // base Error on a bad slave id / transport. sdo_read returns the byte count read.
    virtual void sdo_write(std::uint16_t slave, std::uint16_t index, std::uint8_t sub, std::span<const std::byte> data) = 0;
    virtual std::size_t sdo_read(std::uint16_t slave, std::uint16_t index, std::uint8_t sub, std::span<std::byte> out) = 0;

    // Map the (already remapped) PDOs into the process-data image. Call AFTER the
    // SDO remap. After this, slave_io() returns valid spans. Throws
    // PdoMappingError/base Error naming the offending slave.
    virtual void map_process_data() = 0;

    // Drive `slave` (0 = all) to `target` and wait. Throws Error naming the
    // slave + target (and the state actually reached) on timeout.
    virtual void request_state(std::uint16_t slave, EcatState target) = 0;
    virtual EcatState slave_state(std::uint16_t slave) const = 0;
    // #71: the ESC AL STATUS CODE for a slave (1-based) -- the standard EtherCAT "why the drive
    // refused an AL state transition" (e.g. 0x0027 "Freerun not supported", 0x0030 "Invalid DC
    // sync config", 0x001B "SM watchdog"). Cached from the last state check (no port I/O), so it
    // is safe to read at a bring-up give-up. 0 = no error. Default 0 (the sim overrides to model a
    // refusal); describe_al_code() below turns the code into text.
    virtual std::uint16_t al_status_code(std::uint16_t slave) const noexcept {
        (void)slave;
        return 0;
    }
    // #71/#25: human string for an ARBITRARY AL code (not a per-slave live read) -- lets a consumer
    // describe a latched code (Master::bringup_al_code()). Default: a bare hex rendering; SoemBackend
    // overrides with SOEM's ec_ALstatuscode2string. Non-RT.
    virtual std::string describe_al_code(std::uint16_t code) const {
        char b[16];
        std::snprintf(b, sizeof b, "0x%04X", code);
        return b;
    }
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

    // ec_sample's SAFE-OP->OP recovery nudge (ec_sample.c:143-155), for the OP-await wait:
    // refresh AL state and, per slave (0 = all), ACK a SAFE_OP+ERROR (write SAFE_OP+ACK) or
    // RE-REQUEST OP from a plain SAFE_OP (write OP). The A6's SAFE-OP->OP can take many seconds;
    // ec_sample waits it out with PD flowing + these repeated nudges (NOT a single request).
    // Writes the AL-control register only -- the caller's loop keeps pumping process data, so
    // PD never gaps. Default no-op (sim / free-run drives reach OP from the single set_state).
    virtual void reack_op(std::uint16_t slave) noexcept {
        (void)slave;
    }

    // DC step 1, in PRE-OP: ecx_configdc -- detect DC-capable slaves, designate the
    // reference clock, write each slave's system-time offset (0x0920) + propagation
    // delay (0x0928). Offsets only; SYNC0 is NOT armed here. Per the SOEM author
    // (Arthur Ketels), a DC drive proves sync from synchronized PDO traffic in
    // SAFE-OP, so the canonical order is configdc(PRE-OP) -> SAFE-OP -> dcsync0 ->
    // phase-locked PD -> OP. Default no-op (sim / free-run drives).
    virtual void configure_dc_configdc() {}

    // DC step 2, AFTER SAFE-OP is reached, called IN the caller's RT loop once
    // phase-locked process data is already flowing continuously: arm the ESC SYNC-out
    // unit via stock `ecx_dcsync0`. No prime pump and no hand-rolled ESC sequence --
    // the caller's loop is already pumping PD, so SYNC0 is armed against a live,
    // disciplined clock and is never armed into a process-data gap (the lesson from
    // #17/CLAUDE.md). `cycle_ns` must be valid for the drive (A6: multiple of 250000
    // ns). `sync0_shift_ns` is the SYNC0 pulse phase offset (ecx_dcsync0 CyclShift):
    // the edge fires `sync0_shift_ns` after the DC base time. Default no-op (sim /
    // free-run drives).
    virtual void arm_dc_sync(std::uint32_t cycle_ns, std::int32_t sync0_shift_ns) {
        (void)cycle_ns;
        (void)sync0_shift_ns;
    }

    // Distributed-Clock system time (ns) latched at the last exchange(), for
    // phase-locking the cyclic wakeup to the SYNC0 pulse (DCtime % cycle = the
    // offset to drive to 0 with a PI controller). 0 = no DC (phase-lock no-op).
    virtual std::int64_t dc_time() const noexcept {
        return 0;
    }

    // --- cyclic (RT hot path; noexcept, no alloc, no block) -----------------

    // Process-data windows for a slave (1-based), valid after map_process_data.
    virtual SlaveIo slave_io(std::uint16_t slave) noexcept = 0;

    // One cyclic exchange (send + receive process data). Returns the actual
    // working counter; < 0 signals a link error. Never throws -- the master
    // interprets the WKC (consecutive-error threshold -> latched Error).
    virtual int exchange() noexcept = 0;

    // Expected WKC for a fully-operational bus, computed at map time.
    virtual int expected_wkc() const noexcept = 0;

    // --- teardown -----------------------------------------------------------

    virtual void close() noexcept = 0;
};

}  // namespace ethercat
