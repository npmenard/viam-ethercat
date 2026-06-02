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
    // the PdoCaches + build the flat field tables -> SAFE-OP -> [DC: enable SYNC0] ->
    // [reach_op: phase-lock warmup -> OP] . Re-applies the map every call (A6 map
    // isn't in EEPROM). Throws InitError/PdoMappingError naming the offending slave.
    //
    // reach_op = false: stop at SAFE-OP with DC enabled and DON'T run the warmup or
    // request OP -- for a CONTINUOUS bring-up where the caller's own cyclic loop runs
    // the phase-lock warmup, calls request_op() once locked, and keeps cycling, so a
    // DC drive sees no frame gap through SAFE-OP->OP (the only safe path on the A6,
    // which faults out of OP on a single missed SYNC0 frame).
    void configure(bool reach_op = true);

    // Caller-driven in-loop SYNC0 arm (only after configure(reach_op=false)): arm the
    // ESC SYNC-out unit from the RT loop, AFTER SAFE-OP, once synchronized PD is
    // flowing and the master is phase-locking -- a DC drive (the A6) generates SYNC0 /
    // permits OP only once it has proven sync from that traffic. Call ONCE; then keep
    // pumping + poll dc_sync_status until ready, then request_op(). Never throws.
    void arm_dc_sync() noexcept;

    // Caller-driven post-arm SDO writes (only after configure(reach_op=false) + an
    // in-loop arm_dc_sync()): apply each slave's postdc_sdo_writes -- the ETG.1020
    // cycle-time handshake that needs the ESC SYNC0 cycle (0x09A0) live. configure()
    // skips these on the caller-driven path because the arm is deferred to the loop;
    // the caller invokes this once, just after arming. Never throws (optional+mandatory
    // writes both logged, never propagated into the RT loop).
    void apply_postdc_writes() noexcept;

    // Caller-driven OP request (only after configure(reach_op=false)): writes the
    // OP state request; the caller's cyclic loop pumps the transition. Never throws.
    void request_op() noexcept;

    void close() noexcept;

    // --- RT hot path (noexcept, exception-free) -----------------------------

    // One cyclic exchange: drive the bus, interpret the WKC (latch BusError
    // after max_consecutive_wkc_errors consecutive bad cycles -- never throw),
    // and publish each slave's feedback into its PdoCache.
    void process() noexcept;

    // The RT loop writes a slave's command image (RxPDO: controlword, target)
    // directly into this span. 1-based slave id.
    std::span<std::byte> outputs(std::uint16_t slave) noexcept;

    // The RT loop reads a slave's live feedback image (TxPDO: statusword, actual)
    // directly -- the values from the last exchange(), without going through the
    // seqlock snapshot (which is for the NON-RT side). 1-based slave id.
    std::span<const std::byte> input_image(std::uint16_t slave) const noexcept;

    // --- accessors (non-RT) -------------------------------------------------

    std::size_t slave_count() const noexcept {
        return slaves_.size();
    }
    bool all_operational() const noexcept {
        return operational_.load(std::memory_order_relaxed);
    }
    bool fault() const noexcept {
        // Acquire pairs with the release store in process(), so a reader that
        // sees fault()==true also sees the fault_wkc_ payload last_error() reads.
        return fault_.load(std::memory_order_acquire);
    }
    int working_counter() const noexcept {
        return working_counter_.load(std::memory_order_relaxed);
    }
    // RAW working counter from the last exchange(), good OR bad (unlike
    // working_counter(), which holds the last GOOD value so a transient doesn't
    // flap the reported WKC). Use this to actually SEE per-cycle short WKC.
    int last_wkc() const noexcept {
        return last_wkc_.load(std::memory_order_relaxed);
    }
    int expected_wkc() const noexcept {
        return expected_wkc_;
    }
    // DC system time (ns) from the last process(); for phase-locking the cyclic
    // wakeup to SYNC0. 0 on non-DC backends.
    std::int64_t dc_time() const noexcept {
        return backend_->dc_time();
    }
    // Live DC-sync health for the SAFE-OP -> OP gate: poll while pumping phase-locked
    // PD in SAFE-OP, request OP only once `.ready` (slave clock locked + SYNC0 armed).
    DcSyncStatus dc_sync_status() const noexcept {
        return backend_->dc_sync_status();
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

    // Pump `max_cycles` exchanges while running the DC phase-lock PI, sharing the
    // caller's `next` deadline + `integral` so back-to-back calls stay on ONE
    // continuous cadence (gapless across a state change). `target_streak > 0` returns
    // early once the phase has held in-band that many consecutive cycles; 0 = run all
    // cycles. Returns the final consecutive-locked streak. noexcept (RT-paced setup).
    int phase_lock_pump(std::uint32_t cycle_ns,
                        std::int64_t shift_ns,
                        std::uint32_t max_cycles,
                        int target_streak,
                        timespec& next,
                        std::int64_t& integral) noexcept;

    static std::map<std::uint32_t, FieldLocation> build_field_table(std::uint16_t slave, const PdoMap& map);

    MasterConfig config_;
    std::unique_ptr<EcatBackend> backend_;
    std::deque<SlaveRuntime> slaves_;

    int expected_wkc_ = 0;
    std::uint64_t cycle_ = 0;                   // RT-only
    std::uint32_t consecutive_wkc_errors_ = 0;  // RT-only
    std::uint32_t settle_remaining_ = 0;        // RT-only: post-OP grace cycles left (DC phase settle; no WKC latch)

    std::atomic<int> working_counter_{0};
    std::atomic<int> last_wkc_{0};  // raw WKC from the last exchange (diagnostic; good or bad)
    std::atomic<int> fault_wkc_{0};
    std::atomic<bool> fault_{false};
    std::atomic<bool> operational_{false};
};

}  // namespace ethercat
