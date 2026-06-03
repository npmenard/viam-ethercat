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

// Status of the DC bring-up state machine (Master::bringup_step). The caller drives
// one step per cyclic exchange until it sees Operational (switch to the steady loop)
// or Aborted (surface the fault; do NOT immediately re-enter bring-up -- repeated
// Er74 OP-entry wedges the A6, CLAUDE.md).
enum class BringupStatus : std::uint8_t {
    Settling,     // pumping phase-locked PD; DC clock coming live, not yet armed
    Arming,       // SYNC0 armed this step (stock ecx_dcsync0)
    Gating,       // armed; waiting for "no Er74.1" to hold K cycles before requesting OP
    AwaitingOp,   // OP requested; waiting for all slaves to reach OP with full WKC
    Operational,  // all slaves OPERATIONAL -- bring-up complete
    Aborted,      // a sync fault (Er74.1) appeared during the gate -- SYNC0 did not take
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

    // PRE-OP -> apply the PDO remap per slave -> map the process image -> size the
    // PdoCaches + build the flat field tables -> [DC: configdc (PRE-OP)] -> SAFE-OP ->
    // [fault_reset: clear a latent drive fault via the vendor SDO]. Re-applies the map
    // every call (A6 map isn't in EEPROM). Throws InitError/PdoMappingError naming the
    // offending slave.
    //
    // configure() STOPS AT SAFE-OP: it does NOT arm SYNC0 and does NOT request OP. The
    // caller's single cyclic loop runs the bring-up to OP via bringup_step() -- so a DC
    // drive sees CONTINUOUS process data through SAFE-OP->OP (no frame gap -> no Er74),
    // and SYNC0 is armed only once PD is already flowing (#20 / CLAUDE.md). The blocking
    // fault-reset SDO is the LAST thing configure() does, while it is still the single
    // port owner (before the RT thread spawns).
    void configure();

    // One cyclic step of the DC bring-up state machine, called from the caller's RT loop
    // AFTER configure() (which left the bus at SAFE-OP). It performs the cyclic exchange()
    // and advances SETTLE -> ARM(stock ecx_dcsync0) -> GATE -> request OP -> AWAIT_OP ->
    // OPERATIONAL, returning the new status. The CALLER owns the cadence: it does the
    // clock_nanosleep deadline + dc_phase_correction(dc_time(), ...) around this call, so
    // PD stays phase-locked and gapless. `drive_sync_faulted` is the caller's read of the
    // drive's Er74.1 no-sync fault (0x603F == 0x8700) from the PREVIOUS step's feedback
    // image -- passing it in keeps Master free of CiA402 semantics and lets both
    // ServoController and the thin #21 program reuse this. Never throws. On Operational
    // the caller switches to its steady loop; on Aborted it must surface the fault and
    // NOT immediately re-enter bring-up (bounded -- repeated Er74 OP-entry wedges the A6).
    BringupStatus bringup_step(bool drive_sync_faulted) noexcept;

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

    // Internal phases of the bring-up state machine (bringup_step). Distinct from the
    // public BringupStatus so the "armed this step" edge (Arming) is a transient the
    // caller sees once while the internal phase is already Gate.
    enum class BringupPhase : std::uint8_t { Settle, Arm, Gate, AwaitOp, Done, Aborted };
    BringupPhase bringup_phase_ = BringupPhase::Settle;  // RT-only
    std::uint32_t bringup_settle_count_ = 0;             // RT-only: SETTLE cycles elapsed
    std::uint32_t bringup_gate_streak_ = 0;              // RT-only: consecutive no-Er74.1 cycles in GATE
    bool dc_enabled_ = false;                            // set in configure(): is SYNC0 in play?
    std::uint32_t dc_cycle_ns_ = 0;                      // SYNC0 cycle = 1e9 / loop rate (set in configure())
    std::int32_t dc_sync0_shift_ns_ = 0;                 // ecx_dcsync0 CyclShift (set in configure())

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
