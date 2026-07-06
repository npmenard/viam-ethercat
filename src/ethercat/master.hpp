#pragma once

// Master -- the generic EtherCAT master policy layer over an EcatBackend.
//
// Owns the bus lifecycle (init/configure/process/close), the per-power-on PDO
// remap, the flat {offset,width} field tables, and one PdoCache per slave. It is
// backend-agnostic: the EcatBackend (SoemBackend in production) is injected
// as std::unique_ptr<EcatBackend>.
//

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <map>
#include <memory>
#include <optional>
#include <span>
#include <string>

#include "ethercat/backend.hpp"
#include "ethercat/errors.hpp"
#include "ethercat/field.hpp"
#include "ethercat/pdo_buffer.hpp"
#include "ethercat/pdo_cache.hpp"
#include "ethercat/pdo_mapping.hpp"

namespace ethercat {

// Byte location of a mapped PDO object within a process-data image. Offset-only: every
// access derives its width from the Field's T (sizeof) at the call site (the load_le/
// store_le free functions below), so the location carries no width. The per-object mapped
// width (bit_length) lives in the Master's internal field table (Master::MappedField),
// where resolve_field's configure-time assert verifies sizeof(F::type)*8 == bit_length.
// A FieldLocation is ALWAYS a valid resolved location (only the resolvers construct one);
// "maybe absent" is expressed as std::optional<FieldLocation> (try_resolve_rx/try_resolve_tx).
struct FieldLocation {
    std::size_t byte_offset = 0;
};

// RT cached-offset accessors: read/write sizeof(T) little-endian at a resolved FieldLocation
// on a live process image. noexcept and no bounds-check. Precondition: `loc` came from
// Master::resolve_rx/resolve_tx<F>() (width-asserted vs the mapped bit_length and checked
// in-image) at configure, so offset + sizeof(T) is valid. Resolve and bounds-check happen
// once at configure, so the 1 kHz path never throws, resolves, or allocates. (Overloads the
// span forms in pdo_buffer.hpp.)
template <PdoScalar T>
T load_le(std::span<const std::byte> image, FieldLocation loc) noexcept {
    return load_le<T>(image.subspan(loc.byte_offset, sizeof(T)));
}
template <PdoScalar T>
void store_le(std::span<std::byte> image, FieldLocation loc, T value) noexcept {
    store_le<T>(image.subspan(loc.byte_offset, sizeof(T)), value);
}

// Cyclic WKC health counters, published RT -> non-RT.
struct WkcStats {
    int expected = 0;                // the bus's full working counter (constant post-configure)
    int last = 0;                    // raw WKC of the most recent exchange (good or bad)
    std::uint64_t total_cycles = 0;  // process() cycles since configure()
    std::uint64_t bad_cycles = 0;    // cycles whose WKC was short/abnormal
};

// Status of the DC bring-up state machine (Master::bringup_step). The caller drives one step
// per cyclic exchange until it sees Operational (switch to the steady loop) or Aborted
// (surface the fault; do not immediately re-enter bring-up -- a workaround for drives whose
// EtherCAT interface locks up under repeated failed OP requests, recoverable only by a
// control-power cycle).
enum class BringupStatus : std::uint8_t {
    Gating,
    AwaitingOp,
    Operational,
    Aborted,
};

class Master {
   public:
    // Validates config (no I/O). Throws Error on a bad config.
    Master(MasterConfig config, std::unique_ptr<EcatBackend> backend);

    Master(const Master&) = delete;
    Master& operator=(const Master&) = delete;
    Master(Master&&) = delete;
    Master& operator=(Master&&) = delete;
    ~Master() = default;

    // --- non-RT lifecycle (may throw) ---------------------------------------

    // Open the NIC and enumerate; throws Error if the slave count doesn't
    // match the config.
    void init();

    // PRE-OP, apply the PDO remap per slave, map the process image, size the PdoCaches and
    // build the flat field tables, arm SYNC0 in PRE-OP (DC), run configdc (DC), then go to
    // SAFE-OP. Re-applies the map every call (a remapped PDO assignment is volatile on many
    // drives, not persisted in EEPROM). Throws Error/PdoMappingError naming the offending slave.
    //
    // configure() stops at SAFE-OP: it arms SYNC0 (in PRE-OP, before config_map_group) but
    // does not request OP. The caller's single cyclic loop runs the bring-up to OP via
    // bringup_step(), so a DC drive sees continuous process data through SAFE-OP -> OP with
    // no frame gap. Arming SYNC0 in PRE-OP is what makes drives that latch their sync-type at
    // the PRE-OP -> SAFE-OP transition self-select DC.
    void configure();

    // One cyclic step of the DC bring-up state machine, called from the caller's RT loop after
    // configure() (which left the bus at SAFE-OP with SYNC0 already armed in PRE-OP). It does
    // the cyclic exchange() and advances settle (bounded phase-locked PD), request OP once,
    // await OP (hold for OP + sync), operational, returning the new status. The caller owns the
    // cadence: it does the clock_nanosleep deadline and dc_phase_correction(dc_time(), ...)
    // around this call, so PD stays phase-locked and gapless. `drive_sync_faulted` is the
    // caller's read of the drive's no-sync fault (its device fault code) from the previous
    // step's feedback image; passing it in keeps Master free of CiA402 semantics. Never throws.
    // On Operational the caller switches to its steady loop; on Aborted it must surface the
    // fault and not immediately re-enter bring-up (a workaround for drives whose EtherCAT
    // interface locks up under repeated failed OP requests).
    //
    // `drive_present` is a caller read of whether the drive's feedback looks alive (e.g.
    // statusword != 0). It is an OP-confirm gate beyond the working counter -- a workaround for
    // drives that pass the WKC gate (full WKC) while their PDO data is dead (statusword 0x0)
    // after a refused OP request, where WKC alone would wrongly declare OP; requiring
    // drive_present makes bring-up give up (BringupAborted) on a dead drive instead. Defaults
    // to true (WKC only) so single-signal callers and tests are unaffected; the Runner passes
    // the AND of the controls' drive_present().
    BringupStatus bringup_step(bool drive_sync_faulted, bool drive_present = true) noexcept;

    void close() noexcept;

    // --- RT hot path (noexcept, exception-free) -----------------------------

    // One cyclic exchange: drive the bus, interpret the WKC (latch Error
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
    // Identity and image sizes for a slave (1-based), from enumeration, so callers read
    // identity through the API instead of poking CoE 0x1018 directly. Valid after init().
    // Throws Error if out of range.
    SlaveInfo slave_info(std::uint16_t slave) const {
        return backend_->slave_info(slave);
    }
    // The ESC AL status code for a slave (1-based): the standard EtherCAT reason a drive refused
    // an AL transition (e.g. 0x0027 "Freerun not supported" on a DC-only drive requested into OP
    // without SYNC0). A cached read (no port I/O), so a consumer can read it at a bring-up
    // give-up to name the cause. 0 means no error. Delegates to the backend.
    std::uint16_t al_status_code(std::uint16_t slave) const noexcept {
        return backend_->al_status_code(slave);
    }
    // The last non-zero ESC AL status code observed during AWAIT_OP, latched across the whole
    // bring-up. At a give-up the live al_status_code(slave) can read 0 (reack_op ACKs the
    // SAFE_OP+ERROR on the cycle it times out), so a consumer surfacing why bring-up failed must
    // read this to reliably name the cause (e.g. 0x0027 on a DC-only drive under free-run). 0
    // means no AL error was seen. RT-written, read plainly after the RT thread joins.
    std::uint16_t bringup_al_code() const noexcept {
        return bringup_al_code_;
    }
    // Human-readable text for an AL status code (delegates to the backend / SOEM's
    // ec_ALstatuscode2string), so a consumer can describe the latched bringup_al_code(). Non-RT
    // (allocates); call it at the give-up, off the RT path.
    std::string describe_al_code(std::uint16_t code) const {
        return backend_->describe_al_code(code);
    }
    bool all_operational() const noexcept {
        return operational_.load(std::memory_order_relaxed);
    }
    bool fault() const noexcept {
        // Acquire pairs with the release store in process(), so a reader that sees
        // fault() == true also sees the fault_wkc_ payload last_error() reads.
        return fault_.load(std::memory_order_acquire);
    }
    int working_counter() const noexcept {
        return working_counter_.load(std::memory_order_relaxed);
    }
    // Raw working counter from the last exchange(), good or bad, unlike working_counter(),
    // which holds the last good value so a transient does not flap the reported WKC. Use this
    // to see per-cycle short WKC.
    int last_wkc() const noexcept {
        return last_wkc_.load(std::memory_order_relaxed);
    }
    // Snapshot of the cyclic WKC health counters. The fields are individually relaxed, so a
    // reader may observe total and bad inconsistent by +/-1 cycle. That is by design and fine
    // for diagnostics; do not add a lock. Read on cold paths against counters the RT loop bumps
    // every cycle.
    WkcStats wkc_stats() const noexcept {
        return WkcStats{expected_wkc_,
                        last_wkc_.load(std::memory_order_relaxed),
                        total_cycles_.load(std::memory_order_relaxed),
                        bad_cycles_.load(std::memory_order_relaxed)};
    }
    int expected_wkc() const noexcept {
        return expected_wkc_;
    }
    // Read-only config facts the Runner derives its pacing from: the loop rate (period =
    // 1e9/rate) and whether DC/SYNC0 is in play (set in configure()).
    std::uint32_t loop_rate_hz() const noexcept {
        return config_.target_loop_rate_hz;
    }
    bool dc_enabled() const noexcept {
        return dc_enabled_;
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

    // --- typed PDO field resolution (#30 §5) --------------------------------

    // Resolve a Field<> to its byte location in the slave's command (rx) or feedback (tx)
    // image. Templated so it knows sizeof(F::type): throws PdoMappingError if the object is not
    // mapped, or Error if sizeof(F::type)*8 != the mapped object's bit_length (a wrong-width
    // access, e.g. an int16 alias on a 32-bit-mapped object, which would otherwise read the
    // wrong byte count silently). The RT path calls this once at configure to cache a
    // FieldLocation; the noexcept load_le/store_le(image, loc) free functions then run per cycle.
    template <class F>
    FieldLocation resolve_rx(std::uint16_t slave) const {
        return resolve_field(runtime_for(slave).rx_fields, F::index, F::sub, sizeof(typename F::type), slave, /*is_tx=*/false);
    }
    template <class F>
    FieldLocation resolve_tx(std::uint16_t slave) const {
        return resolve_field(runtime_for(slave).tx_fields, F::index, F::sub, sizeof(typename F::type), slave, /*is_tx=*/true);
    }
    // Optional resolve, for a field a consumer maps only in some modes (0x60FF is absent in a
    // PP-only map, 0x607A in a PV-only map). Returns nullopt when the object is not in the map
    // instead of throwing; the caller stores the std::optional and guards its per-cycle
    // load/store on it (deref with *loc -- never .value(), which can throw, on the RT path).
    // A mapped-but-wrong-width object still throws Error. Configure-time only.
    template <class F>
    std::optional<FieldLocation> try_resolve_rx(std::uint16_t slave) const {
        try {
            return resolve_rx<F>(slave);
        } catch (const PdoMappingError&) {
            return std::nullopt;
        }
    }
    template <class F>
    std::optional<FieldLocation> try_resolve_tx(std::uint16_t slave) const {
        try {
            return resolve_tx<F>(slave);
        } catch (const PdoMappingError&) {
            return std::nullopt;
        }
    }

    // --- public SDO primitive ------------------------------------------------------------
    // A CoE object read/write via the backend's blocking mailbox transfer, run on the caller's
    // thread. Vendor policy (a fault-reset and the like) belongs to consumers, and this is safe
    // to call concurrently with a running RT PDO loop: SOEM v2's port is thread-safe (per-index
    // frame buffers plus PRIO_INHERIT getindex/tx/rx mutexes in nicdrv), and the mailbox
    // SyncManager is distinct from the PDO SM, so a one-shot SDO from a non-RT thread neither
    // corrupts nor (being one-shot, not a tight poll) starves the cyclic LRW. The backend's
    // error tiers pass through (SdoError on a CoE abort, Error on a bad slave id); sdo_read
    // returns the number of bytes read into `out`.
    void sdo_write(std::uint16_t slave, std::uint16_t index, std::uint8_t sub, std::span<const std::byte> data);
    std::size_t sdo_read(std::uint16_t slave, std::uint16_t index, std::uint8_t sub, std::span<std::byte> out);

   private:
    // Internal per-object mapping record: byte offset plus mapped width in bits. The public
    // FieldLocation is offset-only; the width lives here so resolve_field's configure-time
    // assert can check sizeof(F::type)*8 == bit_length without leaking width onto the RT-cached
    // location. build_field_table populates it; resolve_field / rx_field / tx_field convert it
    // to an offset-only FieldLocation.
    struct MappedField {
        std::size_t byte_offset = 0;
        std::uint16_t bit_length = 0;
    };

    // Per-slave runtime state. Holds a (non-movable) PdoCache, so it lives in a
    // std::deque (stable addresses, never moved) rather than a vector.
    struct SlaveRuntime {
        SlaveRuntime(std::uint16_t id, std::size_t rx_feedback_bytes) : slave_id(id), cache(rx_feedback_bytes) {}
        std::uint16_t slave_id;
        SlaveIo io;                                      // spans into backend storage (valid after map_process_data)
        std::map<std::uint32_t, MappedField> rx_fields;  // command image (RxPDO/outputs)
        std::map<std::uint32_t, MappedField> tx_fields;  // feedback image (TxPDO/inputs)
        PdoCache cache;                                  // rx snapshot = FEEDBACK (TxPDO); command image is written directly via outputs()
    };

    SlaveRuntime& runtime_for(std::uint16_t slave);
    const SlaveRuntime& runtime_for(std::uint16_t slave) const;

    static std::map<std::uint32_t, MappedField> build_field_table(std::uint16_t slave, const PdoMap& map);

    // Shared resolution body for resolve_rx/resolve_tx: look the object up in `table`, throwing
    // PdoMappingError if it is not mapped or Error if its mapped bit_length/8 != want_width (the
    // templated callers pass sizeof(F::type) so the width-vs-T check happens at resolve).
    // Returns an offset-only FieldLocation.
    static FieldLocation resolve_field(const std::map<std::uint32_t, MappedField>& table,
                                       std::uint16_t index,
                                       std::uint8_t sub,
                                       std::size_t want_width,
                                       std::uint16_t slave,
                                       bool is_tx);

    // Internal phases of the bring-up state machine (bringup_step). SYNC0 is armed in
    // configure() (PRE-OP). Settle pumps phase-locked PD a bounded settle -- it is not gated on
    // the drive's no-sync fault, which is the normal pre-sync state in SAFE-OP and clears only
    // at OP -- then requests OP once, and AwaitOp holds for OP reached, sync fault cleared, and
    // WKC holding, or aborts (no re-request) if that does not happen within a window.
    enum class BringupPhase : std::uint8_t { Settle, AwaitOp, Done, Aborted };
    BringupPhase bringup_phase_ = BringupPhase::Settle;  // RT-only
    std::uint32_t bringup_settle_count_ = 0;             // RT-only: settle cycles elapsed before requesting OP
    std::uint32_t bringup_await_count_ = 0;              // RT-only: AWAIT_OP cycles since requesting OP
    std::uint32_t bringup_op_hold_streak_ = 0;           // RT-only: consecutive (full-WKC && !sync-faulted) cycles at OP
    std::uint16_t bringup_al_code_ = 0;                  // RT-written: last non-zero AL status code seen during AWAIT_OP
    // AWAIT_OP bounds derived once from MasterConfig in the ctor: pre-clamped cycle counts the
    // bring-up FSM compares against (the counts clamp 0 -> 1; the give-up bound is
    // op_await_timeout_ms converted at target_loop_rate_hz, so the patience is rate-independent).
    std::uint32_t op_hold_confirm_cycles_ = 5;
    std::uint32_t op_nudge_interval_cycles_ = 10;
    std::uint32_t op_await_bound_cycles_ = 30'000;
    bool dc_enabled_ = false;  // set in configure(): is SYNC0 in play? (gates the post-OP settle grace)

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
    std::atomic<std::uint64_t> total_cycles_{0};  // WkcStats: process() cycles since configure()
    std::atomic<std::uint64_t> bad_cycles_{0};    // WkcStats: short/abnormal-WKC cycles
    // sdo_read/sdo_write run the mailbox transfer directly on the caller's thread,
    // concurrency-safe against the RT PDO loop via SOEM v2's thread-safe port (see the public
    // sdo_read/sdo_write doc).
};

}  // namespace ethercat
