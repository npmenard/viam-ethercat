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
#include <cstring>
#include <deque>
#include <map>
#include <memory>
#include <span>
#include <string>

#include "ethercat/backend.hpp"
#include "ethercat/errors.hpp"
#include "ethercat/field.hpp"
#include "ethercat/pdo_buffer.hpp"
#include "ethercat/pdo_cache.hpp"
#include "ethercat/pdo_mapping.hpp"

namespace ethercat {

// Byte location of a mapped PDO object within a process-data image.
// NOTE (#30 §5): byte_width is retained for the existing call sites
// (servo_controller's optional-field "byte_width==0 => unmapped" sentinel +
// a6_validate's load/store width). The new Field<>/Rpdo/Tpdo access path takes
// the width from the Field's T (sizeof) and only reads byte_offset, so the
// eventual offset-only drop is deferred to the P2b/P2c call-site migration.
struct FieldLocation {
    std::size_t byte_offset = 0;
    std::size_t byte_width = 0;
};

// RT cached-offset accessors (#30 §4): read/write sizeof(T) little-endian at a resolved
// FieldLocation on a live process image. noexcept + NO bounds-check -- PRECONDITION: `loc`
// came from Master::resolve_rx/resolve_tx<F>() (width-checked + in-image) at configure, so
// offset + sizeof(T) is valid. This is the RT hot-path form (cached loc + these free fns);
// the THROWING resolve/bounds surface lives ONLY on Rpdo/Tpdo (the copy types), so the 1 kHz
// path physically can't throw/resolve/alloc. (Overloads the span forms in pdo_buffer.hpp.)
template <PdoScalar T>
T load_le(std::span<const std::byte> image, FieldLocation loc) noexcept {
    return load_le<T>(image.subspan(loc.byte_offset, sizeof(T)));
}
template <PdoScalar T>
void store_le(std::span<std::byte> image, FieldLocation loc, T value) noexcept {
    store_le<T>(image.subspan(loc.byte_offset, sizeof(T)), value);
}

// PDO access types (#30), defined in full after Master (they hold a Master* and
// call its resolve/cache surface). read_rpdo()/make_tpdo() return them.
class Rpdo;
class Tpdo;

// Status of the DC bring-up state machine (Master::bringup_step). The caller drives
// one step per cyclic exchange until it sees Operational (switch to the steady loop)
// or Aborted (surface the fault; do NOT immediately re-enter bring-up -- repeated
// Er74 OP-entry wedges the A6, CLAUDE.md).
enum class BringupStatus : std::uint8_t {
    Gating,       // SYNC0 armed in configure(); pumping phase-locked PD a bounded SETTLE before requesting OP
    AwaitingOp,   // OP requested once; awaiting "OP reached + Er74.1 cleared + WKC holds"
    Operational,  // all slaves OPERATIONAL, synced (WKC holding) -- bring-up complete
    Aborted,      // OP did not take within the await window (WKC won't hold / Er74.1 didn't clear) -- no re-request
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
    // AFTER configure() (which left the bus at SAFE-OP with SYNC0 already armed in PRE-OP).
    // It performs the cyclic exchange() and advances SETTLE (bounded phase-locked PD) ->
    // request OP once -> AWAIT_OP (hold for OP + sync) -> OPERATIONAL, returning the new
    // status. The CALLER owns the cadence: it does the
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
    // Identity + image sizes for a slave (1-based), from enumeration -- the library's
    // typed view of the bus, so tools/callers read identity through the API instead of
    // poking CoE 0x1018 directly. Valid after init(). Throws ConfigError if out of range.
    SlaveInfo slave_info(std::uint16_t slave) const {
        return backend_->slave_info(slave);
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

    // --- typed PDO access (#30, ergonomic non-RT/bench form) ----------------
    // These are the throwing, per-call-resolve, COPY form (NOT the 1 kHz hot path -- the RT
    // loop keeps cached-offset load_le/store_le). Both resolve the SAME field tables as
    // rx_field/tx_field; the width comes from the Field's T.

    // Immutable, frame-consistent feedback snapshot for a slave (1-based): ONE seqlock read
    // of the PdoCache RxSnapshot, copied by value. Every get<F>() on the returned Rpdo reads
    // the SAME frame (no tearing across fields); call again for a newer frame. Throws
    // ConfigError on an unknown slave.
    Rpdo read_rpdo(std::uint16_t slave) const;

    // Seeded output builder for a slave (1-based): a COPY of the current command image, so
    // fields you don't put() carry over unchanged. put<F>(v) writes into the copy; submit()
    // hands it to TxStaging (transmitted next process()). An UNSUBMITTED Tpdo never touches
    // the bus (drop it = no-op). BENCH/direct-PDO path only (#30 §6: the servo module command
    // path stays CommandQueue+FSM). Throws ConfigError on an unknown slave.
    //
    // CONTRACT (it's a per-cycle transient, single-consumer; no runtime guard -- the doc IS it):
    //  1. Do NOT MIX a direct outputs() write with a Tpdo/submit() on the SAME slave in the SAME
    //     cycle. make_tpdo() snapshots the seed AT CALL TIME, and process() drains the staged
    //     frame OVER the command image, so submit() wins at drain. The order bites:
    //     make_tpdo -> direct outputs() write -> submit() silently overwrites the direct write
    //     with the pre-write seed (make-EARLY is the trap; a direct write BEFORE make_tpdo is
    //     captured into the seed and is fine). Pick one writer per slave per cycle.
    //  2. Do NOT HOLD a Tpdo across cycles. The seed goes stale; a late submit() lands a stale
    //     frame on the bus. Make -> put -> submit all in one cycle, then drop it.
    Tpdo make_tpdo(std::uint16_t slave);

    // Resolve a Field<> to its byte location in the slave's command (rx) / feedback (tx) image
    // (#30 §5). Templated so it knows sizeof(F::type): asserts the Field's width matches the
    // mapped object -- throws PdoAccessError (clear text) if the object isn't mapped OR if
    // sizeof(F::type)*8 != the mapped object's bit_length (catches a silent wrong-width read,
    // e.g. an int16 alias on a 32-bit-mapped object). This is the resolution used by Rpdo/Tpdo
    // per-call AND the one the RT path calls ONCE at configure to cache a FieldLocation (then
    // the noexcept load_le/store_le(image, loc) free fns run per cycle -- no per-cycle resolve).
    template <class F>
    FieldLocation resolve_rx(std::uint16_t slave) const {
        return resolve_field(runtime_for(slave).rx_fields, F::index, F::sub, sizeof(typename F::type), slave, /*is_tx=*/false);
    }
    template <class F>
    FieldLocation resolve_tx(std::uint16_t slave) const {
        return resolve_field(runtime_for(slave).tx_fields, F::index, F::sub, sizeof(typename F::type), slave, /*is_tx=*/true);
    }

    // NOTE: there is intentionally NO public typed CoE SDO accessor on Master. CoE object
    // access is the LIBRARY's responsibility -- Master::configure() does the PDO-remap /
    // 0x6060 / fault-reset writes internally via the backend, and identity is read through
    // slave_info(). Tools/callers must NOT poke raw objects; they consume the typed API
    // (slave_info / configure / bringup_step / process / input_image / outputs). The raw
    // sdo_read/sdo_write live at the EcatBackend level (library-internal) by design.

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

    // Shared resolution body for resolve_rx/resolve_tx (#30 §5): look the object up in `table`,
    // throw PdoAccessError (clear text) if it isn't mapped or its mapped width != want_width
    // (the templated callers pass sizeof(F::type) so the width-vs-T check happens at resolve).
    FieldLocation resolve_field(const std::map<std::uint32_t, FieldLocation>& table,
                                std::uint16_t index,
                                std::uint8_t sub,
                                std::size_t want_width,
                                std::uint16_t slave,
                                bool is_tx) const;

    // Internal phases of the bring-up state machine (bringup_step). SYNC0 is armed in
    // configure() (PRE-OP, per ec_sample). SETTLE pumps phase-locked PD a bounded settle
    // (NOT gated on Er74.1 -- it is the NORMAL pre-sync state in SAFE-OP and clears AT OP,
    // per ec_sample), requests OP ONCE, then AWAIT_OP holds for "OP reached + Er74.1
    // cleared + WKC holds" -- or aborts (no re-request) if that doesn't happen in a window.
    enum class BringupPhase : std::uint8_t { Settle, AwaitOp, Done, Aborted };
    BringupPhase bringup_phase_ = BringupPhase::Settle;  // RT-only
    std::uint32_t bringup_settle_count_ = 0;             // RT-only: SETTLE cycles elapsed before requesting OP
    std::uint32_t bringup_await_count_ = 0;              // RT-only: AWAIT_OP cycles since requesting OP
    std::uint32_t bringup_op_hold_streak_ = 0;           // RT-only: consecutive (full-WKC && !Er74.1) cycles at OP
    bool dc_enabled_ = false;                            // set in configure(): is SYNC0 in play? (gates the post-OP settle grace)

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

// ---------------------------------------------------------------------------
// Rpdo -- immutable, frame-consistent feedback snapshot (#30 §2)
// ---------------------------------------------------------------------------
// Holds a COPY of one feedback frame (the seqlock read) + a back-reference to
// the Master for offset resolution. Every get<F>() reads the SAME copied frame,
// so there is no tearing across fields. NOT a live view (re-call read_rpdo for a
// newer frame). Per-call resolve + bounds-throw -- the ergonomic, non-RT form.
class Rpdo {
   public:
    // Resolve F's index:sub in the slave's TxPDO (feedback) field table and read
    // sizeof(F::type) little-endian at that offset. Throws PdoMappingError if the
    // object isn't mapped, or PdoAccessError if it would read past the frame. The
    // width comes from F::type -- per-field width-vs-T is the caller's contract (§1).
    template <class F>
    typename F::type get() const {
        // resolve_tx throws PdoAccessError on not-in-map OR width-mismatch (#30 §5).
        const FieldLocation loc = master_->template resolve_tx<F>(slave_);
        if (loc.byte_offset + sizeof(typename F::type) > snap_.size) {
            throw PdoAccessError("Rpdo::get object " + std::to_string(F::index) + ":" + std::to_string(F::sub) + " reads " +
                                 std::to_string(sizeof(typename F::type)) + " byte(s) at offset " + std::to_string(loc.byte_offset) +
                                 " past feedback frame size " + std::to_string(snap_.size));
        }
        return load_le<typename F::type>(std::span<const std::byte>(snap_.bytes.data() + loc.byte_offset, sizeof(typename F::type)));
    }

    // The underlying snapshot (WKC / cycle / liveness) behind this view.
    const PdoSnapshot& snapshot() const noexcept {
        return snap_;
    }

   private:
    friend class Master;
    Rpdo(const PdoSnapshot& snap, const Master* master, std::uint16_t slave) noexcept : snap_(snap), master_(master), slave_(slave) {}

    PdoSnapshot snap_;      // the frame-consistent COPY
    const Master* master_;  // non-owning; for offset resolution
    std::uint16_t slave_;
};

// ---------------------------------------------------------------------------
// Tpdo -- seeded write builder, submit-to-transmit (#30 §3)
// ---------------------------------------------------------------------------
// Holds a COPY of the CURRENT command image (the seed), so fields you don't put()
// carry over unchanged. put<F>() writes into the copy; submit() hands it to the
// slave's TxStaging (the RT process() takes it and transmits next cycle). An
// unsubmitted Tpdo never touches the bus -- dropping it is a no-op.
//
// PER-CYCLE TRANSIENT (see make_tpdo): do NOT hold it across cycles (the seed goes
// stale -> a late submit() lands a stale frame), and do NOT mix it with a direct
// outputs() write on the same slave in the same cycle (submit() wins at drain, and
// make_tpdo's seed is snapshotted at call time). Make -> put -> submit -> drop.
class Tpdo {
   public:
    // Resolve F's index:sub in the slave's RxPDO (command) field table and write
    // sizeof(F::type) little-endian at that offset into the staged copy. Throws
    // PdoAccessError if not mapped / width-mismatch / past the frame.
    template <class F>
    void put(typename F::type v) {
        // resolve_rx throws PdoAccessError on not-in-map OR width-mismatch (#30 §5).
        const FieldLocation loc = master_->template resolve_rx<F>(slave_);
        if (loc.byte_offset + sizeof(typename F::type) > size_) {
            throw PdoAccessError("Tpdo::put object " + std::to_string(F::index) + ":" + std::to_string(F::sub) + " writes " +
                                 std::to_string(sizeof(typename F::type)) + " byte(s) at offset " + std::to_string(loc.byte_offset) +
                                 " past command frame size " + std::to_string(size_));
        }
        store_le<typename F::type>(std::span<std::byte>(staged_.data() + loc.byte_offset, sizeof(typename F::type)), v);
    }

    // Hand the staged frame to TxStaging -> the RT process() transmits it next cycle.
    void submit() noexcept {
        cache_->stage_outputs(std::span<const std::byte>(staged_.data(), size_));
        submitted_ = true;
    }

    bool submitted() const noexcept {
        return submitted_;
    }

   private:
    friend class Master;
    Tpdo(std::span<const std::byte> seed, Master* master, PdoCache* cache, std::uint16_t slave) noexcept
        : size_(seed.size()), master_(master), cache_(cache), slave_(slave) {
        if (size_ > staged_.size()) {
            size_ = staged_.size();  // defensive clamp (image sizes are validated <= kMaxPdoBytes at configure)
        }
        std::memcpy(staged_.data(), seed.data(), size_);
    }

    std::array<std::byte, kMaxPdoBytes> staged_{};  // seed copy, mutated by put()
    std::size_t size_;
    Master* master_;   // non-owning; for offset resolution
    PdoCache* cache_;  // non-owning; submit() target
    std::uint16_t slave_;
    bool submitted_ = false;
};

}  // namespace ethercat
