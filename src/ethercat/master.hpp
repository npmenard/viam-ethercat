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

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
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

// Byte location of a mapped PDO object within a process-data image (#30 §5).
// OFFSET-ONLY: every access derives its width from the Field's T (sizeof) at the
// call site (the load_le/store_le free fns below), so the location carries no
// width. The per-object mapped width (bit_length) lives in
// the Master's INTERNAL field table (Master::MappedField), where the
// configure-time width-assert in resolve_field verifies sizeof(F::type)*8 ==
// bit_length -- it never needs to ride along on the RT-cached location.
// `present` distinguishes a resolved location from a default-constructed
// "not mapped" sentinel (the optional-feedback fields); mapped() is the read.
struct FieldLocation {
    std::size_t byte_offset = 0;
    bool present = false;  // false for a default-constructed (unmapped) sentinel
    // True once resolved by Master::resolve_rx/resolve_tx / rx_field / tx_field.
    // Optional fields cache a default FieldLocation{} when absent -> mapped()==false.
    [[nodiscard]] constexpr bool mapped() const noexcept {
        return present;
    }
};

// RT cached-offset accessors (#30 §4): read/write sizeof(T) little-endian at a resolved
// FieldLocation on a live process image. noexcept + NO bounds-check -- PRECONDITION: `loc`
// came from Master::resolve_rx/resolve_tx<F>() (width-asserted vs the mapped bit_length +
// in-image) at configure, so offset + sizeof(T) is valid. This is the RT hot-path form
// (cached loc + these free fns): resolve/bounds-check happens ONCE at configure, so the
// 1 kHz path physically can't throw/resolve/alloc. (Overloads the span forms in pdo_buffer.hpp.)
template <PdoScalar T>
T load_le(std::span<const std::byte> image, FieldLocation loc) noexcept {
    return load_le<T>(image.subspan(loc.byte_offset, sizeof(T)));
}
template <PdoScalar T>
void store_le(std::span<std::byte> image, FieldLocation loc, T value) noexcept {
    store_le<T>(image.subspan(loc.byte_offset, sizeof(T)), value);
}

// Cyclic WKC health counters (#40 item 4), published RT -> non-RT. Consolidates the
// per-tool tallying (a6_validate's raw/badWKC counters) into the library.
struct WkcStats {
    int expected = 0;                // the bus's full working counter (constant post-configure)
    int last = 0;                    // raw WKC of the most recent exchange (good or bad)
    std::uint64_t total_cycles = 0;  // process() cycles since configure()
    std::uint64_t bad_cycles = 0;    // cycles whose WKC was short/abnormal
};

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

    // PRE-OP -> apply the PDO remap per slave -> map the process image -> size the
    // PdoCaches + build the flat field tables -> [DC: configdc (PRE-OP)] -> SAFE-OP ->
    // [fault_reset: clear a latent drive fault via the vendor SDO]. Re-applies the map
    // every call (A6 map isn't in EEPROM). Throws Error/PdoMappingError naming the
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
    //
    // `drive_present` (#71/#25): a caller read of whether the drive's feedback looks PLAUSIBLE/alive
    // (e.g. statusword != 0). It is an ADDITIONAL OP-confirm gate beyond the working counter -- a
    // DC-only A6 under free-run passes the WKC gate (full WKC) while ZOMBIE-PDOing (dead statusword),
    // so WKC alone wrongly declares OP; requiring drive_present makes bring-up give up (BringupAborted)
    // on a dead drive instead. Defaults true (WKC-only, the old behavior) so single-signal callers and
    // tests are unaffected; the Runner passes the controls' drive_present() AND.
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
    // Identity + image sizes for a slave (1-based), from enumeration -- the library's
    // typed view of the bus, so tools/callers read identity through the API instead of
    // poking CoE 0x1018 directly. Valid after init(). Throws Error if out of range.
    SlaveInfo slave_info(std::uint16_t slave) const {
        return backend_->slave_info(slave);
    }
    // #71: the ESC AL status code / SOEM message for a slave (1-based) -- the standard EtherCAT
    // "why the drive refused an AL transition" (e.g. 0x0027 "Freerun not supported" on a DC-only
    // drive requested into OP without SYNC0). A cached read (no port I/O), so a consumer can read
    // it at a bring-up give-up to name the cause. 0/empty = no error. Delegates to the backend.
    std::uint16_t al_status_code(std::uint16_t slave) const noexcept {
        return backend_->al_status_code(slave);
    }
    // #71/#25: the last NON-ZERO ESC AL status code observed during AWAIT_OP, latched across the
    // whole bring-up. At a give-up the LIVE al_status_code(slave) can read 0 (reack_op ACKs the
    // SAFE_OP+ERROR on the very cycle we time out), so a consumer surfacing "why bring-up failed"
    // must read THIS to reliably name the cause (e.g. 0x0027 on a DC-only drive under free-run).
    // 0 = no AL error was seen the whole bring-up. RT-written, plain read after the RT thread joins.
    std::uint16_t bringup_al_code() const noexcept {
        return bringup_al_code_;
    }
    // Human-readable text for an arbitrary AL status code (delegates to the backend / SOEM's
    // ec_ALstatuscode2string) -- lets a consumer describe the LATCHED bringup_al_code(), not just
    // the live per-slave one. Non-RT (allocates); call it at the give-up, off the RT path.
    std::string describe_al_code(std::uint16_t code) const {
        return backend_->describe_al_code(code);
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
    // Snapshot of the cyclic WKC health counters (#40). FIELDS ARE INDIVIDUALLY RELAXED:
    // a reader may observe total/bad mutually inconsistent by +/-1 cycle. That is FINE
    // for diagnostics and BY DESIGN -- do NOT "fix" it with a lock later (this is read
    // on cold paths against counters the RT loop bumps every cycle).
    WkcStats wkc_stats() const noexcept {
        return WkcStats{expected_wkc_,
                        last_wkc_.load(std::memory_order_relaxed),
                        total_cycles_.load(std::memory_order_relaxed),
                        bad_cycles_.load(std::memory_order_relaxed)};
    }
    int expected_wkc() const noexcept {
        return expected_wkc_;
    }
    // #47: read-only config facts the Runner derives its pacing from -- the loop rate
    // (period = 1e9/rate) and whether DC/SYNC0 is in play (set in configure()).
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

    // Resolve a Field<> to its byte location in the slave's command (rx) / feedback (tx) image
    // (#30 §5). Templated so it knows sizeof(F::type): clear-text throws on the throw-tier split
    // -- PdoMappingError if the object isn't mapped (map-membership), Error if
    // sizeof(F::type)*8 != the mapped object's bit_length (a malformed/wrong-width access, e.g.
    // an int16 alias on a 32-bit-mapped object -- catches a silent wrong-width read). The RT path
    // calls this ONCE at configure to cache a FieldLocation (then the noexcept load_le/store_le(
    // image, loc) free fns run per cycle -- no per-cycle resolve).
    template <class F>
    FieldLocation resolve_rx(std::uint16_t slave) const {
        return resolve_field(runtime_for(slave).rx_fields, F::index, F::sub, sizeof(typename F::type), slave, /*is_tx=*/false);
    }
    template <class F>
    FieldLocation resolve_tx(std::uint16_t slave) const {
        return resolve_field(runtime_for(slave).tx_fields, F::index, F::sub, sizeof(typename F::type), slave, /*is_tx=*/true);
    }
    // OPTIONAL resolve (a field a GENERIC consumer maps only in some modes -- e.g. 0x60FF is
    // absent in a PP-only map, 0x607A in a PV-only map). Returns an UNMAPPED FieldLocation
    // (mapped()==false) when the object isn't in the map, instead of throwing -- the caller
    // guards its per-cycle load/store on mapped(). A mapped-but-WRONG-WIDTH object still throws
    // Error (a real misconfig, never silently tolerated). Configure-time only.
    template <class F>
    FieldLocation resolve_rx_optional(std::uint16_t slave) const {
        try {
            return resolve_rx<F>(slave);
        } catch (const PdoMappingError&) {
            return FieldLocation{};
        }
    }
    template <class F>
    FieldLocation resolve_tx_optional(std::uint16_t slave) const {
        try {
            return resolve_tx<F>(slave);
        } catch (const PdoMappingError&) {
            return FieldLocation{};
        }
    }

    // --- public SDO primitive (#39 vendor policy is consumer-side; #15 concurrent-safe) ------
    // A CoE object read/write via the backend's blocking mailbox transfer, run on the CALLER's
    // thread. #39 exposed this (vendor fault-reset etc. belong to consumers); #15 made it safe to
    // call CONCURRENTLY with a running RT PDO loop and retired the old RT-serviced marshaling queue.
    //
    // CONCURRENCY (#15): SOEM v2's port IS thread-safe -- per-index frame buffers + PRIO_INHERIT
    // getindex/tx/rx mutexes (nicdrv) -- and the mailbox SyncManager is distinct from the PDO SM, so
    // a one-shot SDO from a non-RT thread neither corrupts nor (being one-shot, not a tight poll)
    // starves the cyclic LRW. This reverses the pre-#15 contract ("single port owner only"; SOEM v1
    // was not thread-safe). The backend's error tiers (SdoError on a CoE abort, Error on a bad
    // slave id) pass through; sdo_read returns the number of bytes read into `out`.
    void sdo_write(std::uint16_t slave, std::uint16_t index, std::uint8_t sub, std::span<const std::byte> data);
    std::size_t sdo_read(std::uint16_t slave, std::uint16_t index, std::uint8_t sub, std::span<std::byte> out);

   private:
    // INTERNAL per-object mapping record (#30 §5): byte offset + mapped width in bits.
    // The public FieldLocation is offset-only; the width lives HERE so resolve_field's
    // configure-time assert can check sizeof(F::type)*8 == bit_length without leaking
    // width onto the RT-cached location. build_field_table populates it; resolve_field /
    // rx_field / tx_field convert it to an offset-only FieldLocation (present=true).
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

    // Shared resolution body for resolve_rx/resolve_tx (#30 §5): look the object up in `table`,
    // clear-text throw PdoMappingError if it isn't mapped (map-membership) or Error if
    // its mapped bit_length/8 != want_width (malformed access; the templated callers pass
    // sizeof(F::type) so the width-vs-T check happens at resolve). Returns an offset-only
    // FieldLocation (present=true).
    FieldLocation resolve_field(const std::map<std::uint32_t, MappedField>& table,
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
    std::uint16_t bringup_al_code_ = 0;                  // RT-written: last non-zero AL status code seen during AWAIT_OP (#71/#25)
    // AWAIT_OP bounds derived ONCE from MasterConfig in the ctor (#42): pre-clamped cycle
    // counts the bring-up FSM compares against (the counts clamp 0->1; the give-up bound
    // is op_await_timeout_ms converted at target_loop_rate_hz -- rate-independent patience).
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
    std::atomic<std::uint64_t> total_cycles_{0};  // #40 WkcStats: process() cycles since configure()
    std::atomic<std::uint64_t> bad_cycles_{0};    // #40 WkcStats: short/abnormal-WKC cycles
    // #15: the RT-phase gate (rt_active_) and the #22 RT-serviced SDO request slot are GONE --
    // sdo_read/sdo_write now run the mailbox transfer directly on the caller's thread, concurrency-safe
    // against the RT PDO loop via SOEM v2's thread-safe port (see the public sdo_read/sdo_write doc).
};

}  // namespace ethercat
