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
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
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
// call site (the load_le/store_le free fns below, Rpdo::get/Tpdo::put), so the
// location carries no width. The per-object mapped width (bit_length) lives in
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
// (cached loc + these free fns);
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
    // (#30 §5). Templated so it knows sizeof(F::type): clear-text throws on the throw-tier split
    // -- PdoMappingError if the object isn't mapped (map-membership), PdoAccessError if
    // sizeof(F::type)*8 != the mapped object's bit_length (a malformed/wrong-width access, e.g.
    // an int16 alias on a 32-bit-mapped object -- catches a silent wrong-width read). This is the
    // resolution used by Rpdo/Tpdo per-call AND the one the RT path calls ONCE at configure to
    // cache a FieldLocation (then the noexcept load_le/store_le(image, loc) free fns run per cycle
    // -- no per-cycle resolve).
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
    // PdoAccessError (a real misconfig, never silently tolerated). Configure-time only.
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

    // --- narrow public SDO primitive (#39: vendor POLICY is consumer-side) -------------
    // #23 removed the public SDO surface to stop ad-hoc CoE poking; #39 partially reverses
    // that BY USER DIRECTION: vendor fault-reset (and vendor policy generally) belongs to
    // CONSUMERS, which requires a consumer-reachable primitive. The boundary moved from
    // "no public SDO" to "public SDO with an explicit PORT-OWNERSHIP contract":
    //
    // CONTRACT: callable ONLY while the caller is the SINGLE port owner -- pre-RT-spawn
    // (after init()/configure(), before any cyclic thread) or post-RT-join. NEVER
    // concurrently with a running RT loop: SOEM's port is not thread-safe, and a blocking
    // mailbox transfer interleaved with cyclic LRW starves process data (the ec_sample
    // 0x001B SM-watchdog lesson). Steady-state (RT running) SDO access is NOT this
    // primitive -- that is #22's RT-serviced request queue.
    //
    // STRUCTURAL GUARD: Master cannot self-detect RT activity (consumers own their loop
    // threads and call process()), so the enforceable form is the consumer-DECLARED RT
    // phase: bracket set_rt_active(true/false) EXACTLY around the RT thread spawn/join
    // (ServoController does). While declared active, sdo_read/sdo_write THROW ConfigError
    // (a release-mode throw, not a debug assert -- the module ships release). For external
    // library users the flag is part of the documented contract; the doc-contract stays
    // primary. Single-threaded tools (a6_validate) never set it.
    //
    // It ALSO opens/closes the #22 steady-state SDO SERVICER WINDOW (below): set_rt_active(true)
    // means "the RT loop will service marshaled SDO requests"; set_rt_active(false) (after the
    // join) FAILS any in-flight request + wakes every waiter cleanly (no hang across stop()).
    void set_rt_active(bool active) noexcept;
    // Write/read one CoE object via the backend (blocking mailbox transfer). Throws
    // ConfigError while a consumer-declared RT phase is active (the guard above); the
    // backend's own error tiers (SdoError on a CoE abort, ConfigError on a bad slave id)
    // pass through unchanged. sdo_read returns the number of bytes read into `out`.
    void sdo_write(std::uint16_t slave, std::uint16_t index, std::uint8_t sub, std::span<const std::byte> data);
    std::size_t sdo_read(std::uint16_t slave, std::uint16_t index, std::uint8_t sub, std::span<std::byte> out);

    // --- #22 STEADY-STATE SDO: RT-serviced request queue -----------------------------
    // The complement to the guarded pre-/post-RT primitive above: read a CoE object WHILE
    // the RT loop is running, WITHOUT a second thread ever driving SOEM's (non-thread-safe)
    // port. A non-RT caller MARSHALS the request to the RT thread, which executes exactly
    // ONE blocking mailbox transfer per cycle at a designated point (Runner steady loop ->
    // service_sdo()) and posts the result back. The single-port-owner invariant holds by
    // construction; the cost is a bounded, occasional PD gap on the servicing cycle (the
    // mailbox round-trip, ~1-2 ms), absorbed by the pacer's phase-preserving catch-up and
    // tolerated by the drive's SM watchdog (>=50 ms) -- HW-verified (#22).
    //
    // Single request in flight (a submit mutex serializes concurrent callers). Blocks up to
    // `timeout` for the RT thread to service it; returns the byte count read into `out`.
    // Throws ConfigError if no RT servicer is running (call this only while operational;
    // pre-/post-RT use the guarded sdo_read above), SdoError on a CoE abort or on timeout.
    std::size_t sdo_read_deferred(
        std::uint16_t slave, std::uint16_t index, std::uint8_t sub, std::span<std::byte> out, std::chrono::milliseconds timeout);
    // RT hot path: service AT MOST ONE pending SDO request, else return immediately. Called
    // ONCE per steady cycle by the Runner (the single port owner). noexcept: a CoE abort is
    // captured into the request's result, never thrown across the RT boundary. The idle path
    // is a single acquire-load of an atomic -- no lock, no alloc when nothing is pending.
    void service_sdo() noexcept;

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
        SlaveRuntime(std::uint16_t id, std::size_t rx_feedback_bytes, std::size_t tx_command_bytes)
            : slave_id(id), cache(rx_feedback_bytes, tx_command_bytes) {}
        std::uint16_t slave_id;
        SlaveIo io;                                      // spans into backend storage (valid after map_process_data)
        std::map<std::uint32_t, MappedField> rx_fields;  // command image (RxPDO/outputs)
        std::map<std::uint32_t, MappedField> tx_fields;  // feedback image (TxPDO/inputs)
        PdoCache cache;                                  // NOTE: rx snapshot = FEEDBACK (TxPDO), tx staging = COMMAND (RxPDO)
    };

    SlaveRuntime& runtime_for(std::uint16_t slave);
    const SlaveRuntime& runtime_for(std::uint16_t slave) const;

    static std::map<std::uint32_t, MappedField> build_field_table(std::uint16_t slave, const PdoMap& map);

    // Shared resolution body for resolve_rx/resolve_tx (#30 §5): look the object up in `table`,
    // clear-text throw PdoMappingError if it isn't mapped (map-membership) or PdoAccessError if
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
    // Consumer-declared RT phase (#39): while true, the public sdo_read/sdo_write throw
    // (port-ownership guard). Set/cleared by the consumer around its RT thread spawn/join.
    std::atomic<bool> rt_active_{false};

    // --- #22 steady-state SDO request slot (single in-flight) ------------------------
    // The marshaling handoff between a non-RT caller (sdo_read_deferred) and the RT thread
    // (service_sdo). sdo_pending_ is the RT hot-path fast check (acquire-load; skip the lock
    // entirely when Idle). Everything else is under sdo_mtx_ -- taken only off the idle path
    // (a submit, a completion, or the bounded jitter window of the actual transfer), so the
    // 1 kHz idle cycle never locks/allocs.
    static constexpr std::size_t kMaxSdoReadBytes = 64;  // these objects are <=4 B; 64 is ample headroom
    enum class SdoPhase : std::uint8_t { Idle, Requested, Done };
    struct SdoJob {
        std::uint16_t slave = 0;
        std::uint16_t index = 0;
        std::uint8_t sub = 0;
        std::size_t want = 0;  // bytes requested (out.size())
        std::size_t got = 0;   // bytes actually read
        bool ok = false;       // false => `error` holds the reason
        std::string error;     // set by the RT servicer on abort (jitter-window alloc, never the idle path)
        std::array<std::byte, kMaxSdoReadBytes>
            buf{};  // RT reads INTO here; the waiter copies OUT (caller buffer lifetime is irrelevant to RT)
    };
    std::mutex sdo_mtx_;
    std::condition_variable sdo_cv_;
    std::atomic<bool> sdo_pending_{false};  // RT fast path: is there a Requested job to service?
    bool sdo_service_open_ = false;         // guarded by sdo_mtx_: an RT loop is running to service (mirrors rt_active_)
    SdoPhase sdo_phase_ = SdoPhase::Idle;   // guarded by sdo_mtx_
    SdoJob sdo_job_;                        // guarded by sdo_mtx_ (RT holds the lock across its transfer)
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
    // object isn't mapped (map-membership), or PdoAccessError if the width disagrees
    // with the mapping or the read runs past the frame (malformed access).
    template <class F>
    typename F::type get() const {
        // resolve_tx: PdoMappingError on not-in-map, PdoAccessError on width-mismatch (#30 §5).
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
    // PdoMappingError if not mapped (map-membership); PdoAccessError on width-mismatch
    // or past the frame (malformed access).
    template <class F>
    void put(typename F::type v) {
        // resolve_rx: PdoMappingError on not-in-map, PdoAccessError on width-mismatch (#30 §5).
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
