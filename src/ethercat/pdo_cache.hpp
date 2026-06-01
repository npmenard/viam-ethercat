#pragma once

// The RT <-> non-RT boundary for one EtherCAT servo. Three independent
// primitives, each owning exactly one direction of sharing:
//
//   1. RxSnapshot   -- RT writes the latest input image; non-RT reads it.
//                      SEQLOCK (not a triple-buffer): native multi-reader, no
//                      slot-claim bookkeeping. TSan-clean because every shared
//                      byte/word is touched through std::atomic_ref.
//   2. CommandQueue -- non-RT enqueues commands; RT drains + coalesces them.
//                      boost::lockfree::queue (MPMC), fixed capacity, pimpl'd.
//   3. TxStaging    -- non-RT stages an output image; RT takes it. LOCK-FREE
//                      3-slot announce/re-validate handoff (no RT mutex). This
//                      is the advanced/raw path -- the common path has the RT
//                      loop build outputs directly.
//
// RT rules honored: no heap allocation after construction, no exceptions across
// the boundary, no map walks, and the RT side never blocks (snapshot writer is
// wait-free; Tx take and command drain are lock-free).

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <type_traits>
#include <variant>

namespace ethercat {

// Upper bound on a single PDO image (Rx or Tx) in bytes. The A6-EC freely
// mappable RPDO is <= 40 B; 512 leaves generous headroom for other EtherCAT
// slaves while keeping snapshots trivially copyable and cheap to pass by value.
inline constexpr std::size_t kMaxPdoBytes = 512;

// ----------------------------------------------------------------------------
// (1) RxPDO snapshot -- seqlock
// ----------------------------------------------------------------------------

// A torn-free copy of the latest input image plus its metadata, returned by
// value from the non-RT reader.
struct PdoSnapshot {
    std::array<std::byte, kMaxPdoBytes> bytes{};
    std::size_t size = 0;               // valid prefix of bytes
    std::uint16_t working_counter = 0;  // EtherCAT WKC at publish time
    std::uint64_t cycle = 0;            // RT cycle counter at publish time
    bool valid = false;                 // this read succeeded (was not retry-exhausted)
    bool stale = false;                 // == !valid: the read retry-exhausted. NOTE: the
                                        // cross-cycle "RT loop is dead" staleness that drives
                                        // is_powered()/is_moving()=false is computed by
                                        // ServoController (Phase 5), not here.
};

// Single-writer (RT) / multi-reader (non-RT) seqlock.
class RxSnapshot {
   public:
    explicit RxSnapshot(std::size_t payload_size) noexcept;

    // RT writer, wait-free. `payload.size()` should equal payload_size; the copy
    // is clamped to size_ defensively (a wrong-sized span on this noexcept path
    // must never overrun the fixed array).
    void publish(std::span<const std::byte> payload, std::uint16_t working_counter, std::uint64_t cycle) noexcept;

    // Non-RT reader. Retries up to kMaxReadRetries on a torn read; on exhaustion
    // returns {valid=false, stale=true} and the consumer keeps its own last-good.
    PdoSnapshot read() const noexcept;

    // Test-only seam: force the "write in progress" (odd seq) state so a test can
    // exercise the reader's bounded-retry -> valid=false path. Not used in
    // production. Safe even though it can leave seq at an arbitrary parity:
    // publish() is parity-ROBUST (it computes an explicitly-even final seq
    // regardless of entry parity), so a subsequent publish always restores the
    // stable/even invariant.
    void force_writing_for_test() noexcept {
        seq_.fetch_add(1, std::memory_order_relaxed);
    }

   private:
    static constexpr unsigned kMaxReadRetries = 16;

    alignas(64) std::atomic<std::uint64_t> seq_{0};  // odd = write in progress, even = stable
    std::size_t size_;
    // Payload + metadata: all accessed via std::atomic_ref under the seqlock so
    // there are no non-atomic concurrent accesses (TSan-clean, zero suppressions).
    // mutable so the const reader can form atomic_ref over them (a const lvalue
    // cannot bind to atomic_ref).
    alignas(8) mutable std::uint64_t cycle_ = 0;
    mutable std::array<unsigned char, kMaxPdoBytes> bytes_{};
    mutable std::uint16_t wkc_ = 0;
};

// ----------------------------------------------------------------------------
// (2) Command queue -- non-RT producers, RT consumer
// ----------------------------------------------------------------------------

// Setpoint commands carry data; discrete commands are empty tags.
struct SetTarget {
    std::int32_t counts = 0;             // absolute or relative target, in counts
    std::uint32_t profile_velocity = 0;  // PP profile velocity
    bool relative = false;
};
struct SetVelocity {
    std::int32_t velocity = 0;  // PV target velocity, device units
};
struct Enable {};
struct Disable {};
struct Halt {};
struct QuickStop {};
struct FaultReset {};
struct SetZero {};

using Command = std::variant<SetTarget, SetVelocity, Enable, Disable, Halt, QuickStop, FaultReset, SetZero>;
static_assert(std::is_trivially_copyable_v<Command>, "Command must be trivially copyable for the lock-free queue");
// boost::lockfree::queue additionally requires a trivial destructor; assert it
// so a future non-trivial Command alternative fails at compile, not at link.
static_assert(std::is_trivially_destructible_v<Command>, "Command must be trivially destructible for the lock-free queue");

// Coalesced result of draining the queue for one RT cycle. Setpoints are
// latest-wins; discrete commands are sticky booleans (any occurrence latches).
struct CommandBatch {
    std::optional<SetTarget> set_target;
    std::optional<SetVelocity> set_velocity;
    bool enable = false;
    bool disable = false;
    bool halt = false;
    bool quick_stop = false;
    bool fault_reset = false;
    bool set_zero = false;

    bool any() const noexcept {
        return set_target.has_value() || set_velocity.has_value() || enable || disable || halt || quick_stop || fault_reset || set_zero;
    }
};

// MPSC command queue, capacity fixed at construction (no RT allocation).
// Implemented as boost::lockfree::spsc_queue (a TSan-clean atomic head/tail
// ring) plus a producer-side mutex that serializes the multiple non-RT Viam
// handler threads into the single-producer ring. The single RT consumer pops
// lock-free and NEVER takes the mutex -> zero priority inversion. Move-only;
// boost is hidden behind the pimpl.
//
// THREADING CONTRACT: push() is non-RT (gRPC handlers) and takes the mutex;
// drain() is the RT consumer and is lock-free. The RT thread must never call
// push().
class CommandQueue {
   public:
    explicit CommandQueue(std::size_t capacity);
    ~CommandQueue();
    CommandQueue(CommandQueue&&) noexcept;
    CommandQueue& operator=(CommandQueue&&) noexcept;
    CommandQueue(const CommandQueue&) = delete;
    CommandQueue& operator=(const CommandQueue&) = delete;

    // Non-RT producer (gRPC handler threads). Serializes on the producer mutex.
    // Returns false if the queue is full (setpoints coalesce anyway, so a drop
    // is benign; the caller may still report it).
    bool push(const Command& command) noexcept;

    // RT consumer. Drains everything currently queued and coalesces it. Lock-free
    // (never takes the producer mutex); never allocates, never blocks.
    CommandBatch drain() noexcept;

   private:
    struct Impl;  // hides boost::lockfree::spsc_queue from this header
    std::unique_ptr<Impl> impl_;
};

// ----------------------------------------------------------------------------
// (3) TxPDO staging -- lock-free 3-slot announce/re-validate handoff
// ----------------------------------------------------------------------------

// "Rewritable until sent" output slot, lock-free. The non-RT side stages a
// frame; if the RT side has not yet taken it, a subsequent stage overwrites it
// (newest-wins). The RT side takes the newest unsent frame. Three slots
// guarantee the writer always has a free slot that is neither pending nor being
// read. The announce-FIRST / re-validate protocol (mirrors the seqlock reader)
// is the correctness crux against a torn Tx.
//
// TRIPWIRE: this hazard scheme is married to exactly ONE non-RT writer (the
// advanced/raw staging path) and ONE RT reader. Two concurrent stagers would
// break the 3-slot/hazard invariant. There is no way to static_assert this, so
// stage_outputs has a debug-only reentrancy guard. If an advanced path ever
// needs multiple stagers, wrap stage_outputs in a producer-side mutex the RT
// thread never touches.
//
// All pending_/reading_ accesses are seq_cst on BOTH sides: the announce/
// validate handshake is a Dekker StoreLoad, and release/acquire is insufficient
// for StoreLoad ordering (it reorders on arm64). Slot payload bytes do NOT need
// atomic_ref -- the pending_ release/acquire edge plus the mutual-exclusion
// invariant give a clean happens-before, so a plain memcpy of the slot is
// race-free.
class TxStaging {
   public:
    explicit TxStaging(std::size_t payload_size);

    // Non-RT. Overwrite-if-unsent (newest-wins). Copy clamped to size_.
    void stage_outputs(std::span<const std::byte> payload) noexcept;

    // RT. If a staged frame is pending, copy it into `out` and return the number
    // of bytes written (== size_); returns 0 if nothing new is pending.
    std::size_t take_outputs(std::span<std::byte> out) noexcept;

   private:
    static constexpr std::uint32_t kNone = 0xFFFFFFFFU;  // distinct 4th sentinel (slots are 0/1/2)
    // Cap on the re-validate loop. This is a FRESHNESS bound, not a safety bound:
    // copying a slightly-stale idx is still safe (reading_==idx is held), so the
    // cap just stops a pathological concurrent stager from livelocking the RT
    // thread.
    static constexpr unsigned kMaxTakeSpins = 8;

    // Atomics first (the alignas(64) anchor) so the large slots_ array does not
    // wedge padding between fields.
    alignas(64) std::atomic<std::uint32_t> pending_{kNone};  // newest unsent slot, or kNone
    std::atomic<std::uint32_t> reading_{kNone};              // slot the RT reader is copying (hazard cell)
    std::size_t size_;
    std::array<std::array<std::byte, kMaxPdoBytes>, 3> slots_{};
    // Debug-only single-writer tripwire (see TRIPWIRE above). [[maybe_unused]]
    // because the guard compiles out under NDEBUG.
    [[maybe_unused]] std::atomic<bool> staging_{false};
};

// ----------------------------------------------------------------------------
// PdoCache -- aggregates the Rx snapshot + Tx staging for one slave, matching
// the plan's publish_inputs/read_inputs/stage_outputs/take_outputs surface. The
// CommandQueue is a separate channel owned alongside this by ServoController.
// ----------------------------------------------------------------------------
class PdoCache {
   public:
    // Validates sizes (non-RT setup, so throwing is correct): throws
    // PdoMappingError if either image exceeds kMaxPdoBytes.
    PdoCache(std::size_t rx_size, std::size_t tx_size);

    void publish_inputs(std::span<const std::byte> payload, std::uint16_t wkc, std::uint64_t cycle) noexcept {
        rx_.publish(payload, wkc, cycle);
    }
    PdoSnapshot read_inputs() const noexcept {
        return rx_.read();
    }
    void stage_outputs(std::span<const std::byte> payload) noexcept {
        tx_.stage_outputs(payload);
    }
    std::size_t take_outputs(std::span<std::byte> out) noexcept {
        return tx_.take_outputs(out);
    }

   private:
    RxSnapshot rx_;
    TxStaging tx_;
};

}  // namespace ethercat
