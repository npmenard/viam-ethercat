#pragma once

// The RT <-> non-RT boundary for one EtherCAT servo. Two independent
// primitives, each owning exactly one direction of sharing:
//
//   1. RxSnapshot   -- RT writes the latest input image; non-RT reads it.
//                      SEQLOCK (not a triple-buffer): native multi-reader, no
//                      slot-claim bookkeeping. TSan-clean because every shared
//                      byte/word is touched through std::atomic_ref.
//   2. CommandQueue -- non-RT enqueues commands; RT drains + coalesces them.
//                      boost::lockfree::spsc_queue + producer-mutex (MPSC),
//                      fixed capacity, pimpl'd.
//
// The command image (RxPDO/outputs) is NOT staged here: the RT loop builds it
// directly into the backend's output span.
//
// RT rules honored: no heap allocation after construction, no exceptions across
// the boundary, no map walks, and the RT side never blocks (the snapshot writer
// is wait-free; the command drain is lock-free).

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
    // == !valid: the read retry-exhausted. The cross-cycle "RT loop is dead"
    // staleness that drives is_powered()/is_moving()=false is computed by
    // ServoController (Phase 5), not here.
    bool stale = false;

    // A real frame has been published at least once. `cycle == 0` is the
    // pre-publish stable-all-zero frame (which still reads valid=true, since
    // `valid` means "clean read", not "has data"). Consumers should treat a
    // non-live snapshot as the fail-safe bucket (is_powered/is_moving = false).
    bool is_live() const noexcept {
        return cycle > 0;
    }
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
    // Monotonic move id assigned by the non-RT caller. The RT loop ADOPTS this
    // into its active_generation after drain() coalescing, so "which move is
    // active" always matches the command actually applied (a superseded move's
    // waiter wakes via active_generation > its gen). See ServoController.
    std::uint32_t generation = 0;
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
    // #70 ORDER-PRESERVING sticky-Halt disposition: when a Halt and a new motion command
    // (SetTarget/SetVelocity) coalesce into one drain, the STICKY halt must stick only if the
    // Halt was the LATEST of the two -- a motion issued AFTER a halt (Stop() then GoTo()) means
    // the caller wants to move, so the halt is superseded. drain() sets this to the disposition
    // of the last stop-relevant command. (A Halt still CANCELS any in-flight move regardless; only
    // whether the sticky-halt LATCHES is order-dependent.) Default false = no superseding halt.
    bool halt_supersedes = false;

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
// THREADING CONTRACT (load-bearing for the SPSC correctness):
//  - push() is non-RT (gRPC handlers) and holds the producer mutex across its
//    ENTIRE body, so two producers never interleave inside the single-producer
//    ring. The RT thread must NEVER call push().
//  - drain() has EXACTLY ONE caller, ever: the single RT consumer. spsc_queue
//    permits only one popper. No non-RT path may drain/pop concurrently with the
//    RT consumer -- any queue drain on reconfigure/shutdown must happen only
//    AFTER the RT thread is joined (or be performed by the RT thread itself). A
//    stray non-RT drain silently violates the SPSC contract.
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
// PdoCache -- wraps the Rx feedback snapshot for one slave, matching the plan's
// publish_inputs/read_inputs surface. The CommandQueue is a separate channel
// owned alongside this by ServoController.
// ----------------------------------------------------------------------------
class PdoCache {
   public:
    // Validates the size (non-RT setup, so throwing is correct): throws
    // PdoMappingError if the feedback image exceeds kMaxPdoBytes.
    explicit PdoCache(std::size_t rx_size);

    void publish_inputs(std::span<const std::byte> payload, std::uint16_t wkc, std::uint64_t cycle) noexcept {
        rx_.publish(payload, wkc, cycle);
    }
    PdoSnapshot read_inputs() const noexcept {
        return rx_.read();
    }

   private:
    RxSnapshot rx_;
};

}  // namespace ethercat
