#include "ethercat/pdo_cache.hpp"

#include <algorithm>
#include <cassert>
#include <cstring>
#include <mutex>
#include <string>

#include <boost/lockfree/spsc_queue.hpp>

#include "ethercat/errors.hpp"

namespace ethercat {

// ----------------------------------------------------------------------------
// RxSnapshot -- seqlock
// ----------------------------------------------------------------------------

RxSnapshot::RxSnapshot(std::size_t payload_size) noexcept : size_(std::min(payload_size, kMaxPdoBytes)) {}

void RxSnapshot::publish(std::span<const std::byte> payload, std::uint16_t working_counter, std::uint64_t cycle) noexcept {
    const std::size_t n = std::min(payload.size(), size_);

    // Parity-robust: derive a strictly-greater ODD value regardless of the
    // current parity, and commit at odd+1 (even). `(v + 1) | 1` is the next odd
    // >= v+1 for any v, so this restores the single-writer even invariant even if
    // a test seam left seq at an odd value.
    const std::uint64_t odd = (seq_.load(std::memory_order_relaxed) + 1U) | 1U;
    seq_.store(odd, std::memory_order_relaxed);  // -> odd: write in progress
    // (F1) LOAD-BEARING: orders the odd marker before the payload writes below.
    // Must stay a fence -- making the odd store `release` would NOT order it
    // before the subsequent relaxed payload stores. Do not remove.
    std::atomic_thread_fence(std::memory_order_release);

    // All shared payload/metadata writes go through atomic_ref (relaxed) so there
    // is no non-atomic data race for TSan to flag. Bytes beyond the payload are
    // zero-filled to keep each frame deterministic.
    for (std::size_t i = 0; i < size_; ++i) {
        const auto value = (i < n) ? std::to_integer<unsigned char>(payload[i]) : static_cast<unsigned char>(0);
        std::atomic_ref<unsigned char>(bytes_[i]).store(value, std::memory_order_relaxed);
    }
    std::atomic_ref<std::uint16_t>(wkc_).store(working_counter, std::memory_order_relaxed);
    std::atomic_ref<std::uint64_t>(cycle_).store(cycle, std::memory_order_relaxed);

    // (F2) Redundant given the `release` even-store below, but kept as insurance:
    // it becomes load-bearing if anyone ever weakens the even store to relaxed.
    std::atomic_thread_fence(std::memory_order_release);
    seq_.store(odd + 1U, std::memory_order_release);  // -> even: stable
}

PdoSnapshot RxSnapshot::read() const noexcept {
    PdoSnapshot out;

    for (unsigned attempt = 0; attempt < kMaxReadRetries; ++attempt) {
        const std::uint64_t s0 = seq_.load(std::memory_order_acquire);
        if ((s0 & 1U) != 0U) {
            continue;  // a write is in progress; retry
        }

        for (std::size_t i = 0; i < size_; ++i) {
            out.bytes[i] = static_cast<std::byte>(std::atomic_ref<unsigned char>(bytes_[i]).load(std::memory_order_relaxed));
        }
        const std::uint16_t wkc = std::atomic_ref<std::uint16_t>(wkc_).load(std::memory_order_relaxed);
        const std::uint64_t cyc = std::atomic_ref<std::uint64_t>(cycle_).load(std::memory_order_relaxed);

        // (G2) LOAD-BEARING: orders the payload loads above before the seq
        // re-read below. Must stay a fence -- making the re-read `acquire` would
        // order the re-read before *later* ops, not the *prior* loads before
        // itself (wrong direction). Do not remove. The re-read can stay relaxed
        // precisely because this fence carries the ordering.
        std::atomic_thread_fence(std::memory_order_acquire);
        const std::uint64_t s1 = seq_.load(std::memory_order_relaxed);
        if (s0 == s1) {
            out.size = size_;
            out.working_counter = wkc;
            out.cycle = cyc;
            out.valid = true;
            out.stale = false;
            return out;
        }
    }

    // Retry budget exhausted: the writer kept moving (or is wedged mid-publish,
    // leaving seq permanently odd -> every read exhausts -> stale forever ->
    // is_powered()=false; a correct, safe failure mode). Report not-valid with no
    // payload; the consumer keeps its own last-good frame. The cycle is a
    // best-effort diagnostic only.
    out.size = 0;
    out.valid = false;
    out.stale = true;
    out.cycle = std::atomic_ref<std::uint64_t>(cycle_).load(std::memory_order_relaxed);
    return out;
}

// ----------------------------------------------------------------------------
// CommandQueue -- MPMC lock-free, boost hidden behind the pimpl
// ----------------------------------------------------------------------------

// MPSC via SPSC + a producer-side mutex. We use boost::lockfree::spsc_queue
// (a plain atomic head/tail ring -- NO tagged_index, so it is TSan-clean) and
// serialize the multiple non-RT producers with a mutex. The single RT consumer
// pops lock-free and NEVER touches the mutex, so there is zero priority
// inversion. The mutex release/acquire between successive producers also
// supplies the happens-before that satisfies spsc_queue's single-producer
// contract. Capacity is fixed at construction (no allocation on push/pop).
struct CommandQueue::Impl {
    explicit Impl(std::size_t capacity) : queue(capacity) {}
    boost::lockfree::spsc_queue<Command> queue;
    std::mutex producer_mutex;  // non-RT producers only; the RT consumer never locks it
};

CommandQueue::CommandQueue(std::size_t capacity) : impl_(std::make_unique<Impl>(capacity)) {}
CommandQueue::~CommandQueue() = default;
CommandQueue::CommandQueue(CommandQueue&&) noexcept = default;
CommandQueue& CommandQueue::operator=(CommandQueue&&) noexcept = default;

// push() is the NON-RT producer side. It takes the producer mutex so concurrent
// gRPC-handler threads serialize into the single-producer ring. The RT thread
// must NEVER call push() (it is the consumer) -- that is what keeps the lock off
// the RT path. A std::mutex lock/unlock does not allocate, so the malloc-counter
// stays zero. noexcept: a std::mutex failure is unrecoverable here, so letting
// it terminate is the correct outcome for an RT system.
// NOLINTNEXTLINE(bugprone-exception-escape)
bool CommandQueue::push(const Command& command) noexcept {
    const std::lock_guard<std::mutex> lock(impl_->producer_mutex);
    return impl_->queue.push(command);
}

// drain() is contractually noexcept (RT path). The operations inside cannot
// actually throw: Command is a trivially-copyable variant (so std::visit never
// hits valueless_by_exception and the optional assignments are noexcept), and
// the fixed-size boost queue pop does not allocate or throw. The static analysis
// can't prove that, hence the suppression.
// NOLINTNEXTLINE(bugprone-exception-escape)
CommandBatch CommandQueue::drain() noexcept {
    CommandBatch batch;
    Command command;
    while (impl_->queue.pop(command)) {
        std::visit(
            [&batch](const auto& cmd) {
                using T = std::decay_t<decltype(cmd)>;
                if constexpr (std::is_same_v<T, SetTarget>) {
                    batch.set_target = cmd;  // latest-wins
                } else if constexpr (std::is_same_v<T, SetVelocity>) {
                    batch.set_velocity = cmd;  // latest-wins
                } else if constexpr (std::is_same_v<T, Enable>) {
                    batch.enable = true;
                } else if constexpr (std::is_same_v<T, Disable>) {
                    batch.disable = true;
                } else if constexpr (std::is_same_v<T, Halt>) {
                    batch.halt = true;
                } else if constexpr (std::is_same_v<T, QuickStop>) {
                    batch.quick_stop = true;
                } else if constexpr (std::is_same_v<T, FaultReset>) {
                    batch.fault_reset = true;
                } else if constexpr (std::is_same_v<T, SetZero>) {
                    batch.set_zero = true;
                }
            },
            command);
    }
    return batch;
}

// ----------------------------------------------------------------------------
// TxStaging -- lock-free 3-slot announce/re-validate handoff
// ----------------------------------------------------------------------------

TxStaging::TxStaging(std::size_t payload_size) : size_(std::min(payload_size, kMaxPdoBytes)) {}

void TxStaging::stage_outputs(std::span<const std::byte> payload) noexcept {
#ifndef NDEBUG
    // Single-writer tripwire (see TRIPWIRE in the header): catch a second
    // concurrent stager in tests. Compiles out under NDEBUG.
    const bool already = staging_.exchange(true, std::memory_order_acq_rel);
    assert(!already && "TxStaging::stage_outputs is single-writer-only");
#endif

    // seq_cst on BOTH pending_ loads/stores forms the Dekker StoreLoad with the
    // RT take's announce/validate. A free slot always exists: at most one slot
    // is pending and one is being read, leaving a third.
    const std::uint32_t pend = pending_.load(std::memory_order_seq_cst);
    const std::uint32_t rd = reading_.load(std::memory_order_seq_cst);  // observe the RT reader's hazard
    std::uint32_t slot = 0;
    for (; slot < 3; ++slot) {
        if (slot != pend && slot != rd) {
            break;
        }
    }

    std::memcpy(slots_[slot].data(), payload.data(), std::min(payload.size(), size_));
    pending_.store(slot, std::memory_order_seq_cst);  // newest-wins: overwrites any prior unsent slot

#ifndef NDEBUG
    staging_.store(false, std::memory_order_release);
#endif
}

std::size_t TxStaging::take_outputs(std::span<std::byte> out) noexcept {
    // Protocol: load pending -> ANNOUNCE reading -> RE-VALIDATE pending -> copy
    // -> compare_exchange (consume iff unchanged). Never a bare exchange: that
    // would clear pending_ before the announce, reopening the window for the
    // stager to reuse the slot mid-copy (torn Tx). reading_==idx is held across
    // the whole copy, and the stager's self-exclusion (slot != pend) keeps idx
    // out of its pick set, so the copied slot is never concurrently written.
    // All pending_/reading_ ops are seq_cst (Dekker StoreLoad).
    std::uint32_t idx = pending_.load(std::memory_order_seq_cst);
    for (unsigned spins = 0;; ++spins) {
        if (idx == kNone) {
            reading_.store(kNone, std::memory_order_seq_cst);
            return 0;  // nothing new pending
        }
        reading_.store(idx, std::memory_order_seq_cst);                      // ANNOUNCE
        const std::uint32_t cur = pending_.load(std::memory_order_seq_cst);  // RE-VALIDATE
        if (cur == idx) {
            break;  // stable: safe to copy
        }
        if (spins >= kMaxTakeSpins) {
            break;  // freshness cap: copying idx is still safe (reading_==idx held)
        }
        idx = cur;  // a newer frame was staged; re-announce it
    }

    std::memcpy(out.data(), slots_[idx].data(), std::min(out.size(), size_));
    std::atomic_thread_fence(std::memory_order_acquire);
    // Consume: clear pending_ iff it is still our idx (a newer staged frame is
    // left pending for the next take). Strong CAS; on failure do not reuse idx
    // without re-reading (the CAS already wrote the observed value into idx).
    pending_.compare_exchange_strong(idx, kNone, std::memory_order_seq_cst, std::memory_order_seq_cst);
    reading_.store(kNone, std::memory_order_seq_cst);
    return size_;
}

// ----------------------------------------------------------------------------
// PdoCache
// ----------------------------------------------------------------------------

PdoCache::PdoCache(std::size_t rx_size, std::size_t tx_size) : rx_(rx_size), tx_(tx_size) {
    if (rx_size > kMaxPdoBytes || tx_size > kMaxPdoBytes) {
        throw PdoMappingError("slave PDO image (" + std::to_string(std::max(rx_size, tx_size)) + " B) exceeds kMaxPdoBytes (" +
                              std::to_string(kMaxPdoBytes) + " B); raise kMaxPdoBytes or remap the slave");
    }
}

}  // namespace ethercat
