#include "ethercat/pdo_cache.hpp"

#include <algorithm>
#include <cstring>
#include <string>

#include <boost/lockfree/queue.hpp>

#include "ethercat/errors.hpp"

namespace ethercat {

// ----------------------------------------------------------------------------
// RxSnapshot -- seqlock
// ----------------------------------------------------------------------------

RxSnapshot::RxSnapshot(std::size_t payload_size) noexcept : size_(std::min(payload_size, kMaxPdoBytes)) {}

void RxSnapshot::publish(std::span<const std::byte> payload, std::uint16_t working_counter, std::uint64_t cycle) noexcept {
    const std::size_t n = std::min(payload.size(), size_);

    const std::uint64_t s = seq_.load(std::memory_order_relaxed);
    seq_.store(s + 1, std::memory_order_relaxed);  // -> odd: write in progress
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

    std::atomic_thread_fence(std::memory_order_release);
    seq_.store(s + 2, std::memory_order_release);  // -> even: stable
}

PdoSnapshot RxSnapshot::read() const noexcept {
    PdoSnapshot out;
    out.size = size_;

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

        std::atomic_thread_fence(std::memory_order_acquire);
        const std::uint64_t s1 = seq_.load(std::memory_order_relaxed);
        if (s0 == s1) {
            out.working_counter = wkc;
            out.cycle = cyc;
            out.valid = true;
            out.stale = false;
            return out;
        }
    }

    // Retry budget exhausted: the writer kept moving. Report not-valid; the
    // consumer keeps its own last-good frame.
    out.valid = false;
    out.stale = true;
    return out;
}

// ----------------------------------------------------------------------------
// CommandQueue -- MPMC lock-free, boost hidden behind the pimpl
// ----------------------------------------------------------------------------

struct CommandQueue::Impl {
    explicit Impl(std::size_t capacity) : queue(capacity) {}
    boost::lockfree::queue<Command, boost::lockfree::fixed_sized<true>> queue;
};

CommandQueue::CommandQueue(std::size_t capacity) : impl_(std::make_unique<Impl>(capacity)) {}
CommandQueue::~CommandQueue() = default;
CommandQueue::CommandQueue(CommandQueue&&) noexcept = default;
CommandQueue& CommandQueue::operator=(CommandQueue&&) noexcept = default;

bool CommandQueue::push(const Command& command) noexcept {
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
    const std::uint32_t pend = pending_.load(std::memory_order_relaxed);
    const std::uint32_t rd = reading_.load(std::memory_order_seq_cst);  // observe the RT reader's hazard

    // With three slots and at most one pending + one being-read, a free slot
    // always exists.
    std::uint32_t slot = 0;
    for (; slot < 3; ++slot) {
        if (slot != pend && slot != rd) {
            break;
        }
    }

    std::memcpy(slots_[slot].data(), payload.data(), std::min(payload.size(), size_));
    pending_.store(slot, std::memory_order_seq_cst);  // newest-wins: overwrites any prior unsent slot
}

std::size_t TxStaging::take_outputs(std::span<std::byte> out) noexcept {
    std::uint32_t idx = 0;
    do {
        idx = pending_.load(std::memory_order_acquire);
        if (idx == kNone) {
            reading_.store(kNone, std::memory_order_seq_cst);
            return 0;  // nothing new pending
        }
        reading_.store(idx, std::memory_order_seq_cst);  // ANNOUNCE first (hazard)
    } while (pending_.load(std::memory_order_acquire) != idx);  // RE-VALIDATE the writer didn't move on

    std::memcpy(out.data(), slots_[idx].data(), std::min(out.size(), size_));
    std::atomic_thread_fence(std::memory_order_acquire);
    // Consume: only clear pending_ if it is still our idx (writer may have staged
    // a newer frame, which we leave pending for the next take).
    pending_.compare_exchange_strong(idx, kNone, std::memory_order_acq_rel, std::memory_order_relaxed);
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
