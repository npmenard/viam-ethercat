// Tests for the RT/non-RT boundary (pdo_cache).
//
// This file is built TWICE:
//   * normally (label none)        -- full iteration counts + the malloc-counter
//                                      and jitter cases (which sanitizers perturb).
//   * with -fsanitize=thread (label 'tsan', ETHERCAT_SANITIZE_THREAD=ON) -- the
//     threaded stress cases must pass with ZERO suppressions; iteration counts
//     are reduced (races surface fast) and the malloc/jitter cases compile out.

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <span>
#include <thread>
#include <vector>

#include "ethercat/errors.hpp"
#include "ethercat/pdo_cache.hpp"
#include "test_harness.hpp"

// Detect ThreadSanitizer so we can adjust workload + skip the alloc-counter case.
#if defined(__SANITIZE_THREAD__)
#define ECAT_TSAN 1
#elif defined(__has_feature)
#if __has_feature(thread_sanitizer)
#define ECAT_TSAN 1
#endif
#endif

using ethercat::Command;
using ethercat::CommandQueue;
using ethercat::Disable;
using ethercat::Enable;
using ethercat::FaultReset;
using ethercat::Halt;
using ethercat::PdoCache;
using ethercat::PdoSnapshot;
using ethercat::QuickStop;
using ethercat::RxSnapshot;
using ethercat::SetTarget;
using ethercat::SetVelocity;
using ethercat::SetZero;

namespace {

#ifdef ECAT_TSAN
constexpr std::uint64_t kStressIters = 300'000;
#else
constexpr std::uint64_t kStressIters = 2'000'000;
#endif

constexpr std::size_t kPayload = 32;  // representative A6-ish image size

// Fill a buffer so every byte equals (counter & 0xFF) -- a "uniform frame" whose
// internal consistency a torn read would break.
void fill_uniform(std::span<std::byte> buf, std::uint8_t v) {
    for (auto& b : buf) {
        b = std::byte{v};
    }
}

bool all_equal(std::span<const std::byte> buf, std::size_t n) {
    if (n == 0) {
        return true;
    }
    const std::byte first = buf[0];
    for (std::size_t i = 1; i < n; ++i) {
        if (buf[i] != first) {
            return false;
        }
    }
    return true;
}

// True iff the first n bytes all equal `v`. Used to tie the payload to the
// seqlock-protected cycle: a torn read (payload from one frame, metadata from
// another) breaks this.
bool all_equal_to(std::span<const std::byte> buf, std::size_t n, std::uint8_t v) {
    for (std::size_t i = 0; i < n; ++i) {
        if (std::to_integer<std::uint8_t>(buf[i]) != v) {
            return false;
        }
    }
    return true;
}

}  // namespace

// ---------------------------------------------------------------------------
// Functional: RxSnapshot freshness / valid
// ---------------------------------------------------------------------------

TEST("RxSnapshot: a published frame reads back valid and byte-exact") {
    RxSnapshot rx{kPayload};
    std::array<std::byte, kPayload> frame{};
    fill_uniform(frame, 0xA5);
    rx.publish(frame, 0x1234, 42);

    const PdoSnapshot s = rx.read();
    CHECK(s.valid);
    CHECK(!s.stale);
    CHECK_EQ(s.size, kPayload);
    CHECK_EQ(s.working_counter, std::uint16_t{0x1234});
    CHECK_EQ(s.cycle, std::uint64_t{42});
    CHECK(all_equal({s.bytes.data(), s.size}, s.size));
    CHECK_EQ(std::to_integer<int>(s.bytes[0]), 0xA5);
}

TEST("RxSnapshot: an un-published channel reads valid=true all-zero") {
    RxSnapshot rx{kPayload};
    const PdoSnapshot s = rx.read();  // seq starts even(0); never written
    CHECK(s.valid);
    CHECK_EQ(s.cycle, std::uint64_t{0});
    CHECK(all_equal({s.bytes.data(), s.size}, s.size));
    CHECK_EQ(std::to_integer<int>(s.bytes[0]), 0x00);
}

TEST("RxSnapshot: retry-exhaustion (writer stuck) returns valid=false/stale") {
    RxSnapshot rx{kPayload};
    rx.force_writing_for_test();  // bump seq to odd: "write in progress"
    const PdoSnapshot s = rx.read();
    CHECK(!s.valid);
    CHECK(s.stale);
    CHECK_EQ(s.size, std::size_t{0});  // no payload on the exhausted path

    // publish() is parity-robust: it restores the even/stable invariant even
    // though the seam left seq odd, so a subsequent read is valid again.
    std::array<std::byte, kPayload> frame{};
    fill_uniform(frame, 0x3C);
    rx.publish(frame, 7, 1);
    const PdoSnapshot s2 = rx.read();
    CHECK(s2.valid);
    CHECK_EQ(std::to_integer<int>(s2.bytes[0]), 0x3C);
}

// ---------------------------------------------------------------------------
// Functional: CommandQueue ordering / coalescing / capacity
// ---------------------------------------------------------------------------

TEST("CommandQueue: drain coalesces latest-wins setpoints and sticky discretes") {
    CommandQueue q{64};
    CHECK(q.push(Command{Enable{}}));
    CHECK(q.push(Command{SetTarget{1, 100, false}}));
    CHECK(q.push(Command{SetTarget{2, 200, false}}));
    CHECK(q.push(Command{SetTarget{3, 300, true}}));
    CHECK(q.push(Command{Halt{}}));
    CHECK(q.push(Command{SetZero{}}));

    const auto batch = q.drain();
    CHECK(batch.enable);
    CHECK(batch.halt);
    CHECK(batch.set_zero);
    CHECK(!batch.disable);
    CHECK(!batch.quick_stop);
    CHECK(!batch.fault_reset);
    CHECK(batch.set_target.has_value());
    CHECK_EQ(batch.set_target->counts, std::int32_t{3});  // latest wins
    CHECK_EQ(batch.set_target->profile_velocity, std::uint32_t{300});
    CHECK(batch.set_target->relative);
    CHECK(!batch.set_velocity.has_value());

    // Draining again with nothing queued yields an empty batch.
    CHECK(!q.drain().any());
}

TEST("CommandQueue: SetVelocity coalesces independently of SetTarget") {
    CommandQueue q{16};
    CHECK(q.push(Command{SetVelocity{10}}));
    CHECK(q.push(Command{SetVelocity{-25}}));
    const auto batch = q.drain();
    CHECK(batch.set_velocity.has_value());
    CHECK_EQ(batch.set_velocity->velocity, std::int32_t{-25});
    CHECK(!batch.set_target.has_value());
}

TEST("CommandQueue: push returns false when full (fixed capacity, no RT alloc)") {
    CommandQueue q{2};
    int accepted = 0;
    for (int i = 0; i < 100; ++i) {
        if (q.push(Command{Enable{}})) {
            ++accepted;
        }
    }
    CHECK(accepted >= 1);
    CHECK(accepted < 100);  // push returns false once full (exact capacity is impl-defined)
}

// ---------------------------------------------------------------------------
// Functional: PdoCache size validation
// ---------------------------------------------------------------------------

TEST("PdoCache: oversized image throws PdoMappingError with a clear message") {
    CHECK_THROWS_MSG(PdoCache(ethercat::kMaxPdoBytes + 1), ethercat::PdoMappingError, "exceeds kMaxPdoBytes");
    // In-range sizes construct fine.
    PdoCache ok{kPayload};
    (void)ok;
}

// ---------------------------------------------------------------------------
// Concurrency stress (the TSan gate): RxSnapshot 1 writer + N readers
// ---------------------------------------------------------------------------

TEST("RxSnapshot stress: no torn frames, cycle monotone (1 writer, 4 readers)") {
    RxSnapshot rx{kPayload};
    std::atomic<bool> stop{false};
    std::atomic<int> tears{0};
    std::atomic<int> regressions{0};
    std::atomic<std::uint64_t> valid_reads{0};

    auto reader = [&] {
        std::uint64_t last_cycle = 0;
        while (!stop.load(std::memory_order_relaxed)) {
            const PdoSnapshot s = rx.read();
            if (!s.valid) {
                continue;
            }
            valid_reads.fetch_add(1, std::memory_order_relaxed);
            // Every byte of frame c is (c & 0xFF); the seqlock-protected cycle is
            // c. If the read tore (payload from one frame, cycle from another),
            // the bytes won't all equal (cycle & 0xFF).
            const auto expect = static_cast<std::uint8_t>(s.cycle & 0xFFU);
            if (!all_equal_to({s.bytes.data(), s.size}, s.size, expect)) {
                tears.fetch_add(1, std::memory_order_relaxed);
            }
            if (s.cycle < last_cycle) {
                regressions.fetch_add(1, std::memory_order_relaxed);
            }
            last_cycle = s.cycle;
        }
    };

    constexpr int kReaders = 4;
    std::vector<std::thread> readers;
    readers.reserve(kReaders);
    for (int i = 0; i < kReaders; ++i) {
        readers.emplace_back(reader);
    }

    // #36 de-flake: the old fixed-length burst raced DESCHEDULED readers -- under TSan
    // instrumentation + a loaded machine (parallel builds) the readers could observe
    // <1000 valid frames before the writer finished, failing the non-vacuity guard
    // spuriously. LOAD-INDEPENDENT form: publish the full stress burst, then KEEP
    // publishing (yielding, so the readers actually get cycles) until they reach the
    // observation goal. The ceiling keeps a genuinely-broken read() (never valid) from
    // looping forever -- it then exits and the valid_reads CHECK fails meaningfully.
    constexpr std::uint64_t kReadGoal = 1000;
    std::array<std::byte, kPayload> frame{};
    std::uint64_t c = 0;
    const std::uint64_t ceiling = kStressIters * 100;
    while (c < kStressIters || valid_reads.load(std::memory_order_relaxed) < kReadGoal) {
        if (c >= ceiling) {
            break;  // broken read(): let the CHECK below report it
        }
        ++c;
        fill_uniform(frame, static_cast<std::uint8_t>(c & 0xFFU));
        rx.publish(frame, static_cast<std::uint16_t>(c & 0xFFFFU), c);
        if (c >= kStressIters) {
            std::this_thread::yield();  // extension phase: cede the CPU to the readers
        }
    }
    stop.store(true, std::memory_order_relaxed);
    for (auto& t : readers) {
        t.join();
    }

    CHECK_EQ(tears.load(), 0);
    CHECK_EQ(regressions.load(), 0);
    // Guard against a vacuous pass: if read() always exhausted (e.g. a parity
    // bug), tears would be 0 trivially. Require that readers actually observed
    // many valid frames (load-independent: the writer extended until they did).
    CHECK(valid_reads.load() >= kReadGoal);
}

// ---------------------------------------------------------------------------
// Concurrency: CommandQueue multi-producer integrity (no fabricated values)
// ---------------------------------------------------------------------------

// Runs UNDER TSan too: the CommandQueue is now SPSC-ring + producer-mutex (no
// boost tagged_index), so the contended multi-producer path is TSan-clean with
// zero suppressions -- this is the production scenario (multiple gRPC handler
// threads calling push() concurrently while the RT thread drains).
TEST("CommandQueue: multi-producer pushes are never corrupted/fabricated") {
    CommandQueue q{1024};
    std::atomic<bool> start{false};

    // 4 producers each push SetVelocity values from a disjoint, known range.
    constexpr int kProducers = 4;
    constexpr std::int32_t kPerProducer = 500;
    auto producer = [&](int id) {
        while (!start.load(std::memory_order_acquire)) {
        }
        const std::int32_t base = id * 100'000;
        for (std::int32_t i = 0; i < kPerProducer; ++i) {
            while (!q.push(Command{SetVelocity{base + i}})) {
                // queue momentarily full; spin (consumer below drains)
            }
        }
    };

    std::vector<std::int32_t> seen;
    std::atomic<bool> done{false};
    std::thread consumer([&] {
        for (;;) {
            const auto batch = q.drain();
            if (batch.set_velocity.has_value()) {
                seen.push_back(batch.set_velocity->velocity);
            }
            if (done.load(std::memory_order_acquire)) {
                const auto last = q.drain();  // final sweep after producers joined
                if (last.set_velocity.has_value()) {
                    seen.push_back(last.set_velocity->velocity);
                }
                break;
            }
        }
    });

    std::vector<std::thread> producers;
    producers.reserve(kProducers);
    for (int i = 0; i < kProducers; ++i) {
        producers.emplace_back(producer, i);
    }
    start.store(true, std::memory_order_release);
    for (auto& t : producers) {
        t.join();
    }
    done.store(true, std::memory_order_release);
    consumer.join();

    // Every value the consumer ever saw must be a legitimately-pushed value:
    // id*100000 + i with 0 <= i < kPerProducer and 0 <= id < kProducers.
    for (const std::int32_t v : seen) {
        const std::int32_t id = v / 100'000;
        const std::int32_t i = v % 100'000;
        CHECK(id >= 0 && id < kProducers);
        CHECK(i >= 0 && i < kPerProducer);
    }
}

// ---------------------------------------------------------------------------
// RT-safety extras (normal build only; sanitizers perturb allocation/timing)
// ---------------------------------------------------------------------------

#ifndef ECAT_TSAN

namespace {
std::atomic<std::size_t> g_alloc_count{0};
}  // namespace

void* operator new(std::size_t n) {
    g_alloc_count.fetch_add(1, std::memory_order_relaxed);
    void* p = std::malloc(n != 0 ? n : 1);
    return p;
}
void* operator new[](std::size_t n) {
    g_alloc_count.fetch_add(1, std::memory_order_relaxed);
    void* p = std::malloc(n != 0 ? n : 1);
    return p;
}
void operator delete(void* p) noexcept {
    std::free(p);
}
void operator delete[](void* p) noexcept {
    std::free(p);
}
void operator delete(void* p, std::size_t) noexcept {
    std::free(p);
}
void operator delete[](void* p, std::size_t) noexcept {
    std::free(p);
}

TEST("RT path performs ZERO heap allocations after warm-up") {
    PdoCache cache{kPayload};
    CommandQueue q{256};
    std::array<std::byte, kPayload> frame{};

    // Warm up: prime everything so first-touch allocation (if any) is excluded.
    for (int i = 0; i < 1000; ++i) {
        cache.publish_inputs(frame, 1, static_cast<std::uint64_t>(i));
        (void)cache.read_inputs();
        (void)q.push(Command{SetTarget{i, 0, false}});
        (void)q.drain();
    }

    const std::size_t before = g_alloc_count.load(std::memory_order_relaxed);
    for (std::uint64_t c = 0; c < 100'000; ++c) {
        cache.publish_inputs(frame, 2, c);
        const PdoSnapshot s = cache.read_inputs();
        (void)s;
        (void)q.push(Command{SetVelocity{static_cast<std::int32_t>(c)}});
        (void)q.drain();
    }
    const std::size_t after = g_alloc_count.load(std::memory_order_relaxed);
    CHECK_EQ(after - before, std::size_t{0});
}

TEST("boundary op latency: report percentiles (informational, no hard bound)") {
    PdoCache cache{kPayload};
    std::array<std::byte, kPayload> frame{};
    constexpr int kN = 200'000;
    std::vector<std::uint64_t> ns;
    ns.reserve(static_cast<std::size_t>(kN));
    for (int c = 0; c < kN; ++c) {
        const auto t0 = std::chrono::steady_clock::now();
        cache.publish_inputs(frame, 1, static_cast<std::uint64_t>(c));
        (void)cache.read_inputs();
        const auto t1 = std::chrono::steady_clock::now();
        ns.push_back(static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count()));
    }
    std::sort(ns.begin(), ns.end());
    const auto pct = [&](double p) { return ns[static_cast<std::size_t>(p * static_cast<double>(kN - 1))]; };
    std::fprintf(stderr,
                 "  publish+read latency ns: p50=%llu p99=%llu p999=%llu max=%llu\n",
                 static_cast<unsigned long long>(pct(0.50)),
                 static_cast<unsigned long long>(pct(0.99)),
                 static_cast<unsigned long long>(pct(0.999)),
                 static_cast<unsigned long long>(ns.back()));
    CHECK(ns.back() > 0);  // sanity only
}

#endif  // !ECAT_TSAN

TEST_MAIN()
