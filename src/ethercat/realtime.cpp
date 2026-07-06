#include "ethercat/realtime.hpp"

#include <malloc.h>    // mallopt, M_TRIM_THRESHOLD, M_MMAP_MAX
#include <pthread.h>   // pthread_setschedparam, pthread_self
#include <sched.h>     // sched_param, SCHED_FIFO
#include <sys/mman.h>  // mlockall, MCL_CURRENT, MCL_FUTURE

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <thread>

namespace ethercat::realtime {

namespace {

// Pre-fault `bytes` of the calling thread's stack: grow the stack to its working depth now,
// faulting the pages in, so the RT loop's first deep call chain reuses resident pages instead
// of taking a fault. The pages stay mapped after this frame pops (the stack VMA never shrinks),
// and since setup() calls mlockall(MCL_FUTURE) first, they are locked as they fault. Volatile
// per-page writes and a separate noinline frame stop the compiler from eliding the touch.
// Precondition: the RT thread's stack is at least `bytes` (default 512 KiB, well under the
// 8 MiB default pthread stack).
[[gnu::noinline]] void prefault_stack(std::size_t bytes) noexcept {
    if (bytes == 0) {
        return;
    }
    // NOLINTNEXTLINE(cppcoreguidelines-no-malloc,clang-analyzer-security.insecureAPI.*) -- alloca is the idiom for touching a runtime span
    // of THIS stack
    auto* p = static_cast<volatile unsigned char*>(__builtin_alloca(bytes));
    constexpr std::size_t kPage = 4096;
    for (std::size_t i = 0; i < bytes; i += kPage) {
        p[i] = 0;
    }
    p[bytes - 1] = 0;
}

}  // namespace

bool setup(int priority, std::size_t prefault_bytes) noexcept {
    // Process-global one-shots, called once in the RT prelude, so the concurrency-mt-unsafe
    // lints (global heap/locale state) do not apply.
    // NOLINTBEGIN(concurrency-mt-unsafe)
    // Order matters: lock (including MCL_FUTURE) before pre-faulting, so the freshly faulted
    // stack pages are locked as they map in.
    // mlockall is load-bearing for RT determinism: if the RT working set can be paged out, a
    // fault under host memory pressure stalls the loop. Surface a failure so a missing
    // CAP_IPC_LOCK or too-low 'ulimit -l' is visible in the log instead of degrading determinism
    // invisibly.
    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {  // needs CAP_IPC_LOCK
        (void)std::fprintf(stderr,
                           "[ethercat] WARNING: mlockall(MCL_CURRENT|MCL_FUTURE) failed (%s) -- RT memory NOT locked; "
                           "page faults under memory pressure can stall the RT loop. Needs CAP_IPC_LOCK and adequate "
                           "'ulimit -l'.\n",
                           std::strerror(errno));
        (void)std::fflush(stderr);
    }
    (void)mallopt(M_TRIM_THRESHOLD, -1);  // keep the heap -- no fault from trimming
    (void)mallopt(M_MMAP_MAX, 0);
    prefault_stack(prefault_bytes);

    sched_param param{};
    param.sched_priority = priority;
    // The return is only the SCHED_FIFO result -- the bit a require_realtime caller throws on;
    // mlockall/mallopt/pre-fault above are best-effort and do not gate it.
    return pthread_setschedparam(pthread_self(), SCHED_FIFO, &param) == 0;
    // NOLINTEND(concurrency-mt-unsafe)
}

void lock_current() noexcept {
    // NOLINTNEXTLINE(concurrency-mt-unsafe)
    (void)mlockall(MCL_CURRENT);  // best-effort: resident before the RT thread spawns
}

bool sched_fifo_available(int priority) noexcept {
    // Probe on a scratch thread: the SCHED_FIFO policy dies with the thread, so the caller is
    // never left realtime and there are no process-wide side effects. pthread_create failure
    // (resource exhaustion) reports "unavailable", which errs on the loud side.
    bool ok = false;
    try {
        std::thread probe([&ok, priority]() noexcept {
            sched_param param{};
            param.sched_priority = priority;
            ok = pthread_setschedparam(pthread_self(), SCHED_FIFO, &param) == 0;
        });
        probe.join();
    } catch (...) {
        ok = false;
    }
    return ok;
}

}  // namespace ethercat::realtime
