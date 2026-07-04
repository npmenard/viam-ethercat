#include "ethercat/realtime.hpp"

#include <malloc.h>    // mallopt, M_TRIM_THRESHOLD, M_MMAP_MAX
#include <pthread.h>   // pthread_setschedparam, pthread_self
#include <sched.h>     // sched_param, SCHED_FIFO
#include <sys/mman.h>  // mlockall, MCL_CURRENT, MCL_FUTURE

#include <cerrno>
#include <cstdio>
#include <cstring>

namespace ethercat::realtime {

namespace {

// Pre-fault `bytes` of the CALLING thread's stack: grow the stack to its working
// depth NOW (faulting the pages in) so the RT loop's first deep call chain reuses
// resident pages instead of taking a fault. The pages stay mapped after this frame
// pops (the stack VMA never shrinks), and -- since setup() calls mlockall(MCL_FUTURE)
// FIRST -- they are locked as they fault. Volatile per-page writes + a separate
// noinline frame stop the compiler from eliding the touch. The canonical Linux-RT
// stack_prefault, generalized to a runtime size. PRECONDITION: the RT thread's stack
// is at least `bytes` (default 512 KiB << the 8 MiB default pthread stack).
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
    // Process-global one-shots, called ONCE in the RT prelude -- the
    // concurrency-mt-unsafe lints (global heap/locale state) are N/A here.
    // NOLINTBEGIN(concurrency-mt-unsafe)
    // Order matters: lock (incl. MCL_FUTURE) BEFORE pre-faulting, so the freshly
    // faulted stack pages are locked as they map in.
    // #72 audit: mlockall is load-bearing for RT determinism -- if the RT working set can be paged
    // out, a fault under host memory pressure stalls the loop (a candidate cause of the ~26ms gap).
    // It USED to be silently (void)-cast; surface a failure so a missing CAP_IPC_LOCK / too-low
    // 'ulimit -l' is visible in the log instead of degrading determinism invisibly. One line, once.
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
    // The RETURN is ONLY the SCHED_FIFO result -- the load-bearing bit (a
    // require_realtime caller throws on it); mlockall/mallopt/pre-fault above are
    // best-effort and do not gate it.
    return pthread_setschedparam(pthread_self(), SCHED_FIFO, &param) == 0;
    // NOLINTEND(concurrency-mt-unsafe)
}

void lock_current() noexcept {
    // NOLINTNEXTLINE(concurrency-mt-unsafe)
    (void)mlockall(MCL_CURRENT);  // best-effort: resident BEFORE the RT thread spawns
}

}  // namespace ethercat::realtime
