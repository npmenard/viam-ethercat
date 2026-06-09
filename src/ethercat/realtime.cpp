#include "ethercat/realtime.hpp"

#include <malloc.h>    // mallopt, M_TRIM_THRESHOLD, M_MMAP_MAX
#include <pthread.h>   // pthread_setschedparam, pthread_self
#include <sched.h>     // sched_param, SCHED_FIFO
#include <sys/mman.h>  // mlockall, MCL_CURRENT, MCL_FUTURE

#include "ethercat/master.hpp"

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
    (void)mlockall(MCL_CURRENT | MCL_FUTURE);  // best-effort (needs CAP_IPC_LOCK)
    (void)mallopt(M_TRIM_THRESHOLD, -1);       // keep the heap -- no fault from trimming
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

BringupStatus run_to_operational(Master& master,
                                 DcPacer& pacer,
                                 const std::function<bool()>& sync_faulted,
                                 std::chrono::milliseconds timeout) {
    const auto timeout_ns = static_cast<std::uint64_t>(timeout.count()) * 1'000'000ULL;
    const std::uint64_t give_up_at = monotonic_ns() + timeout_ns;
    for (;;) {
        // Order mirrors the hand-rolled bring-up loops: read the prior step's sync
        // state, advance the bring-up (this does the exchange), then pace the cycle
        // off THIS exchange's DC time -- so PD stays gapless + phase-locked.
        const BringupStatus bs = master.bringup_step(sync_faulted());
        pacer.pace(master.dc_time());
        if (bs == BringupStatus::Operational || bs == BringupStatus::Aborted) {
            return bs;
        }
        if (monotonic_ns() >= give_up_at) {
            return BringupStatus::Aborted;  // bounded give-up (don't hammer a wedged drive)
        }
    }
}

}  // namespace ethercat::realtime
