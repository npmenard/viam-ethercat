// Offline tests for #47 P1: Runner + SlaveControl + CycleContext over a SimBackend.
// The 11 spec cases (§6 P1), numbered to match; broken-baseline requirements are
// DECLARED per test (run against deliberately-broken Runner variants during dev and
// noted in the commit -- the established prove-it-fails pattern).
//
// Timing note: the Runner paces for real (1 kHz -> 1 ms/cycle), so tests use a small
// teardown window + short bring-up bounds to stay fast.

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>
#include <type_traits>
#include <vector>

#include "ethercat/master.hpp"
#include "ethercat/runner.hpp"
#include "ethercat/sim_backend.hpp"
#include "test_harness.hpp"

using ethercat::Cia402Mode;
using ethercat::ConfigContext;
using ethercat::CycleContext;
using ethercat::FieldLocation;
using ethercat::Master;
using ethercat::MasterConfig;
using ethercat::Runner;
using ethercat::RunnerConfig;
using ethercat::RunnerPhase;
using ethercat::SimBackend;
using ethercat::SimSlaveModel;
using ethercat::SlaveConfig;
using ethercat::SlaveControl;
using ethercat::StopReason;

namespace {

// A6-ish slave config (the pdo_access_test shape): Rx = ctrl+target, Tx = status+actual.
SlaveConfig make_slave(std::uint16_t id) {
    SlaveConfig sc;
    sc.slave_id = id;
    sc.rxpdo.assign_index = 0x1C12;
    sc.rxpdo.pdo_indices = {0x1600};
    sc.rxpdo.entries[0x1600] = {{0x6040, 0, 16}, {0x607A, 0, 32}};
    sc.txpdo.assign_index = 0x1C13;
    sc.txpdo.pdo_indices = {0x1A00};
    sc.txpdo.entries[0x1A00] = {{0x6041, 0, 16}, {0x6064, 0, 32}};
    sc.default_mode = Cia402Mode::ProfilePosition;
    return sc;
}

MasterConfig make_config(std::size_t slaves = 1, bool dc = true) {
    MasterConfig cfg;
    cfg.ifname = "sim0";
    cfg.use_distributed_clocks = dc;
    cfg.dc_op_gate_cycles = 2;  // short SETTLE: tests run in tens of ms
    for (std::size_t i = 1; i <= slaves; ++i) {
        cfg.slaves.push_back(make_slave(static_cast<std::uint16_t>(i)));
    }
    return cfg;
}

std::vector<SimSlaveModel> make_models(std::size_t slaves = 1) {
    std::vector<SimSlaveModel> out;
    for (std::size_t i = 0; i < slaves; ++i) {
        SimSlaveModel m;
        m.output_bytes = 6;
        m.input_bytes = 6;
        m.ctrlword_off = 0;
        m.target_off = 2;
        m.statusword_off = 0;
        m.actual_off = 2;
        m.mode = Cia402Mode::ProfilePosition;
        out.push_back(m);
    }
    return out;
}

RunnerConfig fast_runner_cfg() {
    RunnerConfig rc;
    rc.bringup_timeout = std::chrono::milliseconds(5000);
    rc.teardown_cycles = 5;  // small window: tests stay fast; semantics identical
    return rc;
}

// The recording control: every hook appends an event; knobs drive the scenarios.
// All mutation happens on the Runner's RT thread; tests read AFTER stop() (the join
// is the synchronization point), so plain members are race-free.
class TestControl : public SlaveControl {
   public:
    // knobs
    std::uint64_t request_stop_at_cycle = UINT64_MAX;  // ctx.request_stop() at this steady cycle
    bool fail_configure = false;                       // throw from on_configured
    bool sync_fault_always = false;                    // hold the bring-up gate closed
    std::function<void(CycleContext&)> on_step;        // extra per-step behavior (tests 9/10/11)

    // recordings
    std::vector<std::string> events;
    std::uint64_t steps = 0;
    std::uint64_t stopping_steps = 0;
    std::uint64_t first_step_cycle = UINT64_MAX;
    StopReason stop_reason = StopReason::None;
    CycleContext* cached_ctx = nullptr;  // test 10: the misbehaving cache

    void on_configured(ethercat::ConfigContext& cfg) override {
        (void)cfg;
        events.emplace_back("configured");
        if (fail_configure) {
            throw ethercat::ConfigError("TestControl: deliberate on_configured failure");
        }
    }
    void on_operational(CycleContext& ctx) noexcept override {
        events.emplace_back("operational");
        cached_ctx = &ctx;  // harmless unless a test USES it outside a window (test 10)
    }
    void on_stop(StopReason reason) noexcept override {
        events.emplace_back(std::string("stop:") + ethercat::to_string(reason));
        stop_reason = reason;
    }
    void step(CycleContext& ctx) noexcept override {
        if (steps == 0) {
            events.emplace_back("first_step");
            first_step_cycle = ctx.cycle();
        }
        ++steps;
        if (ctx.stopping()) {
            ++stopping_steps;
        }
        if (ctx.cycle() == request_stop_at_cycle) {
            ctx.request_stop();
        }
        if (on_step) {
            on_step(ctx);
        }
    }
    bool sync_faulted(const CycleContext& ctx) const noexcept override {
        (void)ctx;
        return sync_fault_always;
    }
};

}  // namespace

// (1) Full lifecycle + hook ORDER: configured -> operational -> first step ->
// stop:<reason> -> stopping window -> Stopped/closed.
// Baseline (declared): reorder the on_operational dispatch after the first step -> fails.
TEST("#47.1: full lifecycle order configured->operational->step->stop->window->closed") {
    Master m{make_config(), std::make_unique<SimBackend>(make_models())};
    m.init();
    m.configure();
    Runner r{m, fast_runner_cfg()};
    TestControl c;
    c.request_stop_at_cycle = 10;
    r.attach(1, c);
    r.run();  // start + block + teardown
    CHECK(c.events.size() >= 4);
    CHECK(c.events[0] == "configured");
    CHECK(c.events[1] == "operational");
    CHECK(c.events[2] == "first_step");
    CHECK(c.events[3] == "stop:Requested");
    CHECK_EQ(c.first_step_cycle, std::uint64_t{0});  // cycle counter starts at the first step
    CHECK(c.steps > 10);                             // steady ran + the window ran
    CHECK_EQ(c.stopping_steps, std::uint64_t{5});    // exactly teardown_cycles stopping steps
    CHECK(r.status().phase == RunnerPhase::Stopped);
    CHECK(r.status().reason == StopReason::Requested);
}

// (2) step() is NEVER dispatched before Operational.
// Baseline (declared): remove the OP gate (dispatch step during bring-up) -> fails.
TEST("#47.2: not stepped before OPERATIONAL (bring-up held closed -> zero steps)") {
    MasterConfig cfg = make_config();
    cfg.op_await_timeout_ms = 50;  // Master's own give-up, fast (#42 knob)
    Master m{cfg, std::make_unique<SimBackend>(make_models())};
    m.init();
    m.configure();
    Runner r{m, fast_runner_cfg()};
    TestControl c;
    c.sync_fault_always = true;  // the gate never confirms -> bring-up aborts
    r.attach(1, c);
    r.run();
    CHECK_EQ(c.steps, std::uint64_t{0});  // no step ever dispatched pre-OP
    CHECK(c.stop_reason == StopReason::BringupAborted);
    CHECK(r.status().reason == StopReason::BringupAborted);
}

// (3) The exhaustive (phase, event) matrix (§5/§5b): every cell value-asserted --
// outcome + StopReason + hook calls. (The give-up cells ride tests 2/5; the
// window-fault cell is test 11; this covers the remaining live cells.)
TEST("#47.3: (phase,event) matrix -- steady fault, steady request, bring-up stop, stopped inertness") {
    {  // Steady + WKC collapse -> BusFault -> on_stop -> window -> close. NEVER auto-re-walk.
        Master m{make_config(), std::make_unique<SimBackend>(make_models())};
        auto* sim_owner = static_cast<SimBackend*>(nullptr);
        (void)sim_owner;
        // need the sim pointer: rebuild with it held
        auto sim = std::make_unique<SimBackend>(make_models());
        SimBackend* simp = sim.get();
        Master m2{make_config(), std::move(sim)};
        m2.init();
        m2.configure();
        Runner r{m2, fast_runner_cfg()};
        TestControl c;
        c.on_step = [&](CycleContext& ctx) {
            if (ctx.cycle() == 8 && !ctx.stopping()) {
                simp->force_short_wkc(true);  // bus dies under us (atomic test hook)
            }
        };
        r.attach(1, c);
        r.run();
        CHECK(c.stop_reason == StopReason::BusFault);
        CHECK(r.status().reason == StopReason::BusFault);
        CHECK_EQ(c.stopping_steps, std::uint64_t{5});     // the window still ran (PD attempts continue)
        CHECK_EQ(simp->op_requests(), 1);                 // no auto-re-walk to OP after the fault
        CHECK(r.status().phase == RunnerPhase::Stopped);  // latched, inert
        const auto reason_after = r.status().reason;      // Stopped+anything: status just reads the latch
        CHECK(reason_after == StopReason::BusFault);
    }
    {  // BringingUp + stop request -> Requested, NO window (drive never enabled), no steps.
        Master m{make_config(), std::make_unique<SimBackend>(make_models())};
        m.init();
        m.configure();
        TestControl c;               // before r (lifetime contract) + read AFTER the join below
        c.sync_fault_always = true;  // hold in bring-up
        std::chrono::steady_clock::duration stop_elapsed{};
        {
            Runner r{m, fast_runner_cfg()};
            r.attach(1, c);
            r.start();
            std::this_thread::sleep_for(std::chrono::milliseconds(20));  // a few bring-up cycles
            // #TODO-3: stop() is private now -- request the stop, wait for the RT loop to mark
            // Stopped, then drop r (dtor JOINS). PROMPTNESS pinned (P2 cell-check): the pump must
            // exit on the stop FLAG -- a flag-ignoring pump reaching the same end-state via its
            // give-up bound (bringup_timeout) is a DIFFERENT exit path with identical values;
            // only the wall time betrays it. The status().phase poll is RELAXED (liveness only),
            // so the control's non-atomic fields (steps/stop_reason) are read AFTER the dtor
            // JOIN below -- the join is the happens-before edge (a relaxed poll is NOT).
            const auto stop_t0 = std::chrono::steady_clock::now();
            r.request_stop();
            while (r.status().phase != RunnerPhase::Stopped) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
            stop_elapsed = std::chrono::steady_clock::now() - stop_t0;
            CHECK(r.status().phase == RunnerPhase::Stopped);
        }  // r dtor: bounded join (immediate -- already Stopped) -> close; SYNCHRONIZES with on_stop
        CHECK_EQ(c.steps, std::uint64_t{0});
        CHECK_EQ(c.stopping_steps, std::uint64_t{0});  // no stopping window pre-OP
        CHECK(c.stop_reason == StopReason::Requested);
        constexpr auto kStopPromptBound = std::chrono::seconds(1);
        CHECK(fast_runner_cfg().bringup_timeout >= 5 * kStopPromptBound);  // the discriminator guard
        CHECK(stop_elapsed < kStopPromptBound);
    }
}

// (4) request_stop latches FIRST-CAUSE-WINS + on_stop carries the right reason per cause.
// Baseline (declared): replace the CAS latch with a plain overwrite -> fails.
TEST("#47.4: stop-reason latch is first-cause-wins across causes") {
    {  // Requested, with a bus fault chasing it into the stopping entry.
        auto sim = std::make_unique<SimBackend>(make_models());
        SimBackend* simp = sim.get();
        Master m{make_config(), std::move(sim)};
        m.init();
        m.configure();
        Runner r{m, fast_runner_cfg()};
        TestControl c;
        c.on_step = [&](CycleContext& ctx) {
            if (ctx.cycle() == 6) {
                ctx.request_stop();           // FIRST cause: Requested
                simp->force_short_wkc(true);  // a bus fault follows immediately...
            }
        };
        r.attach(1, c);
        r.run();
        CHECK(c.stop_reason == StopReason::Requested);  // ...but the first cause won
        CHECK(r.status().reason == StopReason::Requested);
    }
    {  // BusFault first, then a Requested latch attempt MID-WINDOW. This attempt
       // PROVABLY reaches the latch (request_stop is legal from the window and
       // always calls it) -- so this block is the one that exercises the CAS:
       // an overwrite latch flips status().reason to Requested here.
        auto sim = std::make_unique<SimBackend>(make_models());
        SimBackend* simp = sim.get();
        Master m{make_config(), std::move(sim)};
        m.init();
        m.configure();
        Runner r{m, fast_runner_cfg()};
        TestControl c;
        c.on_step = [&](CycleContext& ctx) {
            if (!ctx.stopping() && ctx.cycle() == 4) {
                simp->force_short_wkc(true);  // FIRST cause: BusFault (latches a few cycles on)
            }
            if (ctx.stopping() && c.stopping_steps == 2) {
                ctx.request_stop();  // SECOND cause, mid-window: must NOT displace BusFault
            }
        };
        r.attach(1, c);
        r.run();
        CHECK(c.stop_reason == StopReason::BusFault);      // on_stop carried the first cause
        CHECK(r.status().reason == StopReason::BusFault);  // and the latch HELD it
    }
}

// (5) NO-HAMMER: exactly ONE OP request per start(), across the normal AND abort paths.
// Baseline (declared): allow bring-up re-entry after an abort -> op_requests()==2 -> fails.
TEST("#47.5: exactly one OP request per start() (normal run + aborted bring-up)") {
    {  // normal
        auto sim = std::make_unique<SimBackend>(make_models());
        SimBackend* simp = sim.get();
        Master m{make_config(), std::move(sim)};
        m.init();
        m.configure();
        Runner r{m, fast_runner_cfg()};
        TestControl c;
        c.request_stop_at_cycle = 5;
        r.attach(1, c);
        r.run();
        CHECK_EQ(simp->op_requests(), 1);
    }
    {  // aborted bring-up: still exactly one (the abort never re-requests)
        auto sim = std::make_unique<SimBackend>(make_models());
        SimBackend* simp = sim.get();
        MasterConfig cfg = make_config();
        cfg.op_await_timeout_ms = 50;
        Master m{cfg, std::move(sim)};
        m.init();
        m.configure();
        Runner r{m, fast_runner_cfg()};
        TestControl c;
        c.sync_fault_always = true;
        r.attach(1, c);
        r.run();
        CHECK(c.stop_reason == StopReason::BringupAborted);
        CHECK_EQ(simp->op_requests(), 1);
    }
}

// (6) Multi-slave: controls stepped in attach/slave order each cycle; ONE control's
// request_stop escalates bus-wide (both stop, both get on_stop).
TEST("#47.6: multi-slave step order + bus-wide stop escalation") {
    Master m{make_config(2), std::make_unique<SimBackend>(make_models(2))};
    m.init();
    m.configure();
    Runner r{m, fast_runner_cfg()};
    TestControl c1;
    TestControl c2;
    std::vector<int> order;  // RT-thread-only writes; read after join
    c1.on_step = [&](CycleContext& ctx) {
        if (!ctx.stopping()) {
            order.push_back(1);
        }
    };
    c2.on_step = [&](CycleContext& ctx) {
        if (!ctx.stopping()) {
            order.push_back(2);
        }
        if (ctx.cycle() == 7) {
            ctx.request_stop();  // ONE control stops the WHOLE runner
        }
    };
    r.attach(1, c1);
    r.attach(2, c2);
    r.run();
    CHECK(order.size() >= 4);
    for (std::size_t i = 0; i + 1 < order.size(); i += 2) {
        CHECK_EQ(order[i], 1);  // slave 1 stepped before slave 2, every cycle
        CHECK_EQ(order[i + 1], 2);
    }
    CHECK(c1.stop_reason == StopReason::Requested);  // both controls saw the stop
    CHECK(c2.stop_reason == StopReason::Requested);
    CHECK_EQ(c1.stopping_steps, std::uint64_t{5});  // both ran the window
    CHECK_EQ(c2.stopping_steps, std::uint64_t{5});
}

// (7) Pacing-input selection: DC and free-run go through the SAME pacer; consumer
// step() is identical -- only ctx.dc_time_ns() differs (sim DC ramp vs 0).
TEST("#47.7: dc vs free-run -- same control, dc_time_ns() is the only difference") {
    std::int64_t dc_seen = -1;
    std::int64_t freerun_seen = -1;
    {
        Master m{make_config(1, /*dc=*/true), std::make_unique<SimBackend>(make_models())};
        m.init();
        m.configure();
        Runner r{m, fast_runner_cfg()};
        TestControl c;
        c.request_stop_at_cycle = 5;
        c.on_step = [&](CycleContext& ctx) { dc_seen = ctx.dc_time_ns(); };
        r.attach(1, c);
        r.run();
    }
    {
        Master m{make_config(1, /*dc=*/false), std::make_unique<SimBackend>(make_models())};
        m.init();
        m.configure();
        Runner r{m, fast_runner_cfg()};
        TestControl c;
        c.request_stop_at_cycle = 5;
        c.on_step = [&](CycleContext& ctx) { freerun_seen = ctx.dc_time_ns(); };
        r.attach(1, c);
        r.run();
    }
    CHECK(dc_seen > 0);                       // the sim's synthetic DC ramp is visible under DC
    CHECK_EQ(freerun_seen, std::int64_t{0});  // and exactly 0 in free-run (the §4 contract)
}

// (8) attach-after-start throws; on_configured-throw aborts start() CLEANLY: nothing
// spawned, no rt_active bracket (master SDO still works), master untouched.
TEST("#47.8: attach-after-start throws; on_configured failure -> clean no-spawn abort") {
    Master m{make_config(), std::make_unique<SimBackend>(make_models())};
    m.init();
    m.configure();
    {
        Runner r{m, fast_runner_cfg()};
        TestControl bad;
        bad.fail_configure = true;
        r.attach(1, bad);
        CHECK_THROWS(r.start(), ethercat::ConfigError);
        CHECK(r.status().phase == RunnerPhase::Idle);  // nothing spawned
        // No bracket was set: the single-port-owner SDO surface still works.
        const std::array<std::byte, 2> one{std::byte{0x01}, std::byte{0x00}};
        m.sdo_write(1, 0x5FFF, 0x01, one);  // generic test object; throws if rt_active leaked
    }
    {
        // #TODO-3 ORDERING: the control MUST outlive the Runner -- the RT thread calls
        // control->step() until ~Runner joins it, so a control destroyed first (declared
        // AFTER the Runner) would be touched dead during the dtor's bounded join. Declare
        // c BEFORE r so r (and its thread-join) tears down first, c still alive.
        TestControl c;
        c.request_stop_at_cycle = 3;
        Runner r{m, fast_runner_cfg()};
        r.attach(1, c);
        r.start();
        TestControl late;
        CHECK_THROWS(r.attach(1, late), ethercat::ConfigError);  // attach-after-start
        // No explicit stop() -- the control self-requests at cycle 3 and the r dtor runs
        // the bounded teardown at scope end (joining while c is still alive).
    }
}

// (9) The stopping window: ctx.stopping() visible to step(); EXACTLY teardown_cycles
// stopping steps; the control's disable policy ships (cw=0 lands in the output image);
// a window-IGNORING control still reaches Stopped/closed.
// Baseline (declared): skip the window -> the disable never ships -> the assert fails.
TEST("#47.9: stopping window delivers the disable policy; ignoring it is still safe") {
    {  // a control that uses the window to disable (cw -> 0x0000)
        auto sim = std::make_unique<SimBackend>(make_models());
        SimBackend* simp = sim.get();
        Master m{make_config(), std::move(sim)};
        m.init();
        m.configure();
        Runner r{m, fast_runner_cfg()};
        TestControl c;
        c.request_stop_at_cycle = 6;
        const FieldLocation cw_loc{0, true};  // ctrlword @ offset 0 (the test map)
        std::atomic<bool> disabled{false};
        c.on_step = [&](CycleContext& ctx) {
            if (ctx.stopping()) {
                ctx.store<std::uint16_t>(cw_loc, 0x0000);  // CiA402 disable-voltage POLICY
                disabled.store(true);
            } else {
                ctx.store<std::uint16_t>(cw_loc, 0x000F);
            }
        };
        r.attach(1, c);
        r.run();
        CHECK(disabled.load());                        // the window gave the policy its cycles
        CHECK_EQ(c.stopping_steps, std::uint64_t{5});  // exactly teardown_cycles
        // The staged disable shipped with a later process(): the sim's command image
        // holds cw==0 after the run (slave_io outputs reflect the last exchange).
        const auto out = simp->slave_io(1).outputs;
        CHECK_EQ(ethercat::load_le<std::uint16_t>(out.subspan(0, 2)), std::uint16_t{0x0000});
    }
    {  // a window-ignoring control still ends Stopped (safety doesn't depend on policy)
        Master m{make_config(), std::make_unique<SimBackend>(make_models())};
        m.init();
        m.configure();
        Runner r{m, fast_runner_cfg()};
        TestControl c;  // does nothing with stopping()
        c.request_stop_at_cycle = 4;
        r.attach(1, c);
        r.run();
        CHECK(r.status().phase == RunnerPhase::Stopped);
    }
}

// CycleContext is non-copyable AND non-movable (#47 TODO-1): a stash is a COMPILE
// error, not a silent value-copy aliasing the owned buffers. (Was implicitly
// copyable -- only the ctor was private, so `auto saved = ctx;` compiled.)
static_assert(!std::is_copy_constructible_v<CycleContext> && !std::is_move_constructible_v<CycleContext>,
              "#47 TODO-1: CycleContext must be non-copyable and non-movable");

// (10) OWNED-DATA value semantics (#47 TODO-1): the ctx holds its images BY VALUE, so
// a control that ESCAPES the ctx (stashes &ctx and touches it outside its dispatch
// window) reads a VALID object with STALE data -- never a dangling/in-flight read --
// and a store through the escaped handle lands in the owned buffer and NEVER reaches
// the wire. (Pre-TODO-1 the ctx held spans into the live process buffers: the same
// escape was UB / a wild read that at 1 kHz could command dangerous motion.)
// (The old stop()-from-RT smoke-check is GONE: #TODO-3 made stop() private, so a control
// literally cannot name r.stop() from step() -- the self-join scenario is now a COMPILE
// error, structurally impossible, not a runtime-degraded guard.) NDEBUG only: in a debug
// build the out-of-window touch would (correctly) trip check_live's assert -- see the
// manual broken-baseline (remove the live_ guard -> the assert no longer fires).
TEST("#47.10: escaped ctx is safe-stale, never on the wire (owned-data value semantics)") {
#ifdef NDEBUG
    auto sim = std::make_unique<SimBackend>(make_models());
    SimBackend* simp = sim.get();
    Master m{make_config(), std::move(sim)};
    m.init();
    m.configure();
    Runner r{m, fast_runner_cfg()};
    TestControl c;
    const FieldLocation cw_loc{0, true};  // ctrlword @ offset 0 (the test map)
    constexpr std::uint16_t kShipped = 0x000F;
    c.request_stop_at_cycle = 6;
    c.on_step = [&](CycleContext& ctx) {
        ctx.store<std::uint16_t>(cw_loc, kShipped);  // a known, deterministic last-shipped value
    };
    r.attach(1, c);
    r.run();

    // cached_ctx was stashed by on_operational (the escape). The Runner is still alive,
    // so it points at a VALID CycleContext holding last-dispatch (stale) data.
    CHECK(c.cached_ctx != nullptr);
    const std::uint64_t stale_cycle = c.cached_ctx->cycle();  // stale-but-DEFINED, no crash/UB
    CHECK(stale_cycle != UINT64_MAX);                         // it's a real prior cycle value, not garbage

    // The wire holds the last value the IN-WINDOW stores shipped...
    const auto before = simp->slave_io(1).outputs;
    CHECK_EQ(ethercat::load_le<std::uint16_t>(before.subspan(0, 2)), kShipped);
    // ...and a store through the ESCAPED handle (outside any window) does NOT reach it:
    // it lands in the owned output buffer, which no dispatch will copy out again.
    c.cached_ctx->store<std::uint16_t>(cw_loc, 0xBEEF);
    const auto after = simp->slave_io(1).outputs;
    CHECK_EQ(ethercat::load_le<std::uint16_t>(after.subspan(0, 2)), kShipped);  // still kShipped, NOT 0xBEEF

    CHECK(r.status().phase == RunnerPhase::Stopped);  // and the run ended safely (no deadlock)
#else
    CHECK(true);  // debug builds: the out-of-window touch asserts -- not runnable in-process
#endif
}

// (11) Fault DURING the stopping window: the window COMPLETES to expiry (the settled
// §5/§5b rule -- DA's exit-early proposal was withdrawn on cost asymmetry): PD
// attempts continue so a partial-fault disable can still reach the drive.
// Baseline (declared): an exit-early implementation cuts the window short -> the
// stopping_steps count assert fails.
TEST("#47.11: a bus fault mid-window does NOT cut the window short") {
    auto sim = std::make_unique<SimBackend>(make_models());
    SimBackend* simp = sim.get();
    Master m{make_config(), std::move(sim)};
    m.init();
    m.configure();
    RunnerConfig rc = fast_runner_cfg();
    rc.teardown_cycles = 8;
    Runner r{m, rc};
    TestControl c;
    c.request_stop_at_cycle = 5;  // FIRST cause: Requested -> the window starts
    c.on_step = [&](CycleContext& ctx) {
        if (ctx.stopping() && c.stopping_steps == 3) {
            simp->force_short_wkc(true);  // the bus faults MID-window
        }
    };
    r.attach(1, c);
    r.run();
    CHECK_EQ(c.stopping_steps, std::uint64_t{8});   // the FULL window ran (not cut at 3)
    CHECK(c.stop_reason == StopReason::Requested);  // first cause won; the fault didn't relatch
    CHECK(r.status().phase == RunnerPhase::Stopped);
}

// White-box access to the PRIVATE teardown (#TODO-3): consumers stop by dropping the
// Runner, but H1/H4 must drive stop() directly. Declared a friend in runner.hpp.
namespace ethercat {
struct RunnerTestPeer {
    static void stop(Runner& r) noexcept {
        r.stop();
    }
};
}  // namespace ethercat

namespace {
// A control whose step() BLOCKS until released -- models a WEDGED RT loop (TODO-3 H1).
class WedgeControl : public SlaveControl {
   public:
    std::atomic<bool> release{false};
    std::atomic<bool> in_step{false};
    void step(CycleContext& ctx) noexcept override {
        (void)ctx;
        in_step.store(true, std::memory_order_release);
        while (!release.load(std::memory_order_acquire)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
};
}  // namespace

// (12) TODO-3 H1 -- the LIVENESS guarantee: a wedged step() must NOT hang teardown.
// stop() bounded-joins, then DETACHES the thread and latches Wedged so it RETURNS. The
// drive stays SAFE (PD gaps upstream of step() -> SM watchdog de-energizes); only the
// MODULE wedges. *Baseline (declared): an UNBOUNDED join hangs forever -> the test never
// returns (timeout) -- which the bounded join replaced.*
// TSan detection: the wedge test below INTENTIONALLY abandons a detached thread (the
// documented Wedged behavior), which TSan's join-everything model flags as racing the
// teardown. It's a LIVENESS test (stop returns, doesn't hang), not a data-race test --
// the race-relevant teardown (clean join) is covered by the other tests, which DO run
// under TSan. Exclude only this one from the TSan lane (#50 spirit preserved: no
// race-correctness test is hidden -- this asserts liveness, not race-freedom).
#if defined(__SANITIZE_THREAD__)
#define ETHERCAT_UNDER_TSAN 1
#elif defined(__has_feature)
#if __has_feature(thread_sanitizer)
#define ETHERCAT_UNDER_TSAN 1
#endif
#endif

#ifndef ETHERCAT_UNDER_TSAN
TEST("#47.12 (TODO-3 H1): wedged step() -> bounded stop returns + Wedged, no hang") {
    // A real wedge ABANDONS the module: the detached RT thread outlives the teardown and
    // keeps touching the master/control. So the test must NOT destruct (or even release)
    // them while that thread lives -- it heap-allocates and keeps them reachable forever
    // (no free -> no race/UAF with the detached thread; no leak report -> still reachable).
    // The wedge is never released: the thread stays harmlessly blocked in step().
    static std::vector<void*> kept_alive;  // reachable owner of the abandoned objects
    auto* m = new Master(make_config(), std::make_unique<SimBackend>(make_models()));
    m->init();
    m->configure();
    auto* c = new WedgeControl();
    RunnerConfig rc = fast_runner_cfg();
    rc.stop_join_timeout = std::chrono::milliseconds(120);  // small ceiling -> fast test
    auto* r = new Runner(*m, rc);
    r->attach(1, *c);
    r->start();
    while (!c->in_step.load(std::memory_order_acquire)) {  // wait until the wedge is engaged
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    const auto t0 = std::chrono::steady_clock::now();
    ethercat::RunnerTestPeer::stop(*r);  // bounded join -> wedge detected -> detach + Wedged, RETURNS
    const auto elapsed = std::chrono::steady_clock::now() - t0;
    CHECK(elapsed < std::chrono::milliseconds(600));  // returned, did NOT hang
    CHECK(r->status().reason == StopReason::Wedged);
    CHECK(r->status().phase == RunnerPhase::Stopped);
    kept_alive.push_back(r);  // abandon (never freed): the detached wedged thread keeps using them
    kept_alive.push_back(m);
    kept_alive.push_back(c);
}
#endif  // !ETHERCAT_UNDER_TSAN

// (13) TODO-3 -- de-energize on destruction is STRUCTURAL: destroying the Runner with NO
// explicit stop still runs the graceful teardown (the disable policy ships, then close()).
// This is the Tier-2 dtor path (SDK never called stop()). *Baseline (declared): a
// non-owning Runner whose dtor doesn't chain -> the disable never ships -> this fails.*
TEST("#47.13 (TODO-3): ~Runner de-energizes even with NO explicit stop (Tier-2 dtor)") {
    auto sim = std::make_unique<SimBackend>(make_models());
    SimBackend* simp = sim.get();
    Master m{make_config(), std::move(sim)};
    m.init();
    m.configure();
    TestControl c;  // before r
    const FieldLocation cw_loc{0, true};
    c.request_stop_at_cycle = 4;
    c.on_step = [&](CycleContext& ctx) {
        ctx.store<std::uint16_t>(cw_loc, ctx.stopping() ? std::uint16_t{0x0000} : std::uint16_t{0x000F});
    };
    {
        Runner r{m, fast_runner_cfg()};
        r.attach(1, c);
        r.start();
        // NO stop()/run() -- drop r at scope end. The control self-requests at cycle 4; the
        // ~Runner bounded teardown ships the cw=0 disable through the window, then close().
    }
    const auto out = simp->slave_io(1).outputs;
    CHECK_EQ(ethercat::load_le<std::uint16_t>(out.subspan(0, 2)), std::uint16_t{0x0000});  // disable shipped
}

TEST_MAIN()
