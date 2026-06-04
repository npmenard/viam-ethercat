// a6_validate -- TEMPORARY hardware bring-up validation for the A6-EC servo.
//
// Exercises the ethercat master library against a REAL A6 drive in safe stages:
//   Stage A (always):  open NIC -> enumerate -> SDO identity + key objects
//                       (PRE-OP, read-only; no PDO config, no motion).
//   Stage B (always):  configure (PDO remap + 0x6060 mode + OP) -> cyclic
//                       process() -> drive controlword to READY-TO-SWITCH-ON
//                       (motor NOT energized) and read live feedback.
//   --enable           additionally step the CiA402 ladder to OPERATION-ENABLED
//                       and HOLD at the current position (motor energizes,
//                       holding torque, NO commanded motion).
//   --move-pp R [RPM]  after enabling, command a RELATIVE PP move of R revs at
//                       RPM (default 60) via the bit4 new-setpoint handshake and
//                       watch convergence. *** MOTION -- opt-in only. ***
//
// Defaults are non-energizing and motionless. Runtime needs CAP_NET_RAW (raw
// socket); --enable/--move also benefit from RT scheduling but this tool runs
// best-effort (no SCHED_FIFO) -- fine for validation. NOT a production path;
// the real driver is the Viam module.

#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <iostream>
#include <memory>
#include <span>
#include <string>
#include <vector>

#include <malloc.h>
#include <pthread.h>
#include <sched.h>
#include <sys/mman.h>

#include "ethercat/cia402.hpp"
#include "ethercat/dc_sync.hpp"
#include "ethercat/errors.hpp"
#include "ethercat/master.hpp"
#include "ethercat/pdo_buffer.hpp"
#include "ethercat/pdo_mapping.hpp"
#include "ethercat/soem_backend.hpp"
#include "tools/sine_move.hpp"

namespace {

using namespace ethercat;

// --- CiA402 / A6 object indices (hex; the JSON config carries them in decimal) ---
constexpr std::uint16_t kControlword = 0x6040;
constexpr std::uint16_t kStatusword = 0x6041;
constexpr std::uint16_t kModeDisplay = 0x6061;
constexpr std::uint16_t kTargetPosition = 0x607A;
constexpr std::uint16_t kPositionActual = 0x6064;
constexpr std::uint16_t kProfileVelocity = 0x6081;
constexpr std::uint16_t kVelocityActual = 0x606C;
constexpr std::uint16_t kTorqueActual = 0x6077;
constexpr std::uint16_t kFaultCode = 0x603F;
// NOTE: this tool reads NO raw CoE objects -- identity comes from Master::slave_info()
// and live status/position from the PDO snapshot. The DC sync-type / health-counter
// objects (0x1C32 / 0x1C33) are the LIBRARY's to own; reaching OP via the bring-up FSM
// is the DC-confirmation. (The DC error-counter knowledge lives in the runbook DC §7,
// with a documented re-add path if a future DC-timing issue needs live counts.)

constexpr double kCountsPerRev = 131072.0;  // A6 single-turn encoder = 2^17

// Little-endian object values for an SDO write.
std::vector<std::byte> le16(std::uint16_t v) {
    return {static_cast<std::byte>(v & 0xFFU), static_cast<std::byte>((v >> 8U) & 0xFFU)};
}

std::atomic<bool> g_stop{false};
extern "C" void on_sigint(int) {
    g_stop.store(true);
}

// Build the MasterConfig for one A6, mirroring etc/a6-hardware.example.json (RxPDO
// 0x1600 = ctrl + target-pos + profile-vel; TxPDO 0x1A00 = fault + status +
// mode-display + pos + vel + torque). `mode` selects 0x6060: ProfilePosition (the
// bit4-handshake --move-pp) or CyclicSyncPosition (the streamed --move-sine). Both
// reuse the SAME PDO map -- 0x607A serves the PP target AND the CSP streamed target --
// so one builder covers both; only the post-enable control semantics differ.
MasterConfig build_a6_config(const std::string& ifname, std::int32_t dc_sync0_shift_ns, bool reset_fault, Cia402Mode mode) {
    MasterConfig cfg;
    cfg.ifname = ifname;
    cfg.target_loop_rate_hz = 1000;             // 1 ms SYNC0 = 4 x 250 us (A6-legal)
    cfg.use_distributed_clocks = true;          // A6 supports ONLY DC sync
    cfg.dc_sync0_shift_ns = dc_sync0_shift_ns;  // SYNC0 CyclShift; --dc-shift-ns sweep
    cfg.dc_settle_cycles = 1000;                // ~1 s post-OP grace while the phase finishes locking
    cfg.max_consecutive_wkc_errors = 5;
    // The bring-up SETTLE bound uses MasterConfig's default (dc_op_gate_cycles). SYNC0 is
    // armed in PRE-OP inside configure() (before config_map_group); the cyclic loop then
    // runs SETTLE (phase-locked PD) -> request OP once -> AWAIT_OP (hold for OP + sync),
    // all gapless.

    SlaveConfig a6;
    a6.slave_id = 1;
    a6.default_mode = mode;  // 0x6060 set in configure(); PP=1 (handshake) or CSP=8 (streamed sine)
    // The A6's fault-reset is the VENDOR SDO 0x2031:01 = 1 (NOT CiA402 bit7, CLAUDE.md).
    // --reset-fault clears a latent fault ONCE at bring-up (configure(), after SAFE-OP,
    // single port owner). Width per the A6 OD (U16 assumed); a wrong width just logs a
    // length abort (best-effort) -- adjust if first light shows one.
    if (reset_fault) {
        a6.fault_reset = SdoWrite{0x2031, 0x01, le16(1)};
    }
    // #20: we DELIBERATELY do NOT write 0x1C32:01 (SM sync-type). v2's config_map_group
    // lets the A6 self-select DC SYNC0; forcing it was the self-inflicted AL 0x0030
    // (CLAUDE.md). The generic preop/postremap SDO mechanisms remain for drives that
    // need them; the A6 needs none here.

    a6.rxpdo.assign_index = 0x1C12;
    a6.rxpdo.pdo_indices = {0x1600};
    a6.rxpdo.entries[0x1600] = {
        {kControlword, 0, 16},
        {kTargetPosition, 0, 32},
        {kProfileVelocity, 0, 32},
    };

    a6.txpdo.assign_index = 0x1C13;
    a6.txpdo.pdo_indices = {0x1A00};
    a6.txpdo.entries[0x1A00] = {
        {kFaultCode, 0, 16},
        {kStatusword, 0, 16},
        {kModeDisplay, 0, 8},
        {kPositionActual, 0, 32},
        {kVelocityActual, 0, 32},
        {kTorqueActual, 0, 16},
    };

    cfg.slaves.push_back(std::move(a6));
    return cfg;
}

// Read a scalar out of a slave's feedback image using the flat field table.
template <PdoScalar T>
T read_tx(const Master& m, std::uint16_t slave, std::span<const std::byte> img, std::uint16_t index, std::uint8_t sub = 0) {
    const FieldLocation loc = m.tx_field(slave, index, sub);
    return load_le<T>(img.subspan(loc.byte_offset, loc.byte_width));
}

// Write a scalar into a slave's command image using the flat field table.
template <PdoScalar T>
void write_rx(const Master& m, std::uint16_t slave, std::span<std::byte> img, std::uint16_t index, T value, std::uint8_t sub = 0) {
    const FieldLocation loc = m.rx_field(slave, index, sub);
    store_le<T>(img.subspan(loc.byte_offset, loc.byte_width), value);
}

// Read a scalar back out of a slave's COMMAND image (RxPDO) for logging -- shows what
// the master is actually sending the drive this cycle, decoded from the live image.
template <PdoScalar T>
T read_rx(const Master& m, std::uint16_t slave, std::span<const std::byte> img, std::uint16_t index, std::uint8_t sub = 0) {
    const FieldLocation loc = m.rx_field(slave, index, sub);
    return load_le<T>(img.subspan(loc.byte_offset, loc.byte_width));
}

// Lock memory + go SCHED_FIFO so the cyclic loop's jitter stays inside the DC
// SYNC0 window. WITHOUT this, best-effort scheduling jitter makes the A6 miss the
// sync window -> WKC drops to 0 and the drive faults out of OP. Best-effort: warns
// and continues if it lacks CAP_IPC_LOCK / CAP_SYS_NICE (run under sudo for DC).
bool setup_realtime(int priority) {
    // These are process-global, called ONCE at startup before any cyclic work --
    // the mt-unsafe lints (global locale/errno/heap state) are N/A here.
    // NOLINTBEGIN(concurrency-mt-unsafe)
    bool ok = true;
    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
        std::cerr << "    [rt] mlockall failed (need CAP_IPC_LOCK): " << std::strerror(errno) << '\n';
        ok = false;
    }
    (void)mallopt(M_TRIM_THRESHOLD, -1);  // keep the heap -- no page faults from trimming
    (void)mallopt(M_MMAP_MAX, 0);
    sched_param sp{};
    sp.sched_priority = priority;
    if (sched_setscheduler(0, SCHED_FIFO, &sp) != 0) {
        std::cerr << "    [rt] SCHED_FIFO(prio " << priority << ") failed (need CAP_SYS_NICE): " << std::strerror(errno) << '\n';
        ok = false;
    }
    return ok;
    // NOLINTEND(concurrency-mt-unsafe)
}

void sleep_until(struct timespec& next, long delta_ns) {
    next.tv_nsec += delta_ns;
    while (next.tv_nsec >= 1'000'000'000L) {
        next.tv_nsec -= 1'000'000'000L;
        next.tv_sec += 1;
    }
    while (next.tv_nsec < 0) {  // a DC phase correction can push the target slightly negative
        next.tv_nsec += 1'000'000'000L;
        next.tv_sec -= 1;
    }
    (void)clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, nullptr);
}

struct Options {
    std::string ifname = "enp86s0";
    bool enable = false;
    bool move_pp = false;
    bool move_sine = false;  // --move-sine: CSP streamed soft-started sine (energized)
    bool csp_probe = false;  // --csp-probe: bring up in CSP mode (0x6060=8) but DO NOT enable --
                             // just read+print feedback. Diagnostic: confirms whether read_tx
                             // returns valid sw/pos in CSP without energizing (CSP-feedback vs
                             // wedge isolation). Non-energizing; safe.
    bool reset_fault = false;
    double move_revs = 0.0;
    double move_rpm = 60.0;
    double sine_amplitude = 20000.0;       // counts (peak); --sine-amplitude
    double sine_period = 4.0;              // seconds; --sine-period
    std::int32_t follow_err_limit = 5000;  // counts; CSP tool-level following-error abort; --follow-err-limit
    int seconds = 6;
    std::int32_t dc_sync0_shift_ns = 0;  // SYNC0 CyclShift passed to ecx_dcsync0; sweep with --dc-shift-ns
};

}  // namespace

int main(int argc, char** argv) {
    Options opt;
    std::vector<std::string> args(argv + 1, argv + argc);
    for (std::size_t i = 0; i < args.size(); ++i) {
        const std::string& a = args[i];
        if (a == "--enable") {
            opt.enable = true;
        } else if (a == "--reset-fault") {
            opt.reset_fault = true;
        } else if (a == "--move-pp" && i + 1 < args.size()) {
            opt.move_pp = true;  // requires an EXPLICIT --enable (checked below) -- no implicit energize
            opt.move_revs = std::stod(args[++i]);
            if (i + 1 < args.size() && args[i + 1].rfind("--", 0) != 0) {
                opt.move_rpm = std::stod(args[++i]);
            }
        } else if (a == "--move-sine") {
            opt.move_sine = true;  // requires an EXPLICIT --enable (checked below) -- no implicit energize
        } else if (a == "--csp-probe") {
            opt.csp_probe = true;  // CSP mode, NO enable -- read+print feedback only (diagnostic)
        } else if (a == "--sine-amplitude" && i + 1 < args.size()) {
            opt.sine_amplitude = std::stod(args[++i]);
        } else if (a == "--sine-period" && i + 1 < args.size()) {
            opt.sine_period = std::stod(args[++i]);
        } else if (a == "--follow-err-limit" && i + 1 < args.size()) {
            opt.follow_err_limit = std::stoi(args[++i]);
        } else if (a == "--seconds" && i + 1 < args.size()) {
            opt.seconds = std::stoi(args[++i]);
        } else if (a == "--dc-shift-ns" && i + 1 < args.size()) {
            opt.dc_sync0_shift_ns = std::stoi(args[++i]);  // SYNC0 CyclShift passed to ecx_dcsync0
        } else if (a.rfind("--", 0) != 0) {
            opt.ifname = a;
        } else {
            std::cerr << "usage: a6_validate [ifname] [--enable] [--reset-fault] [--move-pp REVS [RPM]]\n"
                      << "                   [--move-sine [--sine-amplitude N] [--sine-period S]] [--csp-probe] [--seconds N] "
                         "[--dc-shift-ns NS]\n"
                      << "  The DC bring-up (#20) is automatic: configure() arms SYNC0 in PRE-OP + reaches SAFE-OP,\n"
                      << "  then the cyclic loop runs SETTLE (phase-locked PD) -> request OP once -> AWAIT_OP (hold\n"
                      << "  for OP + sync), all gapless. Er74.1 in SAFE-OP is normal pre-sync, clears at OP.\n"
                      << "  --enable: energize to OperationEnabled (holding torque). REQUIRED for any move below --\n"
                      << "            --move-pp/--move-sine no longer imply it, so a forgotten --enable fails closed.\n"
                      << "  --move-sine: *** ENERGIZED MOTION (needs --enable) *** CSP-mode soft-started position sine,\n"
                      << "               relative to the enable position. pos(t)=pos_enable + A*min(1,t/T)*sin(2*pi*t/T);\n"
                      << "               A=--sine-amplitude (counts, def 20000), T=--sine-period (s, def 4.0). CSP-safe\n"
                      << "               (no jump) + ramped (no velocity step). --follow-err-limit N (counts, def 5000):\n"
                      << "               abort+disable if |commanded-actual| exceeds it. Mutually exclusive with --move-pp.\n"
                      << "  --move-pp REVS [RPM]: *** MOTION (needs --enable) *** PP-mode relative move via the bit4 handshake.\n"
                      << "  --csp-probe: NON-energizing diagnostic -- bring up in CSP mode (0x6060=8), hold at\n"
                      << "               ReadyToSwitchOn (NO enable), print feedback. Confirms whether read_tx returns\n"
                      << "               valid sw/pos in CSP without energizing (isolates CSP-feedback vs a wedged drive).\n"
                      << "  --dc-shift-ns: SYNC0 pulse CyclShift (ecx_dcsync0) -- sweep to move the SYNC0 edge if needed.\n"
                      << "  --reset-fault: clear a latent drive fault at bring-up via the A6 vendor SDO 0x2031:01=1.\n";
            return 2;
        }
    }

    const int mode_flags = static_cast<int>(opt.move_pp) + static_cast<int>(opt.move_sine) + static_cast<int>(opt.csp_probe);
    if (mode_flags > 1) {
        std::cerr << "error: --move-pp / --move-sine / --csp-probe are mutually exclusive (one mode of operation at a time)\n";
        return 2;
    }
    // SAFETY: a move must NOT silently energize. --move-pp/--move-sine no longer imply --enable;
    // they require it explicitly, so a forgotten --enable FAILS CLOSED (refuses to energize)
    // instead of moving the shaft. (Near-miss: --move-sine used to imply enable -> a "no-enable"
    // invocation would have energized + moved at the default amplitude.) For a non-energizing CSP
    // feedback read, use --csp-probe.
    if ((opt.move_pp || opt.move_sine) && !opt.enable) {
        std::cerr << "error: --move-pp / --move-sine command ENERGIZED MOTION and require an explicit --enable\n"
                  << "       (safety: motion must be a deliberate opt-in -- a forgotten --enable will not silently move the shaft).\n"
                  << "       For a non-energizing CSP feedback read, use --csp-probe instead.\n";
        return 2;
    }
    // --csp-probe and --move-sine both select CSP (0x6060=8); --move-pp and the plain/no-move
    // default select ProfilePosition. --csp-probe is the non-energizing CSP feedback diagnostic.
    const Cia402Mode mode = (opt.move_sine || opt.csp_probe) ? Cia402Mode::CyclicSyncPosition : Cia402Mode::ProfilePosition;

    (void)std::signal(SIGINT, on_sigint);

    std::cout << "=== A6-EC validation on '" << opt.ifname << "' ===\n"
              << "mode: " << to_string(mode) << " | enable=" << (opt.enable ? "YES (motor energizes)" : "no") << " | move="
              << (opt.move_sine ? "SINE A=" + std::to_string(static_cast<long>(opt.sine_amplitude)) +
                                      "ct T=" + std::to_string(opt.sine_period) + "s (CSP, soft-started)"
                  : opt.move_pp ? std::to_string(opt.move_revs) + " rev @ " + std::to_string(opt.move_rpm) + " rpm (PP)"
                                : "none (hold)")
              << "\n\n";

    // DC SYNC0 cycle = loop period; the A6 requires an integer multiple of 250 us
    // (else Er74.0 "invalid sync cycle"). 1 kHz -> 1 ms = 4 x 250 us.
    constexpr std::uint32_t kCycleNs = 1'000'000'000U / 1000U;
    static_assert(kCycleNs % 250'000U == 0, "SYNC0 cycle must be a 250us multiple for the A6");

    // DC sync is timing-critical: lock memory + SCHED_FIFO so jitter stays inside
    // the sync window. Without it the A6 drops WKC and faults out of OP.
    if (setup_realtime(80)) {
        std::cout << "[rt] SCHED_FIFO + mlockall engaged (DC-safe timing).\n\n";
    } else {
        std::cerr << "    [rt] continuing best-effort -- DC SYNC0 may fault under jitter; run with sudo.\n\n";
    }

    std::cout << "[dc] phase-lock target = auto(mid-cycle) | SYNC0 CyclShift = " << opt.dc_sync0_shift_ns << "ns"
              << " | fault-reset at bring-up = " << (opt.reset_fault ? "ON (vendor 0x2031:01=1)" : "off") << "\n\n";

    Master master(build_a6_config(opt.ifname, opt.dc_sync0_shift_ns, opt.reset_fault, mode), std::make_unique<SoemBackend>());

    // --- Stage A: open + enumerate + read-only SDO identity (PRE-OP) ---
    try {
        master.init();
    } catch (const Error& e) {
        std::cerr << "init failed: " << e.what() << '\n';
        return 1;
    }
    // init() throws unless the enumerated count matches the config (1 slave), so
    // reaching here means the A6 was found. (slave_count()/slaves_ isn't populated
    // until configure(), so don't read it yet.)
    std::cout << "[A] bus up: A6 enumerated (1 slave, matches config).\n";
    // Identity from the library's enumeration view (SlaveInfo) -- NOT a raw CoE 0x1018
    // poke. This tool consumes the library API only; the library OWNS EtherCAT object
    // access (the user's boundary). Live status/position come from the PDO snapshot in
    // the loop below, so no pre-loop CoE feedback peek is needed either.
    const SlaveInfo info = master.slave_info(1);
    std::cout << "    identity: vendor=0x" << std::hex << info.vendor_id << " product=0x" << info.product_code << " rev=0x" << info.revision
              << std::dec << " name=\"" << info.name << "\" (Rx " << info.output_bytes << "B / Tx " << info.input_bytes << "B)\n"
              << "[A] OK -- EtherCAT enumeration confirmed.\n\n";

    const std::uint16_t slave = 1;

    // --- Stage B: configure to SAFE-OP + DC, then ONE continuous loop that
    // phase-locks, requests OP, and runs the CiA402 sequence -- all on an unbroken
    // cadence. The A6 faults out of OP on a SINGLE missed SYNC0 frame, so we must NOT
    // have a gap between bring-up and the steady loop. configure() stops at SAFE-OP + DC
    // (SYNC0 armed in PRE-OP); the loop below owns every frame from there.
    try {
        master.configure();
    } catch (const Error& e) {
        std::cerr << "[B] configure failed: " << e.what() << "\n"
                  << "    (an AL-reject at the SAFE-OP transition lands here; the AL status code in the message names why.)\n";
        return 1;
    }
    std::cout << "[B] configured to SAFE-OP, DC enabled (expected WKC=" << master.expected_wkc()
              << "); running the bring-up FSM (SETTLE -> request OP -> AWAIT_OP), gapless + phase-locked...\n";

    const auto profile_vel = static_cast<std::uint32_t>(opt.move_rpm / 60.0 * kCountsPerRev);
    const Cia402Fsm fsm;
    const Cia402State goal = opt.enable ? Cia402State::OperationEnabled : Cia402State::ReadyToSwitchOn;
    constexpr std::uint32_t kLoopHz = 1000;
    const std::uint32_t period_ns = 1'000'000'000U / kLoopHz;
    // Phase-lock TARGET: mid-cycle (period/2 from the DC base) -- locking on the SYNC0
    // edge leaves no jitter margin. The single DC phase knob is the ecx_dcsync0 CyclShift
    // (--dc-shift-ns); the PI send-phase target is derived here (#20 consolidated).
    const std::int64_t dc_shift = static_cast<std::int64_t>(period_ns) / 2;

    const auto bringup_label = [](BringupStatus s) -> const char* {
        switch (s) {
            case BringupStatus::Gating:
                return "GATE";
            case BringupStatus::AwaitingOp:
                return "AWAIT_OP";
            case BringupStatus::Operational:
                return "OPERATIONAL";
            case BringupStatus::Aborted:
                return "ABORTED";
        }
        return "?";
    };

    struct timespec next{};
    (void)clock_gettime(CLOCK_MONOTONIC, &next);
    std::int64_t dc_integral = 0;
    long dc_off = 0;

    // --- Phase 1: DC bring-up (#20). configure() armed SYNC0 in PRE-OP + left the bus at
    // SAFE-OP; the Master FSM runs SETTLE (phase-locked PD) -> request OP once -> AWAIT_OP
    // (hold for OP + Er74.1-cleared + WKC), while THIS loop owns the gapless, phase-locked
    // cadence. drive_sync_faulted = our read of 0x603F == 0x8700 (Er74.1) from the prior
    // cycle; it's the NORMAL pre-sync state in SAFE-OP (clears at OP), so it gates the
    // post-OP hold, not the request -- see bringup_step.
    bool reached_op = false;
    {
        std::uint64_t btick = 0;
        while (!g_stop.load()) {
            sleep_until(next, static_cast<long>(period_ns) + dc_off);
            const std::span<const std::byte> in = master.input_image(slave);
            std::uint16_t fc = 0;
            try {
                fc = read_tx<std::uint16_t>(master, slave, in, kFaultCode);
            } catch (const Error&) {  // 0x603F not mapped -> treat as no sync fault
            }
            const BringupStatus bs = master.bringup_step(fc == 0x8700);
            dc_off = dc_phase_correction(master.dc_time(), static_cast<std::int64_t>(period_ns), dc_integral, dc_shift);
            if (++btick % 200 == 0) {
                std::cout << "[B] bring-up t=" << btick << " " << bringup_label(bs) << " wkc=" << master.last_wkc() << "/"
                          << master.expected_wkc() << " dcPhase=" << (master.dc_time() % static_cast<std::int64_t>(period_ns)) << "ns\n";
            }
            if (bs == BringupStatus::Operational) {
                reached_op = true;
                break;
            }
            if (bs == BringupStatus::Aborted) {
                std::cerr << "[B] !!! BRING-UP ABORTED: OP did not hold within the await window -- the drive did not reach\n"
                          << "    OP with WKC 3/3 + Er74.1 cleared (SYNC0 likely not truly established). OP was requested\n"
                          << "    ONCE + not re-requested (repeated Er74 OP-entry wedges the A6). Power-cycle + check DC\n"
                          << "    wiring/cycle; sweep --dc-shift-ns. last_error: " << master.last_error() << '\n';
                break;
            }
        }
    }
    if (!reached_op) {
        std::cout << "\n[B] bring-up did not reach OP; closing. (last_error: " << master.last_error() << ")\n";
        master.close();
        return 1;
    }
    std::cout << "[B] *** OPERATIONAL *** WKC=" << master.last_wkc() << "/" << master.expected_wkc()
              << " -- DC bring-up complete (no Er74.1), entering CiA402 control loop\n";

    // --- Phase 2: steady CiA402 control loop (hold / optional PP move), phase-locked.
    const auto t0 = std::chrono::steady_clock::now();
    std::uint16_t last_cw = 0;
    bool announced_op = false;
    bool setpoint_latched = false;
    bool move_done = false;
    bool was_faulted = false;
    bool hold_captured = false;
    std::int32_t hold_pos = 0;
    std::int32_t target = 0;
    std::uint64_t tick = 0;
    int wkc_bad = 0;
    // CSP sine state: pos_enable (origin captured at OperationEnabled), enable_tick (t=0
    // reference), last_sine_target (held during graceful shutdown so the shaft isn't yanked).
    std::int32_t pos_enable = 0;
    std::uint64_t enable_tick = 0;
    std::int32_t last_sine_target = 0;
    bool safety_abort = false;  // CSP: bit13/0x603F tripped -> disable immediately (no hold)

    while (!g_stop.load()) {
        sleep_until(next, static_cast<long>(period_ns) + dc_off);
        master.process();
        ++tick;
        const int raw_wkc = master.last_wkc();
        if (raw_wkc != master.expected_wkc()) {
            ++wkc_bad;
        }
        const std::int64_t dct = master.dc_time();
        dc_off = dc_phase_correction(dct, static_cast<std::int64_t>(period_ns), dc_integral, dc_shift);

        const std::span<const std::byte> in = master.input_image(slave);
        const Status status{read_tx<std::uint16_t>(master, slave, in, kStatusword)};
        const std::int32_t pos = read_tx<std::int32_t>(master, slave, in, kPositionActual);
        const std::int32_t vel = read_tx<std::int32_t>(master, slave, in, kVelocityActual);

        if (!hold_captured) {
            hold_pos = pos;
            target = pos;
            hold_captured = true;
        }

        const bool faulted = status.decode() == Cia402State::Fault;
        if (faulted && !was_faulted) {
            const auto fault_code = read_tx<std::uint16_t>(master, slave, in, kFaultCode);
            std::cout << "[B] !!! DRIVE FAULT t=" << tick / kLoopHz << "s: 0x603F=0x" << std::hex << fault_code << " sw=0x" << status.raw
                      << std::dec << " dcPhase=" << (dct % static_cast<std::int64_t>(period_ns)) << "ns\n";
        } else if (!faulted && was_faulted) {
            std::cout << "[B] *** FAULT CLEARED *** -> " << to_string(status.decode()) << '\n';
        }
        was_faulted = faulted;

        // CSP energized-motion safety net (#24): once we're STREAMING the sine, ANY
        // drive-unhappy signal -- Fault (bit3), following-error / position-deviation
        // (statusword bit13), or a nonzero 0x603F -- aborts the move and disables
        // IMMEDIATELY (no hold). We do NOT auto-reset + re-energize mid-motion. At ~14 rpm
        // soft-started this won't trip (0x6065 deviation window is ~24 revs); it's the
        // guard for the first energized run. (The enable-ladder fault path above still
        // clears a PRE-enable latent fault; this only arms after OperationEnabled.)
        if (opt.move_sine && announced_op) {
            const auto fc_now = read_tx<std::uint16_t>(master, slave, in, kFaultCode);
            // Tool-level following-error abort (architect-required): how far actual lags the
            // last commanded target. This is the EARLY net -- the drive's own bit13 fires
            // only at 0x6065 (~24 revs on the A6), a full runaway; a moderate deviation
            // (can't track / mechanical bind / tuning surprise) trips this ~5000-count
            // (~0.04 rev) limit first. Skip the seed cycle (no commanded target streamed yet).
            const std::int32_t follow_err = (tick > enable_tick) ? (last_sine_target - pos) : 0;
            if (status.fault() || status.following_error() || fc_now != 0 || std::abs(follow_err) > opt.follow_err_limit) {
                std::cerr << "[B] !!! CSP SAFETY ABORT: fault(bit3)=" << status.fault()
                          << " followingError(bit13)=" << status.following_error() << " 0x603F=0x" << std::hex << fc_now << std::dec
                          << " follow_err=" << follow_err << " (limit " << opt.follow_err_limit
                          << ") -- stopping the sine + disabling immediately.\n";
                safety_abort = true;
                break;
            }
        }

        const std::span<std::byte> out = master.outputs(slave);
        std::uint16_t cw = fsm.step(status, goal);
        if (faulted) {
            // Generic CiA402 bit7 fault-reset edge (the A6's real reset is the vendor
            // 0x2031:01 SDO, issued at bring-up; this is the in-loop steady-state fallback).
            cw = (last_cw & ControlWord::kFaultResetBit) ? 0x0000 : ControlWord::fault_reset();
        } else if (opt.enable && status.operation_enabled()) {
            if (!announced_op) {
                pos_enable = pos;  // CSP-safe origin: actual position the cycle OperationEnabled is reached
                enable_tick = tick;
                announced_op = true;
                std::cout << "[B] *** OPERATION ENABLED *** (motor energized at pos=" << pos_enable << ")\n";
            }
            if (opt.move_sine) {
                // CSP: stream the soft-started relative-to-enable sine every cycle; cw held
                // at 0x0F, NO bit4 handshake (that's PP). CSP-safe: sin(0)=0 -> the first
                // target == pos_enable (zero jump); the amplitude ramps in over the first
                // period so velocity starts gentle (see sine_move.hpp). No profile-velocity
                // (0x6081) write -- CSP follows the streamed position directly.
                const double t = static_cast<double>(tick - enable_tick) / static_cast<double>(kLoopHz);
                target = ethercat::tools::csp_target_counts(true, pos, pos_enable, opt.sine_amplitude, opt.sine_period, t);
                last_sine_target = target;
                write_rx<std::int32_t>(master, slave, out, kTargetPosition, target);
                cw = ControlWord::enable_operation();  // 0x0F
            } else {
                // --move-pp (bit4 handshake) or plain hold at the enable position.
                if (opt.move_pp && !move_done) {
                    target = hold_pos + static_cast<std::int32_t>(opt.move_revs * kCountsPerRev);
                }
                write_rx<std::int32_t>(master, slave, out, kTargetPosition, target);
                write_rx<std::uint32_t>(master, slave, out, kProfileVelocity, profile_vel);
                std::uint16_t base = ControlWord::enable_operation();  // 0x0F
                if (opt.move_pp && !move_done) {
                    // bit4 handshake: assert new-setpoint, hold until the drive acks (bit12),
                    // then drop it so the next move can re-arm.
                    if (!setpoint_latched) {
                        base = ControlWord::with_new_setpoint(base, true);  // 0x1F
                        if (status.setpoint_acknowledged()) {
                            setpoint_latched = true;
                        }
                    } else {
                        base = ControlWord::with_new_setpoint(base, false);  // back to 0x0F
                        if (std::abs(pos - target) < 300) {
                            move_done = true;
                            std::cout << "[B] move complete: pos=" << pos << " (target " << target << ")\n";
                        }
                    }
                }
                cw = base;
            }
        } else if (opt.enable && opt.move_sine) {
            // CSP enable-jump fix: we are climbing the CiA402 ladder (06->07->0F) toward
            // OperationEnabled but not there yet (and not faulted). cw is already the ladder
            // step from fsm.step() above; the critical bit is 0x607A. In CSP the drive latches
            // its FIRST setpoint from the 0x607A on the cw=0x0F frame that TRIGGERS OE -- which
            // is one of THESE ladder frames, not a post-OE frame. So stream target = live actual
            // every ladder cycle -> that frame carries target==actual -> genuine zero jump
            // (csp_target_counts(false,...) returns pos_actual). Without this, 0x607A stays 0
            // and the drive slews from the absolute-encoder position toward 0 on enable.
            target = ethercat::tools::csp_target_counts(false, pos, pos_enable, opt.sine_amplitude, opt.sine_period, 0.0);
            last_sine_target = target;  // keep coherent for the graceful-hold + follow-err seed
            write_rx<std::int32_t>(master, slave, out, kTargetPosition, target);
            // cw left as fsm.step()'s ladder climb (06/07/0F as appropriate).
        } else if (!opt.enable) {
            cw = ControlWord::shutdown();  // 0x06 -> ReadyToSwitchOn, NOT energized
        }
        write_rx<std::uint16_t>(master, slave, out, kControlword, cw);
        last_cw = cw;

        if (tick % 200 == 0) {  // ~5 Hz decoded-PDO print
            const std::span<const std::byte> outimg = master.outputs(slave);
            const auto rx_cw = read_rx<std::uint16_t>(master, slave, outimg, kControlword);
            const auto rx_tpos = read_rx<std::int32_t>(master, slave, outimg, kTargetPosition);
            const auto fc = read_tx<std::uint16_t>(master, slave, in, kFaultCode);
            const auto mode_now = read_tx<std::int8_t>(master, slave, in, kModeDisplay);
            // CSP tracking: commanded target (0x607A) vs actual (0x6064) + following error.
            const std::int32_t follow_err = rx_tpos - pos;
            std::cout << "    t=" << tick / kLoopHz << "s " << to_string(status.decode()) << " sw=0x" << std::hex << status.raw
                      << " 0x603F=0x" << fc << std::dec << " mode=" << static_cast<int>(mode_now) << " Rx.cw=0x" << std::hex << rx_cw
                      << std::dec << " cmdTarget=" << rx_tpos << " pos=" << pos << " followErr=" << follow_err << " vel=" << vel
                      << " wkc=" << raw_wkc << "/" << master.expected_wkc() << " badWKC=" << wkc_bad
                      << " dcPhase=" << (dct % static_cast<std::int64_t>(period_ns)) << "ns" << (master.fault() ? " [BUS FAULT]" : "")
                      << '\n';
        }

        if (master.fault()) {
            std::cerr << "[B] bus fault latched: " << master.last_error() << '\n';
            break;
        }
        if (std::chrono::steady_clock::now() - t0 > std::chrono::seconds(opt.seconds)) {
            break;
        }
        if (opt.move_pp && move_done && tick % 500 == 0) {
            break;  // let it settle a moment, then finish
        }
    }

    // --- graceful shutdown ---
    // CSP: stop streaming the sine but HOLD the last commanded position a few cycles
    // (cw stays 0x0F, target frozen at last_sine_target) before disabling -- do NOT snap
    // to 0 or to pos_enable, which would yank the shaft. The drive is still in OP here, so
    // keep PD phase-locked. (Skipped if we never energized / never started the sine.)
    if (opt.move_sine && announced_op && !master.fault() && !safety_abort) {
        std::cout << "\n[B] sine stopped -- holding last commanded pos=" << last_sine_target << " for ~100ms, then disabling...\n";
        for (int i = 0; i < 100; ++i) {
            sleep_until(next, static_cast<long>(period_ns) + dc_off);
            master.process();
            dc_off = dc_phase_correction(master.dc_time(), static_cast<std::int64_t>(period_ns), dc_integral, dc_shift);
            write_rx<std::int32_t>(master, slave, master.outputs(slave), kTargetPosition, last_sine_target);
            write_rx<std::uint16_t>(master, slave, master.outputs(slave), kControlword, ControlWord::enable_operation());
        }
    }
    std::cout << "\n[B] disabling drive (controlword -> 0x00) and closing...\n";
    for (int i = 0; i < 50; ++i) {
        master.process();
        write_rx<std::uint16_t>(master, slave, master.outputs(slave), kControlword, ControlWord::disable_voltage());
        sleep_until(next, static_cast<long>(period_ns));
    }
    master.close();

    std::cout << "=== done. bad-WKC cycles: " << wkc_bad << " / " << tick << " (raw per-cycle, not the masked working_counter) ===\n";
    return 0;
}
