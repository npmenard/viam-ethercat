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

#include <algorithm>
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
constexpr std::uint16_t kIdentity = 0x1018;

// SM synchronization-TYPE objects (ETG std). 0x1C32:01 = SM2/output (RxPDO) sync
// type, 0x1C33:01 = SM3/input (TxPDO) sync type. Value 2 = "DC SYNC0". These tell
// the drive's APPLICATION layer to synchronize to the SYNC0 pulse. The bench panel
// shows Er74.1 "no sync signal" even with the ESC SYNC0 active (configdc TRUE,
// hasdc=1, DCtime advancing) -- the classic cause is the drive still running
// free-run/SM-sync because 0x1C32:01 was never set to 2. Opt-in via --sm-dc-sync
// (a read-only-object abort here would otherwise block the timing-sweep runs).
constexpr std::uint16_t kSm2SyncType = 0x1C32;  // SM2 (outputs/RxPDO)
constexpr std::uint16_t kSm3SyncType = 0x1C33;  // SM3 (inputs/TxPDO)
constexpr std::uint8_t kSyncTypeSub = 0x01;
constexpr std::uint16_t kSyncTypeDcSync0 = 2;  // ETG sync type 2 = DC SYNC0

constexpr double kCountsPerRev = 131072.0;  // A6 single-turn encoder = 2^17

// Little-endian 2-byte (U16) object value for an SDO write.
std::vector<std::byte> le16(std::uint16_t v) {
    return {static_cast<std::byte>(v & 0xFFU), static_cast<std::byte>((v >> 8U) & 0xFFU)};
}

std::atomic<bool> g_stop{false};
extern "C" void on_sigint(int) {
    g_stop.store(true);
}

// Build the PROFILE POSITION MasterConfig for one A6, mirroring
// etc/a6-hardware.example.json (RxPDO 0x1600 = ctrl + target-pos + profile-vel;
// TxPDO 0x1A00 = fault + status + mode-display + pos + vel + torque).
MasterConfig build_a6_pp_config(const std::string& ifname, std::int32_t dc_target_ns, std::int32_t dc_sync0_shift_ns, bool sm_dc_sync) {
    MasterConfig cfg;
    cfg.ifname = ifname;
    cfg.target_loop_rate_hz = 1000;             // 1 ms SYNC0 = 4 x 250 us (A6-legal)
    cfg.use_distributed_clocks = true;          // A6 supports ONLY DC sync
    cfg.dc_lock_cycles = 2000;                  // up to 2 s phase-locking warmup -> enter OP aligned
    cfg.dc_settle_cycles = 1000;                // ~1 s post-OP grace while the phase finishes locking
    cfg.dc_sync_shift_ns = dc_target_ns;        // send-phase target (-1 = auto mid-cycle); --dc-target-ns sweep
    cfg.dc_sync0_shift_ns = dc_sync0_shift_ns;  // SYNC0 CyclShift; --dc-shift-ns sweep
    cfg.max_consecutive_wkc_errors = 5;

    SlaveConfig a6;
    a6.slave_id = 1;
    a6.default_mode = Cia402Mode::ProfilePosition;

    // --sm-dc-sync: put the SM2/SM3 application sync into DC SYNC0 mode (PRE-OP).
    // Targeted Er74.1 "no sync signal" fix -- the ESC SYNC0 is active but the drive
    // app may still be free-run. Opt-in: a read-only-object abort would otherwise
    // block the plain timing-sweep runs. (Panel-confirmed jitter is NOT the issue,
    // so the C13 sync-tolerance writes were dropped -- the generic preop_sdo_writes
    // mechanism stays for exactly this kind of drive-config write.)
    if (sm_dc_sync) {
        // optional=true: 0x1C33:01 (input SM) reads a non-standard value on the A6 and
        // may be read-only -- a rejection there must NOT abort the run before we learn
        // whether 0x1C32:01 (output SM, the one consuming our RxPDO) took =2.
        a6.preop_sdo_writes = {
            {kSm2SyncType, kSyncTypeSub, le16(kSyncTypeDcSync0), /*optional=*/true},  // SM2 outputs -> DC SYNC0
            {kSm3SyncType, kSyncTypeSub, le16(kSyncTypeDcSync0), /*optional=*/true},  // SM3 inputs  -> DC SYNC0
        };
    }

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
    bool reset_fault = false;
    double move_revs = 0.0;
    double move_rpm = 60.0;
    int seconds = 6;
    std::int32_t dc_target_ns = -1;      // send-phase lock target (-1 = auto mid-cycle); sweep with --dc-target-ns
    std::int32_t dc_sync0_shift_ns = 0;  // SYNC0 CyclShift; sweep with --dc-shift-ns
    bool sm_dc_sync = false;             // --sm-dc-sync: write SM2/SM3 sync type = DC SYNC0 (Er74.1 fix)
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
            opt.move_pp = true;
            opt.enable = true;  // a move requires enabling
            opt.move_revs = std::stod(args[++i]);
            if (i + 1 < args.size() && args[i + 1].rfind("--", 0) != 0) {
                opt.move_rpm = std::stod(args[++i]);
            }
        } else if (a == "--seconds" && i + 1 < args.size()) {
            opt.seconds = std::stoi(args[++i]);
        } else if (a == "--dc-target-ns" && i + 1 < args.size()) {
            opt.dc_target_ns = std::stoi(args[++i]);  // send-phase lock target (-1 = auto mid-cycle)
        } else if (a == "--dc-shift-ns" && i + 1 < args.size()) {
            opt.dc_sync0_shift_ns = std::stoi(args[++i]);  // SYNC0 CyclShift passed to ecx_dcsync0
        } else if (a == "--sm-dc-sync") {
            opt.sm_dc_sync = true;  // write SM2/SM3 sync type = DC SYNC0 (targeted Er74.1 fix)
        } else if (a.rfind("--", 0) != 0) {
            opt.ifname = a;
        } else {
            std::cerr << "usage: a6_validate [ifname] [--enable] [--reset-fault] [--move-pp REVS [RPM]] [--seconds N]\n"
                      << "                   [--dc-target-ns NS] [--dc-shift-ns NS] [--sm-dc-sync]\n";
            return 2;
        }
    }

    (void)std::signal(SIGINT, on_sigint);

    std::cout << "=== A6-EC validation on '" << opt.ifname << "' ===\n"
              << "mode: PROFILE POSITION | enable=" << (opt.enable ? "YES (motor energizes)" : "no")
              << " | move=" << (opt.move_pp ? std::to_string(opt.move_revs) + " rev @ " + std::to_string(opt.move_rpm) + " rpm" : "none")
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

    std::cout << "[dc] send-phase target = " << (opt.dc_target_ns < 0 ? "auto(mid-cycle)" : std::to_string(opt.dc_target_ns) + "ns")
              << " | SYNC0 CyclShift = " << opt.dc_sync0_shift_ns << "ns"
              << " | SM DC-sync write = " << (opt.sm_dc_sync ? "ON (0x1C32/33:01=2)" : "off") << "\n\n";

    Master master(build_a6_pp_config(opt.ifname, opt.dc_target_ns, opt.dc_sync0_shift_ns, opt.sm_dc_sync), std::make_unique<SoemBackend>());

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
    try {
        const auto vendor = master.sdo_read<std::uint32_t>(1, kIdentity, 1);
        const auto product = master.sdo_read<std::uint32_t>(1, kIdentity, 2);
        const auto revision = master.sdo_read<std::uint32_t>(1, kIdentity, 3);
        const auto mode_disp = master.sdo_read<std::int8_t>(1, kModeDisplay, 0);
        const auto sw = master.sdo_read<std::uint16_t>(1, kStatusword, 0);
        const auto pos = master.sdo_read<std::int32_t>(1, kPositionActual, 0);
        std::cout << "    identity 0x1018: vendor=0x" << std::hex << vendor << " product=0x" << product << " rev=0x" << revision << std::dec
                  << "\n"
                  << "    statusword 0x6041 = 0x" << std::hex << sw << std::dec << " -> " << to_string(Status{sw}.decode()) << "\n"
                  << "    mode display 0x6061 = " << static_cast<int>(mode_disp) << " | position 0x6064 = " << pos << " counts\n";
    } catch (const Error& e) {
        std::cerr << "[A] SDO read failed (CoE mailbox issue): " << e.what() << '\n';
        return 1;
    }
    std::cout << "[A] OK -- EtherCAT comms + CoE SDO confirmed.\n\n";

    // --- Stage B: configure to SAFE-OP + DC, then ONE continuous loop that
    // phase-locks, requests OP, and runs the CiA402 sequence -- all on an unbroken
    // cadence. The A6 faults out of OP on a SINGLE missed SYNC0 frame, so we must
    // NOT have a gap between bring-up and the steady loop (configure() reaching OP
    // then handing off to a fresh loop dropped a frame -> Er74). reach_op=false
    // stops configure at SAFE-OP + DC; the loop below owns every frame from there.
    try {
        master.configure(false);
    } catch (const Error& e) {
        std::cerr << "[B] configure failed: " << e.what() << "\n"
                  << "    (a PdoMappingError here means the A6 rejected our assumed PDO map -- check the ESI.)\n";
        return 1;
    }
    std::cout << "[B] configured to SAFE-OP, DC enabled (expected WKC=" << master.expected_wkc()
              << "); phase-locking, then requesting OP with NO frame gap...\n";

    const std::uint16_t slave = 1;

    // SM sync-type diagnostic (ALWAYS -- these are risk-free SDO reads). Tells us the
    // drive's ACTUAL application sync mode (2 = DC SYNC0) vs FreeRun(0)/SM(1): if it's
    // not 2 while we're driving SYNC0, that's the Er74.1 cause. 0x1C32:04 is the
    // SUPPORTED-types bitmask (does the A6 even offer DC?), 0x1C32:05 the min cycle.
    // With --sm-dc-sync the values reflect our PRE-OP write; without, the drive default.
    try {
        const auto sm2 = master.sdo_read<std::uint16_t>(slave, kSm2SyncType, kSyncTypeSub);
        const auto sm3 = master.sdo_read<std::uint16_t>(slave, kSm3SyncType, kSyncTypeSub);
        const auto supported = master.sdo_read<std::uint16_t>(slave, kSm2SyncType, 0x04);
        const auto min_cycle = master.sdo_read<std::uint32_t>(slave, kSm2SyncType, 0x05);
        const bool dc = sm2 == kSyncTypeDcSync0 && sm3 == kSyncTypeDcSync0;
        std::cout << "[dc] SM sync-type: 0x1C32:01(SM2)=" << sm2 << " 0x1C33:01(SM3)=" << sm3
                  << (dc ? "  -> DC SYNC0 active" : "  -> NOT DC SYNC0 (0=FreeRun,1=SM,2=DC) -- candidate Er74.1 cause") << "\n"
                  << "[dc]   0x1C32:04 supportedTypes=0x" << std::hex << supported << std::dec << " 0x1C32:05 minCycle=" << min_cycle
                  << "ns" << (opt.sm_dc_sync ? "  [--sm-dc-sync wrote :01=2]" : "") << '\n';
    } catch (const Error& e) {
        std::cerr << "[dc] SM sync-type read failed (object absent?): " << e.what() << '\n';
    }
    const auto profile_vel = static_cast<std::uint32_t>(opt.move_rpm / 60.0 * kCountsPerRev);
    const Cia402Fsm fsm;
    const Cia402State goal = opt.enable ? Cia402State::OperationEnabled : Cia402State::ReadyToSwitchOn;
    constexpr std::uint32_t kLoopHz = 1000;
    const std::uint32_t period_ns = 1'000'000'000U / kLoopHz;
    // Send-phase lock target: --dc-target-ns overrides the default mid-cycle. The
    // master phase-locks (dc_time mod cycle) to this offset; sweeping it (with the
    // SYNC0 CyclShift) is how we find the window where the A6 latches a FRESH frame.
    const std::int64_t dc_shift =
        opt.dc_target_ns < 0 ? static_cast<std::int64_t>(period_ns) / 2 : static_cast<std::int64_t>(opt.dc_target_ns);

    // Short label for the EtherCAT state phase (SAFE-OP -> ->OP -> OP) used in prints.
    const auto phase_label = [](bool in_op, bool requested) -> const char* {
        if (in_op) {
            return "OP";
        }
        return requested ? "->OP" : "SAFEOP";
    };

    struct timespec next{};
    (void)clock_gettime(CLOCK_MONOTONIC, &next);
    const auto t0 = std::chrono::steady_clock::now();

    std::uint16_t last_cw = 0;
    bool announced_op = false;
    bool op_reached_announced = false;
    bool op_requested = false;
    bool setpoint_latched = false;
    bool move_done = false;
    bool was_faulted = false;
    bool hold_captured = false;
    std::int32_t hold_pos = 0;
    std::int32_t target = 0;
    std::uint64_t tick = 0;
    int wkc_bad = 0;
    int wkc_bad_streak = 0;
    int wkc_bad_max_streak = 0;
    int locked_streak = 0;
    std::int64_t dc_integral = 0;
    long dc_off = 0;

    while (!g_stop.load()) {
        // DEADLINE-FIRST: advance the phase-corrected deadline and sleep to it BEFORE
        // exchanging -- one unbroken cadence from the first frame, so the drive never
        // sees a gap through SAFE-OP -> OP.
        sleep_until(next, static_cast<long>(period_ns) + dc_off);
        master.process();
        ++tick;

        const int raw_wkc = master.last_wkc();
        if (raw_wkc != master.expected_wkc()) {
            ++wkc_bad;
            ++wkc_bad_streak;
            wkc_bad_max_streak = std::max(wkc_bad_max_streak, wkc_bad_streak);
        } else {
            wkc_bad_streak = 0;
        }

        // DC phase-lock correction for the NEXT cycle (mid-cycle target).
        const std::int64_t dct = master.dc_time();
        dc_off = dc_phase_correction(dct, static_cast<std::int64_t>(period_ns), dc_integral, dc_shift);
        locked_streak = dc_phase_locked(dct, static_cast<std::int64_t>(period_ns), dc_shift) ? locked_streak + 1 : 0;

        // Once locked in SAFE-OP, request OP -- the loop keeps cycling through the
        // transition, so the drive sees no gap.
        if (!op_requested && locked_streak >= 50) {
            master.request_op();
            op_requested = true;
            std::cout << "[B] phase locked (dcPhase~" << (dct % static_cast<std::int64_t>(period_ns)) << "ns) -> requesting OP\n";
        }
        // Full WKC == outputs processing == in OP with live command flow.
        const bool op = op_requested && raw_wkc == master.expected_wkc();

        const std::span<const std::byte> in = master.input_image(slave);
        const Status status{read_tx<std::uint16_t>(master, slave, in, kStatusword)};
        const std::int32_t pos = read_tx<std::int32_t>(master, slave, in, kPositionActual);
        const std::int32_t vel = read_tx<std::int32_t>(master, slave, in, kVelocityActual);

        if (op && !hold_captured) {
            hold_pos = pos;
            target = pos;
            hold_captured = true;
        }
        if (op && !op_reached_announced) {
            std::cout << "[B] *** OPERATIONAL *** WKC=" << raw_wkc << "/" << master.expected_wkc() << ", holding at " << hold_pos << '\n';
            op_reached_announced = true;
        }

        // DIAGNOSTIC: capture the CiA402 error code 0x603F on EVERY Fault entry (not
        // just under --reset-fault). The A6 faults AT OP ENTRY on a DC timing miss --
        // 0x8700 = Er74.1 "no SYNC0", 0x6320 = Er74.0 "cycle error" -- so the exact
        // code at the transition tells the bench which way to sweep --dc-target-ns /
        // --dc-shift-ns. Printed immediately so it's never lost between status prints.
        const bool faulted = status.decode() == Cia402State::Fault;
        if (faulted && !was_faulted) {
            const auto fault_code = read_tx<std::uint16_t>(master, slave, in, kFaultCode);
            std::cout << "[B] !!! DRIVE FAULT @ " << phase_label(op, op_requested) << " t=" << tick / kLoopHz << "s: 0x603F=0x" << std::hex
                      << fault_code << " sw=0x" << status.raw << std::dec << " dcPhase=" << (dct % static_cast<std::int64_t>(period_ns))
                      << "ns";
            if (opt.reset_fault) {
                std::cout << " -- running CiA402 fault-reset (no energize)...";
            }
            std::cout << '\n';
        } else if (!faulted && was_faulted) {
            std::cout << "[B] *** FAULT CLEARED *** -> " << to_string(status.decode()) << '\n';
        }
        was_faulted = faulted;

        // Decide the controlword for this cycle.
        std::uint16_t cw = fsm.step(status, goal);
        // Fault-reset edge: step() returns the 0x80 LEVEL while faulted; create
        // the rising edge by dropping bit7 for one cycle when it's already set.
        if ((cw & ControlWord::kFaultResetBit) && (last_cw & ControlWord::kFaultResetBit)) {
            cw = 0x0000;
        }

        const std::span<std::byte> out = master.outputs(slave);
        if (op && opt.enable && status.operation_enabled()) {
            if (!announced_op) {
                std::cout << "[B] *** OPERATION ENABLED *** (motor energized, holding at " << hold_pos << ")\n";
                announced_op = true;
            }
            // Hold or move. Keep profile velocity + target populated every cycle.
            if (opt.move_pp && !move_done) {
                target = hold_pos + static_cast<std::int32_t>(opt.move_revs * kCountsPerRev);
            }
            write_rx<std::int32_t>(master, slave, out, kTargetPosition, target);
            write_rx<std::uint32_t>(master, slave, out, kProfileVelocity, profile_vel);

            std::uint16_t base = ControlWord::enable_operation();  // 0x0F
            if (opt.move_pp && !move_done) {
                // bit4 handshake: assert new-setpoint, hold until the drive acks
                // (bit12), then drop it so the next move can re-arm.
                if (!setpoint_latched) {
                    base = ControlWord::with_new_setpoint(base, true);  // 0x1F
                    if (status.setpoint_acknowledged()) {
                        setpoint_latched = true;
                    }
                } else {
                    base = ControlWord::with_new_setpoint(base, false);  // back to 0x0F
                    // A6 bit10 is useless; use actual-vs-target within tolerance.
                    if (std::abs(pos - target) < 300) {
                        move_done = true;
                        std::cout << "[B] move complete: pos=" << pos << " (target " << target << ")\n";
                    }
                }
            }
            cw = base;
        } else if (!opt.enable) {
            cw = ControlWord::shutdown();  // 0x06 -> ReadyToSwitchOn, NOT energized
        }

        write_rx<std::uint16_t>(master, slave, out, kControlword, cw);
        last_cw = cw;

        if (tick % 200 == 0) {  // ~5 Hz status print
            const std::int64_t dc_phase = period_ns != 0 ? dct % static_cast<std::int64_t>(period_ns) : 0;
            std::cout << "    t=" << tick / kLoopHz << "s " << phase_label(op, op_requested) << " " << to_string(status.decode())
                      << " sw=0x" << std::hex << status.raw << std::dec;
            if (faulted) {  // surface the CiA402 error code alongside the state on every faulted print
                std::cout << " 0x603F=0x" << std::hex << read_tx<std::uint16_t>(master, slave, in, kFaultCode) << std::dec;
            }
            // mode display 0x6061 (in TxPDO): confirm the drive is in PP(1)/PV(3),
            // NOT CSP(8) -- a stuck CSP default is itself a candidate "no sync" cause.
            const auto mode_now = read_tx<std::int8_t>(master, slave, in, kModeDisplay);
            std::cout << " mode=" << static_cast<int>(mode_now) << " pos=" << pos << " vel=" << vel << " wkc=" << raw_wkc << "/"
                      << master.expected_wkc() << " badWKC=" << wkc_bad << "(maxRun=" << wkc_bad_max_streak << ") dcPhase=" << dc_phase
                      << "ns(off=" << dc_off << ")" << (master.fault() ? " [BUS FAULT]" : "") << '\n';
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

    // --- graceful shutdown: disable the drive, flush a few cycles, close ---
    std::cout << "\n[B] disabling drive (controlword -> 0x00) and closing...\n";
    for (int i = 0; i < 50; ++i) {
        master.process();
        write_rx<std::uint16_t>(master, slave, master.outputs(slave), kControlword, ControlWord::disable_voltage());
        sleep_until(next, static_cast<long>(period_ns));
    }
    master.close();

    std::cout << "=== done. bad-WKC cycles: " << wkc_bad << " / " << tick << " (max consecutive run: " << wkc_bad_max_streak
              << "; raw per-cycle, not the masked working_counter) ===\n";
    return 0;
}
