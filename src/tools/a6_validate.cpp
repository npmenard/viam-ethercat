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

// SM synchronization objects 0x1C32 (SM2/outputs) / 0x1C33 (SM3/inputs), per
// ETG.1020. Confirmed against the A6's REAL object dictionary (slaveinfo -sdo):
//   :01 Sync Type      R/W  -- 2 = DC SYNC0 (default 1 = SM-synchron). Writable ONLY
//                              before DC activation; post-configdc it 0x08000022-rejects.
//   :02 Cycle Time     RO   -- the value SafeOp validates; the A6 does NOT auto-derive
//                              it from the ESC 0x09A0. Must be populated via :0a + :08.
//   :04 Supported      RO   -- 0x0004 = DC SYNC0 only (confirms SYNC1 N/A).
//   :05 Min Cycle      RO   -- 250000 ns.
//   :08 Get Cycle Time R/W  -- 1 = measure/calc the cycle time (ETG.1020 handshake).
//   :0a Sync0 Cycle T. R/W  -- the master TELLS the drive the SYNC0 cycle (ns) here;
//                              the drive then fills the RO :02 -> valid DC config.
constexpr std::uint16_t kSm2SyncType = 0x1C32;     // SM2 (outputs/RxPDO)
constexpr std::uint16_t kSm3SyncType = 0x1C33;     // SM3 (inputs/TxPDO)
constexpr std::uint8_t kSyncTypeSub = 0x01;        // :01 sync type (R/W, pre-DC)
constexpr std::uint8_t kSyncCycleSub = 0x02;       // :02 cycle time (U32 ns, RO -- SafeOp validates this)
constexpr std::uint8_t kGetCycleSub = 0x08;        // :08 Get Cycle Time (U16, R/W): 1 = measure
constexpr std::uint8_t kSync0CycleSub = 0x0A;      // :0a Sync0 Cycle Time (U32 ns, R/W)
constexpr std::uint16_t kSyncTypeDcSync0 = 2;      // ETG sync type 2 = DC SYNC0
constexpr std::uint32_t kSyncCycleNs = 1'000'000;  // SYNC0/SM cycle = loop period (1 ms); must match ESC 0x09A0
// ETG.1020 measurement diagnostic counters (sub-indices per the A6's REAL OD dump --
// note SyncError is :13 on this drive, not the ETG-standard :20). Populated by :08=1.
constexpr std::uint8_t kSmMissedSub = 0x0B;       // :0b SM-event missed counter (U16)
constexpr std::uint8_t kCycleTooSmallSub = 0x0C;  // :0c Cycle Time Too Small counter (U16)
constexpr std::uint8_t kSyncErrorSub = 0x13;      // :13 Sync Error (BOOL)

constexpr double kCountsPerRev = 131072.0;  // A6 single-turn encoder = 2^17

// Little-endian object values for an SDO write.
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
MasterConfig build_a6_pp_config(const std::string& ifname, std::int32_t dc_sync0_shift_ns, bool reset_fault) {
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
    a6.default_mode = Cia402Mode::ProfilePosition;
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
    bool reset_fault = false;
    double move_revs = 0.0;
    double move_rpm = 60.0;
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
            opt.move_pp = true;
            opt.enable = true;  // a move requires enabling
            opt.move_revs = std::stod(args[++i]);
            if (i + 1 < args.size() && args[i + 1].rfind("--", 0) != 0) {
                opt.move_rpm = std::stod(args[++i]);
            }
        } else if (a == "--seconds" && i + 1 < args.size()) {
            opt.seconds = std::stoi(args[++i]);
        } else if (a == "--dc-shift-ns" && i + 1 < args.size()) {
            opt.dc_sync0_shift_ns = std::stoi(args[++i]);  // SYNC0 CyclShift passed to ecx_dcsync0
        } else if (a.rfind("--", 0) != 0) {
            opt.ifname = a;
        } else {
            std::cerr << "usage: a6_validate [ifname] [--enable] [--reset-fault] [--move-pp REVS [RPM]] [--seconds N] [--dc-shift-ns NS]\n"
                      << "  The DC bring-up (#20) is automatic: configure() arms SYNC0 in PRE-OP + reaches SAFE-OP,\n"
                      << "  then the cyclic loop runs SETTLE (phase-locked PD) -> request OP once -> AWAIT_OP (hold\n"
                      << "  for OP + sync), all gapless. No manual 0x1C32:01 force, no SYNC0 start-delay hack;\n"
                      << "  Er74.1 in SAFE-OP is normal pre-sync and clears at OP (CLAUDE.md / spec #20).\n"
                      << "  --dc-shift-ns: SYNC0 pulse CyclShift (ecx_dcsync0) -- sweep to move the SYNC0 edge if needed.\n"
                      << "  --reset-fault: clear a latent drive fault at bring-up via the A6 vendor SDO 0x2031:01=1.\n";
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

    std::cout << "[dc] phase-lock target = auto(mid-cycle) | SYNC0 CyclShift = " << opt.dc_sync0_shift_ns << "ns"
              << " | fault-reset at bring-up = " << (opt.reset_fault ? "ON (vendor 0x2031:01=1)" : "off") << "\n\n";

    Master master(build_a6_pp_config(opt.ifname, opt.dc_sync0_shift_ns, opt.reset_fault), std::make_unique<SoemBackend>());

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

    const std::uint16_t slave = 1;

    // Dump the drive's SM sync configuration (type :01, cycle :02, supported :04, min
    // :05). Defined here so we can call it on BOTH the success path AND the
    // configure-FAILED path: a SafeOp AL-reject (0x0030 "invalid DC SYNC config")
    // makes configure() throw, but the bus stays open in PRE-OP with the post-assign
    // writes already applied -- so these SDO reads still work and show exactly what
    // the drive validated. Each group is its own try so a missing sub-index can't
    // suppress the others.
    const auto dump_sm_config = [&master]() {
        try {
            const auto sm2 = master.sdo_read<std::uint16_t>(slave, kSm2SyncType, kSyncTypeSub);
            const auto sm3 = master.sdo_read<std::uint16_t>(slave, kSm3SyncType, kSyncTypeSub);
            const auto supported = master.sdo_read<std::uint16_t>(slave, kSm2SyncType, 0x04);
            const auto min_cycle = master.sdo_read<std::uint32_t>(slave, kSm2SyncType, 0x05);
            const bool dc = sm2 == kSyncTypeDcSync0 && sm3 == kSyncTypeDcSync0;
            std::cout << "[dc] SM sync-type: 0x1C32:01(SM2)=" << sm2 << " 0x1C33:01(SM3)=" << sm3
                      << (dc ? "  -> DC SYNC0 active (drive self-selected; we never force it)" : "  -> NOT DC SYNC0 (0=FreeRun,1=SM,2=DC)")
                      << "\n"
                      << "[dc]   0x1C32:04 supportedTypes=0x" << std::hex << supported << std::dec << " 0x1C32:05 minCycle=" << min_cycle
                      << "ns" << '\n';
        } catch (const Error& e) {
            std::cerr << "[dc] SM sync-type read failed (object absent?): " << e.what() << '\n';
        }
        // Cycle-time handshake state: :02 (RO Cycle Time), :0a (Sync0 Cycle Time -- what
        // we told the drive), :08 (Get Cycle Time -- the measure trigger; auto-resets to 0
        // when done). NOTE: :02 is NOT the 0x0030 cause -- DISPROVEN on HW (bench saw :02
        // populated 999120 AND still 0x0030; and :02=0 with SM-sync passes SAFE-OP). The
        // A6 measures :02 at OP entry, not SAFE-OP, so 0 here is NORMAL. The 0x0030 reject
        // with DC sync-type is drive-internal (not visible master-side). Shown FYI only.
        try {
            const auto cyc2 = master.sdo_read<std::uint32_t>(slave, kSm2SyncType, kSyncCycleSub);
            const auto cyc0a = master.sdo_read<std::uint32_t>(slave, kSm2SyncType, kSync0CycleSub);
            const auto get08 = master.sdo_read<std::uint16_t>(slave, kSm2SyncType, kGetCycleSub);
            const char* note = "  (drive's own measurement)";
            if (cyc2 == 0) {
                note = "  (0 = not yet measured; NORMAL in SAFE-OP, NOT the 0x0030 cause)";
            } else if (cyc2 == kSyncCycleNs) {
                note = "  (= 1ms)";
            }
            std::cout << "[dc]   0x1C32:02 CycleTime(RO)=" << cyc2 << "ns 0x1C32:0a Sync0CycleTime=" << cyc0a
                      << "ns 0x1C32:08 GetCycleTime=" << get08 << note << '\n';
        } catch (const Error& e) {
            std::cerr << "[dc]   0x1C32 cycle-time readback failed: " << e.what() << '\n';
        }
        // ETG.1020 measurement diagnostics (FYI): SM-event-missed / cycle-too-small /
        // SyncError, updated by :08=1. (0x0030 persists even with :02 populated + these
        // clear, so they are not the cause either -- the reject is drive-internal.)
        try {
            const auto missed = master.sdo_read<std::uint16_t>(slave, kSm2SyncType, kSmMissedSub);
            const auto too_small = master.sdo_read<std::uint16_t>(slave, kSm2SyncType, kCycleTooSmallSub);
            const auto sync_err = master.sdo_read<std::uint8_t>(slave, kSm2SyncType, kSyncErrorSub);
            std::cout << "[dc]   0x1C32 diag: :0b SMmissed=" << missed << " :0c cycleTooSmall=" << too_small
                      << " :13 syncError=" << static_cast<int>(sync_err) << '\n';
        } catch (const Error& e) {
            std::cerr << "[dc]   0x1C32 diag-counter readback failed: " << e.what() << '\n';
        }
    };

    // --- Stage B: configure to SAFE-OP + DC, then ONE continuous loop that
    // phase-locks, requests OP, and runs the CiA402 sequence -- all on an unbroken
    // cadence. The A6 faults out of OP on a SINGLE missed SYNC0 frame, so we must NOT
    // have a gap between bring-up and the steady loop. reach_op=false stops configure
    // at SAFE-OP + DC; the loop below owns every frame from there.
    try {
        master.configure();
    } catch (const Error& e) {
        std::cerr << "[B] configure failed: " << e.what() << "\n"
                  << "    (an AL-reject at the SAFE-OP transition lands here; SM config below shows what the drive saw.)\n";
        dump_sm_config();  // bus still open in PRE-OP -> reads work; capture the SM config even on failure
        return 1;
    }
    std::cout << "[B] configured to SAFE-OP, DC enabled (expected WKC=" << master.expected_wkc()
              << "); running the bring-up FSM (SETTLE -> request OP -> AWAIT_OP), gapless + phase-locked...\n";

    dump_sm_config();

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
        dump_sm_config();
        std::cout << "\n[B] bring-up did not reach OP; closing.\n";
        master.close();
        return 1;
    }
    std::cout << "[B] *** OPERATIONAL *** WKC=" << master.last_wkc() << "/" << master.expected_wkc()
              << " -- DC bring-up complete (no Er74.1), entering CiA402 control loop\n";
    dump_sm_config();

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

        const std::span<std::byte> out = master.outputs(slave);
        std::uint16_t cw = fsm.step(status, goal);
        if (faulted) {
            // Generic CiA402 bit7 fault-reset edge (the A6's real reset is the vendor
            // 0x2031:01 SDO, issued at bring-up; this is the in-loop steady-state fallback).
            cw = (last_cw & ControlWord::kFaultResetBit) ? 0x0000 : ControlWord::fault_reset();
        } else if (opt.enable && status.operation_enabled()) {
            if (!announced_op) {
                std::cout << "[B] *** OPERATION ENABLED *** (motor energized, holding at " << hold_pos << ")\n";
                announced_op = true;
            }
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
            std::cout << "    t=" << tick / kLoopHz << "s " << to_string(status.decode()) << " sw=0x" << std::hex << status.raw
                      << " 0x603F=0x" << fc << std::dec << " mode=" << static_cast<int>(mode_now) << " Rx.cw=0x" << std::hex << rx_cw
                      << std::dec << " targetPos=" << rx_tpos << " pos=" << pos << " vel=" << vel << " wkc=" << raw_wkc << "/"
                      << master.expected_wkc() << " badWKC=" << wkc_bad << " dcPhase=" << (dct % static_cast<std::int64_t>(period_ns))
                      << "ns" << (master.fault() ? " [BUS FAULT]" : "") << '\n';
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

    std::cout << "=== done. bad-WKC cycles: " << wkc_bad << " / " << tick << " (raw per-cycle, not the masked working_counter) ===\n";
    return 0;
}
