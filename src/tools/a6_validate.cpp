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
constexpr std::uint16_t kGetCycleMeasure = 1;      // :08 = 1 -> measure ONCE, populates :02 (needs SYNC0 live)
constexpr std::uint16_t kGetCycleReset = 0;        // :08 = 0 -> reset any stale measurement before triggering
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
std::vector<std::byte> le32(std::uint32_t v) {
    return {static_cast<std::byte>(v & 0xFFU),
            static_cast<std::byte>((v >> 8U) & 0xFFU),
            static_cast<std::byte>((v >> 16U) & 0xFFU),
            static_cast<std::byte>((v >> 24U) & 0xFFU)};
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
        // PRE-DC (postremap = post-assign, before configdc): switch SM2/SM3 to DC SYNC0.
        // 0x1C32:01 is R/W only BEFORE DC activation -- written post-configdc the A6
        // rejects it 0x08000022 (wrong state). optional=true: 0x1C33:01 reads a
        // non-standard 0x22 and may reject; must not abort the run.
        a6.postremap_sdo_writes = {
            {kSm2SyncType, kSyncTypeSub, le16(kSyncTypeDcSync0), /*optional=*/true},  // SM2 outputs -> DC SYNC0
            {kSm3SyncType, kSyncTypeSub, le16(kSyncTypeDcSync0), /*optional=*/true},  // SM3 inputs  -> DC SYNC0
        };
        // POST-DC (after dcsync0 set 0x09A0=1ms + SYNC0 pulsing): the ETG.1020
        // cycle-time handshake. The A6's OD shows 0x1C32:02 (Cycle Time) is READ-ONLY
        // and is NOT auto-derived -- so we TELL the drive the SYNC0 cycle via the R/W
        // :0a (Sync0 Cycle Time = 1ms) and trigger a measurement via :08 (Get Cycle
        // Time = 1). The drive then fills the RO :02 -> SafeOp validates a non-zero
        // cycle. :0a first (provides the value), then :08 (triggers). All optional.
        a6.postdc_sdo_writes = {
            {kSm2SyncType, kGetCycleSub, le16(kGetCycleReset), /*optional=*/true},    // reset any stale measurement first
            {kSm3SyncType, kGetCycleSub, le16(kGetCycleReset), /*optional=*/true},    //
            {kSm2SyncType, kSync0CycleSub, le32(kSyncCycleNs), /*optional=*/true},    // SM2 Sync0 Cycle Time = 1 ms
            {kSm3SyncType, kSync0CycleSub, le32(kSyncCycleNs), /*optional=*/true},    // SM3 Sync0 Cycle Time = 1 ms
            {kSm2SyncType, kGetCycleSub, le16(kGetCycleMeasure), /*optional=*/true},  // SM2 Get Cycle Time = measure once
            {kSm3SyncType, kGetCycleSub, le16(kGetCycleMeasure), /*optional=*/true},  // SM3 Get Cycle Time = measure once
        };
        // After the handshake, pump up to 250 cycles so the drive measures + populates
        // the RO 0x1C32:02; break early the moment :02 reads non-zero. Without this
        // settle the drive validates :02=0 at SafeOp -> AL 0x0030.
        cfg.dc_postwrite_settle_cycles = 250;
        cfg.dc_settle_poll_index = kSm2SyncType;  // poll 0x1C32...
        cfg.dc_settle_poll_sub = kSyncCycleSub;   // ...:02 (SM cycle time) until non-zero
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
                      << "                   [--dc-target-ns NS] [--dc-shift-ns NS] [--sm-dc-sync]\n"
                      << "  --dc-target-ns: master send-phase lock target within the 1ms cycle (-1=auto mid ~500000).\n"
                      << "                  SWEEP on Er74.0 cycle-error: try 100000 (just after SYNC0) or 900000 (just\n"
                      << "                  before) to find where the drive accepts the frame relative to its SYNC0 edge.\n"
                      << "  --dc-shift-ns:  SYNC0 pulse CyclShift (ecx_dcsync0) -- moves the SYNC0 edge itself.\n";
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

    const std::uint16_t slave = 1;

    // Dump the drive's SM sync configuration (type :01, cycle :02, supported :04, min
    // :05). Defined here so we can call it on BOTH the success path AND the
    // configure-FAILED path: a SafeOp AL-reject (0x0030 "invalid DC SYNC config")
    // makes configure() throw, but the bus stays open in PRE-OP with the post-assign
    // writes already applied -- so these SDO reads still work and show exactly what
    // the drive validated. Each group is its own try so a missing sub-index can't
    // suppress the others.
    const auto dump_sm_config = [&master, &opt]() {
        try {
            const auto sm2 = master.sdo_read<std::uint16_t>(slave, kSm2SyncType, kSyncTypeSub);
            const auto sm3 = master.sdo_read<std::uint16_t>(slave, kSm3SyncType, kSyncTypeSub);
            const auto supported = master.sdo_read<std::uint16_t>(slave, kSm2SyncType, 0x04);
            const auto min_cycle = master.sdo_read<std::uint32_t>(slave, kSm2SyncType, 0x05);
            const bool dc = sm2 == kSyncTypeDcSync0 && sm3 == kSyncTypeDcSync0;
            std::cout << "[dc] SM sync-type: 0x1C32:01(SM2)=" << sm2 << " 0x1C33:01(SM3)=" << sm3
                      << (dc ? "  -> DC SYNC0 active" : "  -> NOT DC SYNC0 (0=FreeRun,1=SM,2=DC)") << "\n"
                      << "[dc]   0x1C32:04 supportedTypes=0x" << std::hex << supported << std::dec << " 0x1C32:05 minCycle=" << min_cycle
                      << "ns" << (opt.sm_dc_sync ? "  [--sm-dc-sync wrote :01=2 :02=1ms]" : "") << '\n';
        } catch (const Error& e) {
            std::cerr << "[dc] SM sync-type read failed (object absent?): " << e.what() << '\n';
        }
        // Cycle-time handshake state: :02 (RO Cycle Time -- the value SafeOp validates),
        // :0a (Sync0 Cycle Time -- what we told the drive), :08 (Get Cycle Time -- the
        // measure trigger; auto-resets to 0 when done). :02 flipping 0 -> 1000000 after
        // the :0a/:08 writes is the smoking gun that the DC config is now valid.
        try {
            const auto cyc2 = master.sdo_read<std::uint32_t>(slave, kSm2SyncType, kSyncCycleSub);
            const auto cyc0a = master.sdo_read<std::uint32_t>(slave, kSm2SyncType, kSync0CycleSub);
            const auto get08 = master.sdo_read<std::uint16_t>(slave, kSm2SyncType, kGetCycleSub);
            const char* note = "  (non-1ms -- unexpected)";
            if (cyc2 == 0) {
                note = "  *** :02=0 -> drive hasn't populated Cycle Time -> AL 0x0030 ***";
            } else if (cyc2 == kSyncCycleNs) {
                note = "  -> Cycle Time populated, DC config VALID";
            }
            std::cout << "[dc]   0x1C32:02 CycleTime(RO)=" << cyc2 << "ns 0x1C32:0a Sync0CycleTime=" << cyc0a
                      << "ns 0x1C32:08 GetCycleTime=" << get08 << note << '\n';
        } catch (const Error& e) {
            std::cerr << "[dc]   0x1C32 cycle-time readback failed: " << e.what() << '\n';
        }
        // ETG.1020 measurement diagnostics: if :02 populates but 0x0030 persists, these
        // say why -- SM-event-missed / cycle-too-small / SyncError (updated by :08=1).
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
        master.configure(false);
    } catch (const Error& e) {
        std::cerr << "[B] configure failed: " << e.what() << "\n"
                  << "    (a SafeOp AL-reject 0x0030 'invalid DC SYNC config' lands here; SM config below shows what the drive saw.)\n";
        dump_sm_config();  // bus still open in PRE-OP -> reads work; capture the SM config even on failure
        return 1;
    }
    std::cout << "[B] configured to SAFE-OP, DC enabled (expected WKC=" << master.expected_wkc()
              << "); phase-locking, then requesting OP with NO frame gap...\n";

    dump_sm_config();

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
    // The SOEM-author (Arthur Ketels) flow, in ONE gapless loop:
    //   (1) pump phase-locked PD in SAFE-OP until the MASTER is send-phase locked;
    //   (2) THEN arm SYNC0 IN-LOOP (master.arm_dc_sync) -- on a live clock the drive is
    //       already seeing synchronized LRW on, which is what makes it generate SYNC0;
    //   (3) keep pumping + poll the slave DC-sync health (0x0984 arm + 0x092C lock);
    //   (4) request OP only once dc_sync_status().ready (Arthur's "proven in sync").
    // Poll the ESC DC regs at ~20 Hz (acyclic FPRD ADDITIVE to the per-cycle PD send,
    // never substitutive) only after arming. A cap (relative to the arm) still requests
    // OP so a never-arms run loudly surfaces the drive's Er74 reaction.
    DcSyncStatus dcs{};
    bool dc_ready = false;
    bool dc_sync_announced = false;
    int op_fault_streak = 0;                             // consecutive cycles faulted-in-OP (persistent-cause give-up)
    constexpr int kPersistentFaultGiveUp = 2000;         // ~2 s faulted in OP despite reset -> STOP (protect the drive)
    bool dc_armed = false;                               // SYNC0 armed in-loop yet?
    std::int64_t sm_cycle = -1;                          // latest 0x1C32:02 read (ns), -1 = not yet read (DIAGNOSTIC only)
    std::uint64_t arm_tick = 0;                          // tick at which we armed (cap is relative to this)
    constexpr int kArmAfterLockStreak = 200;             // arm SYNC0 once the master holds phase-lock this long (~200 ms)
    constexpr std::uint64_t kDcPollEvery = 50;           // poll dc_sync_status every 50 cycles (~20 Hz), ADDITIVE to PD
    constexpr std::uint64_t kOpRequestCapCycles = 8000;  // ~8 s AFTER arming: SYNC0 never armed -> abort (failure path)
    // NOTE on 0x1C32:02 (RO SM cycle): on a CLEAN drive it reads 0 in SAFE-OP and is only
    // MEASURED/populated by the drive AT the OP transition (bench: the 999120 we chased
    // earlier was a polluted-drive residual). So we do NOT gate OP on it -- a "verify
    // ==1ms before OP" check can never pass (the value only exists after the OP it would
    // block). Instead the PRE-OP phase-lock makes the OP-ENTRY measurement clean, and the
    // bounded give-up is the safety net. 0x1C32:02 stays a DIAGNOSTIC (poll trace +
    // fault-edge read) so we can see what the drive measured at/after OP.

    // Read the RO SM2 cycle time (0x1C32:02) the drive validates at OP -- watch it
    // re-derive toward 1000000 once a clean SYNC0 is pulsing (vs a stale cached value
    // like 999680 a prior dirty bring-up burned in -> Er74.0 cycle error). SDO read =
    // a mailbox round-trip that BLOCKS the loop, so only call it where a stalled cycle
    // is harmless: pre-OP (no cycle enforcement yet) or when already faulted. -1 = not
    // readable this cycle.
    const auto read_sm_cycle = [&master]() -> std::int64_t {
        try {
            return static_cast<std::int64_t>(master.sdo_read<std::uint32_t>(slave, kSm2SyncType, kSyncCycleSub));
        } catch (const Error&) {
            return -1;
        }
    };

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

        // STEP 2 -- ARM SYNC0 IN-LOOP, once the master holds phase-lock, while
        // synchronized PD is already flowing. This is the crux of the SOEM-author flow:
        // dcsync0 runs HERE (post-SAFE-OP, PD live, master disciplined), NOT in
        // configure(). My bet (and the team-lead's): the A6 generates SYNC0 only when it
        // arms against a clock it is already observing in sync.
        if (!dc_armed && locked_streak >= kArmAfterLockStreak) {
            master.arm_dc_sync();
            dc_armed = true;
            arm_tick = tick;
            std::cout << "[B] master phase-locked (lockStreak=" << locked_streak << ", dcPhase~"
                      << (dct % static_cast<std::int64_t>(period_ns)) << "ns) -> ARMING SYNC0 in-loop (post-SAFE-OP, PD flowing)\n";
            // --sm-dc-sync: NOW that the arm set the ESC SYNC0 cycle (0x09A0) live, run
            // the ETG.1020 cycle-time handshake (0x1C32:0a/:08) so the drive populates
            // the RO 0x1C32:02 it validates at the OP transition. configure() deferred
            // these to here (post-arm) on the caller-driven path.
            if (opt.sm_dc_sync) {
                master.apply_postdc_writes();
                std::cout << "[B] applied post-arm ETG.1020 cycle handshake (0x1C32:0a/:08) -> populating 0x1C32:02\n";
            }
        }

        // STEP 3 -- after arming, poll the slave DC-sync health at ~20 Hz (pre-OP, SDO
        // read safe). 0x0984 (SYNC0 armed) is the load-bearing DC-sync bit (single-slave:
        // 0x092C ~ 0 trivially). 0x1C32:02 is read as a DIAGNOSTIC only (it is 0 in
        // SAFE-OP on a clean drive -- the drive measures it AT OP entry -- so it is NOT a
        // gate; see the NOTE above). Trace until SYNC0 is armed (the OP go-signal).
        if (dc_armed && !op_requested && (tick % kDcPollEvery == 0)) {
            dcs = master.dc_sync_status();
            dc_ready = dcs.ready;
            sm_cycle = read_sm_cycle();
            if (!dc_ready) {  // trace through the proving window (stops once SYNC0 armed -> ready for OP)
                std::cout << "[B]   poll t=" << tick << " 0x0984=" << (dcs.sync0_active ? "ARMED" : "0")
                          << " 0x092C=" << dcs.sys_time_diff_ns << "ns lockStreak=" << locked_streak << " AL=0x" << std::hex
                          << dcs.al_status << std::dec << " 0x1C32:02=" << sm_cycle << "ns (0 in SAFE-OP is normal; measured at OP)\n";
            }
            if (dcs.sync0_active && !dc_sync_announced) {
                std::cout << "[B] *** SYNC0 ARMED *** (0x0984 b0=1), clock " << (dcs.clock_locked ? "LOCKED" : "unlocked")
                          << " (0x092C=" << dcs.sys_time_diff_ns << "ns)\n";
                dc_sync_announced = true;
            }
        }

        // STEP 4 -- request OP once master-locked + SYNC0-armed + clock-locked (dc_ready).
        // We deliberately do NOT gate on 0x1C32:02: on a clean drive it is 0 in SAFE-OP and
        // is only measured AT the OP transition, so a verify-before-OP gate can never pass.
        // The PRE-OP phase-lock is the actual fix -- the master crosses into SAFE-OP already
        // locked, so the drive's OP-entry cycle measurement should be a clean 1 ms (no
        // Er74.0). This is the real test of that hypothesis. The bounded give-up below is
        // the safety net if the OP-entry measurement is still off (stops at ~2 s, no wedge).
        const bool cap_hit = dc_armed && tick >= arm_tick + kOpRequestCapCycles;
        if (!op_requested && dc_armed && dc_ready) {
            master.request_op();
            op_requested = true;
            std::cout << "[B] DC-sync READY (master locked + SYNC0 armed + clock locked) -> requesting OP "
                      << "(0x1C32:02 is measured at OP entry; PRE-OP lock should make it a clean 1ms)\n";
        } else if (!op_requested && dc_armed && cap_hit) {
            // SYNC0 never armed within the cap. Arming was solved (d5ff073), so this is
            // unexpected -- surface it but do NOT request OP into a no-SYNC0 state (Er74.1).
            std::cout << "[B] !!! SYNC0 NEVER ARMED: 0x0984=0 through " << kOpRequestCapCycles
                      << " cycles after the in-loop arm -- NOT requesting OP (would Er74.1). Unexpected (arming was solved); check the "
                      << "[dc] arm readback (0x0981 activation) above.\n";
            break;  // abort cleanly
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
            // Read 0x1C32:02 on the fault edge (drive already faulted -> a stalled cycle
            // from the SDO read is harmless): a 0x6320 Er74.0 "cycle error" with :02 !=
            // 1000000 (e.g. a stale 999680) is the smoking gun for the cached-cycle theory.
            std::cout << "[B] !!! DRIVE FAULT @ " << phase_label(op, op_requested) << " t=" << tick / kLoopHz << "s: 0x603F=0x" << std::hex
                      << fault_code << " sw=0x" << status.raw << std::dec << " dcPhase=" << (dct % static_cast<std::int64_t>(period_ns))
                      << "ns 0x1C32:02=" << read_sm_cycle() << "ns";
            if (opt.reset_fault) {
                std::cout << " -- running CiA402 fault-reset (no energize)...";
            }
            std::cout << '\n';
        } else if (!faulted && was_faulted) {
            std::cout << "[B] *** FAULT CLEARED *** -> " << to_string(status.decode()) << '\n';
        }
        was_faulted = faulted;

        // BOUNDED GIVE-UP: if the drive stays faulted in OP despite the auto fault-reset
        // -- a persistent-CAUSE fault like Er74.0 cycle-error, where the bit7 reset edge
        // fires but the cause is still active -- STOP cleanly instead of spinning the
        // reset for the whole --seconds. Repeated OP-entry faults are what wedged the
        // drive (NO-CARRIER) last run, so giving up protects the (power-cycle-scarce)
        // drive AND keeps the result legible (the live drive code is named, not masked).
        // A transient fault that clears resets the streak and the run continues.
        if (op_requested && faulted) {
            ++op_fault_streak;
            if (op_fault_streak >= kPersistentFaultGiveUp) {
                std::cout << "[B] !!! fault-reset INEFFECTIVE: drive fault 0x" << std::hex
                          << read_tx<std::uint16_t>(master, slave, in, kFaultCode) << std::dec << " persists " << op_fault_streak
                          << " cycles after OP (cause not cleared) -- STOPPING to protect "
                          << "the drive (repeated OP-entry faults wedge it). Fix the cycle cause; do not re-run blind.\n";
                break;
            }
        } else if (!faulted) {
            op_fault_streak = 0;
        }

        // Decide the controlword for this cycle.
        const std::span<std::byte> out = master.outputs(slave);
        std::uint16_t cw = fsm.step(status, goal);

        if (faulted) {
            // A latched fault (Er74) takes PRECEDENCE over the CiA402 ladder and the
            // shutdown/enable branches below: it must be cleared before the drive will
            // engage SYNC0 / accept commands, and it must clear in SAFE-OP (DURING
            // sync-proving) as well as OP. CiA402 fault-reset = 0x00 -> 0x80 (bit7)
            // rising edge -> 0x06. Generate the edge by toggling bit7: 0x80 when it was
            // low last cycle, 0x00 when it was high. The fault code was already captured
            // + printed on the latch edge above, so auto-resetting loses no diagnostic.
            cw = (last_cw & ControlWord::kFaultResetBit) ? 0x0000 : ControlWord::fault_reset();
        } else if (op && opt.enable && status.operation_enabled()) {
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

        if (tick % 200 == 0) {  // ~5 Hz decoded-PDO print, through SAFE-OP AND OP (red_test-style)
            const std::int64_t dc_phase = period_ns != 0 ? dct % static_cast<std::int64_t>(period_ns) : 0;
            // Decode BOTH directions from the live process image (one snapshot/cycle, per
            // #16): RxPDO = what we command the drive; TxPDO = what it feeds back.
            const std::span<const std::byte> outimg = master.outputs(slave);
            const auto rx_cw = read_rx<std::uint16_t>(master, slave, outimg, kControlword);
            const auto rx_tpos = read_rx<std::int32_t>(master, slave, outimg, kTargetPosition);
            const auto rx_pvel = read_rx<std::uint32_t>(master, slave, outimg, kProfileVelocity);
            const auto fc = read_tx<std::uint16_t>(master, slave, in, kFaultCode);
            const auto mode_now = read_tx<std::int8_t>(master, slave, in, kModeDisplay);
            const auto torque = read_tx<std::int16_t>(master, slave, in, kTorqueActual);
            std::cout << "    t=" << tick / kLoopHz << "s " << phase_label(op, op_requested) << '\n';
            std::cout << "      Rx(cmd) : cw=0x" << std::hex << rx_cw << std::dec << " targetPos=" << rx_tpos << " profVel=" << rx_pvel
                      << '\n';
            std::cout << "      Tx(fb)  : " << to_string(status.decode()) << " sw=0x" << std::hex << status.raw << " 0x603F=0x" << fc
                      << std::dec << " mode=" << static_cast<int>(mode_now) << " pos=" << pos << " vel=" << vel << " torq=" << torque
                      << '\n';
            std::cout << "      bus/DC  : wkc=" << raw_wkc << "/" << master.expected_wkc() << " badWKC=" << wkc_bad
                      << "(maxRun=" << wkc_bad_max_streak << ") DCtime=" << dct << " dcPhase=" << dc_phase << "ns(off=" << dc_off << ")"
                      << " lockStreak=" << locked_streak << (master.fault() ? " [BUS FAULT]" : "") << '\n';
            const char* dc_sync_note = " (proving sync...)";
            if (dc_ready) {
                dc_sync_note = " -> READY for OP";
            } else if (op) {
                dc_sync_note = "";
            }
            std::cout << "      DC-sync : SYNC0=" << (dcs.sync0_active ? "ARMED" : "off")
                      << " clock=" << (dcs.clock_locked ? "LOCKED" : "unlocked") << " 0x092C=" << dcs.sys_time_diff_ns << "ns AL=0x"
                      << std::hex << dcs.al_status << std::dec << dc_sync_note << '\n';
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
