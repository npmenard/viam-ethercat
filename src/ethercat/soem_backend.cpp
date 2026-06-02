#include "ethercat/soem_backend.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>
#include <ctime>
#include <iostream>
#include <string>

#include <soem/ethercat.h>

#include "ethercat/errors.hpp"

namespace ethercat {

namespace {

// Paced exchanges between ecx_configdc and ecx_dcsync0 so the slave's DC system
// time (ESC 0x0910) is live + forward-moving before dcsync0 computes the SYNC0
// start time from it. Too few -> start time off a stale clock -> SYNC0 misfires
// (Er74.1 / ESC 0x0134 = 0x2D). ~50 ms at a 1 ms cycle is ample for the offset +
// drift to settle without a wrong start.
constexpr std::uint32_t kDcStartPrimeCycles = 50;

// After ecx_dcsync0, SOEM schedules the FIRST SYNC0 pulse ~SyncDelay (100 ms) after
// the DC base time (its internal constant). A DC-mode drive validates its DC config
// at the PRE-OP->SAFE-OP transition and AL-rejects 0x0030 "Invalid DC SYNC config"
// if SYNC0 isn't actively PULSING yet. So pump paced PD until the DC clock passes the
// SYNC0 start before returning (the master requests SafeOp next). Cap the wait so a
// bad start time can't hang here -- 250 ms covers the 100 ms SyncDelay + margin.
constexpr int kSync0StartWaitCap = 250;

// EcatState <-> SOEM AL-state value.
std::uint16_t to_soem_state(EcatState state) noexcept {
    switch (state) {
        case EcatState::Init:
            return EC_STATE_INIT;
        case EcatState::PreOp:
            return EC_STATE_PRE_OP;
        case EcatState::SafeOp:
            return EC_STATE_SAFE_OP;
        case EcatState::Op:
            return EC_STATE_OPERATIONAL;
        case EcatState::None:
            return EC_STATE_NONE;
    }
    return EC_STATE_NONE;
}

EcatState from_soem_state(std::uint16_t soem) noexcept {
    switch (soem & 0x0FU) {  // mask off the ERROR/ACK high bits
        case EC_STATE_INIT:
            return EcatState::Init;
        case EC_STATE_PRE_OP:
            return EcatState::PreOp;
        case EC_STATE_SAFE_OP:
            return EcatState::SafeOp;
        case EC_STATE_OPERATIONAL:
            return EcatState::Op;
        default:
            return EcatState::None;
    }
}

std::string hex32(std::uint32_t v) {
    static constexpr char kDigits[] = "0123456789ABCDEF";
    std::string out = "0x00000000";
    for (int i = 0; i < 8; ++i) {
        out[static_cast<std::size_t>(9 - i)] = kDigits[(v >> (4U * static_cast<unsigned>(i))) & 0xFU];
    }
    return out;
}

// Drain SOEM's error stack and, if a CoE abort is present, return its detail.
// SOEM reports an SDO abort by pushing an ec_errort (with .AbortCode) even when
// the mailbox working counter is non-zero, so checking the WKC alone can miss it.
std::string pop_coe_abort(ecx_contextt* ctx) {
    std::string detail;
    ec_errort err{};
    while (ecx_poperror(ctx, &err)) {
        if (err.Etype == EC_ERR_TYPE_SDO_ERROR) {
            detail = ", CoE abort " + hex32(static_cast<std::uint32_t>(err.AbortCode));
        }
    }
    return detail;
}

}  // namespace

// All SOEM-touching state lives here, behind the pimpl. The reentrant API wants
// a context that points at caller-owned buffers (the global ec_slave[]/ec_*
// API is exactly this struct wired to file-scope statics); we own them per
// instance instead, so there is no SOEM global state.
struct SoemBackend::Impl {
    ecx_portt port{};
    ec_slavet slavelist[EC_MAXSLAVE]{};
    int slavecount{};
    ec_groupt grouplist[EC_MAXGROUP]{};
    uint8 esibuf[EC_MAXEEPBUF]{};
    uint32 esimap[EC_MAXEEPBITMAP]{};
    ec_eringt elist{};
    ec_idxstackT idxstack{};
    boolean ecaterror{};
    int64 dctime{};
    ec_SMcommtypet smcommtype[EC_MAX_MAPT]{};
    ec_PDOassignt pdoassign[EC_MAX_MAPT]{};
    ec_PDOdesct pdodesc[EC_MAX_MAPT]{};
    ec_eepromSMt eepsm{};
    ec_eepromFMMUt eepfmmu{};
    ecx_contextt ctx{};

    std::array<std::byte, 8192> iomap{};
    int expected_wkc = 0;
    int slave_count = 0;
    std::uint32_t dc_cycle_ns = 0;  // SYNC0 cycle once DC is enabled; paces the OP-transition PD pump
    bool open = false;

    Impl() {
        ctx.port = &port;
        ctx.slavelist = &slavelist[0];
        ctx.slavecount = &slavecount;
        ctx.maxslave = EC_MAXSLAVE;
        ctx.grouplist = &grouplist[0];
        ctx.maxgroup = EC_MAXGROUP;
        ctx.esibuf = &esibuf[0];
        ctx.esimap = &esimap[0];
        ctx.esislave = 0;
        ctx.elist = &elist;
        ctx.idxstack = &idxstack;
        ctx.ecaterror = &ecaterror;
        ctx.DCtO = 0;
        ctx.DCl = 0;
        ctx.DCtime = &dctime;
        ctx.SMcommtype = &smcommtype[0];
        ctx.PDOassign = &pdoassign[0];
        ctx.PDOdesc = &pdodesc[0];
        ctx.eepSM = &eepsm;
        ctx.eepFMMU = &eepfmmu;
        ctx.FOEhook = nullptr;
        ctx.EOEhook = nullptr;
        ctx.manualstatechange = 0;
    }
};

SoemBackend::SoemBackend() : impl_(std::make_unique<Impl>()) {}
SoemBackend::~SoemBackend() {
    if (impl_->open) {
        ecx_close(&impl_->ctx);
    }
}

std::size_t SoemBackend::open(std::string_view ifname) {
    if (impl_->open) {
        throw ConfigError("SoemBackend::open: bus already open (one master per backend; close() first)");
    }
    const std::string name(ifname);
    if (ecx_init(&impl_->ctx, name.c_str()) <= 0) {
        throw InitError("failed to open EtherCAT interface '" + name +
                        "': need CAP_NET_RAW (run with setcap or as root) and the interface must exist");
    }
    const int count = ecx_config_init(&impl_->ctx, FALSE);
    if (count <= 0) {
        ecx_close(&impl_->ctx);
        throw InitError("no EtherCAT slaves found on '" + name + "' (is the bus wired and powered?)");
    }
    impl_->open = true;
    impl_->slave_count = count;
    return static_cast<std::size_t>(count);
}

SlaveInfo SoemBackend::slave_info(std::uint16_t slave) const {
    if (slave < 1 || slave > impl_->slave_count) {
        throw ConfigError("SoemBackend::slave_info: slave " + std::to_string(slave) + " out of range");
    }
    const ec_slavet& s = impl_->slavelist[slave];
    SlaveInfo info;
    info.position = slave;
    info.vendor_id = s.eep_man;
    info.product_code = s.eep_id;
    info.name = s.name;
    info.input_bytes = s.Ibytes;
    info.output_bytes = s.Obytes;
    return info;
}

void SoemBackend::sdo_write(std::uint16_t slave, std::uint16_t index, std::uint8_t sub, std::span<const std::byte> data) {
    // SOEM's psize is an int; PDO/SDO payloads are tiny, so the cast is safe.
    const int size = static_cast<int>(data.size());
    const int wkc = ecx_SDOwrite(&impl_->ctx, slave, index, sub, FALSE, size, const_cast<std::byte*>(data.data()), EC_TIMEOUTRXM);
    // A CoE abort can return wkc > 0 but push an error, so check both.
    if (wkc <= 0 || ecx_iserror(&impl_->ctx)) {
        const std::string abort = pop_coe_abort(&impl_->ctx);
        throw PdoMappingError("SDO write to slave " + std::to_string(slave) + " object " + std::to_string(index) + ":" +
                              std::to_string(sub) + " failed (working counter " + std::to_string(wkc) + ")" + abort);
    }
}

std::size_t SoemBackend::sdo_read(std::uint16_t slave, std::uint16_t index, std::uint8_t sub, std::span<std::byte> out) {
    int size = static_cast<int>(out.size());
    const int wkc = ecx_SDOread(&impl_->ctx, slave, index, sub, FALSE, &size, out.data(), EC_TIMEOUTRXM);
    if (wkc <= 0 || ecx_iserror(&impl_->ctx)) {
        const std::string abort = pop_coe_abort(&impl_->ctx);
        throw BusError("SDO read from slave " + std::to_string(slave) + " object " + std::to_string(index) + ":" + std::to_string(sub) +
                       " failed (working counter " + std::to_string(wkc) + ")" + abort);
    }
    return static_cast<std::size_t>(size < 0 ? 0 : size);
}

void SoemBackend::map_process_data() {
    const int used = ecx_config_map_group(&impl_->ctx, impl_->iomap.data(), 0);
    if (used <= 0 || static_cast<std::size_t>(used) > impl_->iomap.size()) {
        throw PdoMappingError("ec_config_map produced an invalid IOmap size (" + std::to_string(used) + "); the bus image may exceed " +
                              std::to_string(impl_->iomap.size()) + " bytes");
    }
    const ec_groupt& g = impl_->grouplist[0];
    impl_->expected_wkc = (g.outputsWKC * 2) + g.inputsWKC;
}

void SoemBackend::request_state(std::uint16_t slave, EcatState target) {
    const std::uint16_t want = to_soem_state(target);
    impl_->slavelist[slave].state = want;
    ecx_writestate(&impl_->ctx, slave);

    std::uint16_t reached = 0;
    if (target == EcatState::Op) {
        // A DC-only drive rejects OP (AL 0x0027) unless it sees LIVE process data +
        // SYNC0 events during the transition -- so pump PD while statechecking, and
        // PACE the pump at the SYNC0 cycle so the sends align to the slave's DC
        // pulse (an unpaced burst doesn't). ~200 cycles of settle.
        for (int chk = 0; chk < 200; ++chk) {
            ecx_send_processdata(&impl_->ctx);
            ecx_receive_processdata(&impl_->ctx, EC_TIMEOUTRET);
            reached = ecx_statecheck(&impl_->ctx, slave, want, 50000);
            if (reached == want) {
                break;
            }
            if (impl_->dc_cycle_ns > 0) {
                const timespec ts{.tv_sec = 0, .tv_nsec = static_cast<long>(impl_->dc_cycle_ns)};
                (void)nanosleep(&ts, nullptr);
            }
        }
    } else {
        reached = ecx_statecheck(&impl_->ctx, slave, want, EC_TIMEOUTSTATE);
    }

    if (reached != want) {
        // Refresh every slave's AL state + AL status code so the error names WHY
        // (e.g. "Invalid DC SYNC Configuration", "SM watchdog") -- a SAFE-OP->OP
        // refusal is otherwise opaque on the bench.
        std::string detail;
        ecx_readstate(&impl_->ctx);
        for (int i = 1; i <= impl_->slavecount; ++i) {
            const std::uint16_t al = impl_->slavelist[i].ALstatuscode;
            detail += " [slave " + std::to_string(i) + " state=" + to_string(from_soem_state(impl_->slavelist[i].state)) +
                      " ALstatuscode=" + hex32(al) + " (" + ec_ALstatuscode2string(al) + ")]";
        }
        throw InitError("slave " + std::to_string(slave) + " did not reach state " + to_string(target) + " (reached " +
                        to_string(from_soem_state(reached)) + ")" + detail);
    }
}

void SoemBackend::set_state(std::uint16_t slave, EcatState target) noexcept {
    // Write the state request ONLY -- no pump, no statecheck, no throw. The caller's
    // cyclic loop pumps process data through the transition so a DC drive never sees
    // a gap. slave_state() reports progress.
    impl_->slavelist[slave].state = to_soem_state(target);
    ecx_writestate(&impl_->ctx, slave);
}

EcatState SoemBackend::slave_state(std::uint16_t slave) const {
    if (slave > impl_->slave_count) {
        return EcatState::None;
    }
    return from_soem_state(impl_->slavelist[slave].state);
}

SlaveIo SoemBackend::slave_io(std::uint16_t slave) noexcept {
    if (slave < 1 || slave > impl_->slave_count) {
        return {};
    }
    const ec_slavet& s = impl_->slavelist[slave];
    // SOEM hands out raw uint8* into the IOmap; reinterpret as std::byte spans.
    auto* out = reinterpret_cast<std::byte*>(s.outputs);
    const auto* in = reinterpret_cast<const std::byte*>(s.inputs);
    return SlaveIo{std::span<std::byte>(out, out != nullptr ? s.Obytes : 0), std::span<const std::byte>(in, in != nullptr ? s.Ibytes : 0)};
}

void SoemBackend::configure_dc_sync(std::uint32_t cycle_ns, std::int32_t sync0_shift_ns) {
    // ecx_configdc detects DC-capable slaves + syncs the DC reference clock (ESC
    // 0x0910). It MUST run, set each slave's hasdc, and return TRUE before
    // ecx_dcsync0 -- otherwise dcsync0 is a silent no-op and a DC-only drive (the
    // A6) refuses OP with AL 0x0027 "Freerun not supported". Verify both so a DC
    // misconfig names itself instead of surfacing as an opaque OP refusal.
    const boolean dc_found = ecx_configdc(&impl_->ctx);
    std::cerr << "[dc] ecx_configdc() returned " << (dc_found == TRUE ? "TRUE (DC slaves found)" : "FALSE (NO DC slaves)") << '\n';
    if (dc_found == FALSE) {
        throw InitError("use_distributed_clocks is set but ecx_configdc() found NO DC-capable slave on the bus");
    }

    // ORDER IS LOAD-BEARING: ecx_dcsync0 computes the SYNC0 START TIME from the
    // slave's LIVE local DC system time (it FPRDs ESC 0x0910 at call time) and
    // schedules the first pulse at the next cycle boundary. Right after configdc the
    // DC clocks are NOT yet disciplined -- the offset/drift is only propagated by the
    // ARMW/FRMW datagram inside ecx_send/receive_processdata. Call dcsync0 too early
    // and the start time is derived from a stale/zero clock -> the first SYNC0 lands
    // in the past -> the ESC never fires a pulse -> the drive reports Er74.1 "no sync
    // signal" (and ESC 0x0134 = 0x2D "DC start time invalid"). So PRIME first: pace a
    // handful of paced exchanges so 0x0910 is live and forward-moving, THEN dcsync0.
    for (std::uint32_t p = 0; p < kDcStartPrimeCycles; ++p) {
        ecx_send_processdata(&impl_->ctx);
        (void)ecx_receive_processdata(&impl_->ctx, EC_TIMEOUTRET);  // refreshes the DC clocks
        const timespec ts{.tv_sec = 0, .tv_nsec = static_cast<long>(cycle_ns)};
        (void)nanosleep(&ts, nullptr);
    }

    std::int64_t latest_sync0_start = 0;  // newest SYNC0 start time across slaves (wait past it before SafeOp)
    for (int i = 1; i <= impl_->slavecount; ++i) {
        if (impl_->slavelist[i].hasdc == FALSE) {
            throw InitError("slave " + std::to_string(i) + " is not DC-capable (hasdc=0) -- cannot enable SYNC0");
        }
        const auto si = static_cast<std::uint16_t>(i);
        // SYNC0 on, `cycle_ns` period, `sync0_shift_ns` CyclShift. The A6 is SYNC0-ONLY
        // (manual: activation 0x0981 = 0x03 = cyclic+SYNC0, SYNC1 off), so plain
        // ecx_dcsync0. SOEM writes ESC 0x0980/0x0981/0x0990/0x09A0 to activate the
        // cyclic pulse `shift` after the DC base time.
        ecx_dcsync0(&impl_->ctx, si, TRUE, cycle_ns, sync0_shift_ns);

        // SILICON-LEVEL PROOF: read the ESC DC registers straight back via FPRD so the
        // bench can see whether SYNC0 actually activated. Decisive reads:
        //   0x0981 activation -- expect 0x03 (cyclicEn + SYNC0en); bit1 clear => the
        //     activation didn't take.
        //   0x0134 AL status code -- 0x2D => "DC start time invalid" = the start-time/
        //     ordering bug (would mean the prime above wasn't enough). Smoking gun.
        //   0x09A0 SYNC0 cycle -- expect 1000000; 0x0990 start time -- sane near-future.
        const std::uint16_t adp = impl_->slavelist[i].configadr;
        std::uint8_t cyclic_ctrl = 0;  // 0x0980 cyclic unit control
        std::uint8_t activation = 0;   // 0x0981 activation: b0 cyclic, b1 SYNC0, b2 SYNC1
        std::uint16_t al_status = 0;   // 0x0134 AL status code (0x2D = DC start time invalid)
        std::uint32_t sync0_cyc = 0;   // 0x09A0 SYNC0 cycle time (ns)
        std::uint64_t start_time = 0;  // 0x0990 start time / next SYNC0 system time
        std::uint64_t sys_time = 0;    // 0x0910 current DC system time (for start-vs-now sanity)
        (void)ecx_FPRD(&impl_->port, adp, 0x0980, sizeof(cyclic_ctrl), &cyclic_ctrl, EC_TIMEOUTRET);
        (void)ecx_FPRD(&impl_->port, adp, 0x0981, sizeof(activation), &activation, EC_TIMEOUTRET);
        (void)ecx_FPRD(&impl_->port, adp, 0x0134, sizeof(al_status), &al_status, EC_TIMEOUTRET);
        (void)ecx_FPRD(&impl_->port, adp, 0x09A0, sizeof(sync0_cyc), &sync0_cyc, EC_TIMEOUTRET);
        (void)ecx_FPRD(&impl_->port, adp, 0x0990, sizeof(start_time), &start_time, EC_TIMEOUTRET);
        (void)ecx_FPRD(&impl_->port, adp, 0x0910, sizeof(sys_time), &sys_time, EC_TIMEOUTRET);
        std::cerr << "[dc] slave " << i << " hasdc=1, requested SYNC0 @ " << cycle_ns << " ns shift " << sync0_shift_ns << " ns\n"
                  << "[dc]   ESC 0x0980 cyclicCtrl=0x" << std::hex << static_cast<unsigned>(cyclic_ctrl) << " 0x0981 activation=0x"
                  << static_cast<unsigned>(activation) << std::dec << " [cyclicEn=" << ((activation & 0x01U) != 0)
                  << " SYNC0en=" << ((activation & 0x02U) != 0) << "]"
                  << " 0x0134 ALstatus=0x" << std::hex << al_status << std::dec << "\n"
                  << "[dc]   ESC 0x09A0 SYNC0cyc=" << sync0_cyc << "ns 0x0990 startTime=" << start_time << " 0x0910 sysTime=" << sys_time
                  << " (start-now=" << (static_cast<std::int64_t>(start_time) - static_cast<std::int64_t>(sys_time)) << "ns)\n";
        if ((activation & 0x02U) == 0) {
            std::cerr << "[dc]   *** WARNING: ESC SYNC0 enable bit (0x0981 b1) CLEAR -- SYNC0 NOT activated at silicon ***\n";
        }
        if (al_status == 0x2DU) {
            std::cerr << "[dc]   *** WARNING: ESC 0x0134 = 0x2D 'DC start time invalid' -- SYNC0 start was mis-scheduled ***\n";
        }
        latest_sync0_start = std::max(latest_sync0_start, static_cast<std::int64_t>(start_time));
    }

    // WAIT for SYNC0 to actually start pulsing before returning (-> the master
    // requests SAFE-OP next). SOEM's 100 ms SyncDelay puts the first pulse ~100 ms
    // out; a DC-mode drive AL-rejects 0x0030 at the SafeOp transition if it validates
    // DC config before any SYNC0 edge has fired. Pump paced PD until the DC clock is a
    // few cycles past the start time (a handful of pulses fired), capped.
    const std::int64_t sync0_live_target = latest_sync0_start + (3 * static_cast<std::int64_t>(cycle_ns));
    int waited = 0;
    for (; waited < kSync0StartWaitCap && impl_->dctime < sync0_live_target; ++waited) {
        ecx_send_processdata(&impl_->ctx);
        (void)ecx_receive_processdata(&impl_->ctx, EC_TIMEOUTRET);  // advances impl_->dctime
        const timespec ts{.tv_sec = 0, .tv_nsec = static_cast<long>(cycle_ns)};
        (void)nanosleep(&ts, nullptr);
    }
    std::cerr << "[dc] waited " << waited << " cycles for SYNC0 to start: DCtime=" << impl_->dctime << " vs start=" << latest_sync0_start
              << (impl_->dctime >= latest_sync0_start ? "  -> SYNC0 PULSING (ready for SafeOp)"
                                                      : "  -> *** start not reached (cap hit) -- SafeOp may still 0x0030 ***")
              << '\n';

    // POST-WAIT silicon check: NOW that the ~100 ms SyncDelay start has elapsed, read
    // the SYNC-out-unit GENERATION status -- this is the "is SYNC0 actually pulsing?"
    // evidence (vs the pre-wait readback above, which is always 0 because the start is
    // ~100 ms in the future). 0x0984 = activation status (b0 = SYNC0 cyclic op active);
    // 0x098E = SYNC0 status/event (read twice with PD pumped between -- if it CHANGES,
    // pulses are firing); 0x0980 b0 = SYNC-unit control source (1 = PDI/uC owns it, so
    // ECAT's activation is ignored -> would explain a stuck 0). 0x0134 = 0x2D => the
    // start was mis-scheduled (clock not settled before dcsync0).
    //
    // DECISIVE A-vs-B DISAMBIGUATION (architect + team-lead): the question is whether
    // SYNC0-won't-arm is (A) the DC CLOCK being dead/unlocked upstream, or (B) the
    // SYNC-out unit refusing despite a correct, running, properly-started clock. Three
    // reads settle it, all on the SAME pump as the 0x098E double-read:
    //   * 0x0910 read TWICE around one pump -> is the 64-bit system time ADVANCING? Not
    //     advancing => clock DEAD (branch A, upstream ecx_configdc). Advancing => alive.
    //   * 0x092C System Time Difference (the REAL lock signal -- NOT 0x0930, which is the
    //     Speed-Counter-Start CONFIG reg whose default 0x1000=4096 is a red herring).
    //     b31 = sign, b0..30 = |ns|. ~0 => the reference clock is locked/disciplined.
    //   * (0x0910 sampleB - 0x0990 start) signed -> PROVES we are past the scheduled
    //     start when we read, so a stuck 0x0984 is not a "read too early" artifact.
    // Verdict: clock advancing + 0x092C~0 + past start + 0x0984=0 & 0x098E static + 0x0980
    // b0=0 (ECAT owns) => BRANCH B, silicon refusal -> next move is a pcap-diff capture.
    for (int i = 1; i <= impl_->slavecount; ++i) {
        const std::uint16_t adp = impl_->slavelist[i].configadr;
        std::uint8_t cyc_ctrl = 0;      // 0x0980 cyclic unit control (b0: 0=ECAT, 1=PDI owns SYNC unit)
        std::uint8_t act_status = 0;    // 0x0984 activation status (b0 SYNC0 active, b1 SYNC1 active)
        std::uint8_t sync0_stat_a = 0;  // 0x098E SYNC0 status, sample A
        std::uint8_t sync0_stat_b = 0;  // 0x098E SYNC0 status, sample B (after a pump)
        std::uint16_t al_status = 0;    // 0x0134 AL status code
        std::uint64_t sys_a = 0;        // 0x0910 system time, sample A (clock-advance check)
        std::uint64_t sys_b = 0;        // 0x0910 system time, sample B (after the pump)
        std::uint32_t time_diff = 0;    // 0x092C System Time Difference (the real lock signal)
        std::uint64_t start_990 = 0;    // 0x0990 SYNC0 start time (prove we are past it)
        (void)ecx_FPRD(&impl_->port, adp, 0x0980, sizeof(cyc_ctrl), &cyc_ctrl, EC_TIMEOUTRET);
        (void)ecx_FPRD(&impl_->port, adp, 0x0984, sizeof(act_status), &act_status, EC_TIMEOUTRET);
        (void)ecx_FPRD(&impl_->port, adp, 0x098E, sizeof(sync0_stat_a), &sync0_stat_a, EC_TIMEOUTRET);
        (void)ecx_FPRD(&impl_->port, adp, 0x0910, sizeof(sys_a), &sys_a, EC_TIMEOUTRET);
        ecx_send_processdata(&impl_->ctx);
        (void)ecx_receive_processdata(&impl_->ctx, EC_TIMEOUTRET);
        const timespec ts{.tv_sec = 0, .tv_nsec = static_cast<long>(cycle_ns)};
        (void)nanosleep(&ts, nullptr);
        (void)ecx_FPRD(&impl_->port, adp, 0x098E, sizeof(sync0_stat_b), &sync0_stat_b, EC_TIMEOUTRET);
        (void)ecx_FPRD(&impl_->port, adp, 0x0910, sizeof(sys_b), &sys_b, EC_TIMEOUTRET);
        (void)ecx_FPRD(&impl_->port, adp, 0x092C, sizeof(time_diff), &time_diff, EC_TIMEOUTRET);
        (void)ecx_FPRD(&impl_->port, adp, 0x0990, sizeof(start_990), &start_990, EC_TIMEOUTRET);
        (void)ecx_FPRD(&impl_->port, adp, 0x0134, sizeof(al_status), &al_status, EC_TIMEOUTRET);
        const bool pulsing = act_status != 0 || sync0_stat_a != sync0_stat_b;
        const bool clock_advancing = sys_b > sys_a;
        const std::int64_t clock_step = static_cast<std::int64_t>(sys_b) - static_cast<std::int64_t>(sys_a);
        const std::int64_t past_start = static_cast<std::int64_t>(sys_b) - static_cast<std::int64_t>(start_990);
        // 0x092C: b31 sign (1 => local ahead of reference), b0..30 magnitude in ns.
        const bool diff_sign = (time_diff & 0x80000000U) != 0;
        const std::uint32_t diff_mag = time_diff & 0x7FFFFFFFU;
        std::cerr << "[dc]   slave " << i << " SYNC0-GEN (post-start): 0x0984 actStatus=0x" << std::hex << static_cast<unsigned>(act_status)
                  << " 0x098E sync0Status=0x" << static_cast<unsigned>(sync0_stat_a) << "->0x" << static_cast<unsigned>(sync0_stat_b)
                  << " 0x0980 unitCtrl=0x" << static_cast<unsigned>(cyc_ctrl) << " 0x0134 AL=0x" << al_status << std::dec
                  << (pulsing ? "  -> SYNC0 GENERATING" : "  -> *** SYNC0 NOT generating (0x0984=0 & 0x098E static) ***") << '\n';
        std::cerr << "[dc]   slave " << i << " DC-CLOCK: 0x0910 " << sys_a << "->" << sys_b << " (step=" << clock_step << "ns "
                  << (clock_advancing ? "ADVANCING" : "*** DEAD ***") << ")"
                  << " 0x092C sysTimeDiff=" << (diff_sign ? "-" : "+") << diff_mag << "ns"
                  << (diff_mag < 1000 ? " (LOCKED~0)" : " (*** UNLOCKED ***)")
                  << " | 0x0910-0x0990 startDelta=" << past_start << "ns " << (past_start >= 0 ? "(PAST start)" : "(*** before start ***)")
                  << "\n[dc]   slave " << i << " VERDICT: "
                  << (clock_advancing && diff_mag < 1000 && past_start >= 0 && !pulsing
                          ? "BRANCH B -- clock alive+locked+past-start yet SYNC0 refuses -> SILICON, needs pcap-diff"
                      : !clock_advancing ? "BRANCH A -- DC clock DEAD, dig ecx_configdc reference setup"
                      : pulsing          ? "SYNC0 OK -- arming succeeded"
                                         : "INCONCLUSIVE -- see flags above")
                  << '\n';
        if ((cyc_ctrl & 0x01U) != 0) {
            std::cerr << "[dc]   *** 0x0980 b0=1: the SYNC unit is PDI/uC-controlled -- ECAT activation is IGNORED ***\n";
        }
    }

    impl_->dc_cycle_ns = cycle_ns;  // pace the upcoming OP-transition PD pump at this period
}

int SoemBackend::exchange() noexcept {
    ecx_send_processdata(&impl_->ctx);
    return ecx_receive_processdata(&impl_->ctx, EC_TIMEOUTRET);
}

std::int64_t SoemBackend::dc_time() const noexcept {
    // ctx.DCtime -> &impl_->dctime, refreshed by SOEM on each receive_processdata.
    return impl_->dctime;
}

DcSyncStatus SoemBackend::dc_sync_status() noexcept {
    // Single-shot acyclic FPRD of the DC-sync LEVEL registers, AND-reduced across
    // every DC slave, for the SAFE-OP -> OP gate. The caller pumps phase-locked PD
    // each cycle and polls this until `ready`. Reads (per Arthur Ketels' diagnosis):
    //   0x0984 b0 -- SYNC-out unit ARMED. This is the signal that only goes high once
    //                the slave has OBSERVED synchronized DC-phase-locked PDO in SAFE-OP
    //                (the whole point: it stays 0 if we poke PRE-OP regs and ask cold).
    //   0x092C     -- System Time Difference (the REAL lock signal; NOT 0x0930, which is
    //                the Speed-Counter-Start config reg whose 0x1000=4096 default misled
    //                the bench). b31 sign, b0..30 |ns|. Within band => slave clock locked.
    //   0x0134     -- AL status (0x2D = DC start invalid) surfaced for diagnostics.
    constexpr std::uint32_t kLockBandNs = 1000;  // |0x092C| < 1 us => disciplined/locked
    DcSyncStatus st{};
    if (impl_->slavecount < 1) {
        return st;  // no slaves -> not ready
    }
    bool all_locked = true;
    bool all_armed = true;
    std::uint32_t worst_mag = 0;  // largest |0x092C| across slaves -> reported as the signed sample
    for (int i = 1; i <= impl_->slavecount; ++i) {
        const std::uint16_t adp = impl_->slavelist[i].configadr;
        std::uint8_t act_status = 0;  // 0x0984 activation status (b0 = SYNC0 active)
        std::uint32_t time_diff = 0;  // 0x092C system time difference
        std::uint16_t al = 0;         // 0x0134 AL status code
        (void)ecx_FPRD(&impl_->port, adp, 0x0984, sizeof(act_status), &act_status, EC_TIMEOUTRET);
        (void)ecx_FPRD(&impl_->port, adp, 0x092C, sizeof(time_diff), &time_diff, EC_TIMEOUTRET);
        (void)ecx_FPRD(&impl_->port, adp, 0x0134, sizeof(al), &al, EC_TIMEOUTRET);
        const std::uint32_t diff_mag = time_diff & 0x7FFFFFFFU;
        const bool neg = (time_diff & 0x80000000U) != 0;
        all_armed = all_armed && ((act_status & 0x01U) != 0);
        all_locked = all_locked && (diff_mag < kLockBandNs);
        if (diff_mag >= worst_mag) {
            worst_mag = diff_mag;
            st.sys_time_diff_ns = neg ? -static_cast<std::int32_t>(diff_mag) : static_cast<std::int32_t>(diff_mag);
        }
        if (al != 0) {
            st.al_status = al;  // surface any non-zero AL code
        }
    }
    st.clock_locked = all_locked;
    st.sync0_active = all_armed;
    st.sync0_pulsing = all_armed;  // arm level; edge proof is the configure-time 0x098E double-read
    st.ready = all_locked && all_armed;
    return st;
}

int SoemBackend::expected_wkc() const noexcept {
    return impl_->expected_wkc;
}

void SoemBackend::close() noexcept {
    if (impl_->open) {
        // Turn SYNC0 OFF before closing if we enabled DC -- otherwise the drive is
        // left expecting a sync pulse that stops coming, which sync-faults it (A6
        // Er74) and wedges its CoE mailbox until a control-power cycle. Disabling
        // SYNC0 first lets the drive fall back cleanly between runs.
        if (impl_->dc_cycle_ns != 0) {
            for (int i = 1; i <= impl_->slavecount; ++i) {
                ecx_dcsync0(&impl_->ctx, static_cast<std::uint16_t>(i), FALSE, 0, 0);
            }
            impl_->dc_cycle_ns = 0;
        }
        ecx_close(&impl_->ctx);
        impl_->open = false;
    }
}

}  // namespace ethercat
