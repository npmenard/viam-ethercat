#include "ethercat/soem_backend.hpp"

#include <array>
#include <cassert>
#include <cstdint>
#include <iostream>
#include <string>

#include <soem/soem.h>

#include "ethercat/errors.hpp"
#include "ethercat/hex.hpp"

namespace ethercat {

namespace {

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

// Drain SOEM's error stack and, if a CoE abort is present, return its detail.
// SOEM reports an SDO abort by pushing an ec_errort (with .AbortCode) even when
// the mailbox working counter is non-zero, so checking the WKC alone can miss it.
std::string pop_coe_abort(ecx_contextt* ctx) {
    std::string detail;
    ec_errort err{};
    while (ecx_poperror(ctx, &err)) {
        if (err.Etype == EC_ERR_TYPE_SDO_ERROR) {
            detail = ", CoE abort " + hex(static_cast<std::uint32_t>(err.AbortCode));
        }
    }
    return detail;
}

}  // namespace

// All SOEM-touching state lives here, behind the pimpl. SOEM v2's ecx_contextt
// OWNS its buffers as direct members (port, slavelist[], slavecount, grouplist[],
// DCtime, the internal eeprom/SM/PDO/mailbox pools, ...) -- so unlike v1.4.0 (where
// the context was a struct of pointers we wired to per-instance arrays), we just
// hold ONE zero-initialized context per Master. Still no SOEM global state.
struct SoemBackend::Impl {
    ecx_contextt ctx{};

    std::array<std::byte, 8192> iomap{};
    int expected_wkc = 0;
    int slave_count = 0;
    std::uint32_t dc_cycle_ns = 0;  // SYNC0 cycle once DC is enabled; gates the close() SYNC0-disable
    bool open = false;
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
    const int count = ecx_config_init(&impl_->ctx);
    if (count <= 0) {
        ecx_close(&impl_->ctx);
        throw InitError("no EtherCAT slaves found on '" + name + "' (is the bus wired and powered?)");
    }
    impl_->slave_count = count;

    // PRE-OP confirm -- matches ec_sample's bring-up (ec_sample.c:300-327). config_init leaves
    // slaves nominally in PRE-OP; manualstatechange=1 makes WE own every AL transition (stops
    // config_map_group auto-jumping to SAFE-OP, so configdc still runs in PRE-OP, #20), then we
    // drive all slaves to PRE-OP and CONFIRM it. (We do NOT do an INIT->PRE-OP writestate bounce
    // or an SM/mailbox-counter reprogram -- those were a misdiagnosis; ec_sample does neither, and
    // the wire showed the steady case reaches PRE-OP cleanly without them.) The one thing the A6
    // DOES need after PRE-OP is the patient CoE-handler warm-up below.
    impl_->ctx.manualstatechange = 1;
    ecx_readstate(&impl_->ctx);
    impl_->ctx.slavelist[0].state = EC_STATE_PRE_OP;
    ecx_writestate(&impl_->ctx, 0);
    const std::uint16_t reached = ecx_statecheck(&impl_->ctx, 0, EC_STATE_PRE_OP, 3 * EC_TIMEOUTSTATE);
    if ((reached & 0x0FU) != EC_STATE_PRE_OP) {
        std::string detail;
        ecx_readstate(&impl_->ctx);
        for (int i = 1; i <= count; ++i) {
            const std::uint16_t al = impl_->ctx.slavelist[i].ALstatuscode;
            detail += " [slave " + std::to_string(i) + " state=" + to_string(from_soem_state(impl_->ctx.slavelist[i].state)) +
                      " ALstatuscode=" + hex(static_cast<std::uint32_t>(al)) + " (" + ec_ALstatuscode2string(al) + ")]";
        }
        ecx_close(&impl_->ctx);
        throw InitError("EtherCAT slaves did not reach PRE-OP on '" + name + "' (reached " + to_string(from_soem_state(reached)) + ")" +
                        detail);
    }

    // CRITICAL: refresh EACH slave's cached state to PRE-OP. statecheck(slave 0) only updates the
    // group state (slavelist[0]); slavelist[1..n].state stays at whatever the pre-transition
    // readstate saw (INIT). ecx_mbxsend's direct-send path is gated on slavelist[slave].state >=
    // PRE_OP -- with a stale INIT it takes NEITHER the cyclic NOR the direct path and returns 0
    // WITHOUT transmitting (verified on the wire: 0 mailbox frames). ec_sample calls ecx_readstate
    // after reaching PRE-OP (ec_sample.c:303) for exactly this reason.
    ecx_readstate(&impl_->ctx);

    // PATIENT CoE mailbox readiness gate (two parts, per CoE-capable slave). The A6 in some
    // states is SLOW on EVERYTHING -- the same drive-slowness that makes SAFE-OP->OP take >10s
    // (handled by the patient OP-await) also makes its CoE mailbox slow to ready after the PRE-OP
    // transition. Two distinct slow phases, both waited out patiently here, BEFORE configure()'s
    // first stateful remap write:
    //   (1) SEND side  -- SM0 (mailbox-out) must be writable or ecx_mbxsend never transmits.
    //   (2) HANDLER side -- even once writable, the A6 ignores the FIRST SDO for up to seconds
    //                       (mailbox-out ACKs, mailbox-in never fills), then every SDO works.
    // ec_sample tolerates both by being patient (and its config_init happens to cycle the mailbox
    // first -- its first real send is already counter 5). We do NOT reprogram SMs / reset the
    // mailbox counter / bounce through INIT (2009d47's misdiagnosis -- ec_sample does none of it).
    for (int i = 1; i <= count; ++i) {
        const auto slave = static_cast<std::uint16_t>(i);
        if (impl_->ctx.slavelist[i].mbx_l == 0) {
            continue;  // no CoE mailbox on this slave (e.g. simple I/O) -> nothing to warm up
        }
        // (1) PATIENT send-side gate -- SM0 mailbox-out must be EMPTY/WRITABLE. If it is not,
        // ecx_mbxsend bails WITHOUT putting a frame on the wire (no FPWR to 0x1000), so the warm-up
        // read below never actually asks the slave anything -- it just times out. (Wire-proven:
        // 60583c7 dropped this wait on the theory that the read's own mbxsend covers it; it does
        // NOT on the A6 -> 0 mailbox frames sent, 0/7.) The A6's mailbox-out can stay non-writable
        // for SECONDS when cold (ec_sample's config_init cycles the mailbox first -- its first real
        // send is already counter 5; we hit a cold mailbox), so wait it out PATIENTLY.
        constexpr int kMbxEmptyTimeoutUs = 10'000'000;  // ~10s patient -- same philosophy as the warm-up/OP-await
        if (ecx_mbxempty(&impl_->ctx, slave, kMbxEmptyTimeoutUs) <= 0) {
            ecx_readstate(&impl_->ctx);
            const std::uint16_t al = impl_->ctx.slavelist[i].ALstatuscode;
            std::string detail = " [slave " + std::to_string(i) + " state=" + to_string(from_soem_state(impl_->ctx.slavelist[i].state)) +
                                 " ALstatuscode=" + hex(static_cast<std::uint32_t>(al)) + " (" + ec_ALstatuscode2string(al) + ")]";
            ecx_close(&impl_->ctx);
            throw InitError("slave " + std::to_string(i) + " CoE mailbox-out (SM0) not writable within ~10s after PRE-OP on '" + name +
                            "' -- mbxsend would not transmit" + detail);
        }
        // (2) PATIENT handler warm-up -- now that SM0 is writable, the read actually goes out.
        constexpr int kWarmupTries = 300;         // ~15s @ ~50ms/try -- patient, matching drive slowness
        constexpr int kWarmupTimeoutUs = 50'000;  // 50 ms/try: warm round-trip ~1.4ms, cold fails fast
        constexpr std::uint32_t kWarmupGapUs = 2'000;
        std::uint32_t vendor = 0;
        int warm_wkc = 0;
        for (int attempt = 0; attempt < kWarmupTries; ++attempt) {
            int psize = static_cast<int>(sizeof(vendor));
            warm_wkc = ecx_SDOread(&impl_->ctx, slave, 0x1018, 0x01, FALSE, &psize, &vendor, kWarmupTimeoutUs);
            if (warm_wkc > 0) {
                break;
            }
            (void)osal_usleep(kWarmupGapUs);
        }
        // Drain the errors the ignored attempts queued so they don't bleed into configure()'s
        // first real SDO (its ecx_iserror() check).
        ec_errort err{};
        while (ecx_poperror(&impl_->ctx, &err)) {
        }
        if (warm_wkc <= 0) {
            ecx_readstate(&impl_->ctx);
            const std::uint16_t al = impl_->ctx.slavelist[i].ALstatuscode;
            std::string detail = " [slave " + std::to_string(i) + " state=" + to_string(from_soem_state(impl_->ctx.slavelist[i].state)) +
                                 " ALstatuscode=" + hex(static_cast<std::uint32_t>(al)) + " (" + ec_ALstatuscode2string(al) + ")]";
            ecx_close(&impl_->ctx);
            throw InitError("slave " + std::to_string(i) +
                            " CoE handler did not answer a warm-up SDO read (0x1018:01) within ~15s after PRE-OP on '" + name + "'" +
                            detail);
        }
    }

    impl_->open = true;
    return static_cast<std::size_t>(count);
}

SlaveInfo SoemBackend::slave_info(std::uint16_t slave) const {
    if (slave < 1 || slave > impl_->slave_count) {
        throw ConfigError("SoemBackend::slave_info: slave " + std::to_string(slave) + " out of range");
    }
    const ec_slavet& s = impl_->ctx.slavelist[slave];
    SlaveInfo info;
    info.position = slave;
    info.vendor_id = s.eep_man;
    info.product_code = s.eep_id;
    info.revision = s.eep_rev;
    info.name = s.name;
    info.input_bytes = s.Ibytes;
    info.output_bytes = s.Obytes;
    return info;
}

void SoemBackend::sdo_write(std::uint16_t slave, std::uint16_t index, std::uint8_t sub, std::span<const std::byte> data) {
    // SOEM's psize is an int; PDO/SDO payloads are tiny, so the cast is safe. Single-shot, NO
    // WKC-0 retry: with the mailbox-counter resync in open() the first SDO lands first time, and
    // a blind re-send is actively harmful -- it advances the mailbox counter / can double-apply a
    // remap write, desyncing the map (the "OP did not hold" failure). A genuine CoE abort returns
    // WKC > 0 with an error pushed and is surfaced below; WKC 0 is now a real, reportable fault.
    if (slave < 1 || slave > impl_->slave_count) {
        throw ConfigError("SoemBackend::sdo_write: slave " + std::to_string(slave) + " out of range (configured " +
                          std::to_string(impl_->slave_count) + ")");
    }
    const int size = static_cast<int>(data.size());
    const int wkc = ecx_SDOwrite(&impl_->ctx, slave, index, sub, FALSE, size, data.data(), EC_TIMEOUTRXM);
    // A CoE abort can return wkc > 0 but push an error, so check both. This is the GENERIC SDO
    // tier -> SdoError (carries the abort code); it is NOT PdoMappingError. apply_pdo_map wraps
    // its mapping-object writes (0x1C1x/0x16xx/0x1Axx) to surface PdoMappingError where that name
    // is correct -- a non-mapping abort (mode 0x6060, vendor/tuning, fault-reset) stays SdoError.
    if (wkc <= 0 || ecx_iserror(&impl_->ctx)) {
        const std::string abort = pop_coe_abort(&impl_->ctx);
        throw SdoError("SDO write to slave " + std::to_string(slave) + " object " + std::to_string(index) + ":" + std::to_string(sub) +
                       " failed (working counter " + std::to_string(wkc) + ")" + abort);
    }
}

std::size_t SoemBackend::sdo_read(std::uint16_t slave, std::uint16_t index, std::uint8_t sub, std::span<std::byte> out) {
    if (slave < 1 || slave > impl_->slave_count) {
        throw ConfigError("SoemBackend::sdo_read: slave " + std::to_string(slave) + " out of range (configured " +
                          std::to_string(impl_->slave_count) + ")");
    }
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
    const ec_groupt& g = impl_->ctx.grouplist[0];
    impl_->expected_wkc = (g.outputsWKC * 2) + g.inputsWKC;
}

void SoemBackend::request_state(std::uint16_t slave, EcatState target) {
    // OP is reached ONLY via set_state() + bringup_step()'s pumped RT loop; request_state
    // is PRE-OP/SAFE-OP only. It statechecks WITHOUT pumping process data, so requesting OP
    // here would gap a DC drive (no PD during the transition -> Er74). Loud trap, not a
    // silent one (the old OP-pump branch here was dead after the #20 fold).
    assert(target != EcatState::Op && "request_state: OP goes via set_state + bringup_step; this path doesn't pump PD");

    const std::uint16_t want = to_soem_state(target);
    impl_->ctx.slavelist[slave].state = want;
    ecx_writestate(&impl_->ctx, slave);

    const std::uint16_t reached = ecx_statecheck(&impl_->ctx, slave, want, EC_TIMEOUTSTATE);
    if (reached != want) {
        // Refresh every slave's AL state + AL status code so the error names WHY
        // (e.g. "Invalid DC SYNC Configuration", "SM watchdog") -- a SAFE-OP->OP
        // refusal is otherwise opaque on the bench.
        std::string detail;
        ecx_readstate(&impl_->ctx);
        for (int i = 1; i <= impl_->ctx.slavecount; ++i) {
            const std::uint16_t al = impl_->ctx.slavelist[i].ALstatuscode;
            detail += " [slave " + std::to_string(i) + " state=" + to_string(from_soem_state(impl_->ctx.slavelist[i].state)) +
                      " ALstatuscode=" + hex(static_cast<std::uint32_t>(al)) + " (" + ec_ALstatuscode2string(al) + ")]";
        }
        throw InitError("slave " + std::to_string(slave) + " did not reach state " + to_string(target) + " (reached " +
                        to_string(from_soem_state(reached)) + ")" + detail);
    }
}

void SoemBackend::set_state(std::uint16_t slave, EcatState target) noexcept {
    // Write the state request ONLY -- no pump, no statecheck, no throw. The caller's
    // cyclic loop pumps process data through the transition so a DC drive never sees
    // a gap. slave_state() reports progress.
    impl_->ctx.slavelist[slave].state = to_soem_state(target);
    ecx_writestate(&impl_->ctx, slave);
}

void SoemBackend::reack_op(std::uint16_t slave) noexcept {
    // ec_sample's SAFE-OP->OP recovery nudge (ec_sample.c:143-155): refresh AL state, then per
    // slave ACK a SAFE_OP+ERROR (write SAFE_OP+ACK) or RE-REQUEST OP from a plain SAFE_OP (write
    // OP). The A6's SAFE-OP->OP can take many seconds; ec_sample waits it out with PD flowing +
    // these repeated nudges (NOT a single request), so the bring-up FSM calls this periodically
    // during the OP-await wait. Writes the AL-control register only (the caller keeps pumping PD,
    // so the SyncManager watchdog never starves -> no AL 0x001B). Best-effort, no throw.
    ecx_readstate(&impl_->ctx);
    const int lo = (slave == 0) ? 1 : slave;
    const int hi = (slave == 0) ? impl_->slave_count : slave;
    for (int i = lo; i <= hi; ++i) {
        ec_slavet& s = impl_->ctx.slavelist[i];
        if (s.state == (EC_STATE_SAFE_OP + EC_STATE_ERROR)) {
            s.state = EC_STATE_SAFE_OP + EC_STATE_ACK;  // ACK the error (ec_sample.c:143-147)
            ecx_writestate(&impl_->ctx, static_cast<std::uint16_t>(i));
        } else if (s.state == EC_STATE_SAFE_OP) {
            s.state = EC_STATE_OPERATIONAL;  // re-request OP (ec_sample.c:149-154)
            if (s.mbxhandlerstate == ECT_MBXH_LOST) {
                s.mbxhandlerstate = ECT_MBXH_CYCLIC;
            }
            ecx_writestate(&impl_->ctx, static_cast<std::uint16_t>(i));
        }
    }
}

EcatState SoemBackend::slave_state(std::uint16_t slave) const {
    if (slave > impl_->slave_count) {
        return EcatState::None;
    }
    return from_soem_state(impl_->ctx.slavelist[slave].state);
}

SlaveIo SoemBackend::slave_io(std::uint16_t slave) noexcept {
    if (slave < 1 || slave > impl_->slave_count) {
        return {};
    }
    const ec_slavet& s = impl_->ctx.slavelist[slave];
    // SOEM hands out raw uint8* into the IOmap; reinterpret as std::byte spans.
    auto* out = reinterpret_cast<std::byte*>(s.outputs);
    const auto* in = reinterpret_cast<const std::byte*>(s.inputs);
    return SlaveIo{std::span<std::byte>(out, out != nullptr ? s.Obytes : 0), std::span<const std::byte>(in, in != nullptr ? s.Ibytes : 0)};
}

void SoemBackend::configure_dc_configdc() {
    // DC step 1 (PRE-OP): ecx_configdc detects DC-capable slaves, designates the
    // reference clock, and writes each slave's system-time offset (0x0920) +
    // propagation delay (0x0928). It MUST run + return TRUE before dcsync0 -- else a
    // DC-only drive (the A6) refuses with AL 0x0027 "Freerun not supported". SYNC0 is
    // NOT armed here: the canonical SOEM-author order arms it AFTER SAFE-OP, on a
    // disciplined clock that has seen synchronized PDO traffic.
    const boolean dc_found = ecx_configdc(&impl_->ctx);
    std::cerr << "[dc] ecx_configdc() returned " << (dc_found == TRUE ? "TRUE (DC slaves found)" : "FALSE (NO DC slaves)") << '\n';
    if (dc_found == FALSE) {
        throw InitError("use_distributed_clocks is set but ecx_configdc() found NO DC-capable slave on the bus");
    }
}

void SoemBackend::arm_dc_sync(std::uint32_t cycle_ns, std::int32_t sync0_shift_ns) {
    impl_->dc_cycle_ns = cycle_ns;  // remember it so close() disables SYNC0

    // Arm SYNC0 with stock ecx_dcsync0, per ec_sample -- called in PRE-OP, BEFORE
    // config_map_group. The A6 latches its SM sync-type (SM vs DC) at the PRE-OP->SAFE-OP
    // transition based on whether SYNC0 is ALREADY armed: arm here and the drive
    // self-selects DC (0x1C32:01 reads 2) and holds OP; arm only after SAFE-OP and the
    // drive has already chosen SM-sync -> Er74.1 "no sync signal" ~1s into OP (bench:
    // team-lead). No hasdc guard: hasdc is not set until config_map_group/configdc, and
    // ec_sample arms unconditionally here (ecx_dcsync0 writes the ESC SYNC0 registers
    // directly via configadr). The ~50ms-watchdog worry (CLAUDE.md lesson 5) does not
    // bite: the arm sits in PRE-OP with the long config_map+configdc before OP, and the
    // RT loop is pumping PD before SYNC0's first edge (stock 100ms SyncDelay).
    for (int i = 1; i <= impl_->ctx.slavecount; ++i) {
        ecx_dcsync0(&impl_->ctx, static_cast<std::uint16_t>(i), TRUE, cycle_ns, sync0_shift_ns);
    }
}

int SoemBackend::exchange() noexcept {
    ecx_send_processdata(&impl_->ctx);
    return ecx_receive_processdata(&impl_->ctx, EC_TIMEOUTRET);
}

std::int64_t SoemBackend::dc_time() const noexcept {
    // ctx.DCtime is refreshed by SOEM on each receive_processdata.
    return impl_->ctx.DCtime;
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
            for (int i = 1; i <= impl_->ctx.slavecount; ++i) {
                ecx_dcsync0(&impl_->ctx, static_cast<std::uint16_t>(i), FALSE, 0, 0);
            }
            impl_->dc_cycle_ns = 0;
        }
        // Walk the drive DOWN to INIT before dropping the master -- the standard EtherCAT
        // teardown. Leaving it in OP with SYNC0 just disabled + the socket dropped left the
        // drive's DC subsystem un-re-syncable on the IMMEDIATELY following bring-up (a perfect
        // odd/even alternation -- a successful run's locked-then-killed DC poisoned the next run;
        // a failed run that never locked DC did not -- which blocked the energized first move,
        // 4/4 "OP did not hold"). The INIT transition RESETS the drive's SMs + DC state, so the
        // next config_init starts from a clean slate regardless of what the SYNC0-disable left.
        // ec_sample sidesteps this by being killed in OP (the drive falls to INIT on carrier
        // loss); this is the deterministic equivalent. Best-effort + bounded -- close() is
        // noexcept and ecx_writestate/ecx_statecheck do not throw.
        impl_->ctx.slavelist[0].state = EC_STATE_INIT;
        ecx_writestate(&impl_->ctx, 0);
        ecx_statecheck(&impl_->ctx, 0, EC_STATE_INIT, EC_TIMEOUTSTATE);
        ecx_close(&impl_->ctx);
        impl_->open = false;
    }
}

}  // namespace ethercat
