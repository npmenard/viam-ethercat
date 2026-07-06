#include "ethercat/soem_backend.hpp"

#include <array>
#include <cassert>
#include <cstdint>
#include <iostream>
#include <string>

#include <soem/soem.h>

#include "ethercat/errors.hpp"
#include "ethercat/util.hpp"

namespace ethercat {

const char* to_string(EcatState state) noexcept {
    switch (state) {
        case EcatState::None:
            return "None";
        case EcatState::Init:
            return "Init";
        case EcatState::PreOp:
            return "PreOp";
        case EcatState::SafeOp:
            return "SafeOp";
        case EcatState::Op:
            return "Op";
    }
    return "Unknown";
}

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

struct SoemBackend::Impl {
    ecx_contextt ctx{};

    std::array<std::byte, 8192> iomap{};
    int expected_wkc = 0;
    int slave_count = 0;
    std::uint32_t dc_cycle_ns = 0;  // SYNC0 cycle once DC is enabled
    bool is_open = false;
};

SoemBackend::SoemBackend() : impl_(std::make_unique<Impl>()) {}
SoemBackend::~SoemBackend() {
    if (impl_->is_open) {
        ecx_close(&impl_->ctx);
    }
}

std::size_t SoemBackend::open(std::string_view ifname) {
    if (impl_->is_open) {
        throw Error("SoemBackend::open: bus already open (one master per backend; close() first)");
    }
    const std::string name(ifname);
    if (ecx_init(&impl_->ctx, name.c_str()) <= 0) {
        throw Error("failed to open EtherCAT interface '" + name +
                    "': need CAP_NET_RAW (run with setcap or as root) and the interface must exist");
    }
    const int count = ecx_config_init(&impl_->ctx);
    if (count <= 0) {
        ecx_close(&impl_->ctx);
        throw Error("no EtherCAT slaves found on '" + name + "' (is the bus wired and powered?)");
    }
    impl_->slave_count = count;

    // Confirm PRE-OP. config_init leaves slaves nominally in PRE-OP; manualstatechange=1 makes
    // this code own every AL transition (so config_map_group does not auto-jump to SAFE-OP and
    // configdc still runs in PRE-OP), then drive all slaves to PRE-OP and confirm it. No
    // INIT->PRE-OP writestate bounce and no SM/mailbox-counter reprogram are needed. Slow-mailbox
    // drives do need the patient CoE-handler warm-up below after PRE-OP.
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
        throw Error("EtherCAT slaves did not reach PRE-OP on '" + name + "' (reached " + to_string(from_soem_state(reached)) + ")" +
                    detail);
    }

    // Refresh each slave's cached state to PRE-OP. statecheck(slave 0) only updates the group
    // state (slavelist[0]); slavelist[1..n].state stays at whatever the pre-transition readstate
    // saw (INIT). ecx_mbxsend's direct-send path is gated on slavelist[slave].state >= PRE_OP,
    // so with a stale INIT it takes neither the cyclic nor the direct path and returns 0 without
    // transmitting. ec_sample calls ecx_readstate after reaching PRE-OP for the same reason.
    ecx_readstate(&impl_->ctx);

    // CoE mailbox readiness gate (two parts, per CoE-capable slave). Workaround for drives whose
    // CoE mailbox is slow to ready after the PRE-OP transition (the same drives whose SAFE-OP->OP
    // takes many seconds). Two distinct slow phases, both waited out here, before configure()'s
    // first stateful remap write:
    //   (1) send side: SM0 (mailbox-out) must be writable or ecx_mbxsend never transmits.
    //   (2) handler side: even once writable, some drives ignore the first SDO for up to seconds
    //                     (mailbox-out ACKs, mailbox-in never fills), then every SDO works.
    // No SM reprogram, mailbox-counter reset, or INIT bounce is needed.
    for (int i = 1; i <= count; ++i) {
        const auto slave = static_cast<std::uint16_t>(i);
        if (impl_->ctx.slavelist[i].mbx_l == 0) {
            continue;  // no CoE mailbox on this slave (e.g. simple I/O) -> nothing to warm up
        }
        // (1) Send-side gate: SM0 mailbox-out must be empty/writable. If it is not, ecx_mbxsend
        // bails without putting a frame on the wire (no FPWR to 0x1000), so the warm-up read below
        // never asks the slave anything and just times out. A cold drive's mailbox-out can stay
        // non-writable for seconds, so wait it out.
        constexpr int kMbxEmptyTimeoutUs = 10'000'000;  // ~10s
        if (ecx_mbxempty(&impl_->ctx, slave, kMbxEmptyTimeoutUs) <= 0) {
            ecx_readstate(&impl_->ctx);
            const std::uint16_t al = impl_->ctx.slavelist[i].ALstatuscode;
            const std::string detail = " [slave " + std::to_string(i) +
                                       " state=" + to_string(from_soem_state(impl_->ctx.slavelist[i].state)) +
                                       " ALstatuscode=" + hex(static_cast<std::uint32_t>(al)) + " (" + ec_ALstatuscode2string(al) + ")]";
            ecx_close(&impl_->ctx);
            std::string msg = "slave " + std::to_string(i);
            msg += " CoE mailbox-out (SM0) not writable within ~10s after PRE-OP on '" + name + "' -- mbxsend would not transmit";
            msg += detail;
            throw Error(msg);
        }
        // (2) Handler warm-up: now that SM0 is writable, the read actually goes out.
        constexpr int kWarmupTries = 300;
        constexpr int kWarmupTimeoutUs = 50'000;
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
            const std::string detail = " [slave " + std::to_string(i) +
                                       " state=" + to_string(from_soem_state(impl_->ctx.slavelist[i].state)) +
                                       " ALstatuscode=" + hex(static_cast<std::uint32_t>(al)) + " (" + ec_ALstatuscode2string(al) + ")]";
            ecx_close(&impl_->ctx);
            std::string msg = "slave " + std::to_string(i);
            msg += " CoE handler did not answer a warm-up SDO read (0x1018:01) within ~15s after PRE-OP on '" + name + "'";
            msg += detail;
            throw Error(msg);
        }
    }

    impl_->is_open = true;
    return static_cast<std::size_t>(count);
}

SlaveInfo SoemBackend::slave_info(std::uint16_t slave) const {
    if (slave < 1 || slave > impl_->slave_count) {
        throw Error("SoemBackend::slave_info: slave " + std::to_string(slave) + " out of range");
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
    // SOEM's psize is an int; PDO/SDO payloads are tiny, so the cast is safe. Single-shot, no
    // WKC-0 retry: the first SDO lands first time, and a blind re-send is harmful -- it advances
    // the mailbox counter and can double-apply a remap write, desyncing the map. A genuine CoE
    // abort returns WKC > 0 with an error pushed and is surfaced below; WKC 0 is a real,
    // reportable fault.
    if (slave < 1 || slave > impl_->slave_count) {
        throw Error("SoemBackend::sdo_write: slave " + std::to_string(slave) + " out of range (configured " +
                    std::to_string(impl_->slave_count) + ")");
    }
    const int size = static_cast<int>(data.size());
    const int wkc = ecx_SDOwrite(&impl_->ctx, slave, index, sub, FALSE, size, data.data(), EC_TIMEOUTRXM);
    // A CoE abort can return wkc > 0 but push an error, so check both. This is the generic SDO
    // tier, so it throws SdoError (carrying the abort code), not PdoMappingError. apply_pdo_map
    // wraps its mapping-object writes (0x1C1x/0x16xx/0x1Axx) to surface PdoMappingError where that
    // name is correct; a non-mapping abort (mode 0x6060, vendor/tuning, fault-reset) stays SdoError.
    if (wkc <= 0 || ecx_iserror(&impl_->ctx)) {
        const std::string abort = pop_coe_abort(&impl_->ctx);
        throw SdoError("SDO write to slave " + std::to_string(slave) + " object " + std::to_string(index) + ":" + std::to_string(sub) +
                       " failed (working counter " + std::to_string(wkc) + ")" + abort);
    }
}

std::size_t SoemBackend::sdo_read(std::uint16_t slave, std::uint16_t index, std::uint8_t sub, std::span<std::byte> out) {
    if (slave < 1 || slave > impl_->slave_count) {
        throw Error("SoemBackend::sdo_read: slave " + std::to_string(slave) + " out of range (configured " +
                    std::to_string(impl_->slave_count) + ")");
    }
    int size = static_cast<int>(out.size());
    const int wkc = ecx_SDOread(&impl_->ctx, slave, index, sub, FALSE, &size, out.data(), EC_TIMEOUTRXM);
    if (wkc <= 0 || ecx_iserror(&impl_->ctx)) {
        const std::string abort = pop_coe_abort(&impl_->ctx);
        throw Error("SDO read from slave " + std::to_string(slave) + " object " + std::to_string(index) + ":" + std::to_string(sub) +
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
    // OP is reached only via set_state() plus bringup_step()'s pumped RT loop; request_state is
    // PRE-OP/SAFE-OP only. It statechecks without pumping process data, so requesting OP here
    // would gap a DC drive (no PD during the transition -> its sync watchdog faults it). The
    // assert is a loud trap.
    assert(target != EcatState::Op && "request_state: OP goes via set_state + bringup_step; this path doesn't pump PD");

    const std::uint16_t want = to_soem_state(target);
    impl_->ctx.slavelist[slave].state = want;
    ecx_writestate(&impl_->ctx, slave);

    const std::uint16_t reached = ecx_statecheck(&impl_->ctx, slave, want, EC_TIMEOUTSTATE);
    if (reached != want) {
        // Refresh every slave's AL state and AL status code so the error names why (e.g.
        // "Invalid DC SYNC Configuration", "SM watchdog"); a SAFE-OP->OP refusal is otherwise
        // opaque.
        std::string detail;
        ecx_readstate(&impl_->ctx);
        for (int i = 1; i <= impl_->ctx.slavecount; ++i) {
            const std::uint16_t al = impl_->ctx.slavelist[i].ALstatuscode;
            detail += " [slave " + std::to_string(i) + " state=" + to_string(from_soem_state(impl_->ctx.slavelist[i].state)) +
                      " ALstatuscode=" + hex(static_cast<std::uint32_t>(al)) + " (" + ec_ALstatuscode2string(al) + ")]";
        }
        throw Error("slave " + std::to_string(slave) + " did not reach state " + to_string(target) + " (reached " +
                    to_string(from_soem_state(reached)) + ")" + detail);
    }
}

void SoemBackend::set_state(std::uint16_t slave, EcatState target) noexcept {
    // Write the state request only: no pump, no statecheck, no throw. The caller's cyclic loop
    // pumps process data through the transition so a DC drive never sees a gap. slave_state()
    // reports progress.
    impl_->ctx.slavelist[slave].state = to_soem_state(target);
    ecx_writestate(&impl_->ctx, slave);
}

void SoemBackend::reack_op(std::uint16_t slave) noexcept {
    // SAFE-OP->OP recovery nudge: refresh AL state, then per slave ACK a SAFE_OP+ERROR (write
    // SAFE_OP+ACK) or re-request OP from a plain SAFE_OP (write OP). Workaround for drives whose
    // SAFE-OP->OP takes many seconds and is waited out with PD flowing and these repeated nudges,
    // not a single request; the bring-up FSM calls this periodically during the OP-await wait. Writes the
    // AL-control register only; the caller keeps pumping PD, so the SyncManager watchdog never
    // starves (no AL 0x001B). Best-effort, no throw.
    ecx_readstate(&impl_->ctx);
    const int lo = (slave == 0) ? 1 : slave;
    const int hi = (slave == 0) ? impl_->slave_count : slave;
    for (int i = lo; i <= hi; ++i) {
        ec_slavet& s = impl_->ctx.slavelist[i];
        if (s.state == (EC_STATE_SAFE_OP + EC_STATE_ERROR)) {
            s.state = EC_STATE_SAFE_OP + EC_STATE_ACK;  // ACK the error
            ecx_writestate(&impl_->ctx, static_cast<std::uint16_t>(i));
        } else if (s.state == EC_STATE_SAFE_OP) {
            s.state = EC_STATE_OPERATIONAL;  // re-request OP
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

std::uint16_t SoemBackend::al_status_code(std::uint16_t slave) const noexcept {
    // The ESC AL status code cached from the last state check: why the drive refused an AL
    // transition (e.g. 0x0027 free-run not supported on a DC-only drive). A plain field read, no I/O.
    if (slave < 1 || slave > impl_->slave_count) {
        return 0;
    }
    return impl_->ctx.slavelist[slave].ALstatuscode;
}

std::string SoemBackend::describe_al_code(std::uint16_t code) {
    // SOEM's human string for an arbitrary (latched) AL code; no slave read, no I/O.
    return ec_ALstatuscode2string(code);
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
    // ecx_configdc (PRE-OP) detects DC-capable slaves, designates the reference clock, and writes
    // each slave's system-time offset (0x0920) and propagation delay (0x0928). It must run and
    // return TRUE, or a DC-only drive refuses OP with AL 0x0027 "Freerun not supported".
    // SYNC0 is not armed here; arm_dc_sync does that, in PRE-OP.
    const boolean dc_found = ecx_configdc(&impl_->ctx);
    std::cerr << "[dc] ecx_configdc() returned " << (dc_found == TRUE ? "TRUE (DC slaves found)" : "FALSE (NO DC slaves)") << '\n';
    if (dc_found == FALSE) {
        throw Error("use_distributed_clocks is set but ecx_configdc() found NO DC-capable slave on the bus");
    }
}

void SoemBackend::arm_dc_sync(std::uint32_t cycle_ns, std::int32_t sync0_shift_ns) {
    impl_->dc_cycle_ns = cycle_ns;  // remember it so close() disables SYNC0

    // Arm SYNC0 with stock ecx_dcsync0, in PRE-OP, before config_map_group. Workaround for drives
    // that latch their SM sync-type (SM vs DC) at the PRE-OP->SAFE-OP transition based on whether
    // SYNC0 is already armed: arm here and such a drive self-selects DC (0x1C32:01 reads 2) and
    // holds OP; arm only after SAFE-OP and it has already chosen SM-sync, then sync-faults
    // shortly into OP. No hasdc guard: hasdc is not set until config_map_group/configdc, and the
    // arm is unconditional here (ecx_dcsync0 writes the ESC SYNC0 registers directly via
    // configadr). The ~50ms watchdog does not bite: the arm sits in PRE-OP with the long
    // config_map+configdc before OP, and the RT loop is pumping PD before SYNC0's first edge
    // (stock 100ms SyncDelay).
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
    if (impl_->is_open) {
        // Turn SYNC0 off before closing if DC was enabled -- a workaround for drives that, left
        // expecting a sync pulse that stops coming, sync-fault and wedge their CoE mailbox until
        // a control-power cycle. Disabling SYNC0 first lets the drive fall back cleanly between
        // runs.
        if (impl_->dc_cycle_ns != 0) {
            for (int i = 1; i <= impl_->ctx.slavecount; ++i) {
                ecx_dcsync0(&impl_->ctx, static_cast<std::uint16_t>(i), FALSE, 0, 0);
            }
            impl_->dc_cycle_ns = 0;
        }
        // Walk the drive down to INIT before dropping the master, the standard EtherCAT teardown.
        // Leaving it in OP with SYNC0 just disabled and the socket dropped left the drive's DC
        // subsystem un-re-syncable on the immediately following bring-up (a locked-then-killed DC
        // poisoned the next run). The INIT transition resets the drive's SMs and DC state, so the
        // next config_init starts from a clean slate. Best-effort and bounded: close() is noexcept
        // and ecx_writestate/ecx_statecheck do not throw.
        impl_->ctx.slavelist[0].state = EC_STATE_INIT;
        ecx_writestate(&impl_->ctx, 0);
        ecx_statecheck(&impl_->ctx, 0, EC_STATE_INIT, EC_TIMEOUTSTATE);
        ecx_close(&impl_->ctx);
        impl_->is_open = false;
    }
}

}  // namespace ethercat
