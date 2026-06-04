#include "ethercat/soem_backend.hpp"

#include <array>
#include <cassert>
#include <cstdint>
#include <iostream>
#include <string>

#include <soem/soem.h>

#include "ethercat/errors.hpp"

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

    // PRE-OP settle -- the ec_sample recovery the A6 needs before any SDO. config_init
    // leaves slaves nominally in PRE-OP, but the CoE mailbox is NOT reliably ready until
    // we drive a CONFIRMED PRE-OP: the A6 WKC-0's the first SDO otherwise, especially
    // after a prior faulted run left it in a bad AL state. So (mirroring ec_sample):
    //   - manualstatechange = 1: WE own every AL transition; stops ecx_config_map_group
    //     from auto-jumping to SAFE-OP later, so configdc still runs in PRE-OP (#20).
    //   - bounce any slave NOT in PRE-OP through INIT first (clears a latent AL error a
    //     prior faulted run burned in), then drive all slaves PRE-OP and CONFIRM it.
    impl_->ctx.manualstatechange = 1;
    ecx_readstate(&impl_->ctx);
    for (int i = 1; i <= count; ++i) {
        if ((impl_->ctx.slavelist[i].state & 0x0FU) != EC_STATE_PRE_OP) {
            impl_->ctx.slavelist[i].state = EC_STATE_INIT;
            ecx_writestate(&impl_->ctx, static_cast<std::uint16_t>(i));
            ecx_statecheck(&impl_->ctx, static_cast<std::uint16_t>(i), EC_STATE_INIT, EC_TIMEOUTSTATE);
        }
    }
    impl_->ctx.slavelist[0].state = EC_STATE_PRE_OP;
    ecx_writestate(&impl_->ctx, 0);
    const std::uint16_t reached = ecx_statecheck(&impl_->ctx, 0, EC_STATE_PRE_OP, 3 * EC_TIMEOUTSTATE);
    if ((reached & 0x0FU) != EC_STATE_PRE_OP) {
        std::string detail;
        ecx_readstate(&impl_->ctx);
        for (int i = 1; i <= count; ++i) {
            const std::uint16_t al = impl_->ctx.slavelist[i].ALstatuscode;
            detail += " [slave " + std::to_string(i) + " state=" + to_string(from_soem_state(impl_->ctx.slavelist[i].state)) +
                      " ALstatuscode=" + hex32(al) + " (" + ec_ALstatuscode2string(al) + ")]";
        }
        ecx_close(&impl_->ctx);
        throw InitError("EtherCAT slaves did not settle to PRE-OP on '" + name + "' (reached " + to_string(from_soem_state(reached)) +
                        "); the CoE mailbox would not be ready for SDO" + detail);
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
    // SOEM's psize is an int; PDO/SDO payloads are tiny, so the cast is safe.
    const int size = static_cast<int>(data.size());
    const int wkc = ecx_SDOwrite(&impl_->ctx, slave, index, sub, FALSE, size, data.data(), EC_TIMEOUTRXM);
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
    impl_->ctx.slavelist[slave].state = to_soem_state(target);
    ecx_writestate(&impl_->ctx, slave);
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
        ecx_close(&impl_->ctx);
        impl_->open = false;
    }
}

}  // namespace ethercat
