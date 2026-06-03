#include "ethercat/soem_backend.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>
#include <ctime>
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
    impl_->open = true;
    impl_->slave_count = count;
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
    const std::uint16_t want = to_soem_state(target);
    impl_->ctx.slavelist[slave].state = want;
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

    // DC step 2 (AFTER SAFE-OP, called from the caller's RT loop while phase-locked PD
    // is already flowing): arm SYNC0 with stock ecx_dcsync0. v2 + ec_sample proved this
    // is all that's needed -- config_map_group lets the drive self-select DC sync-type,
    // and ecx_dcsync0 writes the ESC SYNC0 activation (0x0981) + cycle (0x09A0) + start
    // off the slave's live DC clock. No hand-rolled ESC sequence, no start-delay hack:
    // because the caller pumps PD continuously, the drive sees synchronized traffic
    // before and after the arm, so SYNC0 is never armed into a gap (CLAUDE.md).
    for (int i = 1; i <= impl_->ctx.slavecount; ++i) {
        if (impl_->ctx.slavelist[i].hasdc == FALSE) {
            throw InitError("slave " + std::to_string(i) + " is not DC-capable (hasdc=0) -- cannot enable SYNC0");
        }
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
