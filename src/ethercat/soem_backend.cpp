#include "ethercat/soem_backend.hpp"

#include <array>
#include <cstring>
#include <string>

#include <soem/ethercat.h>

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
        // Reaching OP requires pumping process data while the slaves transition.
        for (int chk = 0; chk < 200; ++chk) {
            ecx_send_processdata(&impl_->ctx);
            ecx_receive_processdata(&impl_->ctx, EC_TIMEOUTRET);
            reached = ecx_statecheck(&impl_->ctx, slave, want, 50000);
            if (reached == want) {
                break;
            }
        }
    } else {
        reached = ecx_statecheck(&impl_->ctx, slave, want, EC_TIMEOUTSTATE);
    }

    if (reached != want) {
        throw InitError("slave " + std::to_string(slave) + " did not reach state " + to_string(target) + " (reached " +
                        to_string(from_soem_state(reached)) + ")");
    }
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

int SoemBackend::exchange() noexcept {
    ecx_send_processdata(&impl_->ctx);
    return ecx_receive_processdata(&impl_->ctx, EC_TIMEOUTRET);
}

int SoemBackend::expected_wkc() const noexcept {
    return impl_->expected_wkc;
}

void SoemBackend::close() noexcept {
    if (impl_->open) {
        ecx_close(&impl_->ctx);
        impl_->open = false;
    }
}

}  // namespace ethercat
