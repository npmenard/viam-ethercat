#include <cstdint>
#include <iostream>
#include <memory>
#include <string>

#include <soem/ethercat.h>

namespace {

// Exit codes: keep them small, distinct, and documented.
constexpr int kUsageError = 2;  // bad command line
constexpr int kBusError = 1;    // could not open NIC or no slaves found

// Backing storage for one *reentrant* SOEM context.
//
// SOEM's `ecx_contextt` is just a bag of pointers into caller-owned buffers --
// the classic global `ec_slave[]` / `ec_slavecount` API is literally this same
// struct wired to file-scope statics. We own the buffers here instead, so there
// is no SOEM global state in play. This is the pattern `soem_backend` will
// reuse in a later phase so the module can safely host more than one master in
// a single process.
struct SoemContext {
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

    SoemContext() {
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

}  // namespace

// Minimal EtherCAT bus scanner. Opens the given NIC with SOEM's reentrant API
// against a local context, enumerates the slaves on the bus, and prints the
// count. It is expected to fail *cleanly* (non-zero, with a clear message) when
// it lacks privileges or no NIC/bus is present -- runtime success needs
// CAP_NET_RAW, which CI does not grant.
int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "usage: ec_scan <interface>\n"
                  << "  e.g. ec_scan eth0\n";
        return kUsageError;
    }

    const std::string ifname = argv[1];

    // Heap-allocated: the backing buffers (slavelist, eeprom caches, ...) are
    // a few hundred KB, too large to want on the stack.
    const auto soem = std::make_unique<SoemContext>();
    ecx_contextt* const ctx = &soem->ctx;

    // ecx_init opens a raw packet socket on the NIC. This needs CAP_NET_RAW
    // (run via setcap or as root). It returns > 0 on success.
    if (ecx_init(ctx, ifname.c_str()) <= 0) {
        std::cerr << "ec_scan: failed to open EtherCAT interface '" << ifname
                  << "': need CAP_NET_RAW (run with setcap or as root) and the interface must exist.\n";
        return kBusError;
    }

    // ecx_config_init enumerates the slaves; its return value (also stored in
    // soem->slavecount) is the number of slaves found.
    const int slave_count = ecx_config_init(ctx, 0);
    if (slave_count <= 0) {
        std::cerr << "ec_scan: no EtherCAT slaves found on '" << ifname << "' (is the bus wired and powered?).\n";
        ecx_close(ctx);
        return kBusError;
    }

    std::cout << "ec_scan: found " << slave_count << " EtherCAT slave(s) on '" << ifname << "':\n";
    for (int i = 1; i <= slave_count; ++i) {
        std::cout << "  slave " << i << ": " << soem->slavelist[i].name << '\n';
    }

    ecx_close(ctx);
    return 0;
}
