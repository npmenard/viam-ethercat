#include <cstdlib>
#include <iostream>
#include <string>

#include <soem/ethercat.h>

namespace {

// Exit codes: keep them small, distinct, and documented.
constexpr int kUsageError = 2;  // bad command line
constexpr int kBusError = 1;    // could not open NIC or no slaves found

}  // namespace

// Minimal EtherCAT bus scanner. Opens the given NIC with SOEM, enumerates the
// slaves on the bus, and prints the count. It is expected to fail *cleanly*
// (non-zero, with a clear message) when it lacks privileges or no NIC/bus is
// present -- runtime success needs CAP_NET_RAW, which CI does not grant.
int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "usage: ec_scan <interface>\n"
                  << "  e.g. ec_scan eth0\n";
        return kUsageError;
    }

    const std::string ifname = argv[1];

    // ec_init opens a raw packet socket on the NIC. This needs CAP_NET_RAW
    // (run via setcap or as root). It returns > 0 on success.
    if (ec_init(ifname.c_str()) <= 0) {
        std::cerr << "ec_scan: failed to open EtherCAT interface '" << ifname
                  << "': need CAP_NET_RAW (run with setcap or as root) and the interface must exist.\n";
        return kBusError;
    }

    // ec_config_init enumerates the slaves; its return value (also reflected in
    // the global ec_slavecount) is the number of slaves found.
    const int slave_count = ec_config_init(0);
    if (slave_count <= 0) {
        std::cerr << "ec_scan: no EtherCAT slaves found on '" << ifname << "' (is the bus wired and powered?).\n";
        ec_close();
        return kBusError;
    }

    std::cout << "ec_scan: found " << slave_count << " EtherCAT slave(s) on '" << ifname << "':\n";
    for (int i = 1; i <= slave_count; ++i) {
        std::cout << "  slave " << i << ": " << ec_slave[i].name << '\n';
    }

    ec_close();
    return 0;
}
