#include <iostream>

// Placeholder entrypoint for the Viam EtherCAT servo motor module.
//
// The real module (servo_motor implementing rdk:component:motor, the
// boost::asio io_context, and ModuleService registration) is implemented in
// Phase 6. This stub exists so the `ethercat-servo` target configures, builds,
// and links against the Viam C++ SDK from Phase 0 onward.
int main() {
    std::cerr << "ethercat-servo: module not yet implemented (arrives in Phase 6)\n";
    return 1;
}
