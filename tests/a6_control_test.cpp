// a6_validate CLI-option coverage. The A6Control drive-behavior tests that used to
// live here (DA-B mode-echo refuse, the PP rising-edge handshake + reached predicate,
// the PV Quick-Stop ramp-then-disable, and the 0x605A/0x6085/VEL configure refusals)
// were RETIRED with the sim-fidelity demotion (#17 item 12): those are drive behaviors,
// now proven HW-first on the bench (a6_validate) and in the RDK campaign, not in sim.
// See docs/offline-test-retirement.md for the retired-case -> HW-check map.

#include "tools/a6_control.hpp"
#include "test_harness.hpp"

using ethercat::tools::Options;

// --- exclusivity (CLI) --------------------------------------------------------------------
TEST("#53 exclusivity: two move flags -> conflict (mode_flag_count > 1)") {
    Options o;
    CHECK_EQ(ethercat::tools::mode_flag_count(o), 0);  // none
    o.move_vel = true;
    CHECK_EQ(ethercat::tools::mode_flag_count(o), 1);  // one is fine
    o.move_pos = true;
    CHECK(ethercat::tools::mode_flag_count(o) > 1);  // two -> the CLI rejects it
    Options p;
    p.move_pp = true;
    p.move_sine = true;
    CHECK(ethercat::tools::mode_flag_count(p) > 1);
}

TEST_MAIN()
