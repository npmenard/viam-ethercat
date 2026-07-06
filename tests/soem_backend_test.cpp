// SoemBackend cannot do real bus I/O without a NIC + CAP_NET_RAW, so this test
// only exercises the no-hardware paths -- but in doing so it forces the linker
// to resolve SoemBackend (and its SOEM ecx_* references) into an actual
// executable, which is the real value: it proves the SOEM containment TU links.

#include <cstddef>

#include "ethercat/soem_backend.hpp"
#include "test_harness.hpp"

using ethercat::EcatState;
using ethercat::SoemBackend;

TEST("SoemBackend constructs and links against SOEM (no hardware paths)") {
    SoemBackend be;
    // Pre-open: no slaves, no WKC, empty IO windows, benign teardown.
    CHECK_EQ(be.expected_wkc(), 0);
    CHECK_EQ(be.slave_state(0), EcatState::None);
    CHECK_EQ(be.slave_io(1).outputs.size(), std::size_t{0});
    CHECK_EQ(be.slave_io(1).inputs.size(), std::size_t{0});
    be.close();  // no-op when not open
}

TEST_MAIN()
