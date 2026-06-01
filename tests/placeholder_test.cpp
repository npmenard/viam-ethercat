#include <cstring>
#include <iostream>

#include "ethercat/version.hpp"

// Placeholder ctest so `make test` has something to run from Phase 0. Real unit
// tests (pdo_buffer, cia402, pdo_cache, conversions, ...) arrive in later phases.
int main() {
    const char* const v = ethercat::version();
    if (v == nullptr || std::strlen(v) == 0) {
        std::cerr << "FAIL: ethercat::version() returned an empty string\n";
        return 1;
    }
    std::cout << "ethercat library version: " << v << '\n';
    return 0;
}
