# cmake/soem.cmake
#
# Acquire SOEM (Simple Open EtherCAT Master) via FetchContent and expose a
# single consumable CMake target: `soem`.
#
# All SOEM-specific knowledge is intentionally isolated in this file so the
# rest of the build never has to know how SOEM is fetched, named, or wired.
# A later migration to Conan / a system package should only need to touch
# this file (provide a `soem` target some other way) and nothing else.
#
# ----------------------------------------------------------------------------
# This project is pinned to **SOEM v2.0.0** (+10 commits; see the pin below)
# — the redesigned API that drops the global `ec_slave[]`/`ec_slavecount`
# state in favour of a caller-owned `ecx_contextt` and renames the umbrella
# header from `soem/ethercat.h` to `soem/soem.h`. The only translation units
# that touch SOEM — `src/ethercat/soem_backend.cpp` (the master backend) and
# `src/tools/ec_scan.cpp` — have been ported to the v2 API. The rest of the
# codebase is SOEM-free by construction (the EcatBackend abstraction), so the
# v2 migration's blast radius is contained to those two files.
#
# v2.0.0 properties relevant to wiring:
#   * Target name: `soem` (STATIC library; same as v1.4.0).
#   * Public umbrella header: `soem/soem.h` (path provided via the target's
#     own `target_include_directories(... PUBLIC ...)`).
#   * `cmake_minimum_required(VERSION 3.28)` — no policy backport needed.
#   * No `-Werror` in its own flags (unlike v1.4.0).
#   * Samples are gated on `SOEM_BUILD_SAMPLES AND PROJECT_IS_TOP_LEVEL`, so
#     pulling it in via FetchContent suppresses them automatically.
# ----------------------------------------------------------------------------

include(FetchContent)

# Pinned by commit SHA (== v2.0.0+10, ec_sample parity) for reproducibility.
# This is the EXACT commit our known-good reference ~/SOEM/samples/ec_sample
# (which brings the A6-EC to OP in DC mode) is validated against, so any
# behaviour difference on the bench is OUR code, not a SOEM version delta.
set(ETHERCAT_SOEM_GIT_TAG "b410bf6ef599d5c85302ea45cae5f55f8e9aa394"
    CACHE STRING "SOEM git commit SHA to build against (== v2.0.0+10, ec_sample parity)")

FetchContent_Declare(
  soem
  GIT_REPOSITORY https://github.com/OpenEtherCATsociety/SOEM
  GIT_TAG ${ETHERCAT_SOEM_GIT_TAG}
  GIT_SHALLOW FALSE
  SYSTEM
)

FetchContent_MakeAvailable(soem)

if(TARGET soem)
  # `libethercat` ends up as a SHARED library when a transitive dep
  # (viam-cpp-sdk) flips BUILD_SHARED_LIBS=ON. Linking non-PIC static objects
  # into a .so fails with R_X86_64_PC32 relocation errors, so force PIC.
  set_target_properties(soem PROPERTIES POSITION_INDEPENDENT_CODE ON)
endif()
