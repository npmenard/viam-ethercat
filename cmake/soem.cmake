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
# Findings from inspecting https://github.com/OpenEtherCATsociety/SOEM
# (verified 2026-06-01 by cloning and reading the tagged CMakeLists.txt):
#
#   * Tags available: v1.3.1, v1.3.2, v1.3.3-beta.1, v1.4.0, v2.0.0 (sparse).
#
#   * master / v2.0.0 is a *redesigned* API (project version 2.0.0). It drops
#     the classic `ec_slave[]` / `ec_slavecount` global state and the matching
#     reentrant `ecx_*` flow that our master library (and src/tools/ec_scan.cpp)
#     is written against. We therefore deliberately DO NOT track master.
#
#   * We pin the **v1.4.0** release, but BY COMMIT SHA rather than by tag, since
#     the tags are sparse/stale and a SHA is fully reproducible:
#         abbf0d42e38d6cfbaa4c1e9e8e07ace651c386fd  == tag v1.4.0
#     v1.4.0 provides the reentrant `ecx_init(ctx,…)` / `ecx_config_init(ctx,…)`
#     API with a caller-owned `ecx_contextt` (no global state) that we build on.
#
#   * CMake target name is the unqualified **`soem`** (a STATIC library).
#     There is NO namespaced `SOEM::soem` alias and NO installed package
#     config, so `find_package(soem)` is not available — FetchContent is the
#     supported acquisition path here.
#
#   * v1.4.0 uses old-style directory-scoped `include_directories()`, which
#     do NOT propagate to consumers. We therefore attach the public headers
#     to the target ourselves as SYSTEM INTERFACE include dirs below, so
#     downstream code can `#include <soem/ethercat.h>` (and the umbrella
#     header's siblings) and so SOEM's own `-Werror` C warnings never leak
#     into our stricter C++ targets.
#
#   * On Linux SOEM uses an AF_PACKET raw socket and links only `pthread rt`
#     (see its CMakeLists `OS_LIBS` for the linux branch). **libpcap is NOT
#     required on Linux** (pcap is only used on win32/macOS). So the Dockerfile
#     does not need libpcap-dev.
#
#   * v1.4.0 declares `cmake_minimum_required(VERSION 2.8.4)`, which CMake
#     >= 3.31 / 4.0 rejects. We set CMAKE_POLICY_VERSION_MINIMUM=3.5 for the
#     subdirectory to keep it configuring on modern CMake.
# ----------------------------------------------------------------------------

include(FetchContent)

# Pinned by commit SHA (== tag v1.4.0) for reproducibility. GIT_SHALLOW is left
# FALSE on purpose: a shallow clone cannot reliably fetch an arbitrary commit
# SHA on every server/CMake combination, whereas a full clone of this small
# repo always can.
set(ETHERCAT_SOEM_GIT_TAG "abbf0d42e38d6cfbaa4c1e9e8e07ace651c386fd"
    CACHE STRING "SOEM git commit SHA to build against (== tag v1.4.0)")

FetchContent_Declare(
  soem
  GIT_REPOSITORY https://github.com/OpenEtherCATsociety/SOEM
  GIT_TAG ${ETHERCAT_SOEM_GIT_TAG}
  GIT_SHALLOW FALSE
  SYSTEM
  # v1.4.0 hardcodes `set(BUILD_TESTS TRUE)` and builds its sample executables
  # (slaveinfo/eepromtool/simple_test) under their own `-Werror`, which clang-19
  # rejects (e.g. -Wunused-but-set-variable). We only want the `soem` library,
  # so disable those samples. The sed is idempotent (no TRUE left to match on a
  # second run), so reconfigures are safe.
  PATCH_COMMAND sed -i "s/set(BUILD_TESTS TRUE)/set(BUILD_TESTS FALSE)/" CMakeLists.txt
)

# Allow SOEM's ancient cmake_minimum_required(2.8.4) to configure under modern
# CMake. Scope it to the FetchContent population only.
set(_ethercat_saved_policy_min "${CMAKE_POLICY_VERSION_MINIMUM}")
set(CMAKE_POLICY_VERSION_MINIMUM 3.5)

FetchContent_MakeAvailable(soem)

set(CMAKE_POLICY_VERSION_MINIMUM "${_ethercat_saved_policy_min}")

# v1.4.0 does not export its include dirs on the target. Attach them as SYSTEM
# INTERFACE includes so consumers get the headers and SOEM's C warnings stay
# out of our -Werror builds. Linux layout: soem/, osal/, osal/linux/, oshw/linux/.
if(TARGET soem AND DEFINED soem_SOURCE_DIR)
  target_include_directories(soem SYSTEM INTERFACE
    "${soem_SOURCE_DIR}"
    "${soem_SOURCE_DIR}/soem"
    "${soem_SOURCE_DIR}/osal"
    "${soem_SOURCE_DIR}/osal/linux"
    "${soem_SOURCE_DIR}/oshw/linux"
  )
endif()
