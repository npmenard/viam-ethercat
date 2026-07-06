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
#
# DELIBERATELY NOT a CACHE variable (#35 root cause): a `CACHE STRING` never
# self-updates, so a build dir configured under an OLD pin keeps fetching the
# old SOEM forever -- that stale-cache path is exactly how a build dir ended up
# on v1.4.0 (abbf0d4) after the v2 migration. The in-file pin is authoritative
# on every configure. To build against a DIFFERENT SOEM tree, use the explicit
# dev override -DFETCHCONTENT_SOURCE_DIR_SOEM=<path> (the pin check below
# permits it with a loud warning).
set(ETHERCAT_SOEM_GIT_TAG "b410bf6ef599d5c85302ea45cae5f55f8e9aa394")
# Scrub the stale cache entry from dirs configured under the old CACHE form, so
# it cannot shadow anything that still reads the cache.
unset(ETHERCAT_SOEM_GIT_TAG CACHE)

# EXCLUDE_FROM_ALL keeps SOEM's own install() rules out of OUR install tree --
# without it, module.tar.gz grows libsoem.a, the soem/ headers, its cmake
# configs, AND SOEM's README.md/LICENSE.md at the package root (shadowing
# ours). The soem target still builds on demand as a dependency of libethercat.
# (Requires CMake >= 3.28, which SOEM's own cmake_minimum_required demands anyway.)
FetchContent_Declare(
  soem
  GIT_REPOSITORY https://github.com/OpenEtherCATsociety/SOEM
  GIT_TAG ${ETHERCAT_SOEM_GIT_TAG}
  GIT_SHALLOW FALSE
  SYSTEM
  EXCLUDE_FROM_ALL
)

FetchContent_MakeAvailable(soem)

# --- #35: pin assertion -- fail LOUD at configure on a stale checkout -----------
# The incident this guards: a fresh build dir once resolved the FetchContent step
# to a STALE SOEM v1.4.0 tree (abbf0d4) instead of the pin above. The v1 layout has
# no include/soem/soem.h, so the failure surfaced as a cryptic `'soem/soem.h' file
# not found` deep in the build -- and on a bench machine that wastes a hardware
# session. Diagnose it HERE, with the fix in the message.
if(NOT DEFINED soem_SOURCE_DIR OR NOT EXISTS "${soem_SOURCE_DIR}")
  message(FATAL_ERROR
      "SOEM pin check: soem_SOURCE_DIR is unset/missing after FetchContent_MakeAvailable "
      "-- the fetch failed; check network access / the GIT_REPOSITORY URL.")
endif()

# Tier 1 (layout; always runs): the v2 umbrella header must exist. Any v1-era tree
# fails this regardless of how it got there.
if(NOT EXISTS "${soem_SOURCE_DIR}/include/soem/soem.h")
  message(FATAL_ERROR
      "SOEM pin check FAILED: '${soem_SOURCE_DIR}' is NOT a SOEM v2 tree "
      "(missing include/soem/soem.h; a v1.4.0-era checkout has the old header layout). "
      "This is the stale-FetchContent failure mode. Fix: delete this build dir's "
      "_deps (soem-src/soem-build/soem-subbuild) and re-configure, or set "
      "-DFETCHCONTENT_SOURCE_DIR_SOEM=<path to a checkout of ${ETHERCAT_SOEM_GIT_TAG}>.")
endif()

# Tier 2 (exact SHA; when the tree is a git checkout): HEAD must equal the pin.
# An explicit FETCHCONTENT_SOURCE_DIR_SOEM dev override at a different commit is
# permitted -- but loudly, because bench results are only comparable at the pin.
find_package(Git QUIET)
if(GIT_FOUND AND EXISTS "${soem_SOURCE_DIR}/.git")
  execute_process(
    COMMAND "${GIT_EXECUTABLE}" -C "${soem_SOURCE_DIR}" rev-parse HEAD
    OUTPUT_VARIABLE _soem_head
    OUTPUT_STRIP_TRAILING_WHITESPACE
    RESULT_VARIABLE _soem_git_rc
    ERROR_QUIET)
  if(_soem_git_rc EQUAL 0 AND NOT _soem_head STREQUAL "${ETHERCAT_SOEM_GIT_TAG}")
    if(DEFINED FETCHCONTENT_SOURCE_DIR_SOEM AND FETCHCONTENT_SOURCE_DIR_SOEM)
      message(WARNING
          "SOEM pin check: the FETCHCONTENT_SOURCE_DIR_SOEM override tree is at "
          "${_soem_head}, not the pinned ${ETHERCAT_SOEM_GIT_TAG} (v2.0.0+10, ec_sample "
          "parity). Building anyway (explicit override), but bench behaviour is only "
          "comparable against the pin.")
    else()
      message(FATAL_ERROR
          "SOEM pin check FAILED: the FetchContent checkout at '${soem_SOURCE_DIR}' is at "
          "${_soem_head}, expected ${ETHERCAT_SOEM_GIT_TAG}. Stale _deps state. Fix: delete "
          "this build dir's _deps (soem-src/soem-build/soem-subbuild) and re-configure.")
    endif()
  endif()
endif()
message(STATUS "SOEM pin check OK: v2 layout at ${soem_SOURCE_DIR}")

if(TARGET soem)
  # `libethercat` ends up as a SHARED library when a transitive dep
  # (viam-cpp-sdk) flips BUILD_SHARED_LIBS=ON. Linking non-PIC static objects
  # into a .so fails with R_X86_64_PC32 relocation errors, so force PIC.
  set_target_properties(soem PROPERTIES POSITION_INDEPENDENT_CODE ON)
endif()
