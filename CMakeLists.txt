cmake_minimum_required(VERSION 3.25 FATAL_ERROR)

project(
  ethercat
  DESCRIPTION "SOEM-based EtherCAT master library + generic servo Viam motor module"
  HOMEPAGE_URL https://github.com/viam-modules/ethercat
  LANGUAGES C CXX
)

#
# Options
#

# Build with -Wall / -Werror (and friends) on our own targets.
option(VIAM_USE_WALL_WERROR "Build with -Wall and -Werror flags" ON)

# Build the Viam motor module (servo_motor + main.cpp). This pulls in the
# viam-cpp-sdk. Turn OFF to build only the SOEM-based library, tools, and the
# pure unit tests without the SDK dependency (useful for early phases / CI of
# the library in isolation). Default ON so the full module always builds.
option(ETHERCAT_BUILD_MODULE "Build the Viam motor module (requires viam-cpp-sdk)" ON)

# When ON, additionally build the RT-boundary tests under ThreadSanitizer as
# separate targets, registered with the ctest label "tsan". These must pass with
# ZERO suppressions. The source for these tests is compiled directly (with TSan)
# rather than linked from libethercat, so the seqlock / lock-free code under test
# is itself instrumented.
option(ETHERCAT_SANITIZE_THREAD "Also build RT-boundary tests under ThreadSanitizer (ctest label 'tsan')" OFF)

# Function to apply strict warning flags to one of OUR targets. SOEM and the
# viam-cpp-sdk are pulled in as SYSTEM dependencies, so their warnings never
# reach this.
function(viam_apply_wall_werror target_name)
  if (NOT VIAM_USE_WALL_WERROR)
    return()
  endif()

  set(GNULIKE_COMPILERS "Clang" "AppleClang" "GNU")
  if (CMAKE_CXX_COMPILER_ID IN_LIST GNULIKE_COMPILERS)
    target_compile_options(${target_name} PRIVATE
      -Wall
      -Werror
      -Wextra
      -pedantic
      -Wconversion
      -Wdouble-promotion
      -Wnull-dereference
      -Wunused
    )
  elseif (CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
    target_compile_options(${target_name} PRIVATE /W4 /WX)
  else()
    message(FATAL_ERROR "VIAM_USE_WALL_WERROR is set, but not known how to enable for compiler ID ${CMAKE_CXX_COMPILER_ID}")
  endif()
endfunction()


# If no build type is selected, build optimized but retain debug info. This is
# the right build type for packaging and release.
if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "RelWithDebInfo")
endif()


# Enforce the C++ standard, and disable extensions.
if (NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_CXX_EXTENSIONS OFF)

# Minimum-toolchain floor (#TODO-9): GCC 13. GCC 13 (2023) ships in all current LTS
# distros; the build image is ubuntu:noble (24.04, default g++-13/14). The floor buys
# a complete C++20 stdlib -- libstdc++ 13 has <format> (libstdc++ 12 does NOT) and
# <expected> (libstdc++ 12+), so it unblocks std::format adoption (#TODO-5) and lets
# the ethercat::Expected shim retire to std::expected (#51). clang builds inherit the
# newest installed libstdc++, so the clang lanes get the same stdlib. Fail loud on an
# older GCC so a stale toolchain surfaces here, not as a cryptic missing-<format> error.
if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 13)
  message(FATAL_ERROR
      "GCC ${CMAKE_CXX_COMPILER_VERSION} is below the project floor (GCC 13, #TODO-9). "
      "GCC 13's libstdc++ is required for complete C++20 (<format>/<expected>). "
      "Use g++-13+ (ubuntu:noble ships it) or build with clang-16+ against libstdc++ 13.")
endif()


# Produce a compilation database for clang-tidy / editors.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)


# Produce GNU-style install variables, and default the install prefix to be
# local to the build. Override with -DCMAKE_INSTALL_PREFIX=... if installing
# elsewhere.
if (CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  set(CMAKE_INSTALL_PREFIX "${CMAKE_BINARY_DIR}/install" CACHE PATH "..." FORCE)
endif()
include(GNUInstallDirs)

# Configure runtime paths for relocatable installs.
set(CMAKE_BUILD_WITH_INSTALL_RPATH false)
set(CMAKE_BUILD_RPATH_USE_ORIGIN true)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH true)
set(CMAKE_SKIP_BUILD_RPATH false)

list(FIND CMAKE_PLATFORM_IMPLICIT_LINK_DIRECTORIES "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}" isSystemDir)
if("${isSystemDir}" STREQUAL "-1")
  file(RELATIVE_PATH BINTOPREFIXRELPATH ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_BINDIR} ${CMAKE_INSTALL_PREFIX})
  file(RELATIVE_PATH PREFIXTOLIBRELPATH ${CMAKE_INSTALL_PREFIX} ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR})

  if (CMAKE_SYSTEM_NAME STREQUAL "Darwin")
    list(PREPEND CMAKE_INSTALL_RPATH "@loader_path/${BINTOPREFIXRELPATH}/${PREFIXTOLIBRELPATH}")
  else()
    list(PREPEND CMAKE_INSTALL_RPATH "$ORIGIN/${BINTOPREFIXRELPATH}/${PREFIXTOLIBRELPATH}")
  endif()
endif()


#
# Dependencies
#

include(FetchContent)

# SOEM: all specifics (tag, target name, include wiring) live in this file.
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/soem.cmake)

# Everything needs threads; prefer -pthread if available.
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

# Boost headers (header-only usage for now: asio, etc.).
find_package(Boost CONFIG REQUIRED
  COMPONENTS
    headers
)

# The Viam C++ SDK is only needed by the module. Pull it in exactly the way
# universal-robots does (SYSTEM + PATCH_COMMAND to drop the static protobuf
# gens + find_package fallback), but gated behind ETHERCAT_BUILD_MODULE so the
# library + pure tests can build without it.
if (ETHERCAT_BUILD_MODULE)
  FetchContent_Declare(
    viam-cpp-sdk
    GIT_REPOSITORY https://github.com/viamrobotics/viam-cpp-sdk
    GIT_TAG releases/v0.31.0 # Keep in sync with Dockerfile
    GIT_SHALLOW TRUE
    SYSTEM
    # Without this patch command, using this as a SYSTEM package results in
    # failed compilation due to the statically included protobuf gens getting
    # in the wrong order. Just delete them, we don't need them.
    PATCH_COMMAND find ./src/viam/api -name "*.pb.h" -type f -exec rm {} +
    FIND_PACKAGE_ARGS 0.31.0 CONFIG COMPONENTS viamsdk
  )
  FetchContent_MakeAvailable(viam-cpp-sdk)
endif()

enable_testing()


#
# The EtherCAT master library
#
# SOEM-only, no Viam types. Public headers must not include SOEM headers
# (only the SOEM backend TU will, in a later phase). For now this is a small
# placeholder so the project configures and builds; later phases add the real
# sources (master, pdo_*, cia402, backends, ...).
#

add_library(ethercat)

target_sources(ethercat
  PRIVATE
    src/ethercat/cia402.cpp
    src/ethercat/pdo_cache.cpp
    src/ethercat/backend.cpp
    src/ethercat/pdo_mapping.cpp
    src/ethercat/master.cpp
    src/ethercat/realtime.cpp
    src/ethercat/runner.cpp
    src/ethercat/soem_backend.cpp
    src/viam/lib/motion_profile.cpp
    src/viam/lib/servo_config.cpp
    src/viam/lib/servo_controller.cpp
)

target_include_directories(ethercat
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(ethercat
  PUBLIC
    Threads::Threads
    Boost::headers
  PRIVATE
    # SOEM is an implementation detail of the library; it must not leak into
    # public headers. Linked here so the wiring is in place for later phases.
    soem
)

viam_apply_wall_werror(ethercat)

install(
  TARGETS
    ethercat
)


#
# Tools
#

# slaveinfo: SOEM's own bundled bus-inspection tool, built from the FetchContent
# source (SOEM's CMake samples are disabled, so we add the target ourselves). It
# dumps each slave's identity, SM/FMMU config, DC info, and -- with -sdo -- the
# full CoE object dictionary (e.g. the whole 0x1C32/0x1C33 sync-manager sub-tree:
# sync type, cycle time, supported types). Authoritative drive-side DC/SM config
# for A6 bring-up, straight off the hardware. NOT our code, so NO -Wall -Werror.
#   sudo ./slaveinfo <ifname>          # identity + SM + DC summary
#   sudo ./slaveinfo <ifname> -map     # PDO mapping (per CoE + per SII) + SM sizes
#   sudo ./slaveinfo <ifname> -sdo     # full CoE OD incl 0x1C32/0x1C33 sub-tree
if(DEFINED soem_SOURCE_DIR AND EXISTS "${soem_SOURCE_DIR}/test/linux/slaveinfo/slaveinfo.c")
  add_executable(slaveinfo "${soem_SOURCE_DIR}/test/linux/slaveinfo/slaveinfo.c")
  target_link_libraries(slaveinfo PRIVATE soem)
endif()

# a6_validate: TEMPORARY hardware bring-up validation against a real A6 drive.
# Links the ethercat library (Master + SoemBackend); SOEM is bundled inside it.
add_executable(a6_validate)

target_sources(a6_validate
  PRIVATE
    src/tools/a6_validate.cpp
)

target_link_libraries(a6_validate
  PRIVATE
    ethercat
)

viam_apply_wall_werror(a6_validate)


#
# The Viam motor module (servo)
#

if (ETHERCAT_BUILD_MODULE)
  add_executable(ethercat-servo)

  target_sources(ethercat-servo
    PRIVATE
      src/viam/module/main.cpp
      src/viam/module/servo_motor.cpp
  )

  # Tell boost::dll to use std::filesystem rather than boost::filesystem.
  target_compile_definitions(ethercat-servo
    PRIVATE
      BOOST_DLL_USE_STD_FS
  )

  target_link_libraries(ethercat-servo
    PRIVATE
      ethercat
      viam-cpp-sdk::viamapi # Proxy for gRPC
      viam-cpp-sdk::viamsdk
  )

  viam_apply_wall_werror(ethercat-servo)
  # The Viam SDK's logging header (boost::log -> boost::mpl) trips clang-19's
  # -Wenum-constexpr-conversion. Suppress it ONLY on the SDK-linked module targets,
  # NOT in viam_apply_wall_werror -- so genuine enum-constexpr issues in our library
  # TUs still fail. Known boost-mpl issue (boostorg/mpl#69); UR carries the same flag.
  target_compile_options(ethercat-servo PRIVATE $<$<CXX_COMPILER_ID:Clang,AppleClang>:-Wno-enum-constexpr-conversion>)

  # DEPLOYMENT runtime story (#TODO-9): statically link libstdc++ + libgcc INTO the
  # shipped module binary. The module is built on noble (gcc-13/14 libstdc++ 13) but
  # deployed to robots whose system libstdc++ may be OLDER -- libstdc++ is forward- but
  # NOT backward-compatible, so a binary referencing libstdc++ 13 symbols (the
  # GLIBCXX_3.4.32 that <format> pulls, once #TODO-5 lands) would fail to load on a
  # gcc-11/12 robot with a missing-symbol error. Static-linking makes the module self-
  # contained: it carries its own libstdc++/libgcc and runs regardless of the target's
  # stdlib. ONLY the shipped executable gets this (library/tests run in-image). libc
  # stays dynamic (glibc IS backward-compatible + must match the host). This is the
  # spec's "settle the deployment story" decision; reversible if a thin-runtime target
  # is ever preferred. NOTE: applies to GNU/Clang; both honor these flags.
  target_link_options(ethercat-servo PRIVATE -static-libstdc++ -static-libgcc)

  install(
    TARGETS
      ethercat-servo
    RUNTIME_DEPENDENCY_SET ethercat_servo_deps
  )

  # #37 Finding B(a): SELF-BUNDLE the module's shared-library dependencies (viam-cpp-sdk +
  # gRPC/absl/protobuf/re2/... -- ~250 .so) into the package's lib/ so module.tar.gz is PORTABLE:
  # it loads on any host (whatever viam-server env the user runs) via ethercat-servo's own RUNPATH
  # ($ORIGIN/../lib), with NO Viam-SDK/gRPC install required on the target. This is what a registry
  # deploy needs (and replaces the manual .so copy used to prove the sim RDK smoke).
  # EXCLUDE the base glibc/loader set (those MUST come from the target host's own glibc, which is
  # backward-compatible); libstdc++/libgcc are already static-linked (-static-libstdc++/-libgcc above),
  # so they never appear as deps. Everything else (the SDK/gRPC/absl tree, ssl/crypto/z) is bundled.
  install(
    RUNTIME_DEPENDENCY_SET ethercat_servo_deps
    DESTINATION ${CMAKE_INSTALL_LIBDIR}
    PRE_EXCLUDE_REGEXES
      "^ld-linux.*"      "^libc\\.so.*"       "^libm\\.so.*"        "^libdl\\.so.*"
      "^libpthread\\.so.*" "^librt\\.so.*"     "^libgcc_s\\.so.*"    "^libstdc\\+\\+\\.so.*"
      "^libresolv\\.so.*"  "^libnsl\\.so.*"    "^libanl\\.so.*"      "^libutil\\.so.*"
    POST_EXCLUDE_REGEXES
      "^/lib/ld-linux" "^/lib64/ld-linux"
      "/libc\\.so"     "/libm\\.so"           "/libpthread\\.so"    "/libdl\\.so"
      "/librt\\.so"    "/libgcc_s\\.so"       "/libstdc\\+\\+\\.so"
  )

  # #69 RUNPATH IS NOT TRANSITIVE: the RUNTIME_DEPENDENCY_SET above copies the third-party .so
  # files (viam-cpp-sdk + gRPC/absl/protobuf/re2/...) into lib/ VERBATIM -- each keeps its OWN
  # build/system RUNPATH. So when the loader resolves a BUNDLED lib's own NEEDED entries (e.g. an
  # absl lib -> libabsl_raw_hash_set, libupb -> absl), it uses that lib's RUNPATH, which points at
  # system paths only -- NOT the sibling libs sitting right next to it in lib/. The exe's
  # $ORIGIN/../lib RUNPATH is NOT inherited by the libs it pulls in. On a clean host with no system
  # gRPC/absl this is a loader failure (a prior bundle "worked" only by accident, when the exe
  # directly NEEDed every lib -- a flat SDK-vendored dep tree; it broke once the build image gained
  # system gRPC/absl, which links a transitive-NEEDED universe).
  #
  # Make the bundle DETERMINISTICALLY self-resolving: stamp $ORIGIN into the RUNPATH of every
  # bundled REAL .so at install time, so each finds its siblings in the same lib/ dir regardless of
  # the host. Runs under BOTH `cmake --install` (DESTDIR) AND `cpack` (staging prefix) via
  # $ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}. Symlinks are skipped (patchelf the real files only).
  # patchelf is a build-image dep (see Dockerfile).
  #
  # #16 STRIP the bundled libs in the SAME pass. The viam-cpp-sdk is built RelWithDebInfo, so its two
  # libs (libviamsdk ~110MB, libviamapi ~110MB) ship UNSTRIPPED -- ~181MB of debug info that is dead
  # weight in a DEPLOY bundle (they dominate the ~254MB lib/). `strip --strip-unneeded` keeps the
  # dynamic symbol table (the .so stays loadable) and drops .symtab/.debug -> the bundle falls to
  # ~73MB. This is a PACKAGING step, NOT a build-type change: RelWithDebInfo stays for LOCAL debugging
  # (the un-installed build tree keeps full symbols); only the packaged/bundled copies are stripped.
  # strip is best-effort (a failure leaves a larger-but-working lib, never breaks the package);
  # patchelf stays a hard requirement. Order: strip THEN patchelf (rpath edit is the last write).
  install(CODE [==[
    find_program(_ethercat_patchelf patchelf)
    if(NOT _ethercat_patchelf)
      message(FATAL_ERROR "#69: patchelf is required to stamp $ORIGIN into the bundled deps -- install it in the build image")
    endif()
    find_program(_ethercat_strip strip)
    if(NOT _ethercat_strip)
      message(WARNING "#16: strip not found -- bundling UNSTRIPPED libs (~181MB of SDK debug info retained). Install binutils to shrink the bundle.")
    endif()
    set(_ethercat_root "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
    file(GLOB_RECURSE _ethercat_bundled LIST_DIRECTORIES false "${_ethercat_root}/*.so*")
    set(_ethercat_n 0)
    foreach(_so IN LISTS _ethercat_bundled)
      if(NOT IS_SYMLINK "${_so}")
        if(_ethercat_strip)
          execute_process(
            COMMAND "${_ethercat_strip}" --strip-unneeded "${_so}"
            RESULT_VARIABLE _ethercat_strip_rc)
          if(NOT _ethercat_strip_rc EQUAL 0)
            message(WARNING "#16: strip --strip-unneeded failed on ${_so} (continuing; lib is unstripped but valid)")
          endif()
        endif()
        execute_process(
          COMMAND "${_ethercat_patchelf}" --set-rpath "$ORIGIN" "${_so}"
          RESULT_VARIABLE _ethercat_rc)
        if(NOT _ethercat_rc EQUAL 0)
          message(FATAL_ERROR "#69: patchelf --set-rpath failed on ${_so}")
        endif()
        math(EXPR _ethercat_n "${_ethercat_n}+1")
      endif()
    endforeach()
    message(STATUS "#69/#16: stamped $ORIGIN RUNPATH into (and stripped) ${_ethercat_n} bundled lib file(s) under ${_ethercat_root}")
  ]==])

  # Offline validation of the shipped example configs through the real parser +
  # ServoConfig::validate() (JSON -> protobuf::Struct -> ProtoStruct -> parse).
  add_executable(a6_config_parse_test
    tests/a6_config_parse_test.cpp
    src/viam/module/servo_motor.cpp
  )
  target_link_libraries(a6_config_parse_test
    PRIVATE
      ethercat
      viam-cpp-sdk::viamapi
      viam-cpp-sdk::viamsdk
  )
  target_compile_definitions(a6_config_parse_test PRIVATE
    BOOST_DLL_USE_STD_FS
    "A6_HW_CONFIG_PATH=\"${CMAKE_CURRENT_SOURCE_DIR}/etc/a6-hardware.example.json\""
    "A6_MINIMAL_CONFIG_PATH=\"${CMAKE_CURRENT_SOURCE_DIR}/etc/a6-minimal.example.json\""
  )
  target_include_directories(a6_config_parse_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/tests)
  viam_apply_wall_werror(a6_config_parse_test)
  target_compile_options(a6_config_parse_test PRIVATE $<$<CXX_COMPILER_ID:Clang,AppleClang>:-Wno-enum-constexpr-conversion>)
  add_test(NAME a6_config_parse_test COMMAND a6_config_parse_test)

endif()


#
# Tests
#
# One executable per test file, registered with ctest. Each links the ethercat
# library and gets the header-only test harness on its include path.

function(viam_add_test test_name source)
  add_executable(${test_name} ${source})
  target_link_libraries(${test_name} PRIVATE ethercat)
  target_include_directories(${test_name} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/tests)
  viam_apply_wall_werror(${test_name})
  add_test(NAME ${test_name} COMMAND ${test_name})
endfunction()

viam_add_test(pdo_buffer_test tests/pdo_buffer_test.cpp)
viam_add_test(cia402_test tests/cia402_test.cpp)
viam_add_test(pdo_cache_test tests/pdo_cache_test.cpp)
viam_add_test(soem_backend_test tests/soem_backend_test.cpp)
viam_add_test(conversion_test tests/conversion_test.cpp)
viam_add_test(dc_sync_test tests/dc_sync_test.cpp)
viam_add_test(realtime_test tests/realtime_test.cpp)
viam_add_test(rt_preflight_test tests/rt_preflight_test.cpp)
viam_add_test(sine_move_test tests/sine_move_test.cpp)
viam_add_test(a6_control_test tests/a6_control_test.cpp)
viam_add_test(servo_config_test tests/servo_config_test.cpp)

# ThreadSanitizer build of the RT-boundary test. Compiles the pdo_cache TU
# directly (with -fsanitize=thread) so the seqlock / lock-free handoff code is
# instrumented, rather than linking the un-instrumented libethercat.
# #40 item 6: the TSan lane is CLANG-ONLY by policy -- gcc-14 cannot build it
# (`atomic_thread_fence not supported with -fsanitize=thread` -Werror=tsan, hit in
# pdo_cache.cpp's LOAD-BEARING seqlock fences, which must not be weakened). Gate it
# so nobody burns an hour rediscovering that.
if (ETHERCAT_SANITIZE_THREAD AND NOT CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  message(STATUS "ETHERCAT_SANITIZE_THREAD: skipped -- the TSan lane requires clang "
                 "(gcc rejects atomic_thread_fence under -fsanitize=thread; the pdo_cache "
                 "seqlock fences are load-bearing). Configure with clang to build it.")
endif()
if (ETHERCAT_SANITIZE_THREAD AND CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # #50: the TSan lane MUST be an NDEBUG config. Concurrency tests may be gated
  # `#ifdef NDEBUG` (paths that in a debug build would trip an assert instead of
  # running); under a Debug build such tests compile to a `CHECK(true)` STUB, so
  # the races they exist to catch go UN-SANITIZED while the lane reports green.
  # Fail loud: require a build type whose flags carry -DNDEBUG.
  if (NOT CMAKE_BUILD_TYPE MATCHES "^(Release|RelWithDebInfo|MinSizeRel)$")
    message(FATAL_ERROR
        "ETHERCAT_SANITIZE_THREAD requires an NDEBUG build type (got "
        "CMAKE_BUILD_TYPE='${CMAKE_BUILD_TYPE}'). NDEBUG-gated concurrency tests "
        "(#ifdef NDEBUG) silently stub out otherwise, so the TSan lane would run green "
        "without sanitizing the paths it exists for. Re-configure with "
        "-DCMAKE_BUILD_TYPE=RelWithDebInfo (keeps -g for symbolized races + defines NDEBUG).")
  endif()
  add_executable(pdo_cache_test_tsan
    tests/pdo_cache_test.cpp
    src/ethercat/pdo_cache.cpp
  )
  target_include_directories(pdo_cache_test_tsan
    PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/src
      ${CMAKE_CURRENT_SOURCE_DIR}/tests
  )
  target_link_libraries(pdo_cache_test_tsan
    PRIVATE
      Threads::Threads
      Boost::headers
  )
  target_compile_options(pdo_cache_test_tsan PRIVATE -fsanitize=thread -O1 -g)
  target_link_options(pdo_cache_test_tsan PRIVATE -fsanitize=thread)
  viam_apply_wall_werror(pdo_cache_test_tsan)
  add_test(NAME pdo_cache_test_tsan COMMAND pdo_cache_test_tsan)
  set_tests_properties(pdo_cache_test_tsan PROPERTIES LABELS "tsan")
endif()


#
# Installation and packaging
#

configure_file(
  meta.json.in
  ${CMAKE_CURRENT_BINARY_DIR}/meta.json
  @ONLY
)

install(
  FILES ${CMAKE_CURRENT_BINARY_DIR}/meta.json
  DESTINATION .
)

set(CPACK_PACKAGE_NAME "viam-ethercat")
set(CPACK_PACKAGE_FILE_NAME "module")
set(CPACK_GENERATOR "TGZ")
set(CPACK_INCLUDE_TOPLEVEL_DIRECTORY 0)
include(CPack)
