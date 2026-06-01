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
    src/ethercat/version.cpp
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

# ec_scan: standalone EtherCAT bus scanner. Links SOEM directly (no SDK).
add_executable(ec_scan)

target_sources(ec_scan
  PRIVATE
    src/tools/ec_scan.cpp
)

target_link_libraries(ec_scan
  PRIVATE
    soem
)

viam_apply_wall_werror(ec_scan)

install(
  TARGETS
    ec_scan
)


#
# The Viam motor module (servo)
#

if (ETHERCAT_BUILD_MODULE)
  add_executable(ethercat-servo)

  target_sources(ethercat-servo
    PRIVATE
      src/viam/module/main.cpp
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

  install(
    TARGETS
      ethercat-servo
  )
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
