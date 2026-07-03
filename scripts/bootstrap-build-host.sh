#!/usr/bin/env bash
#
# bootstrap-build-host.sh -- provision a LOCAL host (Debian/Ubuntu) to build the EtherCAT servo
# module + tests WITHOUT the Docker container. Closes #65 durably: the grpc/protobuf/pkg-config dev
# packages a local build needs were never persistently installed on the bench host (they are not
# dropped by any autoremove -- there is no unattended-upgrades and they are all manual-marked once
# installed); this script installs the exact set in one idempotent command. Absorbs #11 (env pinning).
#
# The apt set mirrors the Dockerfile's build deps. The Viam C++ SDK is NOT an apt package, so this
# also builds + installs it to /usr/local (the same version + flags the Dockerfile uses) unless it is
# already present -- otherwise every clean build FetchContent-rebuilds ~220MB of SDK from scratch.
#
# Usage (sudo is used automatically for apt if you are not root):
#   scripts/bootstrap-build-host.sh                 # apt build deps + build/install viam-cpp-sdk
#   scripts/bootstrap-build-host.sh --with-clang    # + the clang-19 / ThreadSanitizer lane toolchain
#   scripts/bootstrap-build-host.sh --skip-sdk      # apt deps only (SDK handled elsewhere / already present)
#   scripts/bootstrap-build-host.sh --help
#
# Idempotent: re-running only installs what is missing (apt skips satisfied packages; the SDK build is
# skipped when already installed). After it succeeds, from the repo root:
#   make build ETHERCAT_BUILD_MODULE=ON     # module + tests   (or =OFF for the library + pure tests)
#
set -euo pipefail

# KEEP IN SYNC with CMakeLists.txt (FetchContent GIT_TAG) and Dockerfile.
SDK_VERSION="releases/v0.31.0"

WITH_CLANG=0
SKIP_SDK=0
for arg in "$@"; do
  case "$arg" in
    --with-clang) WITH_CLANG=1 ;;
    --skip-sdk)   SKIP_SDK=1 ;;
    -h|--help)    sed -n '2,30p' "$0"; exit 0 ;;
    *) echo "unknown option: $arg (see --help)" >&2; exit 2 ;;
  esac
done

SUDO=""
if [ "$(id -u)" -ne 0 ]; then SUDO="sudo"; fi

log() { printf '\n=== %s ===\n' "$*"; }

# --- core build deps (the #65 set) --------------------------------------------------------------
# build toolchain + bundling (patchelf, #69) + the viam-cpp-sdk build/link deps (grpc/protobuf/
# abseil/ssl/c-ares/re2) + boost. --no-install-recommends keeps it lean.
CORE_PKGS=(
  build-essential cmake ninja-build pkg-config git ca-certificates curl wget patchelf
  libgrpc++-dev libprotobuf-dev protobuf-compiler-grpc protobuf-compiler
  libabsl-dev libssl-dev libc-ares-dev libre2-dev
  libboost-all-dev
)

# --- clang-19 / TSan lane (optional) ------------------------------------------------------------
# Only needed to build the ThreadSanitizer lane (CLANG-ONLY by policy -- gcc rejects the pdo_cache
# seqlock fences under -fsanitize=thread). The plain gcc build does NOT need these.
CLANG_PKGS=(
  clang-19 clang-format-19 clang-tidy-19 clangd-19 clang-tools-19 libclang-rt-19-dev lldb-19
)

log "apt-get update"
$SUDO apt-get update

log "installing core build deps (${#CORE_PKGS[@]} packages)"
$SUDO apt-get install -y --no-install-recommends "${CORE_PKGS[@]}"

if [ "$WITH_CLANG" -eq 1 ]; then
  log "installing clang-19 / TSan lane toolchain"
  if ! $SUDO apt-get install -y --no-install-recommends "${CLANG_PKGS[@]}"; then
    cat >&2 <<'EOF'

[!] The clang-19 packages were not found in the configured apt sources.
    Add the LLVM apt repository for your distro codename, then re-run with --with-clang:
      https://apt.llvm.org/    (e.g. `deb http://apt.llvm.org/<codename>/ llvm-toolchain-<codename>-19 main`)
    The clang lane is OPTIONAL -- only the ThreadSanitizer build needs it; the gcc build is unaffected.
EOF
  fi
fi

# --- Viam C++ SDK (built from source; not an apt package) ---------------------------------------
SDK_MARKER="/usr/local/lib/cmake/viam-cpp-sdk/viam-cpp-sdkConfig.cmake"
if [ "$SKIP_SDK" -eq 1 ]; then
  log "skipping viam-cpp-sdk (--skip-sdk)"
elif [ -f "$SDK_MARKER" ]; then
  log "viam-cpp-sdk already installed ($SDK_MARKER) -- skipping (idempotent)"
else
  log "building + installing viam-cpp-sdk $SDK_VERSION to /usr/local (one-time; ~minutes)"
  SDK_SRC="$(mktemp -d)"
  trap 'rm -rf "$SDK_SRC"' EXIT
  git clone --depth 1 --branch "$SDK_VERSION" https://github.com/viamrobotics/viam-cpp-sdk "$SDK_SRC/viam-cpp-sdk" 2>/dev/null \
    || git clone https://github.com/viamrobotics/viam-cpp-sdk "$SDK_SRC/viam-cpp-sdk"
  ( cd "$SDK_SRC/viam-cpp-sdk"
    git checkout "$SDK_VERSION"
    # Same flags as the Dockerfile: examples/tests OFF, dynamic + offline proto generation.
    cmake -S . -B build \
      -DCMAKE_BUILD_TYPE=RelWithDebInfo \
      -DVIAMCPPSDK_USE_DYNAMIC_PROTOS=ON \
      -DVIAMCPPSDK_OFFLINE_PROTO_GENERATION=ON \
      -DVIAMCPPSDK_BUILD_EXAMPLES=OFF \
      -DVIAMCPPSDK_BUILD_TESTS=OFF \
      -G Ninja
    cmake --build build --target all -- -j"$(nproc)"
    $SUDO cmake --install build --prefix /usr/local )
  $SUDO ldconfig || true
fi

log "done. Build with:  make build ETHERCAT_BUILD_MODULE=ON   (or =OFF for the library + pure tests)"
