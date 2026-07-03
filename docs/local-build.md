# Local host build (no container)

Build the module + tests directly on a Debian/Ubuntu host — no Docker needed. This is the LOCAL
counterpart to the containerized `make docker` build.

## One-time host provisioning (#65 / #16)

```sh
scripts/bootstrap-build-host.sh              # apt build deps + build/install viam-cpp-sdk to /usr/local
scripts/bootstrap-build-host.sh --with-clang # also install the clang-19 / ThreadSanitizer lane toolchain
scripts/bootstrap-build-host.sh --skip-sdk   # apt deps only (SDK already installed / handled elsewhere)
```

The script is idempotent — re-run it any time; it installs only what's missing. It installs the exact
apt set a local build needs and (once) builds + installs the Viam C++ SDK, which is not an apt package.

**Why this exists:** the SDK build/link deps (grpc/protobuf/abseil/c-ares/re2 `-dev`, pkg-config) were
never persistently installed on the bench host — that was the #65 "host keeps losing grpc/protobuf"
symptom. There is no autoremove/unattended-upgrades dropping them; they were simply absent. Once this
script installs them they are apt-marked *manual* and stay put. The set (mirrors the Dockerfile):

```
build-essential cmake ninja-build pkg-config git ca-certificates curl wget patchelf
libgrpc++-dev libprotobuf-dev protobuf-compiler-grpc protobuf-compiler
libabsl-dev libssl-dev libc-ares-dev libre2-dev libboost-all-dev
```

clang-19 / TSan lane (behind `--with-clang`): `clang-19 clang-format-19 clang-tidy-19 clangd-19
clang-tools-19 libclang-rt-19-dev lldb-19`. The plain gcc build does not need these; only the
CLANG-ONLY ThreadSanitizer lane does. On a host whose apt sources lack clang-19, add the LLVM
repository (https://apt.llvm.org/) for your distro codename first.

## Building

```sh
make build ETHERCAT_BUILD_MODULE=ON    # module (ethercat-servo) + all tests
make build ETHERCAT_BUILD_MODULE=OFF   # library + pure offline tests only (no SDK link)
make test                              # run the ctest suite
make package                           # cpack -> module.tar.gz (bundles + strips deps, see below)
```

Keep the SDK version in `scripts/bootstrap-build-host.sh` in sync with the `FetchContent` `GIT_TAG`
in `CMakeLists.txt` and the `git checkout` in the `Dockerfile` (currently `releases/v0.31.0`).

## Bundle size (#16)

`make package` bundles the module's shared-library closure into `lib/` and, in the same install step,
`strip --strip-unneeded`s each bundled `.so`. The Viam SDK is built RelWithDebInfo, so its two libs
ship ~181 MB of debug info that is dead weight in a deploy bundle; stripping takes the packaged bundle
from ~254 MB to ~73 MB. This is a **packaging** step only — RelWithDebInfo stays for local debugging
(the un-installed `build/` tree keeps full symbols); just the bundled copies are stripped.
