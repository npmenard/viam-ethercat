FROM ubuntu:jammy

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update

RUN apt-get -y dist-upgrade

# Base toolchain + viam-cpp-sdk build deps (gRPC/protobuf/abseil/ssl) + boost.
# NOTE: SOEM on Linux uses an AF_PACKET raw socket and links only pthread/rt,
# so libpcap is intentionally NOT installed (pcap is win32/macOS-only in SOEM).
RUN apt-get -y --no-install-recommends install \
    build-essential \
    ca-certificates \
    curl \
    g++ \
    gdb \
    git \
    gnupg \
    gpg \
    jq \
    less \
    libabsl-dev \
    libboost-all-dev \
    libgrpc++-dev \
    libprotobuf-dev \
    libssl-dev \
    lsb-release \
    ninja-build \
    pkg-config \
    protobuf-compiler-grpc \
    software-properties-common \
    sudo \
    wget

# Install CMake 3.x from Kitware's official repository
RUN wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | \
    gpg --dearmor - | \
    tee /usr/share/keyrings/kitware-archive-keyring.gpg >/dev/null && \
    echo "deb [signed-by=/usr/share/keyrings/kitware-archive-keyring.gpg] https://apt.kitware.com/ubuntu/ $(lsb_release -cs) main" | \
    tee /etc/apt/sources.list.d/kitware.list >/dev/null && \
    apt-get update && \
    apt-get install -y --no-install-recommends cmake=3.30.* cmake-data=3.30.*


# C++23 toolchain (#38): the library's Cia402Fsm API uses std::expected, which
# needs libstdc++ >= 12 -- jammy's default g++-11 has no <expected> header at
# all (empirically probed). g++-12 is in-distro on jammy; make it the default
# c++/g++ so `make build` (CI) and the SDK build use it, and clang-19 below
# picks up libstdc++-12's headers automatically (it selects the newest GCC dir).
RUN apt-get -y --no-install-recommends install g++-12 && \
    update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-12 100 && \
    update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-12 100 && \
    update-alternatives --install /usr/bin/cc cc /usr/bin/gcc-12 100 && \
    update-alternatives --install /usr/bin/c++ c++ /usr/bin/g++-12 100

RUN bash -c 'wget -O - https://apt.llvm.org/llvm-snapshot.gpg.key|apt-key add -'
RUN apt-add-repository -y 'deb http://apt.llvm.org/jammy/ llvm-toolchain-jammy-19 main'
RUN apt-get update
RUN apt-get -y --no-install-recommends install -t llvm-toolchain-jammy-19 \
    clang-19 \
    clang-format-19 \
    clang-tidy-19 \
    clangd-19 \
    clang-tools-19 \
    libclang-rt-19-dev \
    lldb-19

# ThreadSanitizer runtime layout shim: some clang-19 packagings ship the
# sanitizer runtimes under .../lib/clang/19/lib/linux/ (legacy, arch-suffixed)
# while the driver looks in .../lib/clang/19/lib/<triple>/. Bridge them with
# suffix-stripped symlinks if the per-triple dir is missing, so
# -fsanitize=thread links. No-op when the layout already matches.
RUN set -e; \
    RTDIR="$(dirname "$(clang-19 -print-runtime-dir)")"; \
    TRIPLE="$(basename "$(clang-19 -print-runtime-dir)")"; \
    if [ ! -e "$RTDIR/$TRIPLE/libclang_rt.tsan.a" ] && [ -d "$RTDIR/linux" ]; then \
      mkdir -p "$RTDIR/$TRIPLE"; \
      for f in "$RTDIR"/linux/*-x86_64.a "$RTDIR"/linux/*-x86_64.so "$RTDIR"/linux/*-aarch64.a "$RTDIR"/linux/*-aarch64.so; do \
        [ -e "$f" ] || continue; \
        b="$(basename "$f")"; n="$(echo "$b" | sed -E 's/-(x86_64|aarch64)//')"; \
        ln -sf "../linux/$b" "$RTDIR/$TRIPLE/$n"; \
      done; \
    fi

RUN mkdir -p /root/opt/src

# Install Viam C++ SDK from source. If you change the version here, change it in
# the top level CMakeLists.txt (and cmake/soem.cmake's notes) as well.
RUN cd /root/opt/src && \
    git clone https://github.com/viamrobotics/viam-cpp-sdk && \
    cd viam-cpp-sdk && \
    git checkout releases/v0.31.0 && \
    cmake -S . -B build \
        -DCMAKE_BUILD_TYPE=RelWithDebInfo \
        -DVIAMCPPSDK_USE_DYNAMIC_PROTOS=ON \
        -DVIAMCPPSDK_OFFLINE_PROTO_GENERATION=ON \
        -DVIAMCPPSDK_BUILD_EXAMPLES=OFF \
        -DVIAMCPPSDK_BUILD_TESTS=OFF \
        -G Ninja && \
    cmake --build build --target all -- -j4 && \
    cmake --install build --prefix /usr/local && \
    rm -rf /root/opt/src/viam-cpp-sdk
