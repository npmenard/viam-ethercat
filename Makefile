.PHONY: \
	default \
	format \
	format-check \
	configure \
	build \
	install \
	test \
	package \
	run-clang-tidy \
	run-clang-tidy-parallel \
	run-clang-check \
	clean \
	clean-all \
	docker-amd64 \
	docker-build \
	docker-upload \
	docker \
	docker-amd64-ci

# Build the Viam module (and pull in viam-cpp-sdk) by default. Set to OFF to
# build only the library, tools, and pure tests without the SDK:
#   make build ETHERCAT_BUILD_MODULE=OFF
ETHERCAT_BUILD_MODULE ?= ON

# Our source lives under these trees. clang-format / clang-tidy operate on them.
SOURCE_FILES := $(shell find src/ethercat src/viam src/tools -type f \( -name '*.cpp' -o -name '*.hpp' \) 2>/dev/null)
CPP_FILES := $(filter %.cpp,$(SOURCE_FILES))

default: package

# Format all of our source in place.
format:
	clang-format-19 -i --style=file $(SOURCE_FILES)

format-check:
	clang-format-19 --style=file --dry-run --Werror $(SOURCE_FILES)

configure:
	cmake -S . -B build \
		-DCMAKE_BUILD_TYPE=RelWithDebInfo \
		-DETHERCAT_BUILD_MODULE=$(ETHERCAT_BUILD_MODULE) \
		-G Ninja

build: configure
	cmake --build build --target all -- -j4

install: build
	DESTDIR=build/install cmake --install build --prefix /

# Test executables are EXCLUDE_FROM_ALL (so the Conan package build skips
# them); build them explicitly before running ctest.
test: build
	cmake --build build --target build-tests -- -j4
	ctest --test-dir build --output-on-failure

package: format-check install
	cmake --build build --target package

run-clang-tidy:
	clang-tidy-19 \
		-p build \
		--config-file ./.clang-tidy \
		--header-filter=".*/src/(ethercat|viam|tools)/.*" \
		$(CPP_FILES)

run-clang-tidy-parallel:
	run-clang-tidy-19 \
		-p build \
		-config-file ./.clang-tidy \
		-header-filter=".*/src/(ethercat|viam|tools)/.*" \
		$(CPP_FILES)

run-clang-check:
	clang-check-19 -p build $(CPP_FILES)

clean:
	rm -rf build

clean-all:
	git clean -fxd

# Docker (linux/amd64 only for now)
BUILD_CMD = docker buildx build --pull $(BUILD_PUSH) --force-rm --build-arg MAIN_TAG=$(MAIN_TAG) \
	--build-arg BASE_TAG=$(BUILD_TAG) --platform linux/$(BUILD_TAG) -f $(BUILD_FILE) -t '$(MAIN_TAG):$(BUILD_TAG)' .
BUILD_PUSH = --load
BUILD_FILE = Dockerfile

docker-amd64: MAIN_TAG = ghcr.io/viam-modules/viam-ethercat
docker-amd64: BUILD_TAG = amd64
docker-amd64:
	$(BUILD_CMD)

docker-build: docker-amd64

docker-upload:
	docker push 'ghcr.io/viam-modules/viam-ethercat:amd64'

docker: docker-build docker-upload

# CI target that automatically pushes; avoid for local test-first-then-push flows.
docker-amd64-ci: MAIN_TAG = ghcr.io/viam-modules/viam-ethercat
docker-amd64-ci: BUILD_TAG = amd64
docker-amd64-ci: BUILD_PUSH = --push
docker-amd64-ci:
	$(BUILD_CMD)
