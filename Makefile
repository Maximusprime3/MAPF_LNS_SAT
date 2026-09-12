# Supported Make entry point. CMake owns the explicit source/target graph.
.DEFAULT_GOAL := all
SOURCE_DIR := $(abspath $(dir $(lastword $(MAKEFILE_LIST))))
BUILD_DIR ?= $(SOURCE_DIR)/build-make
CMAKE ?= cmake
CTEST ?= ctest
CMAKE_FLAGS ?= -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS_RELEASE=-O2
BUILD_FLAGS ?=

.PHONY: all configure test run_experiments lns_verification_runner asan clean FORCE
configure:
	$(CMAKE) -S "$(SOURCE_DIR)" -B "$(BUILD_DIR)" $(CMAKE_FLAGS)

all: configure
	$(CMAKE) --build "$(BUILD_DIR)" $(BUILD_FLAGS)

test: all
	$(CTEST) --test-dir "$(BUILD_DIR)" --output-on-failure

run_experiments: configure
	$(CMAKE) --build "$(BUILD_DIR)" --target run_batch_experiments $(BUILD_FLAGS)

lns_verification_runner: configure
	$(CMAKE) --build "$(BUILD_DIR)" --target run_lns_verification $(BUILD_FLAGS)

# Preserve all documented individual test targets (including shell checks).
test_%: all FORCE
	$(CTEST) --test-dir "$(BUILD_DIR)" --output-on-failure -R '^$@$$'

asan:
	$(MAKE) all BUILD_DIR="$(BUILD_DIR)-asan" CMAKE_FLAGS='-DCMAKE_BUILD_TYPE=Debug -DCMAKE_CXX_FLAGS="-fsanitize=address -fno-omit-frame-pointer" -DCMAKE_EXE_LINKER_FLAGS="-fsanitize=address"'

clean:
	$(CMAKE) -E rm -rf "$(BUILD_DIR)"

FORCE:
