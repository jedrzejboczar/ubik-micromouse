###############################################################################
# Basic usage:
# make cmake
# make build

CMAKE_GENERATOR := Unix Makefiles
CMAKE_BUILD_TYPE := Debug
NJOBS := 3

###############################################################################

ROOT_DIR := $(dir $(abspath $(lastword $(MAKEFILE_LIST))))
BUILD_DIR := $(ROOT_DIR)/build
STM32_MODULE_PATH := $(ROOT_DIR)/cmake/stm32-cmake/cmake
CMAKE_TOOLCHAIN_FILE := $(STM32_MODULE_PATH)/gcc_stm32.cmake

CMAKE_FLAGS := \
			  -DCMAKE_MODULE_PATH="$(STM32_MODULE_PATH)" \
			  -DCMAKE_TOOLCHAIN_FILE="$(CMAKE_TOOLCHAIN_FILE)" \
			  -DCMAKE_BUILD_TYPE="$(CMAKE_BUILD_TYPE)" \

###############################################################################

# all targets are just commands (no specific files produced)
.PHONY: build cmake clean cleanall flash gdb

build: $(BUILD_DIR)/CMakeCache.txt
	cmake --build $(BUILD_DIR) -- -j $(NJOBS)

cmake: $(BUILD_DIR)/CMakeCache.txt

clean:
	cmake --build $(BUILD_DIR) --target clean

cleanall:
	rm -rf $(BUILD_DIR)/*

flash: $(BUILD_DIR)/CMakeCache.txt
	cmake --build $(BUILD_DIR) -- flash

gdb: $(BUILD_DIR)/CMakeCache.txt
	cmake --build $(BUILD_DIR) -- gdb


# this is to make given command auto-call 'cmake' when there is not build directory
$(BUILD_DIR)/CMakeCache.txt:
	cmake -E make_directory $(BUILD_DIR)
	cd $(BUILD_DIR) && cmake $(CMAKE_FLAGS) -G "$(CMAKE_GENERATOR)" ..

