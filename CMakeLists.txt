# CMake 3.6 changes, amongst others, the way cross-compilation is done
cmake_minimum_required(VERSION 3.6)

# somehow-toolchain related options
set(STM32_CHIP "STM32F103C8")
enable_language(C CXX ASM)

# project definition and version
project(ubik VERSION 0.0.1)

# include some utility functions defined in the project
include(cmake/openocd.cmake)
include(cmake/compilation_flags.cmake)

# CMake options
set(CMAKE_COLOR_MAKEFILE ON)
set(CMAKE_ERROR_DEPRECATED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_VERBOSE_MAKEFILE OFF)

# Cube options
# static lib allows to define special compilation flags only for user sources
set(CUBE_AS_STATIC_LIB ON)
set(CUBE_EXTERN_MAIN   ON)

# definitions needed for all source files
add_compile_definitions(
    STM32F103xB
    USE_HAL_DRIVER
    )

# main target
add_executable(${CMAKE_PROJECT_NAME} "")
# special for the main target
stm32_set_target_properties(${CMAKE_PROJECT_NAME})
stm32_add_hex_bin_targets(${CMAKE_PROJECT_NAME})
stm32_print_size_of_targets(${CMAKE_PROJECT_NAME})
stm32_add_dump_target(${CMAKE_PROJECT_NAME})
# openocd related commands
openocd_configure_files("stlink-v2" "stm32f1x")
openocd_add_flash_target(${CMAKE_PROJECT_NAME})
openocd_add_gdb_target(${CMAKE_PROJECT_NAME})

# process all other directories
add_subdirectory(src)

# if we built cube as library it has to be linked after adding all sources
if(CUBE_AS_STATIC_LIB)
    target_link_libraries(${CMAKE_PROJECT_NAME}
        # we have to prevent linker from messing up with .weak symbols
        # https://stackoverflow.com/questions/30355013/linking-with-static-library-not-equivalent-to-linking-with-its-objects
        PRIVATE "-Wl,--whole-archive" cube "-Wl,--no-whole-archive")
endif()

set_property(TARGET ${CMAKE_PROJECT_NAME} PROPERTY C_STANDARD 11)
set_property(TARGET ${CMAKE_PROJECT_NAME} PROPERTY CXX_STANDARD 14)

# custom function to add project compilation flags for main target
# it is nice when we build cube as static library, because we won't add any warnings there
append_target_flags(${CMAKE_PROJECT_NAME}
    COMMON_FLAGS
    -fdiagnostics-color    # color compilation errors
    -ggdb3                 # gdb3 additional info
    -Wpedantic             # warn when code not in defined language standard
    -Wextra
    -Wcast-function-type   # when casting to incompatible function pointer
    -Wdouble-promotion     # when accidentaly computing in double instead of float
    -Wduplicated-cond
    -Wfloat-equal          # when comparing floats ==
    -Wformat=2             # more checks for printf and scanf
    -Winit-self            # when variable initialized with itself
    -Winline               # warn if a function is not inlined
    -Wlogical-op           # suspicous usage of logical operators
    -Wmissing-include-dirs # when directory does not exists
    -Wnull-dereference
    -Wrestrict             # name aliasing for restirct
    -Wshadow=local         # when shadowing local variable (usefull in nested loops: `int i`)
    -Wundef                # when undefined identifier used in #if
    -Wuninitialized        # always initialize?
    # -Wcast-align          # (violated by STM32HAL)
    # -Wcast-qual           # (STM32HAL: assert_param())
    # -Wduplicated-branches # (violated by FreeRTOS tasks.c)
    # -Wredundant-decls   # when declaring something twice (STM32HAL)
    # -Wwrite-strings       # when converting string constants  (STM32HAL: Error_Handler())

    C_FLAGS
    -Wjump-misses-init     # not initializing in swtich
    -Wold-style-definition
    -Wstrict-prototypes    # when declaring without argument types
    # -Wbad-function-cast   # (violated by STM32HAL)

    CXX_FLAGS
    -fno-exceptions        # disable exceptions - only pumps-up binary
    -fno-rtti              # no runtime-type-info - only pumps-up binary
    # -Wold-style-cast       # C casting in C++ (voilated when using HAL macros)

    LINKER_FLAGS
    # add generation of memory map and print percentage memory usage
    -Wl,-Map=output.map
    -Wl,--print-memory-usage
    # use newlib-nano, which is a minimal implementation of libc that removes unneeded features
    --specs=nosys.specs
    --specs=nano.specs
    # --specs=rdimon.specs  # when we want semihosting use this instrad of nano.specs
    )

