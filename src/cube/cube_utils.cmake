# I hate CMake language...

# Finds the directory where the .ioc file is located.
function(find_cube_root CUBE_PROJECT_NAME CUBE_PROJECT_ROOT)

    # first find the .ioc file (there should be only 1!)
    file(GLOB_RECURSE CUBE_IOC "${CMAKE_CURRENT_SOURCE_DIR}/*.ioc")
    list(LENGTH CUBE_IOC CUBE_IOC_FOUND)
    if(NOT CUBE_IOC_FOUND EQUAL 1)
        message(FATAL_ERROR "Could not find *.ioc file, found: ${CUBE_IOC}")
    endif()

    # now get the components accordingly
    get_filename_component(CUBE_ROOT "${CUBE_IOC}" DIRECTORY)
    get_filename_component(CUBE_NAME "${CUBE_IOC}" NAME_WE)

    # set the variables
    set(${CUBE_PROJECT_NAME} "${CUBE_NAME}" PARENT_SCOPE)
    set(${CUBE_PROJECT_ROOT} "${CUBE_ROOT}" PARENT_SCOPE)

endfunction()


# Finds the startup_*.s file. Most toolchains provide startup file,
# for GPDSC this will inspect the description to find startup for GCC.
function(find_startup_file STARTUP_FILE CUBE_PROJECT_NAME CUBE_PROJECT_ROOT)

    # try to find it in the project
    file(GLOB_RECURSE STARTUP_F "${CUBE_PROJECT_ROOT}/startup_*.s")
    list(LENGTH STARTUP_F N_FILES)
    if(N_FILES GREATER 1)
        message(FATAL_ERROR "Found more than one startup file: ${STARTUP_F}")
    endif()

    # if not files were found, than inspect GPDSC
    if(NOT N_FILES EQUAL 1)

        # assume the .gpdsc file path
        set(GPDSC_FILE "${CUBE_PROJECT_ROOT}/${CUBE_PROJECT_NAME}.gpdsc")
        if(NOT EXISTS "${GPDSC_FILE}")
            message(FATAL_ERROR "Could not find .gpdsc file, tested path: ${GPDSC_FILE}")
        endif()

        # read the gpdsc
        file(READ "${GPDSC_FILE}" GPDSC)

        # try to match filename
        # string(REGEX MATCH
        #
        #     MATCHED_STR GPDSC)
        # <file category="sourceAsm" condition="GCC Toolchain" name="Drivers\CMSIS\Device\ST\STM32F4xx\Source\Templates\gcc\startup_stm32f407xx.s"/>
        if(GPDSC MATCHES ".*<file category=\"sourceAsm\" condition=\"GCC Toolchain\" name=\"([^\"]*)\"/>.*")
            set(STARTUP_F "${CMAKE_MATCH_1}")
            # replace Windows path separators to make it CMake compatible
            string(REPLACE "\\" "/" STARTUP_F "${STARTUP_F}")
            # add project root
            set(STARTUP_F "${CUBE_PROJECT_ROOT}/${STARTUP_F}")
        endif()

    endif()

    if(NOT EXISTS "${STARTUP_F}")
        message(FATAL_ERROR "Could not find startup_*.s file, found: ${STARTUP_F}")
    endif()

    # success, set the variable
    set(${STARTUP_FILE} "${STARTUP_F}" PARENT_SCOPE)

endfunction()

