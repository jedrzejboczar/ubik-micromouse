# Utilities for adding target compilation flags in a convenient way

function(append_target_flags FLAGS_TARGET)

    # parse arguments
    set(MULTI_VALUE_ARGS
        COMMON_FLAGS C_FLAGS CXX_FLAGS LINKER_FLAGS)
    # only multi-value args, parses all arguments after the expected ones
    cmake_parse_arguments(ARG "" "" "${MULTI_VALUE_ARGS}" ${ARGN})

    # append linker flags
    # link flags have to be specified as string
    string(REPLACE ";" " " ARG_LINKER_FLAGS "${ARG_LINKER_FLAGS}")
    set_property(TARGET "${FLAGS_TARGET}"
        APPEND_STRING
        PROPERTY LINK_FLAGS
        " ${ARG_LINKER_FLAGS}")

    # append compile options with respect to compiler language using CMake generator expressions
    # compile options have to be specified as CMake list
    set_property(TARGET "${FLAGS_TARGET}"
        APPEND PROPERTY COMPILE_OPTIONS
        ${ARG_COMMON_FLAGS}
        $<$<COMPILE_LANGUAGE:C>:${ARG_C_FLAGS}>
        $<$<COMPILE_LANGUAGE:CXX>:${ARG_CXX_FLAGS}>)

endfunction()
