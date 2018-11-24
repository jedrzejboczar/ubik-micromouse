add_library(mpaland_printf STATIC "")

target_include_directories(mpaland_printf SYSTEM
    PUBLIC printf/
    )

target_sources(mpaland_printf
    PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/printf/printf.c")

