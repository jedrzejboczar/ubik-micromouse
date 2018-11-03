add_subdirectory(logging)

target_sources(${CMAKE_PROJECT_NAME}
    PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/ubik.cpp"
    PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/error_hooks.cpp"
    # PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/interrupts.cpp"
    # PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/supervision.h"
    # PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/supervision.cpp"
    )
