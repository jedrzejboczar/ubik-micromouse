add_subdirectory(logging)
add_subdirectory(movement)

target_sources(${CMAKE_PROJECT_NAME}
    PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/ubik.cpp"
    PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/macros.h"
    PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/error_hooks.cpp"
    PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/dynamic_memory.cpp"
    PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/timing.h"
    PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/macros.h"
    PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/hard_fault.c"
    PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/interrupts.cpp"
    PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/system_monitor.h"
    PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/system_monitor.cpp"
    # PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/supervision.h"
    # PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/supervision.cpp"

    PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/distance_sensors.h"
    PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/distance_sensors.cpp"
    )
