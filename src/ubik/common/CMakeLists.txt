target_sources(${CMAKE_PROJECT_NAME}
    PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/macros.h"
    PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/timing.h"
    PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/robot_parameters.h"

    PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/hard_fault.c"
    PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/interrupts.cpp"
    PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/error_hooks.cpp"
    PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/dynamic_memory.cpp"

    PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/as5045.h"
    PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/as5045.cpp"
    PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/spi_devices.h"
    PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/spi_devices.cpp"
    PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/distance_sensors.h"
    PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/distance_sensors.cpp"
    )
