add_subdirectory(common)
add_subdirectory(logging)
add_subdirectory(localization)
add_subdirectory(movement)

target_sources(${CMAKE_PROJECT_NAME}
    PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/ubik.cpp"
    PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/system_monitor.h"
    PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/system_monitor.cpp"
    )
