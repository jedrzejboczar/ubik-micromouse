target_sources(${CMAKE_PROJECT_NAME}
    PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/motor_control.h"
    PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/motor_control.cpp"
    PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/pid.h"
    PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/regulator.h"
    PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/regulator.cpp"
    PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/controller.h"
    PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/controller.cpp"
    PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/controller_interface.cpp"
    PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/correction.h"
    PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/correction.cpp"
    )
