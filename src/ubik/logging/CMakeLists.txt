target_sources(${CMAKE_PROJECT_NAME}
    PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/logging.h"
    PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/uart_logging_config.h"
    PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/uart_logging.cpp"
    PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/buffer.h"
    PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/stats.h"
    PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/stats.cpp"
    PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/print_bits.h"
    )
