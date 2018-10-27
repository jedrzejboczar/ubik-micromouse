### FREE RTOS ##################################################################
### Wrapper of FreeRTOS (v10)
### see: https://www.freertos.org/Creating-a-new-FreeRTOS-project.html

# Definitions
set(FREERTOS_MEMORY_MENAGEMENT_SCHEME "1")
set(FREERTOS_COMPILER "GCC")
set(FREERTOS_ARCHITECTURE "ARM_CM3")

# Includes and sources
target_include_directories(${CMAKE_PROJECT_NAME}
    PUBLIC .
    PUBLIC Source/include
    PUBLIC Source/portable/${FREERTOS_COMPILER}/${FREERTOS_ARCHITECTURE}
    )
file(GLOB FREERTOS_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/Source/*.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/Source/include/*.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/Source/portable/${FREERTOS_COMPILER}/${FREERTOS_ARCHITECTURE}/*.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/Source/portable/${FREERTOS_COMPILER}/${FREERTOS_ARCHITECTURE}/*.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/Source/portable/MemMang/heap_${FREERTOS_MEMORY_MENAGEMENT_SCHEME}.c"
    )
target_sources(${CMAKE_PROJECT_NAME}
    PUBLIC ${FREERTOS_SOURCES})
