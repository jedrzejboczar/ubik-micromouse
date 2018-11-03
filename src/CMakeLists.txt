message("Applying fix [main.c]: disabe SysTick to prevent early FreeRTOS context switch.")
execute_process(COMMAND
    sh "${CMAKE_CURRENT_SOURCE_DIR}/fix_systick.sh"
    )

target_include_directories(${CMAKE_PROJECT_NAME}
    PUBLIC .)

add_subdirectory(cube)
add_subdirectory(freeRTOS)
add_subdirectory(freeRTOS_cpp)
add_subdirectory(ubik)

