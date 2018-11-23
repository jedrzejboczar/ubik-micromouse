message("Applying changes to Cube generated main.c")
execute_process(COMMAND
    sh "${CMAKE_CURRENT_SOURCE_DIR}/apply_cube_main_changes.sh"
    )

target_include_directories(${CMAKE_PROJECT_NAME}
    PUBLIC .)

add_subdirectory(cube)
add_subdirectory(freeRTOS)
add_subdirectory(freeRTOS_cpp)
add_subdirectory(mpaland_printf)
add_subdirectory(ubik)

