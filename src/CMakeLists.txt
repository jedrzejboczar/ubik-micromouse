target_include_directories(${CMAKE_PROJECT_NAME}
    PUBLIC .)

add_subdirectory(cube)
add_subdirectory(freeRTOS)
add_subdirectory(freeRTOScpp)
add_subdirectory(ubik)
