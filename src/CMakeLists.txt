target_include_directories(${CMAKE_PROJECT_NAME}
    PUBLIC .)

add_subdirectory(cube)
add_subdirectory(freeRTOS)
add_subdirectory(freeRTOS_cpp)
add_subdirectory(ubik)
