cmake_minimum_required(VERSION 3.22)

project(lidar_core_tests C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

enable_testing()

add_executable(lidar_core_test
    core_test.c
    ../../../src/lidar/core.c
)

target_include_directories(lidar_core_test PRIVATE
    stubs
    ../../../src
)

target_compile_options(lidar_core_test PRIVATE
    -Wall
    -Wextra
    -Werror
)

add_test(NAME lidar_core_test COMMAND lidar_core_test)
