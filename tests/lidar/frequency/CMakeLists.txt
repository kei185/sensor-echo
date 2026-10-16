cmake_minimum_required(VERSION 3.22)

project(lidar_frequency_tests C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

enable_testing()

add_executable(lidar_frequency_test
    frequency_test.c
    ../../../src/lidar/frequency.c
    ../../../src/lidar/core.c
    ../../../src/lidar/sys.c
)

target_include_directories(lidar_frequency_test PRIVATE
    ../core/stubs
    ../../startup/stubs
    ../../../src
)

target_compile_options(lidar_frequency_test PRIVATE
    -Wall
    -Wextra
    -Werror
)

add_test(NAME lidar_frequency_test COMMAND lidar_frequency_test)
