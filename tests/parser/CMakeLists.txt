cmake_minimum_required(VERSION 3.22)

project(lidar_parser_tests C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

enable_testing()

add_executable(lidar_parser_tests
    parser_test.c
    parser_test_input.c
    ../../src/lidar/parser/meta.c
    ../../src/lidar/parser/health.c
    ../../src/lidar/parser/device_info.c
    ../../src/lidar/parser/scan.c
)

target_include_directories(lidar_parser_tests PRIVATE
    ../../src
)

target_compile_options(lidar_parser_tests PRIVATE
    -Wall
    -Wextra
    -Werror
)

add_test(NAME lidar_parser_tests COMMAND lidar_parser_tests)
