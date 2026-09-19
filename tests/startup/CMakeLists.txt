cmake_minimum_required(VERSION 3.22)

project(startup_tests C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

enable_testing()

add_executable(startup_test
    startup_test.c
    ../../src/startup.c
    ../../src/lidar/sys.c
    ../../src/tx/frame.c
    ../../src/tx/header.c
    ../../lib/libcrc-2.0/src/crc8.c
)

target_include_directories(startup_test PRIVATE
    stubs
    ../../src
    ../../lib/libcrc-2.0/include
)

target_compile_options(startup_test PRIVATE
    -Wall
    -Wextra
    -Werror
)

add_test(NAME startup_test COMMAND startup_test)
