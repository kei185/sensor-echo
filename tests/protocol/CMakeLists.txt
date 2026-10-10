cmake_minimum_required(VERSION 3.22)

project(protocol_tests C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

enable_testing()

add_executable(protocol_test
    protocol_test.c
    ../../src/protocol.c
    ../../src/time_sync.c
    ../../src/tx/frame.c
    ../../src/tx/header.c
    ../../lib/libcrc-2.0/src/crc8.c
)

target_include_directories(protocol_test PRIVATE
    stubs
    ../../src
    ../../lib/libcrc-2.0/include
)

target_compile_options(protocol_test PRIVATE
    -Wall
    -Wextra
    -Werror
)

add_test(NAME protocol_test COMMAND protocol_test)
