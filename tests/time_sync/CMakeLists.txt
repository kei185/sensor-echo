cmake_minimum_required(VERSION 3.22)

project(time_sync_tests C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

enable_testing()

add_executable(time_sync_test
    time_sync_test.c
    ../../src/time_sync.c
    ../../src/tx/frame.c
)

target_include_directories(time_sync_test PRIVATE
    ../../src
)

target_compile_options(time_sync_test PRIVATE
    -Wall
    -Wextra
    -Werror
)

add_test(NAME time_sync_test COMMAND time_sync_test)
