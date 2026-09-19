cmake_minimum_required(VERSION 3.22)

project(arbiter_tests C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

enable_testing()

add_executable(arbiter_test
    arbiter_test.c
    ../../src/arbiter.c
)

target_include_directories(arbiter_test PRIVATE
    stubs
    ../../src
)

target_compile_options(arbiter_test PRIVATE
    -Wall
    -Wextra
    -Werror
)

add_test(NAME arbiter_test COMMAND arbiter_test)
