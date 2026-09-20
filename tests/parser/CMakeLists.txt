cmake_minimum_required(VERSION 3.22)

project(lidar_parser_tests C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

enable_testing()

function(add_parser_test parser_name)
    set(test_target "${parser_name}_test")

    add_executable(${test_target}
        ${test_target}.c
        parser_test_input.c
        ../../src/lidar/parser/${parser_name}.c
    )

    target_include_directories(${test_target} PRIVATE
        ../../src
    )

    target_compile_options(${test_target} PRIVATE
        -Wall
        -Wextra
        -Werror
    )

    if(CMAKE_C_COMPILER_ID MATCHES "Clang|GNU")
        target_compile_options(${test_target} PRIVATE
            -fsanitize=undefined
            -fno-sanitize-recover=undefined
        )
        target_link_options(${test_target} PRIVATE
            -fsanitize=undefined
            -fno-sanitize-recover=undefined
        )
    endif()

    add_test(NAME ${test_target} COMMAND ${test_target})
endfunction()

add_parser_test(meta)
add_parser_test(health)
add_parser_test(device_info)
add_parser_test(scan)
