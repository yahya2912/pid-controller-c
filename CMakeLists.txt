cmake_minimum_required(VERSION 3.16)
project(pid_controller C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra")

include_directories(include)

add_executable(pid_sim
    src/main.c
    src/pid.c
)

add_executable(test_pid
    test/test_pid.c
    src/pid.c
)
