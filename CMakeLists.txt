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

# GTK GUI target
find_package(PkgConfig REQUIRED)
pkg_check_modules(GTK3 REQUIRED gtk+-3.0)

add_executable(pid_gui
    src/gui.c
    src/pid.c
)

target_include_directories(pid_gui PRIVATE ${GTK3_INCLUDE_DIRS} include)
target_link_libraries(pid_gui PRIVATE ${GTK3_LIBRARIES} m)
target_compile_options(pid_gui PRIVATE ${GTK3_CFLAGS_OTHER})
