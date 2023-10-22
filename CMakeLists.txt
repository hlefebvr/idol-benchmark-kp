cmake_minimum_required(VERSION 3.26)
project(idol_benchmark_kp)

set(CMAKE_CXX_STANDARD 17)

find_package(idol REQUIRED)

add_executable(solve_kp main.cpp)

target_link_libraries(solve_kp PUBLIC idol)
