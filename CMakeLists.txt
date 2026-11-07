cmake_minimum_required(VERSION 3.15)
project(hybrid_astar_python)
find_package(Python COMPONENTS Interpreter Development.Module REQUIRED)
find_package(pybind11 CONFIG REQUIRED)

file(GLOB_RECURSE SOURCES "hybrid_astar/*.cpp" "hybrid_astar/*.hpp")

add_executable(test_main  ${SOURCES} test_main.cpp)

