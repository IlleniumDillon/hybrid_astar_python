cmake_minimum_required(VERSION 3.15)
project(hybrid_astar_python LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Python COMPONENTS Interpreter Development.Module REQUIRED)
find_package(pybind11 CONFIG REQUIRED)

add_library(hybrid_astar_core STATIC
    hybrid_astar/src/hybrid_astar.cpp
    hybrid_astar/src/motion_primitives.cpp
    hybrid_astar/src/occupancy_map.cpp
)
target_include_directories(hybrid_astar_core PUBLIC hybrid_astar/include)
set_target_properties(hybrid_astar_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

pybind11_add_module(hybrid_astar hybrid_astar/src/bindings.cpp)
target_link_libraries(hybrid_astar PRIVATE hybrid_astar_core)

add_executable(test_main test_main.cpp)
target_link_libraries(test_main PRIVATE hybrid_astar_core)

enable_testing()
add_test(NAME hybrid_astar_planner COMMAND test_main)
add_test(NAME hybrid_astar_python
    COMMAND ${Python_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/hybrid_astar/test_python.py)
set_tests_properties(hybrid_astar_python PROPERTIES
    ENVIRONMENT "PYTHONPATH=$<TARGET_FILE_DIR:hybrid_astar>")

