cmake_minimum_required(VERSION 3.15)
project(hybrid_astar_python)
find_package(Python COMPONENTS Interpreter Development.Module REQUIRED)
find_package(pybind11 CONFIG REQUIRED)