cmake_minimum_required(VERSION 3.15)
project(EVRP)

set(CMAKE_CXX_STANDARD 17)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

set(dependencies src/EVRP.cpp src/EVRP.hpp src/heuristics.cpp src/heuristics.hpp src/stats.cpp src/stats.hpp src/utils.cpp src/utils.hpp src/constructions.cpp src/constructions.hpp src/LS_operators.cpp src/LS_operators.hpp src/floydWarshall.cpp src/floydWarshall.hpp src/perturbations.cpp src/perturbations.hpp)

add_executable(EVRP src/main.cpp ${dependencies})