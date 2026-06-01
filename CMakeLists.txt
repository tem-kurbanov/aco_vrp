cmake_minimum_required(VERSION 3.20)
project(aco_vrp)

set(CMAKE_CXX_STANDARD 17)
find_package(OpenMP REQUIRED)
add_executable(aco_vrp main.cpp Graph.cpp Graph.h ShrinkGraph.cpp ShrinkGraph.h SVRPAntColony.cpp SVRPAntColony.h VRPAntColony.cpp VRPAntColony.h Label.h)
target_link_libraries(aco_vrp PRIVATE OpenMP::OpenMP_CXX)