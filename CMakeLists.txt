cmake_minimum_required(VERSION 3.20)
project(aco_vrp LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(OpenMP REQUIRED)
add_executable(aco_vrp main.cpp Graph.cpp Graph.h ShrinkGraph.cpp ShrinkGraph.h SVRPAntColony.cpp SVRPAntColony.h VRPAntColony.cpp VRPAntColony.h Label.h SolverResult.h ExperimentIO.cpp ExperimentIO.h)
target_link_libraries(aco_vrp PRIVATE OpenMP::OpenMP_CXX)

include(CTest)
if(BUILD_TESTING)
find_package(Python3 COMPONENTS Interpreter REQUIRED)
add_executable(test_experiment_io tests/test_experiment_io.cpp ExperimentIO.cpp ExperimentIO.h SolverResult.h)
target_include_directories(test_experiment_io PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_definitions(test_experiment_io PRIVATE
        TEST_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
        TEST_BINARY_DIR="${CMAKE_CURRENT_BINARY_DIR}")
# This test uses assert(), including in Release builds.
target_compile_options(test_experiment_io PRIVATE
        $<$<CXX_COMPILER_ID:MSVC>:/UNDEBUG>
        $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-UNDEBUG>)
add_test(NAME experiment_io COMMAND test_experiment_io)
add_test(NAME analysis_tools COMMAND ${Python3_EXECUTABLE} -m unittest
        tests.test_analysis_tools tests.test_ilp_parser tests.test_journal_aco_analysis)
set_tests_properties(analysis_tools PROPERTIES
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        ENVIRONMENT "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/tools")
add_test(NAME submission_scripts COMMAND ${Python3_EXECUTABLE} -m unittest
        tests.test_submission_scripts)
set_tests_properties(submission_scripts PROPERTIES WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
add_test(NAME cli_contract COMMAND ${Python3_EXECUTABLE}
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_cli_contract.py
        $<TARGET_FILE:aco_vrp>
        ${CMAKE_CURRENT_SOURCE_DIR}/art/u100/graph_001_n100_c0_u1_seed20063.csv)
set_tests_properties(cli_contract PROPERTIES WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
add_test(NAME solver_correctness COMMAND ${Python3_EXECUTABLE}
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_solver_correctness.py $<TARGET_FILE:aco_vrp>)
add_test(NAME input_integrity COMMAND ${Python3_EXECUTABLE}
        ${CMAKE_CURRENT_SOURCE_DIR}/tools/verify_instances.py)
add_test(NAME provenance COMMAND ${Python3_EXECUTABLE} -m unittest tests.test_provenance)
set_tests_properties(provenance PROPERTIES
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        ENVIRONMENT "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/tools")
endif()
