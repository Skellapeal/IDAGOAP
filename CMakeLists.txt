cmake_minimum_required(VERSION 3.22)
project(IDAGOAP)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(IDAGOAP STATIC
        library.cpp
        rida_planner.cpp
        goap_action.cpp
        world_state.cpp
        motive.cpp
        transposition_table.cpp
        plan_result.cpp
        tests/test_actions.h
        tests/test_planner.cpp
        goap_types.cpp
        plan_executor.cpp
        plan_executor.h
        goal_selector.cpp
        goal_selector.h
        async_planner.cpp
        async_planner.h
        heuristics.h
)

target_include_directories(IDAGOAP PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
)

enable_testing()

add_subdirectory(tests)

add_test(NAME idagoap_tests COMMAND idagoap_tests)