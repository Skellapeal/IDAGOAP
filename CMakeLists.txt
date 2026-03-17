cmake_minimum_required(VERSION 3.22)
project(IDAGOAP VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(IDAGOAP STATIC
        goap_types.cpp
        world_state.cpp
        goap_action.cpp
        plan_result.cpp
        transposition_table.cpp
        rida_planner.cpp
        async_planner.cpp
        plan_executor.cpp
        motive.cpp
        goal_selector.cpp
        goap_agent.cpp
        goap_agent.h
)

add_library(rida_goap::rida_goap ALIAS IDAGOAP)

target_include_directories(IDAGOAP PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        $<INSTALL_INTERFACE:include/rida_goap>
)

set_target_properties(IDAGOAP PROPERTIES
        PUBLIC_HEADER
        "rida_goap.h;goap_types.h;world_state.h;goap_action.h;heuristic.h;heuristics.h;\
plan_result.h;rida_planner.h;async_planner.h;plan_executor.h;motive.h;\
goal_selector.h;transposition_table.h"
)

include(GNUInstallDirs)
install(TARGETS IDAGOAP
        EXPORT rida_goap_targets
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/rida_goap
)

install(EXPORT rida_goap_targets
        FILE rida_goap-config.cmake
        NAMESPACE rida_goap::
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/rida_goap
)

option(IDAGOAP_BUILD_TESTS "Build IDAGOAP tests" ON)
if(IDAGOAP_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
    add_test(NAME idagoap_tests COMMAND idagoap_tests)
endif()