cmake_minimum_required(VERSION 3.22)
project(IDAGOAP VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(UE_PLUGIN_INCLUDE_DIR
        "E:/Unreal Projects/FinalYearProject/Plugins/IDAGOAP/ThirdParty/IDAGOAPCore/Include"
        CACHE PATH "UE plugin include destination")
set(UE_PLUGIN_LIB_DIR
        "E:/Unreal Projects/FinalYearProject/Plugins/IDAGOAP/ThirdParty/IDAGOAPCore/Lib/Win64"
        CACHE PATH "UE plugin lib destination")

add_library(IDAGOAP STATIC
        src/goap_types.cpp
        src/world_state.cpp
        src/goap_action.cpp
        src/plan_result.cpp
        src/transposition_table.cpp
        src/rida_planner.cpp
        src/async_planner.cpp
        src/plan_executor.cpp
        src/motive.cpp
        src/goal_selector.cpp
        src/goap_agent.cpp
)

add_library(rida_goap::rida_goap ALIAS IDAGOAP)

target_include_directories(IDAGOAP PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include/rida_goap>
)

set_target_properties(IDAGOAP PROPERTIES
        PUBLIC_HEADER
        "include/rida_goap.h;\
        include/goap_types.h;\
        include/world_state.h;\
        include/goap_action.h;\
        include/heuristic.h;\
        include/heuristics.h;\
        include/plan_result.h;\
        include/rida_planner.h;\
        include/async_planner.h;\
        include/plan_executor.h;\
        include/motive.h;\
        include/goal_selector.h;\
        include/transposition_table.h;\
        include/goap_agent.h"
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

add_custom_command(TARGET IDAGOAP POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E make_directory "${UE_PLUGIN_LIB_DIR}"
        COMMAND ${CMAKE_COMMAND} -E make_directory "${UE_PLUGIN_INCLUDE_DIR}"
        COMMAND ${CMAKE_COMMAND} -E copy
        $<TARGET_FILE:IDAGOAP>
        "${UE_PLUGIN_LIB_DIR}/IDAGOAP.lib"
        COMMAND ${CMAKE_COMMAND} -E copy_directory
        "${CMAKE_CURRENT_SOURCE_DIR}/include"
        "${UE_PLUGIN_INCLUDE_DIR}"
        COMMENT "Copying IDAGOAP to UE plugin..."
)

option(IDAGOAP_BUILD_TESTS "Build IDAGOAP tests" ON)
if(IDAGOAP_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
    add_test(NAME idagoap_tests COMMAND idagoap_tests)
endif()