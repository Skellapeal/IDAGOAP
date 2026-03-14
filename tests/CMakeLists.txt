include(FetchContent)

FetchContent_Declare(
        googletest
        URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
        DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

add_executable(idagoap_tests
        test_world_state.cpp
        test_goap_types.cpp
        test_goap_action.cpp
        test_motive.cpp
        test_heuristics.cpp
        test_transposition_table.cpp
        test_goal_selector.cpp
        test_rida_planner.cpp
        test_async_planner.cpp
        test_plan_executor.cpp
        test_goap_types.cpp
        test_goap_action.cpp
        test_motive.cpp
        test_heuristics.cpp
        test_transposition_table.cpp
        test_goal_selector.cpp
        test_rida_planner.cpp
        test_async_planner.cpp
        test_plan_executor.cpp
)

target_link_libraries(idagoap_tests
        IDAGOAP
        GTest::gtest_main
)

target_include_directories(idagoap_tests PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/..
)

include(GoogleTest)
gtest_discover_tests(idagoap_tests)
