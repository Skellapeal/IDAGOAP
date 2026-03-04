add_executable(idagoap_tests
        test_planner.cpp
)

target_link_libraries(idagoap_tests PRIVATE IDAGOAP)

target_include_directories(idagoap_tests PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/..
)