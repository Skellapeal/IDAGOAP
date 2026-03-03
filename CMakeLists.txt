cmake_minimum_required(VERSION 3.22)
project(IDAGOAP)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(IDAGOAP STATIC
        library.cpp idaplanner.cpp gaction.cpp gworld_model.cpp
        gmotive.cpp gheuristic.cpp gtranpos_table.cpp
        gtypes.h
)

target_include_directories(IDAGOAP PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
)