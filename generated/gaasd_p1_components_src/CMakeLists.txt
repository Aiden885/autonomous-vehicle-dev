cmake_minimum_required(VERSION 3.16)
project(carla_p1_components LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# GCC 9 对 C++20 concepts 支持不完整，需显式启用 -fconcepts。
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 10)
    add_compile_options(-fconcepts)
endif()

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

add_library(carla_p1_components STATIC
    src/CARLAObjectList.cpp
    src/CARLALateralCmd.cpp
    src/CARLAControlCmd.cpp
)

target_include_directories(carla_p1_components
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
