cmake_minimum_required(VERSION 3.16)
project(gaasd_lks_lane_components LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# GCC 9 对 C++20 concepts 支持不完整，需显式启用 -fconcepts。
# GCC 10+ 原生支持 C++20 concepts，此标志可忽略，故仅在 GCC 9.x 上追加。
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 10)
    add_compile_options(-fconcepts)
endif()

add_library(gaasd_lks_lane_components STATIC
    src/CARLALKSLaneModel.cpp
)

target_include_directories(gaasd_lks_lane_components
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(gaasd_lks_lane_components
        PRIVATE -Wall -Wextra -Wpedantic -Werror)
endif()
