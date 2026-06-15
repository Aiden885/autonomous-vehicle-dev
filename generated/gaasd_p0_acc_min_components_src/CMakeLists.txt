cmake_minimum_required(VERSION 3.16)
project(carla_acc_min_components LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# GCC 9 对 C++20 concepts 支持不完整，需显式启用 -fconcepts。
# GCC 10+ 原生支持 C++20 concepts，此标志可忽略，故仅在 GCC 9.x 上追加。
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 10)
    add_compile_options(-fconcepts)
endif()

# 自包含编译：仅使用本工程 include/ 头文件，不引用任何外部框架 include 路径。
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

add_library(carla_acc_min_components STATIC
    src/CARLAACCComputeTargetSpeed.cpp
    src/CARLAACCDriverCommand.cpp
    src/CARLAACCEgoSpeed.cpp
    src/CARLAACCLeadSpeed.cpp
    src/CARLAACCLeadDistance.cpp
    src/CARLAACCLongitudinalCmd.cpp
)

target_include_directories(carla_acc_min_components
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(carla_acc_min_components
        PRIVATE -Wall -Wextra -Wpedantic -Werror)
endif()
