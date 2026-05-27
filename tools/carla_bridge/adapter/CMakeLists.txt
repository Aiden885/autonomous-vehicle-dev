cmake_minimum_required(VERSION 3.16)
project(carla_gaasd_adapter LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

find_package(PkgConfig REQUIRED)
pkg_check_modules(ZMQ REQUIRED libzmq)

find_package(nlohmann_json QUIET CONFIG)
if(NOT nlohmann_json_FOUND)
    find_path(NLOHMANN_JSON_INCLUDE_DIR NAMES nlohmann/json.hpp)
    if(NOT NLOHMANN_JSON_INCLUDE_DIR)
        message(FATAL_ERROR "nlohmann/json.hpp not found")
    endif()
endif()

add_library(carla_gaasd_adapter SHARED
    src/carla_gaasd_adapter.cpp
)

target_include_directories(carla_gaasd_adapter
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    PRIVATE
        ${ZMQ_INCLUDE_DIRS}
)

if(nlohmann_json_FOUND)
    target_link_libraries(carla_gaasd_adapter PRIVATE nlohmann_json::nlohmann_json)
else()
    target_include_directories(carla_gaasd_adapter PRIVATE ${NLOHMANN_JSON_INCLUDE_DIR})
endif()

target_link_libraries(carla_gaasd_adapter PRIVATE ${ZMQ_LIBRARIES})
target_compile_options(carla_gaasd_adapter PRIVATE ${ZMQ_CFLAGS_OTHER})
target_link_directories(carla_gaasd_adapter PRIVATE ${ZMQ_LIBRARY_DIRS})

set_target_properties(carla_gaasd_adapter PROPERTIES
    OUTPUT_NAME carla_gaasd_adapter
)

add_executable(carla_gaasd_adapter_smoke
    test/adapter_smoke_test.c
)
target_include_directories(carla_gaasd_adapter_smoke PRIVATE include)
target_link_libraries(carla_gaasd_adapter_smoke PRIVATE carla_gaasd_adapter)

add_executable(carla_gaasd_adapter_mock_loop
    test/adapter_mock_loop_test.cpp
)
target_include_directories(carla_gaasd_adapter_mock_loop PRIVATE include ${ZMQ_INCLUDE_DIRS})
target_link_libraries(carla_gaasd_adapter_mock_loop PRIVATE carla_gaasd_adapter ${ZMQ_LIBRARIES})
target_link_directories(carla_gaasd_adapter_mock_loop PRIVATE ${ZMQ_LIBRARY_DIRS})
if(nlohmann_json_FOUND)
    target_link_libraries(carla_gaasd_adapter_mock_loop PRIVATE nlohmann_json::nlohmann_json)
else()
    target_include_directories(carla_gaasd_adapter_mock_loop PRIVATE ${NLOHMANN_JSON_INCLUDE_DIR})
endif()

add_executable(carla_gaasd_adapter_bridge_live
    test/adapter_bridge_live_test.c
)
target_include_directories(carla_gaasd_adapter_bridge_live PRIVATE include)
target_link_libraries(carla_gaasd_adapter_bridge_live PRIVATE carla_gaasd_adapter)

add_executable(carla_gaasd_adapter_lks_live
    test/adapter_lks_live_test.c
)
target_include_directories(carla_gaasd_adapter_lks_live PRIVATE include)
target_link_libraries(carla_gaasd_adapter_lks_live PRIVATE carla_gaasd_adapter)
