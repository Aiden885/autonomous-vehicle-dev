cmake_minimum_required(VERSION 3.10.2)
project(coms_sdk VERSION 0.1.0)

# # Set compiler to use c++ 11 features
set(CMAKE_CXX_STANDARD 11)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3 ")

# # set build type
set(CMAKE_BUILD_TYPE "Release")

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin) # 设置可执行文件的输出目录
SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib) # 设置库文件的输出目录

# Build libraries
find_package(Threads REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(Protobuf REQUIRED)
pkg_check_modules(ZMQ libzmq)


add_library(msg SHARED include/control.pb.cc)
target_link_libraries(msg PUBLIC ${PROTOBUF_LIBRARIES})
target_include_directories(msg PRIVATE ${PROTOBUF_INCLUDE_DIRS}
    PUBLIC include)

# import asio library
add_subdirectory(asio)

add_library(${PROJECT_NAME}
    src/asyncio/async_serial.cpp
    src/platforms/mobile_base.cpp
    src/platforms/scout_base.cpp
    src/platforms/scout_uart_parser.c
)

target_link_libraries(${PROJECT_NAME} asio Threads::Threads ${PROTOBUF_LIBRARIES})
target_include_directories(${PROJECT_NAME}
    PUBLIC include 
    ${PROTOBUF_LIBRARIES})

add_executable(coms src/coms.cpp)
target_link_libraries(coms msg ${PROJECT_NAME} ${ZMQ_LIBRARIES} ${PROTOBUF_LIBRARIES})
