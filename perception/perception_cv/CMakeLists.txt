cmake_minimum_required(VERSION 3.0 FATAL_ERROR)  

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fPIC")
SET(CMAKE_BUILD_TYPE "Release")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3")

project(perception_cv) #为模块名称，与模块目录名一致，在自动生成工具不具备的情
# 以下内容中为加可修改注释的都是模板，不要修改或增加新的内容 ​

find_package(PkgConfig REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Protobuf REQUIRED)

pkg_check_modules(ZMQ libzmq)
pkg_check_modules(PROTOBUF REQUIRED protobuf)

include_directories(
    /usr/local/include/opencv4
    ./include
    ./proto
    ${YAML_CPP_INCLUDE_DIR}
    ${PROTOBUF_INCLUDE_DIRS}
)

add_library(imgProcess SHARED ./src/img_process.cpp ./src/InversePerspectiveMapping.cpp ./src/CameraInfoOpt.c)
install(TARGETS imgProcess DESTINATION ./lib)

add_library(laneDetection SHARED ./src/lane_detection.cpp)
install(TARGETS laneDetection DESTINATION ./lib)

add_executable (test src/test.cpp)
target_link_libraries (test ${OpenCV_LIBS} yaml-cpp ${PROTOBUF_LIBRARIES} imgProcess laneDetection)

set(CMAKE_BUILD_TYPE Debug)
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g3 -ggdb") #debug模式下开启-g选项