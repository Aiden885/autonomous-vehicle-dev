cmake_minimum_required(VERSION 3.0 FATAL_ERROR)
project(AILight)

set(CMAKE_CXX_STANDARD 11)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fPIC")
SET(CMAKE_BUILD_TYPE "Release")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wno-stringop-overflow")

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

add_subdirectory(src/tools/communicator)
add_subdirectory(src/proto)
#add_subdirectory(src/perception/lidarDetect)
add_subdirectory(src/driver/lidarDriver)
