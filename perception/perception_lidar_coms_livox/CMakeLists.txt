cmake_minimum_required(VERSION 3.0 FATAL_ERROR)  

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fPIC")
SET(CMAKE_BUILD_TYPE "Release")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3")

project(lidarObjectDetection)

find_package(PCL 1.8 REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Protobuf REQUIRED)

pkg_check_modules(ZMQ libzmq)
pkg_check_modules(PROTOBUF REQUIRED protobuf)

include_directories(
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ./include
    ./proto/Imu ./proto/lidarDataProto ./proto/perception ./proto
    ${YAML_CPP_INCLUDE_DIR}
    ${PROTOBUF_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

link_libraries(pthread)
PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS prediction.proto)


add_library(imuMsg SHARED ../proto/Imu/imu.pb.cc)
target_link_libraries(imuMsg PUBLIC ${PROTOBUF_LIBRARIES})
target_include_directories(imuMsg PRIVATE ${PROTOBUF_INCLUDE_DIRS}  PUBLIC ../proto/Imu/)


add_library(lidarMsg SHARED ../proto/lidarDataProto/cloudData.pb.cc)
target_link_libraries(lidarMsg PUBLIC ${PROTOBUF_LIBRARIES})
target_include_directories(lidarMsg PRIVATE ${PROTOBUF_INCLUDE_DIRS}  PUBLIC ../proto/lidarDataProto/)


add_library(objectMsg SHARED ../proto/perception/perception.pb.cc)
target_link_libraries(objectMsg PUBLIC ${PROTOBUF_LIBRARIES})
target_include_directories(objectMsg PRIVATE ${PROTOBUF_INCLUDE_DIRS}  PUBLIC ../proto/perception/)


add_executable (lidarObjectDetection src/baseLidarObjectDetector.cpp src/patchworkpp.cpp src/classicLidarDetector.cpp src/functionLidarDetector.cpp src/perceptionLidarObjectDetection.cpp ${PROTO_SRCS} ${PROTO_HDRS})
target_link_libraries (lidarObjectDetection imuMsg lidarMsg objectMsg ${ZMQ_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} yaml-cpp ${PROTOBUF_LIBRARIES})

# add_executable (test src/test.cpp src/lidarTools.cpp)
# target_link_libraries(test imuMsg lidarMsg objectMsg ${ZMQ_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} yaml-cpp ${PROTOBUF_LIBRARIES})

set(CMAKE_BUILD_TYPE Debug)
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g3 -ggdb") #debug模式下开启-g选项
##SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall") #如果set(CMAKE_BUILD_TYPE RELEASE)则使用该配置



