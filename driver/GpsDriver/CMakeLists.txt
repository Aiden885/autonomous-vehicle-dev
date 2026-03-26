cmake_minimum_required(VERSION 2.8 )
project(GpsDriver)
ADD_DEFINITIONS(-std=c++11)

find_package(Protobuf REQUIRED)

include_directories(${Protobuf_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_BINARY_DIR})

include_directories (/usr/local/include)
link_directories(/usr/local/lib)

find_package(Boost COMPONENTS thread system REQUIRED)
set( BOOST_THREAD_LIBRARY "-lboost_thread")
set( Boost_SYSTEM_LIBRARY "-lboost_system")
set( LPTHREAD "-lpthread")
set( ZMQ_LIBRARIES "-lzmq")
set( JSON_LIBRARIES "-ljsoncpp")

set(CMAKE_CXX_FLAGS "-std=c++11")
set(SRC_LIST gpsDriver.cpp imu.pb.cc) 
add_executable(gpsDriver ${SRC_LIST})
target_link_libraries (gpsDriver ${Boost_SYSTEM_LIBRARY} ${Boost_THREAD_LIBRARY} ${ZMQ_LIBRARIES} ${JSON_LIBRARIES} ${LPTHREAD} ${PROTOBUF_LIBRARIES})



