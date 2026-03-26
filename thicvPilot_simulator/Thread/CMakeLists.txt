cmake_minimum_required(VERSION 3.10)  

project(Thread)  

ADD_DEFINITIONS(-std=c++11)

find_package(PkgConfig REQUIRED)
find_package(Protobuf REQUIRED)

include_directories(${Protobuf_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_BINARY_DIR})

include_directories (/usr/local/include)
link_directories(/usr/local/lib)

#pkg_check_modules(ZMQ libzmq)
find_package(Boost COMPONENTS thread system REQUIRED)
set( BOOST_THREAD_LIBRARY "-lboost_thread")
set( Boost_SYSTEM_LIBRARY "-lboost_system")
set( LPTHREAD "-lpthread")
set( ZMQ_LIBRARIES "-lzmq")
set( JSON_LIBRARIES "-ljsoncpp")

set(CMAKE_CXX_FLAGS "-std=c++11")
#set(SRC_LIST gpsDriver.cpp imu.pb.cc) 
#add_executable(gpsDriver ${SRC_LIST})
set(SRC_LIST stdThreadDemo.cpp  imu.pb.cc control.pb.cc ztgeographycoordinatetransform.cpp localization_MapAnalysis.cpp) 
add_executable(strThreadDemo ${SRC_LIST})
target_link_libraries (strThreadDemo ${Boost_SYSTEM_LIBRARY} ${Boost_THREAD_LIBRARY} ${ZMQ_LIBRARIES} ${JSON_LIBRARIES} ${LPTHREAD} ${PROTOBUF_LIBRARIES}  yaml-cpp)


#add_executable(strThreadDemo stdThreadDemo.cpp imu.pb.cc)












