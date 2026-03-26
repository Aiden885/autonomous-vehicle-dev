cmake_minimum_required(VERSION 3.0)
project(communicator)

find_package(PCL 1.8 REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(ZMQ libzmq)

add_definitions(${PCL_DEFINITIONS})

set_property(GLOBAL PROPERTY ${PROJECT_NAME}_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include)

add_library(${PROJECT_NAME} SHARED src/CommunicatorBase.cpp src/PtCloudCommunicator.cpp)
target_link_libraries(${PROJECT_NAME} PRIVATE lidarMsg ${ZMQ_LIBRARIES} ${PCL_LIBRARIES})
target_include_directories(${PROJECT_NAME} PRIVATE ${ZMQ_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} PUBLIC include)
