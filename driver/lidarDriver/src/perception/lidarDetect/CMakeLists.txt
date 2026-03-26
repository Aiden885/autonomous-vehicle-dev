cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

project(lidarObjectDetection)

find_package(PCL 1.10 REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP jsoncpp)

add_definitions(${PCL_DEFINITIONS})

add_executable(${PROJECT_NAME} src/lidar_detection.cpp src/lidar_process.cpp)
target_link_libraries(${PROJECT_NAME} PRIVATE communicator ${PCL_LIBRARIES} ${JSONCPP_LIBRARIES})
target_include_directories(${PROJECT_NAME} PRIVATE ${PCL_INCLUDE_DIRS} ${JSONCPP_INCLUDE_DIRS} include)

add_executable(test src/test.cpp)
target_link_libraries(test PRIVATE ${PCL_LIBRARIES})
target_include_directories(test PRIVATE ${PCL_INCLUDE_DIRS})
