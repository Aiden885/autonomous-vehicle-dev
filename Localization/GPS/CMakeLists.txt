
cmake_minimum_required(VERSION 3.16.1)
project(GPS)   #project为模块名称，与模块目录名一致，在自动生成工具不具备的情况下，工程名要修改
#以下内容中为加可修改注释的都是模板，不要修改或增加新的内容
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_AUTOMOC ON)
# set(Qt5_ROOT "/home/pnc/Qt5.14.1/5.14.1/gcc_64")
# set(Qt5_DIR "${Qt5_ROOT}/lib/cmake/Qt5")
# set(Qt5_LIBRARY_DIRS "${Qt5_ROOT}/lib")

# find_package(PkgConfig REQUIRED)
find_package(Qt5 COMPONENTS Widgets REQUIRED)
find_package(Qt5 COMPONENTS SerialPort REQUIRED)
find_package(Qt5 COMPONENTS Core REQUIRED)
find_package(Qt5 COMPONENTS Network REQUIRED)
find_package(Protobuf REQUIRED)

# pkg_check_modules(PROTOBUF protobuf)

file(GLOB UIS "${CMAKE_CURRENT_SOURCE_DIR}/ui/*.ui")
Qt5_wrap_ui(UI_HEADERS ${UIS})


link_directories(${Protobuf_LIBRARY_DIRS} ${Qt5_LIBRARY_DIRS} )
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/dependence
                  ${CMAKE_CURRENT_SOURCE_DIR}/proto/Imu
				  ${CMAKE_CURRENT_SOURCE_DIR}/src
				  ${Protobuf_INCLUDE_DIRS})

file(GLOB_RECURSE SRCS_CPP  ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp 
			${CMAKE_CURRENT_SOURCE_DIR}/dependence/*.cpp 
			${CMAKE_CURRENT_SOURCE_DIR}/test/*.cpp 
			${CMAKE_CURRENT_SOURCE_DIR}/proto/Imu/*.cc)

add_executable(${PROJECT_NAME} ${SRCS_CPP})
target_link_libraries(${PROJECT_NAME} Qt5::Widgets)
target_link_libraries(${PROJECT_NAME} Qt5::SerialPort)
target_link_libraries(${PROJECT_NAME} Qt5::Network)
target_link_libraries(${PROJECT_NAME} Qt5::Network)
# target_link_libraries(${PROJECT_NAME} PkgConfig::PROTOBUF)
target_link_libraries(${PROJECT_NAME} zmq)

target_link_libraries(${PROJECT_NAME} ${Protobuf_LIBRARIES})

