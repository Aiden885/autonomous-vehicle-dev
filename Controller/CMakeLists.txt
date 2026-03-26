cmake_minimum_required(VERSION 3.0.0)
project(trajectory_tracking)


find_package(PkgConfig REQUIRED)
pkg_check_modules(ZMQ REQUIRED libzmq)
find_package(Eigen3 REQUIRED)
find_package(Protobuf REQUIRED)
find_package(Qt5 COMPONENTS Widgets REQUIRED)
find_package(OpenCV REQUIRED)
set(CMAKE_CXX_STANDARD 11)
set(PROTO_DIR ${CMAKE_SOURCE_DIR}/proto)

#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)


# command
# protoc -I=. --cpp_out=./ */*.proto


include_directories(
        include
        ${PROTO_DIR}
        ${PROTO_DIR}/Imu
        ${PROTO_DIR}/planning
        ${PROTO_DIR}/common
        ${PROTO_DIR}/control
        ${EIGEN3_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${ZMQ_INCLUDE_DIRS}
        ${Protobuf_INCLUDE_DIRS}
)


file(GLOB RPOTO_CC_FILES
        ${PROTO_DIR}/*/*.pb.cc
)

file(GLOB SRC_C_FILES
        # src/*.c
        src/calcAcceleration.c
        src/getNearestIndex.c
        src/getPreviewIndex.c
        src/mathC.c
        src/pidController.c
        src/purePursuit.c
        src/vehicleModel.c
)



add_executable(controller ${RPOTO_CC_FILES} ${SRC_C_FILES}
        src/main.cpp

        src/controller.cc
        src/debugVisualizer.cpp
        src/steerSafetyFilter.cpp
        src/calcPreviewDistByCurvature.cpp

        src/stateDataStructTrans.cpp
        src/trajDataStructTrans.cpp
        src/
)


target_link_libraries(controller
        ${ZMQ_LIBRARIES}
        ${PROTOBUF_LIBRARIES}
        ${OpenCV_LIBRARIES}
        Qt5::Widgets
        yaml-cpp
        pthread
)

