cmake_minimum_required(VERSION 3.0.0)
project(pidTest)

find_package(OpenCV REQUIRED)

include_directories(
        .
        ../include
        ${OpenCV_INCLUDE_DIRS}
)


set(SRC_FILES
    ../src/vehicleModel.c
    ../src/pidController.c
    ../src/mathC.c
)

add_executable(pidTest 
        pidTestMain.cpp
        ${SRC_FILES}
        plotSpeed.cpp
)
target_link_libraries(pidTest ${OpenCV_LIBRARIES})


add_executable(pidTestC pidTestMain.c ${SRC_FILES}
)
target_link_libraries(pidTestC)

