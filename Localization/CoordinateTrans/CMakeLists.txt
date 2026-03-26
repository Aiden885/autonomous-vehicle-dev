cmake_minimum_required(VERSION 3.16.1)
project(CoordTrans)
find_package (OpenCV REQUIRED)
include_directories(
    ./src
    ./dependence)

add_library(coordTrans SHARED ./src/CoordinateTransFunctional.cpp)
install(TARGETS coordTrans DESTINATION ./lib)

add_library(dependence SHARED ./dependence/ztgeographycoordinatetransform.cpp)
install(TARGETS dependence DESTINATION ./lib)

add_executable(${PROJECT_NAME} ./test/main.cpp)
target_link_libraries (${PROJECT_NAME} coordTrans dependence)
