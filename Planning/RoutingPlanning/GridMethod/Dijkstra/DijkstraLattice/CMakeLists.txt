cmake_minimum_required(VERSION 3.16.1)
project(DijkstraLattice)
find_package (OpenCV 4 REQUIRED)
include_directories(
    ./src
    ./dependence
    /usr/local/include/opencv4)

add_library(dijLatticeFunctional SHARED ./src/dijLatticeFunctional.cpp)
# install(TARGETS dijLatticeFunctional DESTINATION ./lib)

add_library(libDependence SHARED ./dependence/dijkstra_latticeMap.cpp ./dependence/localization_MapAnalysis.cpp)
# install(TARGETS libDependence DESTINATION ./lib)

add_executable(${PROJECT_NAME} ./test/main.cpp ./src/dijLatticeFunctional.cpp)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} libDependence dijLatticeFunctional)
