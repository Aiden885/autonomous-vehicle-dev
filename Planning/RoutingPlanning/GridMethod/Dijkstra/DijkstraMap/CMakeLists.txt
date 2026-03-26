cmake_minimum_required(VERSION 3.16.1)
project(DijkstraMap)
find_package (OpenCV REQUIRED)
include_directories(
    ./src
    ./dependence
    ${OpenCV_INCLUDE_DIRS})

add_library(dijkstraMapFunctional SHARED ./src/dijkstraMapFunctional.cpp)
install(TARGETS dijkstraMapFunctional DESTINATION ./lib)

add_library(dependence SHARED ./dependence/dijkstra_topologyMap.cpp ./dependence/localization_MapAnalysis.cpp)
install(TARGETS dependence DESTINATION ./lib)

add_executable(${PROJECT_NAME} ./test/main_newdijkstra_topologyMap.cpp)
target_link_libraries (${PROJECT_NAME} ${OpenCV_LIBS} dijkstraMapFunctional dependence)
