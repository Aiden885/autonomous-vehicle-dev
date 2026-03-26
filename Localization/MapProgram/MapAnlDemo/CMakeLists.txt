cmake_minimum_required(VERSION 3.16.1)
project(main)

include_directories(
    ./src
    ./dependence)

add_library(libsrc SHARED ./src/mapAnalysisAdd.cpp)
# install(TARGETS libsrc DESTINATION ./lib)

add_library(libdependence SHARED ./dependence/localization_MapAnalysis.cpp)
# install(TARGETS libdependence DESTINATION ./lib)

add_executable(${PROJECT_NAME} ./test/main.cpp)
target_link_libraries (${PROJECT_NAME} libsrc libdependence)
