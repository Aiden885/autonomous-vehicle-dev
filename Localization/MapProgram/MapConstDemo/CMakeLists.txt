
cmake_minimum_required(VERSION 3.16.1)
project(MapConstDemo)   #project为模块名称，与模块目录名一致，在自动生成工具不具备的情况下，工程名要修改
#以下内容中为加可修改注释的都是模板，不要修改或增加新的内容
include_directories(
    ./src 
    ./dependence)

add_library(libdependence SHARED ./dependence/localization_MapConstruction.cpp)
# install(TARGETS libdependence DESTINATION ./lib)

add_library(libsrc SHARED ./src/localizationDeclaration.cpp)
# install(TARGETS libsrc DESTINATION ./lib)

# add_library(${PROJECT_NAME} SHARED ${SRCS_CPP})
add_executable(${PROJECT_NAME} ./test/main.cpp)

target_link_libraries(${PROJECT_NAME} libdependence libsrc)