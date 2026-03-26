
cmake_minimum_required(VERSION 3.16.1)
project(Quinticpolynomials)   #project为模块名称，与模块目录名一致，在自动生成工具不具备的情况下，工程名要修改
#以下内容中为加可修改注释的都是模板，不要修改或增加新的内容
find_package(Eigen3 REQUIRED)

include_directories(./src ${EIGEN3_INCLUDE_DIR})

add_library(quinticpolynomials SHARED ./src/quinticpolynomials.cpp)
install(TARGETS quinticpolynomials DESTINATION ./lib)

# add_library(${PROJECT_NAME} SHARED ${SRCS_CPP})
add_executable(${PROJECT_NAME}_test ./test/main.cpp)

target_link_libraries(${PROJECT_NAME}_test quinticpolynomials)
